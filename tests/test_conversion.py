"""Regression tests for ros_to_omega_prime.py.

Run inside the omega-prime-ros image (ROS 2, perception_msgs and omega-prime are needed), e.g.

    docker run --rm -v "$PWD":/repo omega-prime-ros:ci \
        bash -c "pip install -q pytest && pytest -q /repo/tests"

Each test writes a small rosbag2 (mcap) with rosbag2_py and converts it.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "omega-prime-ros"))

import betterosi  # noqa: E402
import omega_prime  # noqa: E402
import perception_msgs_utils as pmu  # noqa: E402
import polars as pl  # noqa: E402
import ros_to_omega_prime as conv  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from perception_msgs.msg import EgoData, Object, ObjectClassification, ObjectList  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

T0 = 1_790_000_000_000_000_000
DT = 100_000_000  # 10 Hz
OBJ_TOPIC = "/test/object_list"
EGO_TOPIC = "/test/ego_data"
UTM_TO_MAP = (291810.99986987223, 5630289.000249481, 0.0)
MAP_TO_SITE = (276.91971109469887, -830.2784496536478, 255.0)
SITE_ORIGIN = tuple(a + b for a, b in zip(UTM_TO_MAP, MAP_TO_SITE))
_MOT = betterosi.MovingObjectType
_VCT = betterosi.MovingObjectVehicleClassificationType


# --- bag writing -------------------------------------------------------------------------------------------------


def _stamp(msg_stamp, ns: int) -> None:
    msg_stamp.sec = ns // 1_000_000_000
    msg_stamp.nanosec = ns % 1_000_000_000


def tf_msg(parent: str, child: str, ns: int, xyz=(0.0, 0.0, 0.0), yaw: float = 0.0) -> TFMessage:
    t = TransformStamped()
    _stamp(t.header.stamp, ns)
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return TFMessage(transforms=[t])


def make_obj(oid, ns, frame, x, y, yaw=0.0, vlon=0.0, vlat=0.0, cls=4, model=16) -> Object:
    o = Object()
    o.id = oid
    pmu.initialize_state(o, model)
    _stamp(o.state.header.stamp, ns)
    o.state.header.frame_id = frame
    pmu.set_x(o, float(x))
    pmu.set_y(o, float(y))
    if model != 20:  # TRAFFICLIGHT has only x/y/z
        pmu.set_yaw(o, float(yaw))
        pmu.set_vel_lon(o, float(vlon))
        pmu.set_vel_lat(o, float(vlat))
        pmu.set_length(o, 4.5)
        pmu.set_width(o, 1.8)
        pmu.set_height(o, 1.5)
    if cls is not None:
        o.state.classifications = [ObjectClassification(type=cls, probability=1.0)]
    return o


def make_list(ns: int, frame: str, objs: list) -> ObjectList:
    m = ObjectList()
    _stamp(m.header.stamp, ns)
    m.header.frame_id = frame
    m.objects = objs
    return m


def make_ego(ns: int, frame: str, vid: int, x: float, y: float) -> EgoData:
    e = EgoData()
    _stamp(e.header.stamp, ns)
    e.header.frame_id = frame
    e.vehicle_id = vid
    e.length, e.width, e.height = 4.0, 1.8, 1.5
    pmu.initialize_state(e, 1)
    _stamp(e.state.header.stamp, ns)
    e.state.header.frame_id = frame
    pmu.set_x(e, float(x))
    pmu.set_y(e, float(y))
    pmu.set_vel_lon(e, 5.0)
    return e


class Bag:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._writer = SequentialWriter()
        self._writer.open(StorageOptions(uri=str(path), storage_id="mcap"), ConverterOptions("", ""))
        self._topics: set[str] = set()

    def write(self, topic: str, typ: str, msg, t_ns: int) -> None:
        if topic not in self._topics:
            self._writer.create_topic(
                TopicMetadata(id=len(self._topics), name=topic, type=typ, serialization_format="cdr")
            )
            self._topics.add(topic)
        self._writer.write(topic, serialize_message(msg), int(t_ns))

    def static_chain(self, site: str = "intersection") -> None:
        self.write("/tf_static", "tf2_msgs/msg/TFMessage", tf_msg("utm_32N", "map", T0, UTM_TO_MAP), T0)
        self.write("/tf_static", "tf2_msgs/msg/TFMessage", tf_msg("map", site, T0, MAP_TO_SITE), T0)

    def tf(self, msg: TFMessage, t_ns: int) -> None:
        self.write("/tf", "tf2_msgs/msg/TFMessage", msg, t_ns)

    def objects(self, msg: ObjectList, t_ns: int | None = None) -> None:
        self.write(OBJ_TOPIC, "perception_msgs/msg/ObjectList", msg, t_ns if t_ns is not None else _ns(msg))

    def ego(self, msg: EgoData) -> None:
        self.write(EGO_TOPIC, "perception_msgs/msg/EgoData", msg, _ns(msg))

    def close(self) -> Path:
        del self._writer
        return self.path


def _ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def convert(bag: Path, ego: bool = False, projection_frame: str = "intersection") -> Path:
    return conv.convert_bag_to_omega_prime(
        bag_dir=bag,
        output_dir=bag.parent / "out",
        ego_data_topic=EGO_TOPIC if ego else None,
        object_list_topic=OBJ_TOPIC,
        fixed_frame="utm_32N",
        projection_frame=projection_frame,
    )


def load_df(path: Path) -> pl.DataFrame:
    return omega_prime.Recording.from_file(str(path), apply_proj=False).df.sort("total_nanos", "idx")


def read_gts(path: Path) -> list:
    return list(betterosi.read(str(path), return_ground_truth=True))


# --- tests ---------------------------------------------------------------------------------------------------------


def test_values_and_projection_offset(tmp_path):
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    for k in range(10):
        ns = T0 + k * DT
        bag.objects(make_list(ns, "intersection", [make_obj(1, ns, "intersection", 1.0 + k, 2.0, 0.5, 4.0, 1.0)]))
    df = load_df(convert(bag.close()))

    assert df.height == 10
    row = df.row(0, named=True)
    assert (row["x"], row["y"], row["yaw"]) == pytest.approx((1.0, 2.0, 0.5))
    # lon/lat -> parent frame
    assert row["vel_x"] == pytest.approx(4.0 * math.cos(0.5) - 1.0 * math.sin(0.5))
    assert row["vel_y"] == pytest.approx(4.0 * math.sin(0.5) + 1.0 * math.cos(0.5))
    assert (row["length"], row["width"], row["height"]) == pytest.approx((4.5, 1.8, 1.5))
    for gt in read_gts(tmp_path / "out" / "bag.omega-prime.mcap"):
        assert gt.proj_string == "EPSG:32632"
        offset = gt.proj_frame_offset.position
        assert (offset.x, offset.y, offset.z) == pytest.approx(SITE_ORIGIN)


def test_transform_into_projection_frame(tmp_path):
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    bag.write(
        "/tf_static",
        "tf2_msgs/msg/TFMessage",
        tf_msg("intersection", "sensor", T0, (10.0, 0.0, 0.0), math.pi / 2),
        T0,
    )
    for k in range(5):
        ns = T0 + k * DT
        bag.objects(make_list(ns, "sensor", [make_obj(1, ns, "sensor", 1.0, 0.0, 0.0, 5.0, 1.0)]))
    df = load_df(convert(bag.close()))

    row = df.row(0, named=True)
    assert (row["x"], row["y"], row["yaw"]) == pytest.approx((10.0, 1.0, math.pi / 2))
    assert (row["vel_x"], row["vel_y"]) == pytest.approx((-1.0, 5.0))
    assert df["total_nanos"].to_list() == [T0 + k * DT for k in range(5)]


def test_unresolved_messages_do_not_remove_resolved_rows(tmp_path):
    """K1: an unresolvable ObjectList must not delete the EgoData rows of the same timestamp."""
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    for k in range(10):
        ns = T0 + k * DT
        bag.ego(make_ego(ns, "intersection", 999, k * 0.5, 0.0))
        frame = "nowhere" if k % 2 else "intersection"
        bag.objects(make_list(ns, frame, [make_obj(1, ns, frame, 5.0, 5.0)]))
    df = load_df(convert(bag.close(), ego=True))

    assert (df["idx"] == 999).sum() == 10
    assert (df["idx"] == 1).sum() == 5


def test_object_stamps_get_a_projection_offset(tmp_path):
    """K2: object stamps that differ from the header stamp still get the projection offset."""
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    for k in range(10):
        ns = T0 + k * DT
        bag.objects(make_list(ns, "intersection", [make_obj(1, ns - 30_000_000, "intersection", 5.0 + k, 0.0)]))
    out = convert(bag.close())

    gts = read_gts(out)
    assert len(gts) == 10
    assert all(gt.proj_frame_offset is not None for gt in gts)
    assert gts[0].timestamp.nanos == (T0 - 30_000_000) % 1_000_000_000


def test_state_model_without_velocity_and_dimensions(tmp_path):
    """K3: a TRAFFICLIGHT object (x/y/z only) is written with zero velocity and dimensions instead of aborting."""
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    for k in range(3):
        ns = T0 + k * DT
        objs = [make_obj(1, ns, "intersection", 1, 1, 0, 5), make_obj(2, ns, "intersection", 3, 3, model=20, cls=None)]
        bag.objects(make_list(ns, "intersection", objs))
    df = load_df(convert(bag.close()))

    light = df.filter(pl.col("idx") == 2).row(0, named=True)
    assert (light["x"], light["y"]) == pytest.approx((3.0, 3.0))
    assert (light["vel_x"], light["length"], light["width"]) == (0.0, 0.0, 0.0)
    assert light["type"] == int(_MOT.UNKNOWN)


def test_bag_without_objects_raises_clear_error(tmp_path):
    """K4"""
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    for k in range(5):
        bag.objects(make_list(T0 + k * DT, "intersection", []))
    with pytest.raises(RuntimeError, match="No objects to convert"):
        convert(bag.close())


def test_messages_recorded_long_after_their_stamp_resolve(tmp_path):
    """K5: TF lookups for messages recorded 12 s after their stamp (more than tf2's default 10 s cache)."""
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    for k in range(300):
        ns = T0 + k * DT
        bag.tf(tf_msg("intersection", "sensor", ns, (10.0, 0.0, 0.0)), ns)
        if k >= 120:
            obj_ns = ns - 12_000_000_000
            bag.objects(make_list(obj_ns, "sensor", [make_obj(1, obj_ns, "sensor", 1, 0)]), t_ns=ns)
    df = load_df(convert(bag.close()))

    assert df.height == 180
    assert df["x"].to_list() == pytest.approx([11.0] * 180)


def test_unresolvable_frame_with_dynamic_tf_stays_fast(tmp_path):
    """K6: messages in a frame without TF must not be retried on every /tf message."""
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    for k in range(2 * 60 * 50):  # 2 min of /tf at 50 Hz, ObjectLists at 10 Hz, every second one unresolvable
        ns = T0 + k * 20_000_000
        bag.tf(tf_msg("intersection", "dyn", ns), ns)
        if k % 5 == 0:
            frame = "nowhere" if k % 10 == 0 else "intersection"
            bag.objects(make_list(ns, frame, [make_obj(1, ns, frame, 1, 0)]))
    path = bag.close()

    started = time.monotonic()
    df = load_df(convert(path))

    assert df.height == 600
    assert time.monotonic() - started < 15.0  # v1.1.1 needed about 60 s


def test_duplicate_ego_and_object_id_raises(tmp_path):
    """K7"""
    bag = Bag(tmp_path / "bag")
    bag.static_chain()
    for k in range(3):
        ns = T0 + k * DT
        bag.ego(make_ego(ns, "intersection", 1, 0.0, 0.0))
        bag.objects(make_list(ns, "intersection", [make_obj(1, ns, "intersection", 30.0, 0.0)]))
    with pytest.raises(ValueError, match="share timestamp and ID"):
        convert(bag.close(), ego=True)


@pytest.mark.parametrize(
    ("cls", "expected"),
    # expected: (MovingObjectType, vehicle classification type, vehicle role)
    [
        (None, (_MOT.UNKNOWN, -1, -1)),
        (0, (_MOT.UNKNOWN, -1, -1)),
        (1, (_MOT.PEDESTRIAN, -1, -1)),
        (2, (_MOT.VEHICLE, _VCT.BICYCLE, 2)),
        (4, (_MOT.VEHICLE, _VCT.CAR, 2)),
        (5, (_MOT.VEHICLE, _VCT.HEAVY_TRUCK, 2)),
        (7, (_MOT.VEHICLE, _VCT.BUS, 2)),
        (8, (_MOT.ANIMAL, -1, -1)),
        (9, (_MOT.OTHER, -1, -1)),
        (12, (_MOT.VEHICLE, _VCT.WHEELCHAIR, 2)),
        (13, (_MOT.VEHICLE, conv._VEHICLE_MAP[13], 2)),
        (50, (_MOT.VEHICLE, _VCT.CAR, 2)),
        (51, (_MOT.VEHICLE, _VCT.HEAVY_TRUCK, 2)),
        (100, (_MOT.OTHER, -1, -1)),
    ],
)
def test_class_mapping(cls, expected):
    obj = make_obj(1, T0, "intersection", 0, 0, cls=cls)
    mot, role, subtype = conv._class_to_osi(obj)
    assert (mot, subtype, role) == tuple(int(e) for e in expected)


def test_bags_with_the_same_name_are_rejected(tmp_path, monkeypatch):
    """K9"""
    for site in ("a", "b"):
        bag = Bag(tmp_path / "input" / site / "bag")
        bag.static_chain()
        bag.objects(make_list(T0, "intersection", [make_obj(1, T0, "intersection", 1, 0)]))
        bag.close()
    monkeypatch.setattr(
        sys,
        "argv",
        ["ros_to_omega_prime.py", "--bag-dir", str(tmp_path / "input"), "--op-dir", str(tmp_path / "out"),
         "--object_list_topic", OBJ_TOPIC, "--projection_frame", "intersection", "--map", ""],
    )
    with pytest.raises(SystemExit, match="same directory name"):
        conv.main()
    assert not (tmp_path / "out").exists()
