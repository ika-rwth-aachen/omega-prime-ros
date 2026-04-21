"""
ROS 2 to omega-prime converter
    reads perception_msgs/ObjectList messages and perception_msgs/EgoData from ROS 2 bag files
    converts and emits omega-prime mcap files.

The CLI can process specific bag directories or scan a data root for rosbag2
folders (identified via metadata.yaml).
"""

from __future__ import annotations

import argparse
import math
import os
from collections import deque
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import betterosi
import numpy as np
import perception_msgs_utils as pmu
import polars as pl
import yaml
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException
from tf2_perception_msgs import do_transform_ego_data, do_transform_object_list

import omega_prime
from omega_prime.map import ProjectionOffset

# Legacy numpy aliases expected by perception_msgs_utils/tf_transformations
if not hasattr(np, "float"):
    np.float = float  # type: ignore[attr-defined]
if not hasattr(np, "maximum_sctype"):

    def _np_maximum_sctype(dtype):
        return np.dtype(np.float64).type

    np.maximum_sctype = _np_maximum_sctype  # type: ignore[attr-defined]

_VCT = betterosi.MovingObjectVehicleClassificationType
_ROLE = betterosi.MovingObjectVehicleClassificationRole
_MOT = betterosi.MovingObjectType


@dataclass(slots=True)
class MessageSample:
    topic_name: str
    msg_type_name: str
    timestamp_nanos: int
    frame_id: str
    msg: Any


def utm_to_epsg(zone: int, northern: bool = True) -> str:
    if not isinstance(zone, int):
        raise TypeError(f"Zone must be an integer {zone}")
    base = 32600 if northern else 32700
    epsg_str = f"EPSG:{int(base + zone)}"
    return epsg_str


def _class_to_osi(obj) -> tuple[int, int, int]:
    mot = int(_MOT.TYPE_OTHER)
    role = -1
    subtype = -1

    if obj.state.classifications:
        c = pmu.get_class_with_highest_probability(obj)
        ct = int(c.type)
    else:
        ct = 0

    vehicle_map = {
        4: _VCT.TYPE_CAR,
        5: _VCT.TYPE_HEAVY_TRUCK,
        6: _VCT.TYPE_DELIVERY_VAN,
        7: _VCT.TYPE_BUS,
        10: _VCT.TYPE_TRAIN,
        3: _VCT.TYPE_MOTORBIKE,
        2: _VCT.TYPE_BICYCLE,
        11: _VCT.TYPE_TRAILER,
        50: _VCT.TYPE_OTHER,
        51: _VCT.TYPE_OTHER,
        52: _VCT.TYPE_OTHER,
    }

    if ct == 1:
        mot = int(_MOT.TYPE_PEDESTRIAN)
    elif ct == 8:
        mot = int(_MOT.TYPE_ANIMAL)
    elif ct in vehicle_map:
        mot = int(_MOT.TYPE_VEHICLE)
        role = int(_ROLE.ROLE_CIVIL)
        subtype = int(vehicle_map[ct])
    elif ct in (0, 9, 100):
        mot = int(_MOT.TYPE_OTHER)

    return mot, role, subtype


def _stamp_to_nanos(stamp: Any) -> int:
    return int(Time.from_msg(stamp).nanoseconds)


def _message_type_name(msg: Any) -> str:
    return getattr(type(msg), "__name__", str(type(msg)))


def _header_timestamp_nanos(msg: Any) -> int:
    header = getattr(msg, "header", None)
    if header is None or not hasattr(header, "stamp"):
        raise ValueError(f"Message {_message_type_name(msg)} has no header.stamp")
    return _stamp_to_nanos(header.stamp)


def _object_list_header_timestamp_nanos(msg: Any) -> int:
    msg_type_name = _message_type_name(msg)
    if msg_type_name != "ObjectList":
        raise ValueError(f"Expected ObjectList message, got {msg_type_name}")
    return _header_timestamp_nanos(msg)


def _canonical_message_timestamp_nanos(msg: Any) -> int:
    msg_type_name = _message_type_name(msg)
    if msg_type_name == "EgoData":
        return _header_timestamp_nanos(msg)
    if msg_type_name == "ObjectList":
        return _object_list_header_timestamp_nanos(msg)
    raise ValueError(
        f"Unexpected message type: {msg_type_name}. Supported types are EgoData and ObjectList."
    )


def _object_to_row(obj) -> dict[str, Any]:
    obj_type_name = _message_type_name(obj)

    if obj_type_name == "Object":
        total_nanos = _stamp_to_nanos(obj.state.header.stamp)
        idx = int(obj.id)
        width = float(pmu.get_width(obj))
        length = float(pmu.get_length(obj))
        height = float(pmu.get_height(obj))

    elif obj_type_name == "EgoData":
        total_nanos = _canonical_message_timestamp_nanos(obj)
        idx = int(obj.vehicle_id)
        width = float(obj.width)
        length = float(obj.length)
        height = float(obj.height)
    else:
        raise ValueError(f"Unexpected object type: {obj_type_name}. Supported types are Object and EgoData.")

    pos = pmu.get_center_position(obj)

    try:
        vel = pmu.get_velocity(obj)
    except AttributeError:
        vel = pmu.Vector3D(x=0.0, y=0.0, z=0.0)

    try:
        acc = pmu.get_acceleration(obj)
    except AttributeError:
        acc = pmu.Vector3D(x=0.0, y=0.0, z=0.0)

    yaw = 0.0
    roll = 0.0
    pitch = 0.0

    try:
        if pmu.index_yaw(obj.state.model_id) is not None:
            yaw = pmu.get_yaw(obj)
    except pmu.UnknownStateEntryError:
        yaw = 0.0
    try:
        if pmu.index_roll(obj.state.model_id) is not None:
            roll = pmu.get_roll(obj)
    except pmu.UnknownStateEntryError:
        roll = 0.0
    try:
        if pmu.index_pitch(obj.state.model_id) is not None:
            pitch = pmu.get_pitch(obj)
    except pmu.UnknownStateEntryError:
        pitch = 0.0

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vel_x = vel.x * cos_yaw - vel.y * sin_yaw
    vel_y = vel.x * sin_yaw + vel.y * cos_yaw
    acc_x = acc.x * cos_yaw - acc.y * sin_yaw
    acc_y = acc.x * sin_yaw + acc.y * cos_yaw

    mot, role, subtype = _class_to_osi(obj)

    return {
        "total_nanos": int(total_nanos),
        "idx": idx,
        "x": float(pos.x),
        "y": float(pos.y),
        "z": float(getattr(pos, "z", 0.0)),
        "vel_x": float(vel_x),
        "vel_y": float(vel_y),
        "vel_z": float(getattr(vel, "z", 0.0)),
        "acc_x": float(acc_x),
        "acc_y": float(acc_y),
        "acc_z": float(getattr(acc, "z", 0.0)),
        "length": length,
        "width": width,
        "height": height,
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
        "type": int(mot),
        "role": int(role),
        "subtype": int(subtype),
    }


def _load_metadata(bag_dir: Path) -> dict[str, Any]:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.exists():
        raise FileNotFoundError(f"metadata.yaml not found in {bag_dir}")
    with metadata_path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _extract_proj_offset(msg) -> tuple[int, ProjectionOffset]:
    transformation = msg.transforms[0] if hasattr(msg, "transforms") else msg
    translation = transformation.transform.translation
    rotation = transformation.transform.rotation
    ts = _stamp_to_nanos(transformation.header.stamp)

    offset = ProjectionOffset(
        x=float(translation.x if hasattr(translation, "x") else 0.0),
        y=float(translation.y if hasattr(translation, "y") else 0.0),
        z=float(translation.z if hasattr(translation, "z") else 0.0),
        yaw=_yaw_from_quaternion(rotation),
    )
    return ts, offset


def _copy_stamp(dst_stamp: Any, src_stamp: Any) -> None:
    dst_stamp.sec = int(src_stamp.sec)
    dst_stamp.nanosec = int(src_stamp.nanosec)


def _normalize_object_list_object_timestamps(msg: Any) -> int:
    """Remap mismatching object timestamps to the ObjectList header timestamp."""
    if _message_type_name(msg) != "ObjectList":
        raise ValueError(f"Expected ObjectList message, got {_message_type_name(msg)}")

    header = getattr(msg, "header", None)
    if header is None or not hasattr(header, "stamp"):
        raise ValueError("ObjectList message has no header.stamp")

    header_stamp = _object_list_header_timestamp_nanos(msg)
    normalized_count = 0

    for obj in msg.objects:
        state_header = getattr(getattr(obj, "state", None), "header", None)
        if state_header is None or not hasattr(state_header, "stamp"):
            continue

        obj_stamp = _stamp_to_nanos(state_header.stamp)
        if obj_stamp == header_stamp:
            continue

        print(
            f"Info: Normalizing Object ID {obj.id} timestamp from {obj_stamp} to ObjectList header timestamp {header_stamp}."
        )
        _copy_stamp(state_header.stamp, header.stamp)
        normalized_count += 1

    return normalized_count


def check_object_consistency(msg) -> None:
    """Check that all objects in an ObjectList message have the same timestamp and frame_id as the header."""
    if not hasattr(msg, "objects"):
        return

    header_stamp = _object_list_header_timestamp_nanos(msg) if hasattr(msg, "header") else None
    header_frame_id = msg.header.frame_id if hasattr(msg, "header") else None

    for obj in msg.objects:
        obj_stamp = _stamp_to_nanos(obj.state.header.stamp) if hasattr(obj.state, "header") else None
        obj_frame_id = obj.state.header.frame_id if hasattr(obj.state, "header") else None

        if header_stamp != obj_stamp:
            print(f"Warning: Object with ID {obj.id} has different timestamp than header: {obj_stamp} vs {header_stamp}")
        if header_frame_id != obj_frame_id:
            print(f"Warning: Object with ID {obj.id} has different frame_id than header: {obj_frame_id} vs {header_frame_id}")


def _message_to_sample(msg: Any, topic_name: str) -> MessageSample:
    header = getattr(msg, "header", None)
    if header is None or not hasattr(header, "stamp") or not hasattr(header, "frame_id"):
        raise ValueError(f"Message {_message_type_name(msg)} cannot be converted into a MessageSample")

    return MessageSample(
        topic_name=topic_name,
        msg_type_name=_message_type_name(msg),
        timestamp_nanos=_canonical_message_timestamp_nanos(msg),
        frame_id=str(header.frame_id),
        msg=msg,
    )


def _yaw_from_quaternion(rotation) -> float:
    # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#:~:text=1%5D-,Quaternion%20to%20angles%20%28in%20ZYX%20sequence%29%20conversion
    x = float(rotation.x if hasattr(rotation, "x") else 0.0)
    y = float(rotation.y if hasattr(rotation, "y") else 0.0)
    z = float(rotation.z if hasattr(rotation, "z") else 0.0)
    w = float(rotation.w if hasattr(rotation, "w") else 1.0)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _storage_id(meta: dict[str, Any]) -> str:
    return meta["rosbag2_bagfile_information"]["storage_identifier"]


def iter_bag_messages(
    bag_dir: Path,
    object_list_topic: str | None,
    fixed_frame: str,
    projection_frame: str,
    ego_data_topic: str | None,
    projection: dict[Any, Any],
    unresolved_timestamps: set[int] | None = None,
) -> Iterator[MessageSample]:
    metadata = _load_metadata(bag_dir)
    storage_id = _storage_id(metadata)

    reader = SequentialReader()
    storage_options = StorageOptions(uri=str(bag_dir), storage_id=storage_id)
    converter_options = ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

    msg_cls_dict = {}

    def _get_msg_class(topic_name: str) -> Any:
        if topic_name not in type_map:
            available = ", ".join(sorted(type_map))
            raise RuntimeError(f"Topic {topic_name} not found. Available topics: {available}")
        try:
            return get_message(type_map[topic_name])
        except Exception:
            print(f"Warning: Could not get message class for topic {topic_name}. Skipping.")
            return None

    if ego_data_topic:
        msg_class = _get_msg_class(ego_data_topic)
        if getattr(msg_class, "__name__") == "EgoData":
            msg_cls_dict[ego_data_topic] = msg_class
        else:
            raise ValueError(f"{ego_data_topic} is not of type EgoData")

    if object_list_topic:
        msg_class = _get_msg_class(object_list_topic)
        if getattr(msg_class, "__name__") == "ObjectList":
            msg_cls_dict[object_list_topic] = msg_class
        else:
            raise ValueError(f"{object_list_topic} is not of type ObjectList")

    msg_cls_dict["/tf"] = get_message(type_map["/tf"]) if "/tf" in type_map else None
    msg_cls_dict["/tf_static"] = get_message(type_map["/tf_static"]) if "/tf_static" in type_map else None

    # TF buffer for resolving transforms
    buffer = Buffer()

    # Data messages pending because required TF edges are not available yet.
    pending: deque[tuple[Any, str, Time, str, str]] = deque()

    def _transform_msg_to_projection(msg: Any, msg_type_name: str, transform: Any) -> Any:
        if msg_type_name == "EgoData":
            return do_transform_ego_data(msg, transform)
        if msg_type_name == "ObjectList":
            return do_transform_object_list(msg, transform)
        return msg

    def _resolve_message_and_projection(
        msg: Any,
        msg_type_name: str,
        stamp_time: Time,
        msg_frame_id: str,
    ) -> Any | None:
        projection_to_fixed = None
        if projection_frame != fixed_frame:
            try:
                projection_to_fixed = buffer.lookup_transform(fixed_frame, projection_frame, stamp_time)
            except TransformException:
                return None

        # 1) Transform data to projection frame if needed.
        if msg_frame_id == projection_frame:
            transformed_msg = msg
        else:
            try:
                to_projection = buffer.lookup_transform(projection_frame, msg_frame_id, stamp_time)
            except TransformException:
                return None
            transformed_msg = _transform_msg_to_projection(msg, msg_type_name, to_projection)

        # 2) Store projection metadata as projection_frame -> fixed_frame.
        if projection_frame == fixed_frame:
            ts = int(stamp_time.nanoseconds)
            projection[ts] = ProjectionOffset(x=0.0, y=0.0, z=0.0, yaw=0.0)
            return transformed_msg

        assert projection_to_fixed is not None
        ts, proj_offset = _extract_proj_offset(projection_to_fixed)
        projection[int(ts)] = proj_offset
        return transformed_msg

    def retry_pending() -> Iterator[MessageSample]:
        """Retry pending messages after TF updates and yield those that resolve."""
        if not pending:
            return

        new_pending: deque[tuple[Any, str, Time, str, str]] = deque()
        while pending:
            msg, msg_type_name, st, frame_id, topic_name = pending.popleft()
            resolved_msg = _resolve_message_and_projection(msg, msg_type_name, st, frame_id)
            if resolved_msg is None:
                new_pending.append((msg, msg_type_name, st, frame_id, topic_name))
                continue
            yield _message_to_sample(resolved_msg, topic_name)
        pending.extend(new_pending)

    while reader.has_next():
        topic_name, data, _ = reader.read_next()

        if topic_name not in msg_cls_dict.keys():
            continue

        try:
            msg = deserialize_message(data, msg_cls_dict.get(topic_name))
        except Exception:
            print(
                f"Warning: Could not deserialize message on topic {topic_name} of type {msg_cls_dict.get(topic_name)}. Skipping."
            )
            continue

        if topic_name == "/tf_static":
            for transform in msg.transforms:
                buffer.set_transform_static(transform, "bag")
            yield from retry_pending()
            continue

        if topic_name == "/tf":
            for transform in msg.transforms:
                buffer.set_transform(transform, "bag")
            yield from retry_pending()
            continue

        msg_frame_id = msg.header.frame_id
        msg_type_name = getattr(msg_cls_dict.get(topic_name), "__name__", str(msg_cls_dict.get(topic_name)))
        if msg_type_name == "ObjectList":
            _normalize_object_list_object_timestamps(msg)
            check_object_consistency(msg)

        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            stamp_time = Time(nanoseconds=_canonical_message_timestamp_nanos(msg))
            resolved_msg = _resolve_message_and_projection(msg, msg_type_name, stamp_time, msg_frame_id)
            if resolved_msg is None:
                pending.append((msg, msg_type_name, stamp_time, msg_frame_id, topic_name))
                continue
            msg = resolved_msg

        yield _message_to_sample(msg, topic_name)

    # Final retry pass at end (in case TF arrived after last ObjectList)
    yield from retry_pending()
    unresolved_ts = {int(st.nanoseconds) for _, _, st, _, _ in pending}
    if unresolved_timestamps is not None:
        unresolved_timestamps.clear()
        unresolved_timestamps.update(unresolved_ts)

    if pending:
        print(f"Warning: {len(pending)} messages could not be resolved to a projection frame at the end of processing.")


def _warn_if_reappearing_id(
    row: dict[str, Any],
    last_seen_by_idx: dict[int, int],
    warn_gap_seconds: float,
) -> None:
    idx = int(row["idx"])
    total_nanos = int(row["total_nanos"])
    if idx in last_seen_by_idx:
        dt_seconds = (total_nanos - last_seen_by_idx[idx]) / 1e9
        if dt_seconds > warn_gap_seconds:
            print(f"Warning: ID {idx} found again after {dt_seconds:.3f} seconds.")
    last_seen_by_idx[idx] = total_nanos


def convert_bag_to_omega_prime(
    bag_dir: Path,
    output_dir: Path,
    ego_data_topic: str | None,
    object_list_topic: str | None,
    fixed_frame: str,
    projection_frame: str,
    map_path: Path | None = None,
    validate: bool = False,
    warn_gap_seconds: float = 3.0,
) -> Path:
    projections: dict[Any, Any] = {}
    unresolved_projection_timestamps: set[int] = set()
    last_seen_by_idx: dict[int, int] = {}
    host_vehicle_id: int | None = None

    def row_iter() -> Iterable[dict[str, Any]]:
        nonlocal host_vehicle_id
        for sample in iter_bag_messages(
            bag_dir,
            object_list_topic,
            fixed_frame,
            projection_frame,
            ego_data_topic,
            projection=projections,
            unresolved_timestamps=unresolved_projection_timestamps,
        ):
            msg = sample.msg
            msg_type_name = sample.msg_type_name

            if msg_type_name == "EgoData":
                row = _object_to_row(msg)
                if host_vehicle_id is None:
                    host_vehicle_id = int(row["idx"])
                yield row
                continue

            if msg_type_name == "ObjectList":
                for obj in msg.objects:
                    row = _object_to_row(obj)
                    _warn_if_reappearing_id(row, last_seen_by_idx, warn_gap_seconds)
                    yield row

    df = pl.DataFrame(row_iter())
    if unresolved_projection_timestamps:
        unresolved_ts = sorted(unresolved_projection_timestamps)
        unresolved_expr = pl.col("total_nanos").is_in(unresolved_ts)
        unresolved_rows = df.filter(unresolved_expr).sort(["total_nanos", "idx"])
        removed_rows = unresolved_rows.height
        df = df.filter(~unresolved_expr)
        if removed_rows > 0:
            print(f"Warning: Removed {removed_rows} rows with unresolved projection timestamps after final TF retry.")
            for row in unresolved_rows.iter_rows(named=True):
                print(f"  Removed unresolved row: {row}")

    if fixed_frame == "map":
        if map_path and map_path.exists():
            map = omega_prime.MapOdr.from_file(str(map_path), parse_map=True)
            proj_string = map.proj_string
            if not proj_string:
                raise ValueError(f"Map {map_path} has no projection string")
        elif not map_path:
            raise FileNotFoundError("Map file must be provided for fixed_frame 'map'")
        elif not map_path.exists():
            raise FileNotFoundError(f"Map file does not exist: {map_path}")
    else:
        if fixed_frame.split("_")[0] != "utm":
            raise ValueError(f"fixed_frame must be in format 'utm_<zone_number><N|S>', got '{fixed_frame}'")
        if fixed_frame.split("_")[1][-1] not in ["N", "S"]:
            raise ValueError(f"fixed_frame must be in format 'utm_<zone_number><N|S>', got '{fixed_frame}'")
        proj_string = utm_to_epsg(int(fixed_frame.split("_")[1][:-1]), northern=fixed_frame.split("_")[1][-1] == "N")
        if not proj_string:
            raise KeyError(f"No EPSG Code defined for {fixed_frame}")
    projections["proj_string"] = proj_string

    rec = omega_prime.Recording(
        df=df,
        projections=projections,
        validate=validate,
        host_vehicle_idx=host_vehicle_id,
    )

    if map_path and map_path.exists():
        rec.map = omega_prime.MapOdr.from_file(str(map_path))

    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{bag_dir.name}.omega-prime.mcap"
    rec.to_file(out_path)
    return out_path


def _discover_bags(data_dir: Path) -> list[Path]:
    bags = {path.parent for path in data_dir.rglob("metadata.yaml")}
    return sorted(bags)


def _parse_args() -> argparse.Namespace:
    env_bag_dir = os.environ.get("BAG_DIR", "/input")
    env_op_dir = os.environ.get("OP_DIR", "/output")
    env_ego_data_topic = os.environ.get("EGO_DATA_TOPIC", None)
    env_object_list_topic = os.environ.get("OBJECT_LIST_TOPIC", None)
    env_fixed_frame = os.environ.get("FIXED_FRAME", "utm_32N")
    env_projection_frame = os.environ.get("PROJECTION_FRAME", "map")
    env_map = os.environ.get("MAP", "/map/map.xodr")
    env_bag = [p.strip() for p in os.environ.get("BAG", "").split(",") if p.strip()]
    env_validate = os.environ.get("VALIDATE", "").lower() in {"1", "true", "yes"}
    env_warn_gap_seconds_raw = os.environ.get("WARN_GAP_SECONDS", "3.0")
    try:
        env_warn_gap_seconds = float(env_warn_gap_seconds_raw)
    except ValueError as exc:
        raise ValueError(f"WARN_GAP_SECONDS must be a float, got {env_warn_gap_seconds_raw!r}") from exc

    parser = argparse.ArgumentParser(description="Convert ROS 2 ObjectList bags to omega-prime MCAP")
    parser.add_argument(
        "--bag-dir",
        default=env_bag_dir,
        help="Directory containing rosbag2 folders (default: BAG_DIR or /input)",
    )
    parser.add_argument(
        "--op-dir",
        default=env_op_dir,
        help="Directory to write omega-prime mcap files (default: OP_DIR or /output)",
    )
    parser.add_argument(
        "--ego_data_topic",
        default=env_ego_data_topic,
        help="EgoData topic to export",
    )
    parser.add_argument(
        "--object_list_topic",
        default=env_object_list_topic,
        help="ObjectList topic to export",
    )
    parser.add_argument(
        "--fixed_frame",
        default=env_fixed_frame,
        help="Target fixed frame used for TF lookup and projection metadata (default: FIXED_FRAME or utm_32N)",
    )
    parser.add_argument(
        "--projection_frame",
        default=env_projection_frame,
        help="Data gets transformed into this frame (default: PROJECTION_FRAME or None)",
    )
    parser.add_argument(
        "--map",
        dest="map_path",
        default=env_map,
        help="Optional OpenDRIVE map to embed (default: MAP or /map/map.xodr)",
    )
    parser.add_argument(
        "--bag",
        action="append",
        default=env_bag,
        help="Explicit bag directory to convert (repeatable, or comma-separated via BAG)",
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        default=env_validate,
        help="Enable omega-prime schema validation",
    )
    parser.add_argument(
        "--warn-gap-seconds",
        type=float,
        default=env_warn_gap_seconds,
        help="Warning threshold in seconds if the same object ID appears again (default: WARN_GAP_SECONDS or 3.0)",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    if not args.ego_data_topic and not args.object_list_topic:
        raise ValueError("At least one of --ego_data_topic or --object_list_topic must be specified")

    bag_dirs = [Path(b).resolve() for b in args.bag]
    bag_root = Path(args.bag_dir).resolve()
    if bag_root.exists():
        bag_dirs.extend(_discover_bags(bag_root))

    unique = {}
    for bag in bag_dirs:
        if not bag.exists():
            raise FileNotFoundError(f"Bag path not found: {bag}")
        if not (bag / "metadata.yaml").exists():
            raise FileNotFoundError(f"metadata.yaml missing in bag directory: {bag}")
        unique[bag] = None

    bags = sorted(unique)
    if not bags:
        raise SystemExit("No rosbag2 directories with metadata.yaml found")

    out_dir = Path(args.op_dir).resolve()
    map_path = Path(args.map_path).resolve() if args.map_path else None

    if args.fixed_frame == "map" and map_path and not map_path.exists():
        raise ValueError("When --fixed_frame is 'map', --map must be specified")

    for bag in bags:
        if map_path and map_path.exists():
            print(f"[ros_to_omega_prime] Processing bag: {bag} with OpenDRIVE File: {map_path}")
        else:
            print(f"[ros_to_omega_prime] Processing bag: {bag} without OpenDRIVE File")
        out_file = convert_bag_to_omega_prime(
            bag_dir=bag,
            output_dir=out_dir,
            ego_data_topic=args.ego_data_topic,
            object_list_topic=args.object_list_topic,
            fixed_frame=args.fixed_frame,
            projection_frame=args.projection_frame,
            map_path=map_path,
            validate=args.validate,
            warn_gap_seconds=args.warn_gap_seconds,
        )
        print(f"[ros_to_omega_prime] Wrote {out_file}")


if __name__ == "__main__":
    main()
