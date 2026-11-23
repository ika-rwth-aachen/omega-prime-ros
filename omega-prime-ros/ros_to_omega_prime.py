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
from pathlib import Path
from typing import Any

import betterosi
import numpy as np
import perception_msgs_utils as pmu
import polars as pl
import yaml
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
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

# A message whose TF is still missing although TF data this much newer has arrived will never resolve.
_STALE_PENDING_NS = 10_000_000_000

# Classification types of perception_msgs/ObjectClassification that are vehicles in OSI.
_VEHICLE_MAP = {
    2: _VCT.BICYCLE,  # BICYCLE
    3: _VCT.MOTORBIKE,  # MOTORCYCLE
    4: _VCT.CAR,  # CAR
    5: _VCT.HEAVY_TRUCK,  # UTILITY (deprecated TRUCK)
    6: _VCT.DELIVERY_VAN,  # VAN (deprecated)
    7: _VCT.BUS,  # BUS
    10: _VCT.TRAIN,  # TRAIN (deprecated)
    11: _VCT.TRAILER,  # TRAILER (deprecated)
    12: _VCT.WHEELCHAIR,  # VRU (wheelchairs, strollers)
    # MICRO: betterosi >= 0.9 knows the OSI micromobility type, older versions only the stand-up scooter
    13: getattr(_VCT, "MICROMOBILITY_DEVICE", _VCT.STANDUP_SCOOTER),
    50: _VCT.CAR,  # CAR_UNION (deprecated, use CAR)
    51: _VCT.HEAVY_TRUCK,  # TRUCK_UNION (deprecated, use UTILITY)
    52: _VCT.OTHER,  # BIKE_UNION (deprecated, bicycle or motorcycle)
}

# State entries that a model may lack; they are written as 0 and reported once per model.
_missing_state_entries: set[tuple[int, str]] = set()


def utm_to_epsg(zone: int, northern: bool = True) -> str:
    if not isinstance(zone, int):
        raise TypeError(f"Zone must be an integer {zone}")
    base = 32600 if northern else 32700
    epsg_str = f"EPSG:{int(base + zone)}"
    return epsg_str


def _class_to_osi(obj) -> tuple[int, int, int]:
    role = -1
    subtype = -1

    if obj.state.classifications:
        c = pmu.get_class_with_highest_probability(obj)
        ct = int(c.type)
    else:
        ct = 0

    if ct == 0:  # UNCLASSIFIED or no classification
        mot = int(_MOT.UNKNOWN)
    elif ct == 1:
        mot = int(_MOT.PEDESTRIAN)
    elif ct == 8:
        mot = int(_MOT.ANIMAL)
    elif ct in _VEHICLE_MAP:
        mot = int(_MOT.VEHICLE)
        role = int(_ROLE.CIVIL)
        subtype = int(_VEHICLE_MAP[ct])
    else:  # ROAD_OBSTACLE (9), UNKNOWN (100) and types this converter does not know
        mot = int(_MOT.OTHER)

    return mot, role, subtype


def _state_entry(getter, obj, entry: str, default: float = 0.0) -> float:
    """Read a state entry; models without this entry (e.g. TRAFFICLIGHT without velocity) give `default`."""
    try:
        return float(getter(obj))
    except (AttributeError, pmu.UnknownStateEntryError):
        key = (int(obj.state.model_id), entry)
        if key not in _missing_state_entries:
            _missing_state_entries.add(key)
            print(f"Warning: state model {key[0]} has no entry '{entry}'; writing {default}.")
        return default


def _object_to_row(obj) -> dict[str, Any]:
    total_nanos = Time.from_msg(obj.state.header.stamp).nanoseconds

    obj_type_name = getattr(type(obj), "__name__", str(type(obj)))

    if obj_type_name == "Object":
        idx = int(obj.id)
        width = _state_entry(pmu.get_width, obj, "width")
        length = _state_entry(pmu.get_length, obj, "length")
        height = _state_entry(pmu.get_height, obj, "height")

    elif obj_type_name == "EgoData":
        idx = int(obj.vehicle_id)
        width = float(obj.width)
        length = float(obj.length)
        height = float(obj.height)
    else:
        raise ValueError(f"Unexpected object type: {obj_type_name}. Supported types are Object and EgoData.")

    pos = pmu.get_center_position(obj)

    vel_lon = _state_entry(pmu.get_vel_lon, obj, "vel_lon")
    vel_lat = _state_entry(pmu.get_vel_lat, obj, "vel_lat")
    acc_lon = _state_entry(pmu.get_acc_lon, obj, "acc_lon")
    acc_lat = _state_entry(pmu.get_acc_lat, obj, "acc_lat")

    model_id = obj.state.model_id
    yaw = float(pmu.get_yaw(obj)) if pmu.has_yaw(model_id) else 0.0
    roll = float(pmu.get_roll(obj)) if pmu.has_roll(model_id) else 0.0
    pitch = float(pmu.get_pitch(obj)) if pmu.has_pitch(model_id) else 0.0

    # perception_msgs store velocity/acceleration along the object axes (lon/lat), OSI in the parent frame (x/y)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vel_x = vel_lon * cos_yaw - vel_lat * sin_yaw
    vel_y = vel_lon * sin_yaw + vel_lat * cos_yaw
    acc_x = acc_lon * cos_yaw - acc_lat * sin_yaw
    acc_y = acc_lon * sin_yaw + acc_lat * cos_yaw

    mot, role, subtype = _class_to_osi(obj)

    return {
        "total_nanos": int(total_nanos),
        "idx": idx,
        "x": float(pos.x),
        "y": float(pos.y),
        "z": float(getattr(pos, "z", 0.0)),
        "vel_x": float(vel_x),
        "vel_y": float(vel_y),
        "vel_z": 0.0,
        "acc_x": float(acc_x),
        "acc_y": float(acc_y),
        "acc_z": 0.0,
        "length": length,
        "width": width,
        "height": height,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
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


def _extract_proj_offset(msg) -> ProjectionOffset:
    transformation = msg.transforms[0] if hasattr(msg, "transforms") else msg
    translation = transformation.transform.translation
    rotation = transformation.transform.rotation

    return ProjectionOffset(
        x=float(translation.x if hasattr(translation, "x") else 0.0),
        y=float(translation.y if hasattr(translation, "y") else 0.0),
        z=float(translation.z if hasattr(translation, "z") else 0.0),
        yaw=_yaw_from_quaternion(rotation),
    )


def check_object_consistency(msg) -> tuple[int, int]:
    """Count objects of an ObjectList whose timestamp or frame_id differ from the message header."""
    if not hasattr(msg, "objects"):
        return 0, 0
    stamp_mismatch = sum(obj.state.header.stamp != msg.header.stamp for obj in msg.objects)
    frame_mismatch = sum(obj.state.header.frame_id != msg.header.frame_id for obj in msg.objects)
    return stamp_mismatch, frame_mismatch


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


def _bag_duration_ns(meta: dict[str, Any]) -> int:
    try:
        return int(meta["rosbag2_bagfile_information"]["duration"]["nanoseconds"])
    except (KeyError, TypeError, ValueError):
        return 0


def _row_stamps(msg: Any, msg_type_name: str) -> set[int]:
    """Timestamps of the rows a message produces (object/ego state stamps)."""
    if msg_type_name == "ObjectList":
        return {int(Time.from_msg(obj.state.header.stamp).nanoseconds) for obj in msg.objects}
    if msg_type_name == "EgoData":
        return {int(Time.from_msg(msg.state.header.stamp).nanoseconds)}
    return set()


def iter_bag_messages(
    bag_dir: Path,
    object_list_topic: str | None,
    fixed_frame: str,
    projection_frame: str,
    ego_data_topic: str | None,
    projection: dict[Any, Any],
) -> Iterator[Any]:
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

    for tf_topic in ("/tf", "/tf_static"):
        if tf_topic in type_map:
            msg_cls_dict[tf_topic] = get_message(type_map[tf_topic])

    # Only read the topics that are converted; recorded bags often carry much larger raw sensor topics.
    reader.set_filter(StorageFilter(topics=list(msg_cls_dict)))

    # TF buffer for resolving transforms. The default cache of 10 s would drop transforms that messages recorded
    # with a larger delay still need, so keep the whole bag.
    cache_s = max(10.0, _bag_duration_ns(metadata) / 1e9 + 10.0)
    buffer = Buffer(cache_time=Duration(seconds=cache_s))

    # Data messages pending because required TF edges are not available yet, grouped by frame_id.
    pending: dict[str, deque[tuple[Any, Any, Time]]] = {}
    n_dropped = 0
    latest_tf_ns = 0
    n_stamp_mismatch = 0
    n_frame_mismatch = 0

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

        # 2) Store projection metadata as projection_frame -> fixed_frame, under every timestamp the message's
        #    rows get (object stamps may differ from the header stamp).
        if projection_to_fixed is None:
            proj_offset = ProjectionOffset(x=0.0, y=0.0, z=0.0, yaw=0.0)
        else:
            proj_offset = _extract_proj_offset(projection_to_fixed)
        for ts in {int(stamp_time.nanoseconds), *_row_stamps(transformed_msg, msg_type_name)}:
            projection[ts] = proj_offset
        return transformed_msg

    def _frame_connected(frame_id: str) -> bool:
        if frame_id == projection_frame:
            return True
        return buffer.can_transform(projection_frame, frame_id, Time())

    def retry_pending(final: bool = False) -> Iterator[tuple[Any, Any]]:
        """Retry pending messages after TF updates and yield those that resolve."""
        nonlocal n_dropped
        if not pending:
            return
        if projection_frame != fixed_frame and not buffer.can_transform(fixed_frame, projection_frame, Time()):
            return
        for frame_id in list(pending):
            # Frames without any TF connection are skipped as a whole instead of retrying every message.
            if not _frame_connected(frame_id):
                continue
            queue = pending[frame_id]
            still_pending: deque[tuple[Any, Any, Time]] = deque()
            while queue:
                msg, msg_type, st = queue.popleft()
                msg_type_name = getattr(msg_type, "__name__", str(msg_type))
                resolved_msg = _resolve_message_and_projection(msg, msg_type_name, st, frame_id)
                if resolved_msg is not None:
                    yield (resolved_msg, msg_type)
                elif final or latest_tf_ns - int(st.nanoseconds) > _STALE_PENDING_NS:
                    n_dropped += 1
                else:
                    still_pending.append((msg, msg_type, st))
            if still_pending:
                pending[frame_id] = still_pending
            else:
                del pending[frame_id]

    while reader.has_next():
        topic_name, data, _ = reader.read_next()

        if topic_name not in msg_cls_dict:
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
                latest_tf_ns = max(latest_tf_ns, int(Time.from_msg(transform.header.stamp).nanoseconds))
            yield from retry_pending()
            continue

        msg_frame_id = msg.header.frame_id
        msg_type_name = getattr(msg_cls_dict.get(topic_name), "__name__", str(msg_cls_dict.get(topic_name)))
        if msg_type_name == "ObjectList":
            stamp_mismatch, frame_mismatch = check_object_consistency(msg)
            n_stamp_mismatch += stamp_mismatch
            n_frame_mismatch += frame_mismatch

        stamp = msg.header.stamp if hasattr(msg, "header") else None
        if stamp is not None:
            stamp_time = Time.from_msg(stamp)
            resolved_msg = _resolve_message_and_projection(msg, msg_type_name, stamp_time, msg_frame_id)
            if resolved_msg is None:
                pending.setdefault(msg_frame_id, deque()).append((msg, msg_cls_dict[topic_name], stamp_time))
                continue
            msg = resolved_msg

        yield (msg, msg_cls_dict[topic_name])

    # Final retry pass at end (in case TF arrived after last ObjectList)
    yield from retry_pending(final=True)
    n_unresolved = n_dropped + sum(len(q) for q in pending.values())

    if n_stamp_mismatch or n_frame_mismatch:
        print(
            f"Warning: {n_stamp_mismatch} objects have a different timestamp and {n_frame_mismatch} objects a "
            "different frame_id than their ObjectList header; rows use the object timestamp and the header frame_id."
        )
    if n_unresolved:
        print(
            f"Warning: {n_unresolved} messages could not be resolved to a projection frame and were dropped "
            f"(frames: {', '.join(sorted(pending)) or 'resolved too late'})."
        )


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
    last_seen_by_idx: dict[int, int] = {}
    host_vehicle_id: int | None = None

    def row_iter() -> Iterable[dict[str, Any]]:
        nonlocal host_vehicle_id
        for msg, msg_type in iter_bag_messages(
            bag_dir,
            object_list_topic,
            fixed_frame,
            projection_frame,
            ego_data_topic,
            projection=projections,
        ):
            msg_type_name = getattr(msg_type, "__name__", str(msg_type))

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
    if df.height == 0:
        topics = ", ".join(t for t in (object_list_topic, ego_data_topic) if t)
        raise RuntimeError(
            f"No objects to convert in {bag_dir} (topics: {topics}): the messages are empty or could not be "
            "resolved to the projection frame."
        )

    duplicates = df.filter(pl.struct("total_nanos", "idx").is_duplicated())
    if duplicates.height:
        ids = sorted(set(duplicates["idx"].to_list()))[:10]
        raise ValueError(
            f"{duplicates.height} rows share timestamp and ID with another row (IDs {ids}). EgoData.vehicle_id must "
            "differ from all object IDs, and each object ID may appear only once per timestamp."
        )

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
        help="Data gets transformed into this frame (default: PROJECTION_FRAME or map)",
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

    # One output file per bag name: bags with the same directory name would overwrite each other.
    by_name: dict[str, list[Path]] = {}
    for bag in bags:
        by_name.setdefault(bag.name, []).append(bag)
    clashes = {name: paths for name, paths in by_name.items() if len(paths) > 1}
    if clashes:
        details = "; ".join(f"{name}: {', '.join(map(str, paths))}" for name, paths in clashes.items())
        raise SystemExit(f"Bags with the same directory name would write the same output file: {details}")

    out_dir = Path(args.op_dir).resolve()
    map_path = Path(args.map_path).resolve() if args.map_path else None

    if args.fixed_frame == "map" and not (map_path and map_path.exists()):
        raise ValueError(f"--fixed_frame 'map' needs an existing OpenDRIVE map (--map), got {map_path}")

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
