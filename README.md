# omega-prime-ros

## ROS 2 -> omega-prime 

This image bundles ROS 2 Jazzy, its rosbag2 Python bindings, omega-prime (via PyPI), and builds perception_interfaces (messages + Python utils) from GitHub so you can export EgoData and ObjectList topics to omega-prime MCAP using the built-in converter.

## Build Args
- `OMEGA_PRIME_VERSION` (default `latest`): PyPI version to install; use `latest` for newest
- `PERCEPTION_INTERFACES_REPO` (default GitHub repo)
- `PERCEPTION_INTERFACES_REF` (optional): commit/branch/tag; if unset, uses the repo’s default branch

## Local Build
```bash
docker build -t omega-prime-ros \
    --build-arg OMEGA_PRIME_VERSION=latest \
    --build-arg PERCEPTION_INTERFACES_REF=<commit-or-branch> \
    -f Dockerfile .
```

## Run
- Mount your bag directory to `/data` and an output directory to `/out`.
- EgoData can be extracted by setting the topic with `OP_EGO_DATA_TOPIC`.
- Set the topic via `OP_OBJECT_LIST_TOPIC` (ObjectList topic); the container runs the export automatically. 

### Example:
```bash
docker run --rm -it \
    -e OP_OBJECT_LIST_TOPIC=</your/object_list_topic> \
    -e OP_EGO_DATA_TOPIC=</your/egoData/topic> \
    -v <path/to/bags>:/data:ro \
    -v </path/to/map.xodr>:/map/map.xodr:ro \
    -v "$PWD"/out:/out \
    omega-prime-ros
```

## Projections and Fixed Frame
- The converter reads `/tf` + `/tf_static` and resolves each EgoData and ObjectList message frame against the configured `fixed_frame`.
- The `fixed_frame` should be the georeferenced top-level ROS coordinate frame (TF root) of your setup, for example the global UTM/world frame. When `fixed_frame` is `map`, the map must be parsed and the projection string is the one of the map.
- These transforms are stored in omega-prime as per-timestamp `ProjectionOffset` metadata.
- The fixed frame is converted to an EPSG projection string and written as `projections["proj_string"]`.
- Supported fixed frame values are currently: `utm_<zone_nr: int>[N/S]` and `map`. Examples: `utm_30N`, `utm_32S`

## Notes
- The image builds and installs `perception_interfaces` packages needed for Python APIs and messages (`perception_msgs`, `perception_msgs_utils`, `tf2_perception_msgs`).
- The converter scans `/data` for rosbag2 directories containing a `metadata.yaml` and writes one omega-prime `.mcap` per bag into `/out` per default.
- For large bags ensure sufficient RAM.

## Advanced
- Env vars / CLI flags:
  - `OP_DATA` / `--data-dir` (default `/data`)
  - `OP_OUT` / `--output-dir` (default `/out`)
  - `OP_OBJECT_LIST_TOPIC` / `--object_list_topic`
  - `OP_EGO_DATA_TOPIC` / `--ego_data_topic`
  - `OP_VALIDATE` / `--validate`
  - `OP_FIXED_FRAME` / `--fixed_frame` (default `utm_32N`)
  - `--bag` to process explicit bag directories in addition to auto-discovery
  - `--timeout` Set a timeout that prints a warning if the same object ID appears again after `timeout` seconds.

## OpenDRIVE Map Integration

### During export (recommended)
- Place your `.xodr` file under the mounted `/map/map.xodr`
- The export routine embeds the map in each generated omega-prime `.mcap`.

### Notes
- If `/map/map.xodr` does not exist, outputs won’t include a map.
- Map parsing uses a default geometry sampling step size of 0.01 m.
