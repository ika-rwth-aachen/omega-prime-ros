# omega-prime-ros

ROS 2 Jazzy to omega-prime MCAP export image.

## Usage

Use the Docker image to run the converter automatically. It will discovers `rosbag2` folders  and writes `omega-prime` files to the output folder.

### General Process
1. Build or pull the `omega-prime-ros` image
2. Mount your ROS bag folder to `/input`
3. Mount an output directory to `/output`
4. Set at least one topic (`OBJECT_LIST_TOPIC` and/or `EGO_DATA_TOPIC`)
5. Optionally mount `/map/map.xodr` to embed OpenDRIVE map data
6. Run the container

### Run
```bash
docker run --rm -it \
    -e EGO_DATA_TOPIC=</your/ego_data_topic> \
    -e OBJECT_LIST_TOPIC=</your/object_list_topic> \
    -v <path/to/bags>:/input \
    -v </path/to/map.xodr>:/map/map.xodr \
    -v "$PWD"/output:/output \
    gitlab.ika.rwth-aachen.de:5050/fb-fi/data/omega-prime-ros:latest
```

### CLI Description
Environment variables and CLI flags:
- `BAG_DIR` / `--bag-dir` (default `/input`)
- `OP_DIR` / `--op-dir` (default `/output`)
- `EGO_DATA_TOPIC` / `--ego_data_topic`
- `OBJECT_LIST_TOPIC` / `--object_list_topic`
- `FIXED_FRAME` / `--fixed_frame` (default `utm_32N`)
- `MAP` / `--map` (default `/map/map.xodr`)
- `BAG` / `--bag` to process explicit bag directories in addition to auto-discovery (`BAG` supports comma-separated paths)
- `VALIDATE` / `--validate`
- `ID_GAP` / `--id-gap` warning threshold in seconds if the same object ID appears again

### Notes
- The converter scans `/input` for rosbag2 directories containing a `metadata.yaml` and writes one omega-prime `.mcap` per bag into `/output` by default.
- For large bags, ensure sufficient RAM.

## Map Information

- Place your `.xodr` file under the mounted `/map/map.xodr`.
- The export routine embeds the map in each generated omega-prime `.mcap`.
- If `/map/map.xodr` does not exist, outputs will not include a map.
- Map parsing uses a default geometry sampling step size of `0.01 m`.

## Projection Information

- The converter reads `/tf` and `/tf_static` and resolves each EgoData and ObjectList message frame against the configured `fixed_frame`.
- The `fixed_frame` should be the georeferenced top-level ROS coordinate frame (TF root), for example a global UTM/world frame.
- When `fixed_frame=map`, the map must be parsed and the map projection string is used.
- These transforms are stored in omega-prime as per-timestamp `ProjectionOffset` metadata.
- The fixed frame is converted to an EPSG projection string and written as `projections["proj_string"]`.
- Supported `fixed_frame` values: `utm_<zone: int>[N/S]` and `map` (e.g. `utm_30N`).

## Docker Image

This image bundles ROS 2 Jazzy, rosbag2 Python bindings, omega-prime, and builds `perception_interfaces` from GitHub so EgoData and ObjectList topics can be exported to omega-prime MCAP.

### Build Args
- `OMEGA_PRIME_VERSION` (default `latest`): PyPI version to install
- `PERCEPTION_INTERFACES_VERSION` (optional): commit/branch/tag; if unset, the repo default branch is used

### Local Build
```bash
docker build -t gitlab.ika.rwth-aachen.de:5050/fb-fi/data/omega-prime-ros:latest \
    --build-arg OMEGA_PRIME_VERSION=latest \
    --build-arg PERCEPTION_INTERFACES_VERSION=<commit-or-branch> \
    -f Dockerfile .
```
