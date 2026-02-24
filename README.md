<img src="https://github.com/ika-rwth-aachen/omega-prime-ros/blob/github/omega-prime-ros.svg?raw=True" height=150px align="right" style="margin: 10px;">

[![](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/ika-rwth-aachen/omega-prime-ros/blob/master/LICENSE) 
[![](https://img.shields.io/pypi/v/omega-prime-ros.svg)](https://pypi.python.org/pypi/omega-prime-ros)
[![](https://github.com/ika-rwth-aachen/omega-prime-ros/workflows/CI/badge.svg)](https://github.com/ika-rwth-aachen/omega-prime-ros/actions)
[![](https://img.shields.io/pypi/pyversions/omega-prime-ros.svg)](https://pypi.python.org/pypi/omega-prime-ros/)
[![](https://img.shields.io/github/issues-raw/ika-rwth-aachen/omega-prime-ros.svg)](https://github.com/ika-rwth-aachen/omega-prime-ros/issues)

# Omega-Prime-ROS
This repository provides a Dockerized ROS 2 conversion pipeline that exports `rosbag2` data to omega-prime `.mcap` files.

It scans `rosbag2` recordings, reads EgoData and ObjectList topics, and resolves `/tf` + `/tf_static` transforms into a configurable fixed frame. The converter writes one `.mcap` per bag and supports optional OpenDRIVE map embedding and schema validation for downstream analytics workflows.

## Requirements
You need to have installed `Docker` to be able to convert ROS 2 bags to Omega-Prime MCAP files

## Usage

Use the Docker image to run the converter automatically. It will discovers `rosbag2` folders  and writes `omega-prime` files to the output folder.

1. Build or pull the `omega-prime-ros` image
1. Mount your ROS bag folder to `/input`
1. Mount an output directory to `/output`
1. Optionally mount `/map/map.xodr` to embed OpenDRIVE map data
1. Set at least one topic (`EGO_DATA_TOPIC` and/or `OBJECT_LIST_TOPIC`)
1. Run the container

```bash
docker run --rm -it \
    -e EGO_DATA_TOPIC=</your/ego_data_topic> \
    -e OBJECT_LIST_TOPIC=</your/object_list_topic> \
    -v <path/to/bags>:/input \
    -v </path/to/map.xodr>:/map/map.xodr \
    -v "$PWD"/output:/output \
    ghcr.io/ika-rwth-aachen/omega-prime-ros:latest
```

### CLI Description
Environment variables and CLI flags:
- `BAG_DIR` / `--bag-dir` (default `/input`)
- `OP_DIR` / `--op-dir` (default `/output`)
- `EGO_DATA_TOPIC` / `--ego_data_topic`
- `OBJECT_LIST_TOPIC` / `--object_list_topic`
- `FIXED_FRAME` / `--fixed_frame` (default `utm_32N`)
- `MAP` / `--map` (default `/map/map.xodr`)
- `BAG` / `--bag` to process explicit bags (supports comma-separated paths)
- `VALIDATE` / `--validate` enable omega-prime schema validation
- `WARN_GAP_SECONDS` / `--warn-gap-seconds` warning threshold in seconds if same object ID appears multiple times

### Notes
- The converter scans `/input` for rosbag2 directories containing a `metadata.yaml` and writes one omega-prime `.mcap` per bag into `/output` by default.
- For large bags, ensure sufficient RAM.


## Projection Information
- The converter reads `/tf` and `/tf_static` and resolves each EgoData and ObjectList message frame against the configured `fixed_frame`.
- The `fixed_frame` should be the georeferenced top-level ROS coordinate frame (TF root), for example a global UTM/world frame.
- When `fixed_frame=map`, the map must be parsed and the map projection string is used.
- These transforms are stored in omega-prime as per-timestamp `ProjectionOffset` metadata.
- The fixed frame is converted to an EPSG projection string and written as `projections["proj_string"]`.
- Supported `fixed_frame` values: `utm_<zone: int>[N/S]` and `map` (e.g. `utm_30N`).

## Docker Image
The probided image bundles ROS 2 Jazzy, rosbag2 Python bindings, omega-prime, and builds `perception_interfaces` from GitHub so EgoData and ObjectList topics can be exported to omega-prime MCAP.

### Build Args
- `OMEGA_PRIME_VERSION` (default `latest`): PyPI version to install
- `PERCEPTION_INTERFACES_VERSION` (optional): commit/branch/tag; if unset, the repo default branch is used

### Local Build
```bash
docker build -t ghcr.io/ika-rwth-aachen/omega-prime-ros:latest \
    --build-arg OMEGA_PRIME_VERSION=latest \
    --build-arg PERCEPTION_INTERFACES_VERSION=<commit-or-branch> \
    -f Dockerfile .
```

# Notice

> [!IMPORTANT]
> The project is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).
> We cover a wide variety of research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.
> If you would like to learn more about how we can support your automated driving or robotics efforts, feel free to reach out to us!
> Contact: [opensource@ika.rwth-aachen.de](mailto:opensource@ika.rwth-aachen.de)
