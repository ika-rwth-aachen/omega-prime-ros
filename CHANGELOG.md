# Changelog

This file is used to document all notable changes to the project. Each tag/version should have a corresponding section with a list of changes, improvements, and bug fixes.

## 1.0.0

- Initial release.

## 1.0.1

- Use OSI motion and acceleration definition (x,y in parent frame) instead of perception interfaces defintion (lon, lat). For more info see [Issue](https://github.com/ika-rwth-aachen/omega-prime-ros/issues/3)

## 1.1.0

- Use get_center_pos() instead of get_position() to ensure the x,y,z are always centered aligning with the OSI definition
- all messages are transformed to `projection_frame`
- the transformation from `projection_frame` to `fixed_frame` is stored in Recording projection dictionary

## 1.1.1

- Used fixed version for omega-prime (`0.3.2`) and perception_interfaces (`v1.1.0`) in docker build.

## Unreleased

- omega-prime `0.3.8` (betterosi `0.9`) in the docker build: the object height is written to OSI (before, the width was written as height) and no host vehicle ID 0 is written without EgoData.
- Messages that cannot be resolved to the projection frame no longer remove resolved rows of other messages with the same timestamp.
- The projection offset is stored under every row timestamp; objects whose timestamp differs from the ObjectList header no longer end up without offset (and far away after projection).
- State models without velocity, acceleration or dimensions (e.g. TRAFFICLIGHT) are converted with 0 for the missing entries instead of aborting.
- The TF buffer keeps the whole bag (the default 10 s dropped messages recorded later than 10 s after their timestamp).
- Pending messages are retried per frame, only if the frame is connected to the TF tree, and dropped once newer TF data cannot resolve them; a frame without TF no longer makes the conversion quadratic in time.
- Clear errors for a bag without objects, for rows sharing timestamp and ID (e.g. EgoData `vehicle_id` equal to an object ID) and for bags with the same directory name.
- Classification: UNCLASSIFIED and objects without classification become `TYPE_UNKNOWN`, VRU `TYPE_WHEELCHAIR`, MICRO `TYPE_MICROMOBILITY_DEVICE` (betterosi >= 0.9), CAR_UNION `TYPE_CAR`, TRUCK_UNION `TYPE_HEAVY_TRUCK`.
- Only the converted topics are read from the bag (faster for bags with raw sensor data).
- Object timestamp/frame mismatches are reported once per bag instead of once per object.
- Regression tests in `tests/` run in CI inside the built image.