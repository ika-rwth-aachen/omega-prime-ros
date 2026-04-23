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

## 1.2.0
- Add export-time timestamp synchronization for `EgoData` and `ObjectList` with configurable `base_time_message_type` and `match_threshold_nanos`, while keeping TF lookup and projection handling at each message's original timestamp.
- Preserve higher-frequency topic data by snapping only the nearest eligible non-base message to a base-topic timestamp instead of dropping unmatched samples.
- Normalize `ObjectList` object timestamps to `ObjectList.header.stamp` and expose the synchronization settings through CLI arguments and environment variables.