# SO-ARM101 model attribution

The SO-ARM101 macro and mesh assets in this package were imported from:

- repository: `https://github.com/ros-physical-ai/ros2_so_arm`
- upstream package: `so_arm101_description`
- audited commit: `e166df9d51f43b24da9b99047c6c51c306bda74f`
- imported on: `2026-08-29`
- upstream repository license: BSD-3-Clause
- mesh license/provenance: see `meshes/LICENSE`

Only the visual/collision macro and meshes were imported. Upstream real-hardware
configuration, motor offsets, controller configuration, and launch files are not
used by this workspace. The local mock ros2_control declaration lives in
`so101_arm_control` and has no real-hardware plugin path.
