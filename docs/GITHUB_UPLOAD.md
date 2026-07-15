# GitHub Upload Notes

Do not upload large local runtime data or generated outputs:

- `dobot/build/`, `dobot/install/`, `dobot/log/`
- `FoundationPose/demo_data/`
- `FoundationPose/realsense/`
- `FoundationPose/debug/`
- SDFR rendered datasets and temporary capture data
- Python `__pycache__/`

The current `.gitignore` already excludes these paths. Keep only source code, current runtime configuration, and the small set of assets needed to reproduce the active bottle grasping flow.
