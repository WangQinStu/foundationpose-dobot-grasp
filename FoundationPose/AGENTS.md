# Repository Guidelines

## Project Structure & Module Organization
Top-level Python entrypoints include `run_demo.py`, `run_linemod.py`, `run_ycb_video.py`, and `estimater.py`. Core learning code lives under `learning/` with `datasets/`, `models/`, and `training/` subpackages. Neural field and CUDA helpers live in `bundlesdf/`, including the editable extension in `bundlesdf/mycuda/`. Native C++ bindings are in `mycpp/include` and `mycpp/src`, with build artifacts under `mycpp/build/`. Assets, demo inputs, and model weights are expected in `assets/`, `demo_data/`, and `weights/`. Runtime visualizations are typically written to `debug/`.

The workspace also includes SDFR, a pose refinement module using implicit surfaces, located in the sibling `SDFR/` folder.

## Build, Test, and Development Commands
Install Python dependencies with `python -m pip install -r requirements.txt`.

**Recommended setup (Docker):**
```bash
cd docker/ && docker pull wenbowen123/foundationpose && docker tag wenbowen123/foundationpose foundationpose
bash run_container.sh
# First time inside: bash build_all.sh
# Later: docker exec -it foundationpose bash
```

For local Conda builds (experimental):
```bash
conda create -n foundationpose python=3.9 && conda activate foundationpose
python -m pip install -r requirements.txt
python -m pip install git+https://github.com/NVlabs/nvdiffrast.git kaolin==0.15.0 pytorch3d
CMAKE_PREFIX_PATH=$CONDA_PREFIX/lib/python3.9/site-packages/pybind11/share/cmake/pybind11 bash build_all_conda.sh
```

Common smoke tests:
```bash
python run_demo.py
python run_linemod.py --linemod_dir /path/to/LINEMOD --use_reconstructed_mesh 0
python run_ycb_video.py --ycbv_dir /path/to/YCB_Video --use_reconstructed_mesh 0
```

For SDFR (requires CUDA compilation):
```bash
cd ../SDFR/lib/extensions && bash build_ext.sh
python ../SDFR/run_sdfr.py --data_dir ../SDFR/datasets/
```

## Architecture: Two-Stage Prediction Pipeline
FoundationPose uses a two-stage approach:
- **ScorePredictor**: Scores pose hypotheses from multi-view renders (weights in `weights/2024-01-11-20-02-45/`)
- **PoseRefinePredictor**: Iteratively refines selected poses (weights in `weights/2023-10-28-18-33-37/`)
- **FoundationPose** (`estimater.py`): Orchestrates `register()` (initial) → `track()` (temporal)

Core data flow: Mask → Render pose candidates → Score best hypothesis → Refine iteratively.

SDFR provides post-processing refinement using implicit surface optimization.

## YOLO + FoundationPose + SDFR Integration
The `run_realsense_yolo_foundationpose.py` script integrates:
- YOLO for object detection
- FoundationPose for initial pose estimation
- SDFR for pose refinement (if available and enabled)

To enable SDFR, use the `--use_sdfr` flag:
```bash
python run_realsense_yolo_foundationpose.py --use_sdfr
```

This allows for controlled experiments comparing with and without SDFR refinement.

To enable SDFR fully, restore the SDFR code and compile CUDA extensions:
```bash
cd ../SDFR/lib/extensions && bash build_ext.sh
```

The integration adds SDFR refinement after FoundationPose estimation in `yolo_foundationpose/app.py`.

## Coding Style & Naming Conventions
Follow the existing code style: 2-space indentation in Python, compact imports such as `import os,sys`, and `snake_case` for functions, variables, and CLI flags. Keep new modules consistent with current names like `score_network.py` and `predict_score.py`. Prefer small, direct changes over broad refactors. No formatter or linter config is checked in, so match surrounding code exactly.

Project-specific conventions:
- Poses as 4×4 OpenCV matrices; coordinate transform: `glcam_in_cvcam = diag(1,-1,-1,1)`
- Mesh centered around centroid; diameter drives voxel resolution
- Depth clipped to [0.1, 2.0] or dataset-specific ranges
- Config YAML + OmegaConf for models; backward-compatible defaults hardcoded
- Debug output structure: `debug/track_vis/`, `debug/ob_in_cam/`, `debug/realsense_yolo/`

## Testing Guidelines
There is no dedicated `tests/` suite today. Validate changes by running the smallest relevant entrypoint and checking generated outputs in `debug/`. For training or dataset code, prefer a targeted script under `learning/training/` and document required datasets or weights in the PR. Treat demo execution as the baseline regression check.

For SDFR, validate with `python my_eval.py --results_dir results/`.

## Potential Pitfalls & Environment Issues
- CUDA 11.3 failures on RTX 4090+: Use updated Docker image `shingarey/foundationpose_custom_cuda121:latest`
- Unreasonable poses: Check depth clipping, RGB-D alignment, mask quality; inspect `debug/ob_in_cam/` visualizations
- Kaolin import missing: Use `pip install kaolin==0.15.0 -f https://nvidia-kaolin.s3.us-east-2.amazonaws.com/torch-2.0.0_cu118.html`
- First-run slow: JIT compilation expected
- Windows build fails: Not supported
- SDFR CUDA kernels require compilation; mesh quality affects SDF accuracy

## Commit & Pull Request Guidelines
Recent history favors short, imperative commit messages such as `add trouble shooting reference` or `change zmin clip to 0.001`. Keep commits focused and descriptive. PRs should state the scenario affected, list required data or weights, include exact reproduction commands, and attach screenshots or output paths when a change affects pose visualization or tracking behavior.

## Configuration Tips
Do not commit large weights, datasets, or generated `debug/` artifacts. Keep local paths configurable through CLI arguments instead of hardcoding machine-specific directories.

## Key Files & Exemplar Patterns
Entry points (ranked by simplicity):
- [run_demo.py](run_demo.py) → simplest end-to-end
- [run_linemod.py](run_linemod.py) → multi-object template
- [run_ycb_video.py](run_ycb_video.py) → benchmarking
- [run_realsense_yolo_foundationpose.py](run_realsense_yolo_foundationpose.py) → real-time integration

Utilities: [Utils.py](Utils.py) for rendering and geometry; [datareader.py](datareader.py) for BOP datasets.

For SDFR: [../SDFR/run_sdfr.py](../SDFR/run_sdfr.py) for main refinement.

## Documentation Links
- [readme.md](readme.md): Setup, demos, troubleshooting
- [yolo_foundationpose/README.md](yolo_foundationpose/README.md): YOLO integration
- [../SDFR/基于隐式曲面优化的快速高精度6D物体位姿精修方法.md](../SDFR/基于隐式曲面优化的快速高精度6D物体位姿精修方法.md): SDFR paper summary
