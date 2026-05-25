# GitHub Upload Notes

The root of this workspace is intended to become the GitHub repository:

```bash
cd /home/ptcs/wkSpace/FP
git init
git status --short
```

## Important: Nested Git Repositories

Several subdirectories currently contain their own `.git` directories from upstream clones. If you run `git add .` while those directories remain, Git may treat them as embedded repositories instead of normal folders.

To upload all three parts as one normal repository, first archive or remove only the nested Git metadata:

```bash
cd /home/ptcs/wkSpace/FP
mv FoundationPose/.git FoundationPose/.git.upstream-backup
mv SDFR/.git SDFR/.git.upstream-backup
mv SDFR/pytorch3d/.git SDFR/pytorch3d/.git.upstream-backup
mv dobot/src/DOBOT_6Axis_ROS2_V4/.git dobot/src/DOBOT_6Axis_ROS2_V4/.git.upstream-backup
mv dobot/src/Step_Motor_ROS2/src/serial_ros2/.git dobot/src/Step_Motor_ROS2/src/serial_ros2/.git.upstream-backup
```

Those backup folders are ignored by `.gitignore`. After confirming the root repository has everything you need, you can delete the backups or keep them outside the project.

## First Commit

```bash
cd /home/ptcs/wkSpace/FP
git add .gitignore README.md docs FoundationPose SDFR dobot
git status --short
git commit -m "Organize integrated grasping workspace"
```

If large model or dataset files appear in `git status`, either keep them ignored, add them with Git LFS, or upload them separately as GitHub release assets.

## Add Remote And Push

Create an empty repository on GitHub, then:

```bash
git branch -M main
git remote add origin git@github.com:<your-user>/<your-repo>.git
git push -u origin main
```

## Recommended Upload Boundary

Commit source code, launch files, small config files, and lightweight demo assets required to run the current project.

Do not commit generated ROS 2 outputs, debug frames, RealSense recordings, training renders, experiment logs, temporary datasets, or large checkpoints unless they are intentionally managed with Git LFS.
