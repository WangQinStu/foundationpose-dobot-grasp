from yolo_foundationpose.app import run
from yolo_foundationpose.config import parse_args


if __name__=='__main__':
  # 实时运行入口：
  # 1. parse_args() 读取相机、YOLO、FoundationPose 和调试参数；
  # 2. run(args) 进入 yolo_foundationpose/app.py 里的主循环。
  # 这个文件只负责把命令行参数交给主流程，便于保持入口简短。
  run(parse_args())
