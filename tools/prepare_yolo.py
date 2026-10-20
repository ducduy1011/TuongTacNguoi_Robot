#!/usr/bin/env python3
"""Install YOLO in a project venv and download official YOLOv8n weights."""
from pathlib import Path
import subprocess
import sys
import venv


def main():
    root = Path(__file__).resolve().parents[1]
    environment = root / '.venv-yolo'
    # Ubuntu's apt pip is visible through system_site_packages; no ensurepip needed.
    # Re-running also repairs an interrupted environment creation.
    venv.EnvBuilder(with_pip=False, system_site_packages=True).create(environment)
    python = str(environment / 'bin/python')
    subprocess.run([python, '-m', 'pip', 'install', '--upgrade', 'pip'], check=True)
    # CPU wheels avoid downloading CUDA runtimes and leave GPU memory to Gazebo.
    subprocess.run([python, '-m', 'pip', 'install', 'torch==2.5.1', 'torchvision==0.20.1',
                    '--index-url', 'https://download.pytorch.org/whl/cpu'], check=True)
    subprocess.run([python, '-m', 'pip', 'install', '-r', str(root / 'tools/yolo_requirements.txt')],
                   check=True)
    models = root / '.models'
    models.mkdir(exist_ok=True)
    subprocess.run([python, '-c',
                    "from ultralytics import YOLO; from cv_bridge import CvBridge; "
                    "import numpy as np; import torch; torch.set_num_threads(2); "
                    "CvBridge().cv2_to_imgmsg(np.zeros((32,32,3),np.uint8), 'bgr8'); "
                    "YOLO('yolov8n.pt').predict(np.zeros((320,320,3),np.uint8), "
                    "device='cpu', imgsz=320, verbose=False); "
                    "print('YOLO + cv_bridge OK')"], cwd=models, check=True)
    print(f'YOLO ready. Python: {python}\nWeights: {models / "yolov8n.pt"}')


if __name__ == '__main__':
    try:
        main()
    except (OSError, subprocess.CalledProcessError) as error:
        sys.exit(f'YOLO setup failed: {error}. Check internet and install python3-venv first.')
