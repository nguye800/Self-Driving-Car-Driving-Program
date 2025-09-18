# Self-Driving-Car-Driving-Program

# Link to previous stereo depth estimation project in Linux and C++
- https://github.com/lproj/stereo_match

# Stereo Repo: Virtual Environment Setup for Raspberry Pi 5

This project needs a clean Python virtual environment with compatible dependencies.
Choose **one** of the two setups below depending on whether your Raspberry Pi will display GUI windows.

## 0) System packages (Raspberry Pi OS Bookworm)
```bash
sudo apt update
# Core build tools and Python venv
sudo apt install -y python3-venv python3-dev
# Optional linear algebra (recommended for NumPy/Scipy performance)
sudo apt install -y libopenblas-dev liblapack-dev
# GUI stacks (ONLY if you want cv2.imshow windows)
sudo apt install -y libgtk-3-0 libcanberra-gtk3-module
```

## 1) Create & activate a venv
```bash
python3 -m venv .venv
source .venv/bin/activate
# Use the Windows path
./.venv/Scripts/python.exe -m pip install -U pip setuptools wheel
```

## 2A) With GUI (cv2.imshow)
```bash
pip install -r requirements.txt
python verify_opencv_gui.py
```
If it prints `SUCCESS`, you have a working HighGUI backend.


## 3) Running your stereo script
Use the same venv and just run your program:
```bash
python depth_estimation.py
```

## Notes
- Do **not** install both `opencv-python` and `opencv-python-headless` in the same env.
- If you previously had conflicting packages, start fresh in a new `.venv`.
- On Windows/macOS dev machines, the same requirements files work; remove the `apt` section.


