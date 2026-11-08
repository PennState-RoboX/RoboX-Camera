import os
import time
import ctypes
import cv2
import numpy as np
from pathlib import Path
import platform

# DLL / shared-library setup
if platform.system() == "Windows":
    dll_file = "MvCameraControl.dll"
    DLL_DIRS = [
        r"/home/robox/Desktop/RoboX-Camera/MVS/Win64_x64",
    ]
    dll_dir = next((d for d in DLL_DIRS if os.path.exists(os.path.join(d, dll_file))), None)
    if dll_dir is None:
        raise FileNotFoundError(f"{dll_file} not found. Install HIKRobot MVS (Windows x64).")
    os.add_dll_directory(dll_dir)
    os.environ["PATH"] = dll_dir + ";" + os.environ.get("PATH", "")
else:
    lib = "/opt/MVS/lib/64/libMvCameraControl.so"
    if not os.path.exists(lib):
        raise FileNotFoundError(f"{lib} not found. Install the Linux MVS SDK.")
    os.environ.setdefault("MVCAM_COMMON_RUNENV", "/opt/MVS/lib")

from MVS.MvCameraControl_class import *   # <- dedented: runs on both platforms

