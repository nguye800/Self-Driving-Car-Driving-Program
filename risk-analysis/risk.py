from depth_estimation import compute_dispmap
import cv2
import numpy as np
import sys
from collections import deque
import math
from yolo_sgm import detect_obj
from ultralytics import YOLO
# from picamera2 import Picamera2
from depth_estimation import rpi_camera, compute_dispmap_sgbm, initialize_cam
import numpy as np

baseline = 0.05 # meters
pixel_pitch = 0.0000014 # meters/px
focal_len = 0.0036 / pixel_pitch # px
RISK_THRESHOLD = 0.8

def sigmoid(x): 
    return 1.0 / (1.0 + math.exp(-x))

def compute_z(disparity_normalized, min_disp=0, num_disp=128):
    # Convert from 0-255 back to actual disparity in pixels
    actual_disparity = (disparity_normalized / 255.0) * num_disp + min_disp
    
    if actual_disparity <= 0:
        return -1
    return (baseline * focal_len) / actual_disparity

def compute_risks(objects, img_w=1280, img_h=720,
                  min_disp=0, num_disp=128,
                  wA=1.0, wP=2.0, wC=0.5,
                  alpha=1.0, beta=1.0, k=6.0,
                  eps_dist=0.1, sigma_c=0.5):
    """
    objects: dictionary "object_type" "bounding_box" "avg_disparity
    returns: list of dicts with risk and fields
    """
    out = []
    for obj in objects:
        object_type = obj["object_type"]
        x1,y1,x2,y2 = obj["bounding_box"]
        disparity = obj["avg_disparity"]

        # 1) area
        A = max(0.0, ((x2-x1) * (y2-y1)) / (img_w * img_h))

        # 2) proximity inverse distance + epsilon to avoid explosion of value
        P = 1.0 / max(compute_z(disparity, min_disp, num_disp) + eps_dist, eps_dist)

        # 3) center bias (horizontal only here)
        cx = (x1 + (x2-x1)/2.0) / img_w  # [0,1]
        cx_norm = 2.0*cx - 1.0   # [-1,1]
        C = math.exp(-0.5 * (cx_norm / sigma_c)**2)

        # Combine
        lin = wA * (A ** alpha) + wP * (P ** beta) + wC * C
        R = sigmoid(k * lin)

        out.append({
            "distance_m": compute_z(disparity),
            "bbox": (x1, y1, x2, y2),
            "area_norm": A,
            "proximity": P,
            "center_bias": C,
            "risk": R
        })

    # Sort by risk descending
    out.sort(key=lambda d: d["risk"], reverse=True)
    return out

def check_risk(out):
    risk_obj = []
    for i in range(len(out)):
        if out[i]["risk"] >= RISK_THRESHOLD:
            print("COLLISION RISK: ", out[i])
            risk_obj.append(out[i])
    return risk_obj


if __name__ == "__main__":
    print("testing main")
    # Load YOLO model (you can use yolo11n.pt or yolov8m.pt)
    model = YOLO("yolo11n.pt")

    # Open webcam
    capL = rpi_camera(1)
    capR = rpi_camera(0)

    while True:
        detections = detect_obj(capL, capR, model)
        out = compute_risks(detections)
        risk_obj = check_risk(out)