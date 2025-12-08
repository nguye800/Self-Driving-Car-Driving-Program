# ----------------------------
# Stereo Vision Setup
# ----------------------------
import numpy as np
import cv2
import os
from automatic_drive import config
from picamera2 import Picamera2

def rpi_camera(camera):
    picam2 = Picamera2(camera)
    picam2.configure(picam2.create_preview_configuration(raw={"size":(4608,2592)},main={"format":'RGB888',"size": (1280,720)}))
    picam2.start()

    return picam2
    
def compute_dispmap_sgbm(grayL, grayR, minDisp=0, numDisp=64, blocksize=5):
    """Compute disparity map using Semi-Global Block Matching"""
    blocksize = max(3, blocksize | 1)
    P1 = 8 * blocksize * blocksize
    P2 = 32 * blocksize * blocksize
    
    sgbm = cv2.StereoSGBM_create(
        minDisparity=minDisp,
        numDisparities=numDisp,
        blockSize=blocksize,
        P1=P1, P2=P2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    
    disp16 = sgbm.compute(grayL, grayR)
    
    # Convert to float and scale (disparity is in 16-bit fixed point)
    disp_float = disp16.astype(np.float32) / 16.0
    
    return disp_float
  
def save_debug_depth_map(depth, w, h, step_idx, folder="depth_debug"):
    os.makedirs(folder, exist_ok=True)
    
    # Save raw depths
    npy_path = os.path.join(folder, f"raw_depth_{step_idx:05d}.npy")
    # depth is already a float32 array; save directly
    np.save(npy_path, depth)
    print(f"[DEBUG] Saved raw depth array: {npy_path}")

    valid = depth > 0.1
    depth_norm = np.zeros_like(depth, dtype=np.uint8)

    if np.any(valid):
        dv = depth[valid]

        # robust min/max → ignore extreme outliers
        d_min, d_max = np.percentile(dv, [5, 95])
        if d_max <= d_min:
            d_max = d_min + 1e-3

        dv_clipped = np.clip(dv, d_min, d_max)

        # we want RED = close, BLUE = far:
        # normalized t: 0 (far) → 1 (near)
        t = 1.0 - (dv_clipped - d_min) / (d_max - d_min)
        depth_norm[valid] = (t * 255).astype(np.uint8)

    depth_color = cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_JET)

    # ---------- save ----------
    filename = os.path.join(folder, f"depth_{step_idx:05d}.png")
    cv2.imwrite(filename, depth_color)
    print(f"[DEBUG] Saved depth map: {filename}")


STEREO_DECIMATE = 1

def summarize_region_depth(region_depth, min_depth=0.1, max_depth=5.0,
    default_val=5.0, low_quantile=0.2, near_threshold=1.0, min_near_fraction=0.01):
    """
    Summarize a region's depth with a value that:
    - is sensitive to thin, near obstacles (low percentile)
    - ignores noise if only a tiny fraction of pixels are close.
    """
    vals = region_depth.astype(np.float32).ravel()
    # Filter invalid / out-of-range
    vals = vals[(vals > min_depth) & (vals < max_depth)]
    if vals.size == 0:
        return default_val

    # How many pixels are "near"?
    near_mask = vals < near_threshold
    near_fraction = float(np.count_nonzero(near_mask)) / float(vals.size)

    if near_fraction < min_near_fraction:
        # Too few near pixels → probably just noise or background
        return default_val

    # Use a lower percentile so thin clusters of close pixels matter
    q = np.percentile(vals, low_quantile * 100.0)
    return float(q)


def get_obstacle_readings_from_stereo(camL, camR, baseline=0.055, focal_length=2571, visualize=False, save_disp=False):
    imgL = camL.getImage(); imgR = camR.getImage()
    if imgL is None or imgR is None:
        return config._last_readings

    w, h = camL.getWidth(), camL.getHeight()
    L = np.frombuffer(imgL, np.uint8).reshape((h, w, 4))
    R = np.frombuffer(imgR, np.uint8).reshape((h, w, 4))
    gL = cv2.cvtColor(L, cv2.COLOR_BGRA2GRAY)
    gR = cv2.cvtColor(R, cv2.COLOR_BGRA2GRAY)

    if STEREO_DECIMATE > 1:
        gL = cv2.resize(gL, (w//STEREO_DECIMATE, h//STEREO_DECIMATE))
        gR = cv2.resize(gR, (w//STEREO_DECIMATE, h//STEREO_DECIMATE))
        w, h = gL.shape[1], gL.shape[0]

    # lighter SGBM
    disp = compute_dispmap_sgbm(gL, gR, minDisp=0, numDisp=320, blocksize=5)
        
    if focal_length is None:
        focal_length = w * 0.8

    depth = np.zeros_like(disp, np.float32)
    m = disp > 1.0
    depth[m] = (baseline * focal_length) / disp[m]
    depth = np.clip(depth, 0, 5.0)
    
    if save_disp:
        save_debug_depth_map(depth, 2304, 1296, 0)

    h_start = int(0.35 * h)
    h_end   = int(0.65 * h)
    h_band  = slice(h_start, h_end)
    
    left_start = int(0.1 * w)
    left_end = int(0.3 * w)
    center_start = int(0.25 * w)
    center_end = int(0.75 * w)
    right_start = int(0.7 * w)
    right_end = int(0.9 * w)
    
    # region slices (arrays)
    l_region = depth[h_band, left_start:left_end] # 10–30%
    c_region = depth[h_band, center_start:center_end] # 25–75%
    r_region = depth[h_band, right_start:right_end] # 70–90%
    
    # scalar medians
    left   = summarize_region_depth(l_region)
    center = summarize_region_depth(c_region)
    right  = summarize_region_depth(r_region)
    
    config._last_readings = (center, left, right)

    return center, left, right
