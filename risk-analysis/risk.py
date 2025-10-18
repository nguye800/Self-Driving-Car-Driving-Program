from depth_estimation import compute_dispmap
import cv2
import numpy as np
import sys
from collections import deque
import math

'''
Risk Level Flags
0: No risk
1: Soft Slow Down
2: Hard Slow Down
3: Stop

Ultrasonic Distance Sensor
<0.5m: 3
<1m: 2
<1.5m: 1
>1.5m: 0
'''
baseline = 0.02 # meters
pixel_pitch = 0.0000014 # meters/px
focal_len = 0.0036 / pixel_pitch # px
# find all objects in the disp_map with similar pixel disparities within disp_margin
# startx, starty = bottom left corner
# format of objects list: (distance, startx, starty, xlen, ylen)
def find_objects(disp_map, disp_margin):
    seen = [[0]*len(disp_map[0]) for i in range(len(disp_map))]
    objects = []

    for r in range(len(disp_map)):
        for c in range(len(disp_map[0])):
            if seen[r][c] == 1: # skip if seen alr
                continue
            else: # start bfs
                totaldisp = 0
                count = 0

                startx = c
                starty = r
                maxx = c
                maxy = r

                q = deque()
                q.append((r,c))
                base_disp = disp_map[r][c]
                seen[r][c] = 1
                while q:
                    cur_r, cur_c = q.popleft()
                    totaldisp += disp_map[cur_r][cur_c]
                    count += 1
                    # add left
                    if cur_c > 0 and abs(base_disp - disp_map[cur_r][cur_c-1]) <= disp_margin and seen[cur_r][cur_c-1] == 0:
                        q.append((cur_r, cur_c-1))
                        seen[cur_r][cur_c-1] = 1
                        startx = min(cur_c - 1, startx)
                    # add right
                    if cur_c < len(disp_map[0])-1 and abs(base_disp - disp_map[cur_r][cur_c+1]) <= disp_margin and seen[cur_r][cur_c+1] == 0:
                        q.append((cur_r, cur_c+1))
                        seen[cur_r][cur_c+1] = 1
                        maxx = max(cur_c + 1, maxx)
                    # add top
                    if cur_r > 0 and abs(base_disp - disp_map[cur_r-1][cur_c]) <= disp_margin and seen[cur_r-1][cur_c] == 0:
                        q.append((cur_r-1, cur_c))
                        seen[cur_r-1][cur_c] = 1
                        starty = min(cur_r - 1, starty)
                    # add bottom
                    if cur_r < len(disp_map)-1 and abs(base_disp - disp_map[cur_r+1][cur_c]) <= disp_margin and seen[cur_r+1][cur_c] == 0:
                        q.append((cur_r+1, cur_c))
                        seen[cur_r+1][cur_c] = 1
                        maxy = max(cur_r + 1, maxy)
                # add object to objects dictionary
                avg_disp = totaldisp / count
                avg_distance = compute_z(avg_disp)
                objects.append((avg_distance, startx, starty, maxx-startx, maxy-starty))

    return objects

def sigmoid(x): 
    return 1.0 / (1.0 + math.exp(-x))

def compute_risks(objects, img_w, img_h,
                  wA=1.0, wP=2.0, wC=0.5, wM=2.0,
                  alpha=1.0, beta=1.0, k=6.0,
                  eps_dist=0.1, sigma_c=0.5,
                  use_motion=False):
    """
    objects: list of tuples (distance_m, x, y, w)
    returns: list of dicts with risk and fields
    """
    out = []
    for obj in objects:
        if use_motion and len(obj) >= 6:
            dist, x, y, w, h, vz = obj
        else:
            dist, x, y, w, h = obj[:5]
            vz = None

        # 1) area
        A = max(0.0, (w * h) / (img_w * img_h))

        # 2) proximity inverse distance + epsilon to avoid explosion of value
        P = 1.0 / max(dist + eps_dist, eps_dist)

        # 3) center bias (horizontal only here)
        cx = (x + w/2.0) / img_w          # [0,1]
        cx_norm = 2.0*cx - 1.0            # [-1,1]
        C = math.exp(-0.5 * (cx_norm / sigma_c)**2)

        # 4) motion term (optional)
        # if use_motion and vz is not None and vz > 0:  # approaching (positive towards camera)
        #     ttc = dist / max(vz, 1e-3)
        #     M = sigmoid(1.0 / max(ttc, 1e-3))
        # else:
        #     M = 1.0  # neutral if unknown

        # Combine (additive form recommended for easy tuning)
        lin = wA * (A ** alpha) + wP * (P ** beta) + wC * C #+ wM * M - wM  # subtract wM so M~1 doesn't dominate
        R = sigmoid(k * lin)

        out.append({
            "distance_m": dist,
            "bbox": (x, y, w, h),
            "area_norm": A,
            "proximity": P,
            "center_bias": C,
            #"motion_term": M,
            "risk": R
        })

    # Sort by risk descending
    out.sort(key=lambda d: d["risk"], reverse=True)
    return out


# assumes disparity in px
# returns -1 if avg disparity given is 0
def compute_z(disparity):
    if disparity == 0:
        return -1
    return (baseline * focal_len) / disparity

if __name__ == "__main__":
    print("testing main")