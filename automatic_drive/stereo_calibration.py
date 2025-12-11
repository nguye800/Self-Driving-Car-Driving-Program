#!/usr/bin/env python3
import cv2
import numpy as np
import time
import os
from picamera2 import Picamera2

# -------------------------------
# User parameters
# -------------------------------
CHESSBOARD_SIZE = (9, 6)     # inner corners (# of intersections)
SQUARE_SIZE = 0.025          # meters per square (modify to your board)
NUM_IMAGES = 25              # number of pairs to collect
SAVE_DIR = "calib_images"
CALIB_FILE = "stereo_calib.npz"
DISPLAY = True

# -------------------------------
# Helper: start cameras
# -------------------------------
def start_camera(idx):
    cam = Picamera2(idx)
    cam.configure(
        cam.create_preview_configuration(
            raw={"size": (4608, 2592)},
            main={"format": "RGB888", "size": (1920, 1080)}
        )
    )
    cam.start()
    return cam

# -------------------------------
# Helper: capture grayscale frame
# -------------------------------
def grab_gray(cam):
    frame = cam.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    return gray

# -------------------------------
# Collect calibration images
# -------------------------------
def collect_images():
    os.makedirs(SAVE_DIR, exist_ok=True)
    camL = start_camera(1)
    camR = start_camera(0)
    time.sleep(2)

    collected = 0
    print("== Move chessboard slowly in front of both cameras ==")

    while collected < NUM_IMAGES:
        gL = grab_gray(camL)
        gR = grab_gray(camR)

        retL, cornersL = cv2.findChessboardCorners(gL, CHESSBOARD_SIZE)
        retR, cornersR = cv2.findChessboardCorners(gR, CHESSBOARD_SIZE)

        # --- DEBUG PRINTS ---
        print(f"Detect Left: {retL}, Detect Right: {retR}")

        if not retL:
            print(" Left camera did NOT detect chessboard")
        if not retR:
            print(" Right camera did NOT detect chessboard")
        if retL and retR:
            print(" Both cameras detected chessboard!")

        if retL and retR:
            collected += 1
            print(f"[{collected}/{NUM_IMAGES}] Pair captured")

            cv2.imwrite(f"{SAVE_DIR}/left_{collected:02d}.png", gL)
            cv2.imwrite(f"{SAVE_DIR}/right_{collected:02d}.png", gR)

            if DISPLAY:
                visL = gL.copy()
                visR = gR.copy()
                cv2.drawChessboardCorners(visL, CHESSBOARD_SIZE, cornersL, retL)
                cv2.drawChessboardCorners(visR, CHESSBOARD_SIZE, cornersR, retR)
                cv2.imshow("Left", visL)
                cv2.imshow("Right", visR)
                cv2.waitKey(300)
        else:
            if DISPLAY:
                cv2.imshow("Left", gL)
                cv2.imshow("Right", gR)
                cv2.waitKey(1)

    camL.stop()
    camR.stop()
    cv2.destroyAllWindows()


# -------------------------------
# Run stereo calibration
# -------------------------------
def stereo_calibrate():
    # Prepare 3D object points
    objp = np.zeros((CHESSBOARD_SIZE[0]*CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.indices(CHESSBOARD_SIZE).T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []
    imgpointsL = []
    imgpointsR = []

    # Load all pairs
    file_list = sorted(os.listdir(SAVE_DIR))
    left_files = [f for f in file_list if f.startswith("left")]
    right_files = [f for f in file_list if f.startswith("right")]

    for lf, rf in zip(left_files, right_files):
        imgL = cv2.imread(os.path.join(SAVE_DIR, lf), cv2.IMREAD_GRAYSCALE)
        imgR = cv2.imread(os.path.join(SAVE_DIR, rf), cv2.IMREAD_GRAYSCALE)

        retL, cornersL = cv2.findChessboardCorners(imgL, CHESSBOARD_SIZE)
        retR, cornersR = cv2.findChessboardCorners(imgR, CHESSBOARD_SIZE)

        if retL and retR:
            objpoints.append(objp)
            imgpointsL.append(cornersL)
            imgpointsR.append(cornersR)

    print(f"Using {len(objpoints)} valid pairs for calibration...")

    # Mono calibration
    h, w = imgL.shape[:2]

    retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(
        objpoints, imgpointsL, (w, h), None, None
    )
    retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(
        objpoints, imgpointsR, (w, h), None, None
    )

    # Stereo calibration
    flags = (
        cv2.CALIB_FIX_INTRINSIC |
        cv2.CALIB_USE_INTRINSIC_GUESS
    )

    retval, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpointsL,
        imgpointsR,
        mtxL, distL,
        mtxR, distR,
        (w, h),
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 200, 1e-5)
    )

    # Rectification
    RL, RR, PL, PR, Q, validL, validR = cv2.stereoRectify(
        mtxL, distL,
        mtxR, distR,
        (w, h),
        R, T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0
    )

    # Rectification maps
    mapL1, mapL2 = cv2.initUndistortRectifyMap(
        mtxL, distL, RL, PL, (w, h), cv2.CV_16SC2
    )
    mapR1, mapR2 = cv2.initUndistortRectifyMap(
        mtxR, distR, RR, PR, (w, h), cv2.CV_16SC2
    )

    # Save calibration
    np.savez(
        CALIB_FILE,
        mtxL=mtxL, distL=distL,
        mtxR=mtxR, distR=distR,
        R=R, T=T, E=E, F=F,
        RL=RL, RR=RR, PL=PL, PR=PR, Q=Q,
        mapL1=mapL1, mapL2=mapL2,
        mapR1=mapR1, mapR2=mapR2
    )

    print(f"\nSaved calibration to {CALIB_FILE}")


# -------------------------------
# Main
# -------------------------------
if __name__ == "__main__":
    print("=== STEP 1: Capturing images ===")
    collect_images()

    print("=== STEP 2: Running stereo calibration ===")
    stereo_calibrate()

    print("=== DONE ===")
