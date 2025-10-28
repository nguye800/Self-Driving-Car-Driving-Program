#!/usr/bin/env python3
"""
Iterative NCC-based stereo disparity (Python port of the C++ example).

- Gets left/right images from cv2 and converts to grayscale
- For each valid pixel, extracts a (blocksize x blocksize) patch from the left
  image and slides it across a horizontal strip in the right image whose width
  is blocksize + numdisp (bounded by mindisp), computing NCC via matchTemplate.
- Picks the best x-offset (max NCC) as the disparity value at that pixel.
- Displays a colorized disparity map.

Note:
This replicates the original geometry/indexing: the assigned value is the
max-location x index from matchTemplate (0..numdisp), not adjusted by `mindisp`.
"""

import sys
import cv2
import numpy as np
# from picamera2 import Picamera2
import time

def compute_dispmap_sgbm(grayL, grayR, minDisp=16, numDisp=200, blocksize=5):
    blocksize = max(3, blocksize | 1)
    P1 = 8 * blocksize * blocksize
    P2 = 32 * blocksize * blocksize
    sgbm = cv2.StereoSGBM_create(
        minDisparity=minDisp,
        numDisparities=((numDisp + 15)//16)*16,
        blockSize=blocksize,
        P1=P1, P2=P2,
        disp12MaxDiff=1,          # set to -1 to be denser (more outliers)
        uniquenessRatio=8,        # lower (3–8) for density; higher (10–15) for precision
        speckleWindowSize=50,     # start small (0–50); raise later to clean up
        speckleRange=32,
        preFilterCap=31,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    disp16 = sgbm.compute(grayL, grayR)
    disp16[disp16 < (minDisp<<4)] = minDisp<<4
    return cv2.normalize(disp16, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

def test_camera(cam):
    # Get the default frame width and height# Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))
    
    while True:
        ret, frame = cam.read()

        # Write the frame to the output file
        out.write(frame)

        # Display the captured frame
        cv2.imshow('Camera', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    out.release()
    cv2.destroyAllWindows()
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))
    
    while True:
        ret, frame = cam.read()

        # Write the frame to the output file
        out.write(frame)

        # Display the captured frame
        cv2.imshow('Camera', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    # Release the capture and writer objects
    cam.release()
    out.release()
    cv2.destroyAllWindows()

def rpi_camera(camera):
    picam2 = Picamera2(camera)
    picam2.configure(picam2.create_preview_configuration(raw={"size":(4608,2592)},main={"format":'RGB888',"size": (1280,720)}))
    picam2.start()

    return picam2


def test_dual_cameras(camL_index=0, camR_index=1, record=False):
    capL = cv2.VideoCapture(camL_index, cv2.CAP_DSHOW)
    capR = cv2.VideoCapture(camR_index, cv2.CAP_DSHOW)

    # Set both cameras to the same nominal resolution and FPS
    for cap in (capL, capR):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # Initialize video writer if needed
    frame_width = int(capL.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(capL.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if record:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('dual_output.mp4', fourcc, 20.0, (frame_width * 2, frame_height))
    else:
        out = None

    while True:
        retL, frameL = capL.read()
        retR, frameR = capR.read()
        if not (retL and retR):
            print("Error: Could not read from both cameras.")
            break

        # --- Ensure both frames are the same height ---
        hL, wL = frameL.shape[:2]
        hR, wR = frameR.shape[:2]
        if (hL != hR) or (wL != wR):
            frameR = cv2.resize(frameR, (wL, hL), interpolation=cv2.INTER_AREA)

        # Optional: add labels
        cv2.putText(frameL, "Left", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.putText(frameR, "Right", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        # Combine side-by-side
        combined = np.hstack((frameL, frameR))

        # Write to file if recording
        if record:
            out.write(combined)

        # Resize for smaller window if needed
        # combined = cv2.resize(combined, (1280, 480))

        # Show combined framer   # right camera index 1 (0 for 
        combined_display = cv2.resize(combined, (1280, 480))
        cv2.imshow('Dual Cameras (L | R)', combined_display)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    capL.release()
    capR.release()
    if out:
        out.release()
    cv2.destroyAllWindows()

def initialize_cam(cap):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2304)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1296)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

def show_dispmap_colorized(dispmap: np.ndarray, window_name: str = "disparity_map", show=False):
    # Normalize to 0..255 and apply JET colormap
    disp_norm = cv2.normalize(dispmap, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)

    if show:
        return disp_color
    else:
        cv2.imshow(window_name, disp_color)
        cv2.waitKey(1)


if __name__ == "__main__":
    # rpi_camera(0)
    # rpi_camera(1)
    # capL = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    # test_camera(capL)
    # test_dual_cameras()
    # sys.exit(main())
    # Downscale first for speed
    capL = rpi_camera(0)   # left camera index
    # initialize_cam(capL)
    capR = rpi_camera(1)   # right camera index 1 (0 for testing bc only one camera)
    # initialize_cam(capR)
    while True:
        frameL = capL.capture_array()
        frameR = capR.capture_array()
        # frameL_small = cv2.pyrDown(frameL)  # 1280x720 -> 640x360
        # frameR_small = cv2.pyrDown(frameR)
        grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

        dispmap = compute_dispmap_sgbm(grayL, grayR)
        show_dispmap_colorized(dispmap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            capL.stop()
            capR.stop()
            cv2.destroyAllWindows()
            break
