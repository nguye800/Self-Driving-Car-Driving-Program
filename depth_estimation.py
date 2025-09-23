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

def compute_dispmap(
    limg: np.ndarray, # left image (reference/main)
    rimg: np.ndarray, # right image
    numdisp: int, # maximum pixels may shift (that we are checking)
    mindisp: int = 0, # minimum pixel disparity to be considered
    blocksize: int = 21, # pixel block H/W to compare L and R images
) -> np.ndarray:
    if blocksize % 2 == 0 or blocksize < 1:
        raise ValueError("blocksize must be a positive odd number")
    if numdisp <= 0:
        raise ValueError("numdisp must be > 0")

    h, w = limg.shape[:2] # dimensions for input image
    dispmap = np.zeros((h, w), dtype=np.uint8) # size of image provided, gives disparity for each pixel

    w2 = blocksize // 2
    maxdisp = numdisp + mindisp

    # for loop through effective y range (giving a 1/2 blocksize margin for y values bc 21x21 square wouldn't fit)
    for y in range(w2, h - w2 - 1):
        x_start = maxdisp + w2 # minimum x location where a start is valid due to disparity max and window margin
        x_end_excl = w + mindisp - w2 - 1 # maximum x range where window margin will still fit
        if x_start >= x_end_excl: # skip if bounds are invalid
            continue

        for x in range(x_start, x_end_excl):
            lx0, ly0 = x - w2, y - w2 # top left corner coord of the 21x21 window centered around x,y
            feature = limg[ly0 : ly0 + blocksize, lx0 : lx0 + blocksize] # 21x21 window

            # Define a horizontal strip to check disparity (x range: maxdisparity + window, y range: window)
            # assumes already rectified
            rx0 = x - w2 - maxdisp
            ry0 = y - w2
            rx1 = x + w2 + 1 - mindisp  # exclusive
            ry1 = y + w2 + 1            # exclusive

            # Bounds check (skip if the slice would go out-of-bounds)
            if rx0 < 0 or ry0 < 0 or rx1 > w or ry1 > h:
                continue

            rstrip = rimg[ry0:ry1, rx0:rx1] # strip from right image to compare against

            # Slides feature across rstrip and computes normalized cross-correlation
            ncc = cv2.matchTemplate(rstrip, feature, cv2.TM_CCORR_NORMED)
            # max_loc: offset in pixels where normalized cross-correlation is highest (most likely match point)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(ncc)
            # In the original code, disparity written is `max.x` (0..numdisp)
            dispmap[y, x - mindisp] = np.uint8(max_loc[0])

    return dispmap


def show_dispmap_colorized(dispmap: np.ndarray, window_name: str = "disparity_map") -> None:
    # Normalize to 0..255 and apply JET colormap
    disp_norm = cv2.normalize(dispmap, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
    cv2.imshow(window_name, disp_color)
    cv2.waitKey(1)

def test_camera(cam):
    # Get the default frame width and height
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

def initialize_cam(cap):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

def main() -> int:
    try:
        capL = cv2.VideoCapture(0, cv2.CAP_DSHOW)   # left camera index
        initialize_cam(capL)
        capR = cv2.VideoCapture(1, cv2.CAP_DSHOW)   # right camera index 1 (0 for testing bc only one camera)
        initialize_cam(capR)

        while True:
            retL, frameL = capL.read()
            retR, frameR = capR.read()
            if not retL or not retR:
                break
            grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
            grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)
            dispmap = compute_dispmap(
                grayL,
                grayR,
                numdisp=64,
                mindisp=0,
                blocksize=21,
            )
            show_dispmap_colorized(dispmap)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                capL.release()
                capR.release()
                cv2.destroyAllWindows()
                break
        return 0

    except Exception as e:
        print(str(e), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())