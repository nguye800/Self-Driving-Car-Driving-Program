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
    limg: np.ndarray,
    rimg: np.ndarray,
    numdisp: int,
    mindisp: int = 0,
    blocksize: int = 21,
) -> np.ndarray:
    if blocksize % 2 == 0 or blocksize < 1:
        raise ValueError("blocksize must be a positive odd number")
    if numdisp <= 0:
        raise ValueError("numdisp must be > 0")

    h, w = limg.shape[:2]
    dispmap = np.zeros((h, w), dtype=np.uint8)

    w2 = blocksize // 2
    maxdisp = numdisp + mindisp

    # y from w2 to h - w2 - 1 (inclusive on both ends in C++; range end exclusive here)
    for y in range(w2, h - w2 - 1):
        # x from (maxdisp + w2) to (w + mindisp - w2 - 2)
        # Replicates: for (int x = maxdisp + w; x < limg.cols + mindisp - w - 1; x++)
        x_start = maxdisp + w2
        x_end_excl = w + mindisp - w2 - 1  # exclusive end for Python range
        if x_start >= x_end_excl:
            # Nothing to compute on this row; continue
            continue

        for x in range(x_start, x_end_excl):
            # Left feature patch centered at (x, y) with half-width w2
            # In C++: Rect(Point(x - w, y - w), Size(blocksize, blocksize))
            lx0, ly0 = x - w2, y - w2
            feature = limg[ly0 : ly0 + blocksize, lx0 : lx0 + blocksize]

            # Right image slice:
            # Rect(Point(x - w - maxdisp, y - w), Point(x + w + 1 - mindisp, y + w + 1))
            rx0 = x - w2 - maxdisp
            ry0 = y - w2
            rx1 = x + w2 + 1 - mindisp  # exclusive
            ry1 = y + w2 + 1            # exclusive

            # Bounds check (skip if the slice would go out-of-bounds)
            if rx0 < 0 or ry0 < 0 or rx1 > w or ry1 > h:
                continue

            rstrip = rimg[ry0:ry1, rx0:rx1]

            # Sanity: rstrip width should be blocksize + numdisp
            # Result width from matchTemplate (valid mode) = (W - blocksize + 1) = numdisp + 1
            ncc = cv2.matchTemplate(rstrip, feature, cv2.TM_CCORR_NORMED)
            # Grab location of max NCC
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(ncc)
            # In the original code, disparity written is `max.x` (0..numdisp)
            dispmap[y, x - mindisp] = np.uint8(max_loc[0])

    return dispmap


def show_dispmap_colorized(dispmap: np.ndarray, window_name: str = "disparity_map") -> None:
    # Normalize to 0..255 and apply JET colormap
    disp_norm = cv2.normalize(dispmap, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
    cv2.imshow(window_name, disp_color)
    cv2.waitKey(0)
    cv2.destroyWindow(window_name)


def main() -> int:
    try:
        capL = cv2.VideoCapture(0)   # left camera index
        capR = cv2.VideoCapture(1)   # right camera index

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
                break
        return 0

    except Exception as e:
        print(str(e), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())