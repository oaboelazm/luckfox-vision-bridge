from __future__ import annotations

import argparse
import glob
import json

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description="Simple camera calibration utility")
    parser.add_argument("--images", required=True, help="Glob path for checkerboard images")
    parser.add_argument("--board-w", type=int, default=9)
    parser.add_argument("--board-h", type=int, default=6)
    parser.add_argument("--square", type=float, default=0.024)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pattern_size = (args.board_w, args.board_h)

    objp = np.zeros((args.board_w * args.board_h, 3), np.float32)
    objp[:, :2] = np.mgrid[0 : args.board_w, 0 : args.board_h].T.reshape(-1, 2)
    objp *= args.square

    objpoints = []
    imgpoints = []

    images = glob.glob(args.images)
    if not images:
        print("No images found")
        return 1

    gray_shape = None
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if found:
            objpoints.append(objp)
            imgpoints.append(corners)
            gray_shape = gray.shape[::-1]

    if not objpoints or gray_shape is None:
        print("No valid checkerboard detections")
        return 1

    ret, mtx, dist, _rvecs, _tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)
    out = {
        "rms": ret,
        "fx": float(mtx[0, 0]),
        "fy": float(mtx[1, 1]),
        "cx": float(mtx[0, 2]),
        "cy": float(mtx[1, 2]),
        "distortion": [float(x) for x in dist.flatten().tolist()],
    }
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
