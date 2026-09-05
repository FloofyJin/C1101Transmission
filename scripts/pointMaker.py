#!/usr/bin/env python3
"""
vid2strands.py - convert a silhouette video (e.g. Bad Apple) into per-frame
line segments for a vector / XY display.

Pipeline:
    frame -> grayscale -> threshold -> contour trace -> scale to 0..255
          -> Douglas-Peucker simplification, binary-searched to fit a budget
          -> quantize to integer grid -> order contours to minimize beam travel
          -> flatten to independent segment pairs

OUTPUT FORMAT (strands.json)
    [
      [ [x0,y0],[x1,y1],  [x0,y0],[x1,y1],  ... ],   <- frame 0
      [ [x0,y0],[x1,y1],  ... ],                     <- frame 1
      ...
    ]
    Points come in pairs: index 0 is the start of a segment, index 1 its end,
    index 2 the start of the next, and so on. Every frame therefore has an
    even point count, capped at 2 * --budget.

    Origin defaults to BOTTOM-LEFT: (0,0) is bottom-left, (0,255) is top-left,
    y increases upward. Use --origin top-left for image-style coordinates.

Optional --emit-bin writes the same data as strands.bin:
    header (12 bytes):  "STRD", uint8 version=2, uint8 grid, uint16 frame_count,
                        uint16 fps_x100, uint16 max_points_per_frame
    per frame:          uint16 point_count, then point_count * 2 bytes (x, y)
                        consecutive pairs = one segment
    strands.hex is the same bytes as ASCII hex for $readmemh.

Requires: opencv-python, numpy
    pip install opencv-python numpy
"""

import argparse
import json
import os
import struct
import sys

import cv2
import numpy as np


# ----------------------------------------------------------------------------
# frame extraction
# ----------------------------------------------------------------------------

def iter_frames(path, target_fps, max_frames=None):
    """Yield BGR frames, resampled to target_fps by dropping/keeping source frames."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit(f"could not open video: {path}")
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    if not target_fps or target_fps <= 0:
        target_fps = src_fps
    inc = min(1.0, target_fps / src_fps)
    acc = 0.0
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        acc += inc
        if acc >= 1.0 - 1e-9:
            acc -= 1.0
            yield frame
            n += 1
            if max_frames and n >= max_frames:
                break
    cap.release()


# ----------------------------------------------------------------------------
# contour extraction
# ----------------------------------------------------------------------------

def frame_to_contours(frame, grid, letterbox, ink, thresh, min_area, close_px,
                      flip_y=True):
    """Threshold a frame and return contours already scaled into 0..grid space."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    _, bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    if ink == "dark":
        bw = 255 - bw

    if close_px > 0:
        k = np.ones((close_px, close_px), np.uint8)
        bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, k)
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k)

    # RETR_CCOMP keeps holes as their own contours - a vector display draws
    # outlines only, so a hole is just another closed curve to trace.
    cnts, _ = cv2.findContours(bw, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if letterbox:
        s = min(grid / w, grid / h)
        sx = sy = s
        ox = (grid - w * s) / 2.0
        oy = (grid - h * s) / 2.0
    else:
        sx, sy = grid / w, grid / h
        ox = oy = 0.0

    out = []
    for c in cnts:
        pts = c.reshape(-1, 2).astype(np.float64)
        pts[:, 0] = pts[:, 0] * sx + ox
        pts[:, 1] = pts[:, 1] * sy + oy
        if flip_y:  # image y grows downward; output y grows upward
            pts[:, 1] = grid - pts[:, 1]
        if len(pts) < 3:
            continue
        if abs(cv2.contourArea(pts.astype(np.float32))) < min_area:
            continue
        out.append(pts)
    return out


# ----------------------------------------------------------------------------
# simplification to a segment budget
# ----------------------------------------------------------------------------

def quantize(poly, grid):
    """Round to the integer grid and drop consecutive duplicates (incl. wrap)."""
    q = np.clip(np.rint(poly), 0, grid).astype(np.int32)
    keep = [q[0]]
    for p in q[1:]:
        if p[0] != keep[-1][0] or p[1] != keep[-1][1]:
            keep.append(p)
    while len(keep) > 1 and keep[0][0] == keep[-1][0] and keep[0][1] == keep[-1][1]:
        keep.pop()
    return np.array(keep, dtype=np.int32)


def polys_at_eps(contours, eps, grid):
    out = []
    for c in contours:
        a = cv2.approxPolyDP(c.astype(np.float32), eps, True).reshape(-1, 2)
        if len(a) < 3:
            continue
        q = quantize(a, grid)
        if len(q) >= 3:
            out.append(q)
    return out


def segment_count(polys):
    """A closed polygon of n vertices costs n segments (the closing edge included)."""
    return sum(len(p) for p in polys)


def simplify_to_budget(contours, budget, grid, eps_max=64.0):
    """Smallest epsilon (i.e. most detail) whose output still fits the budget."""
    contours = sorted(contours, key=lambda c: -abs(cv2.contourArea(c.astype(np.float32))))
    dropped = 0

    while True:
        coarse = polys_at_eps(contours, eps_max, grid)
        if segment_count(coarse) <= budget or len(contours) <= 1:
            break
        contours = contours[:-1]  # shed the smallest blob and retry
        dropped += 1

    lo, hi = 0.0, eps_max
    best = polys_at_eps(contours, eps_max, grid)
    clamped = segment_count(best) > budget

    for _ in range(22):
        mid = (lo + hi) / 2.0
        cand = polys_at_eps(contours, mid, grid)
        if segment_count(cand) <= budget:
            best, hi = cand, mid
        else:
            lo = mid

    if clamped:  # even max simplification overflows - hard-truncate to fit
        trimmed, used = [], 0
        for p in best:
            if used + len(p) > budget:
                break
            trimmed.append(p)
            used += len(p)
        best = trimmed
    return best, dropped, clamped


# ----------------------------------------------------------------------------
# ordering: cut down beam travel between contours
# ----------------------------------------------------------------------------

def order_polys(polys, start=(0, 0)):
    """Greedy nearest-neighbour over contours, rotating each closed loop so it
    begins at whichever vertex is nearest the beam's current position."""
    remaining = list(polys)
    cur = np.array(start, dtype=np.float64)
    ordered = []
    while remaining:
        best_i, best_j, best_d = 0, 0, float("inf")
        for i, p in enumerate(remaining):
            d = np.sum((p - cur) ** 2, axis=1)
            j = int(np.argmin(d))
            if d[j] < best_d:
                best_i, best_j, best_d = i, j, d[j]
        p = remaining.pop(best_i)
        p = np.roll(p, -best_j, axis=0)
        ordered.append(p)
        cur = p[0].astype(np.float64)  # closed loop ends where it started
    return ordered


def travel_cost(polys, start=(0, 0)):
    cur = np.array(start, dtype=np.float64)
    total = 0.0
    for p in polys:
        total += float(np.hypot(*(p[0] - cur)))
        cur = p[0].astype(np.float64)
    return total


# ----------------------------------------------------------------------------
# emit
# ----------------------------------------------------------------------------

def frame_to_pairs(polys):
    """Flatten closed polygons into independent segments: p0,p1, p0,p1, ...

    A closed polygon of n vertices yields n segments and therefore 2n points."""
    pts = []
    for p in polys:
        n = len(p)
        for i in range(n):
            a, b = p[i], p[(i + 1) % n]
            pts.append([int(a[0]), int(a[1])])
            pts.append([int(b[0]), int(b[1])])
    return pts


def write_json(path, frames_pts):
    """One frame per line - compact but still greppable and hand-editable."""
    with open(path, "w") as f:
        f.write("[\n")
        for i, pts in enumerate(frames_pts):
            row = ",".join(f"[{x},{y}]" for x, y in pts)
            f.write(f"    [{row}]" + (",\n" if i + 1 < len(frames_pts) else "\n"))
        f.write("]\n")


def write_binary(path, frames_pts, grid, fps):
    max_pts = max((len(f) for f in frames_pts), default=0)
    with open(path, "wb") as f:
        f.write(b"STRD")
        f.write(struct.pack("<BBHHH", 2, grid, len(frames_pts),
                            int(round(fps * 100)), max_pts))
        for pts in frames_pts:
            f.write(struct.pack("<H", len(pts)))
            f.write(bytes(v for p in pts for v in p))
    return max_pts


def write_hex(path, binpath):
    data = open(binpath, "rb").read()
    with open(path, "w") as f:
        f.write("\n".join(f"{b:02x}" for b in data) + "\n")


def render_preview(path, frames_pts, grid, fps, scale=3, flip_y=True):
    """Render straight from the emitted pairs - this validates the output data,
    not the intermediate polygons."""
    size = (grid + 1) * scale
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (size, size))
    for pts in frames_pts:
        img = np.zeros((size, size, 3), np.uint8)
        def px(p):
            y = (grid - p[1]) if flip_y else p[1]
            return (p[0] * scale, y * scale)
        for i in range(0, len(pts) - 1, 2):
            a, b = px(pts[i]), px(pts[i + 1])
            cv2.line(img, a, b, (80, 255, 120), 1, cv2.LINE_AA)
        vw.write(img)
    vw.release()


# ----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("-o", "--out", default="strands_out")
    ap.add_argument("--budget", type=int, default=180,
                    help="max segments per frame (points per frame = 2x this)")
    ap.add_argument("--grid", type=int, default=255, help="coords span 0..grid")
    ap.add_argument("--fps", type=float, default=20.0, help="output frame rate")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--ink", choices=["dark", "light"], default="dark",
                    help="which side of the threshold is the figure")
    ap.add_argument("--threshold", type=int, default=127)
    ap.add_argument("--min-area", type=float, default=6.0,
                    help="drop blobs smaller than this, in output units^2")
    ap.add_argument("--close-px", type=int, default=3,
                    help="morphological open+close kernel; 0 disables")
    ap.add_argument("--origin", choices=["bottom-left", "top-left"],
                    default="bottom-left",
                    help="bottom-left: (0,0) bottom-left, y up (default)")
    ap.add_argument("--stretch", action="store_true",
                    help="fill the square instead of letterboxing (distorts 4:3)")
    ap.add_argument("--emit-bin", action="store_true",
                    help="also write strands.bin / strands.hex")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    flip_y = args.origin == "bottom-left"

    frames_pts, stats = [], []
    for i, frame in enumerate(iter_frames(args.video, args.fps,
                                          args.max_frames or None)):
        cnts = frame_to_contours(frame, args.grid, not args.stretch, args.ink,
                                 args.threshold, args.min_area, args.close_px,
                                 flip_y)
        polys, dropped, clamped = simplify_to_budget(cnts, args.budget, args.grid)
        raw_travel = travel_cost(polys)
        polys = order_polys(polys)
        pts = frame_to_pairs(polys)

        assert len(pts) % 2 == 0, f"frame {i}: odd point count"
        assert len(pts) <= 2 * args.budget, f"frame {i}: {len(pts)} points over cap"

        frames_pts.append(pts)
        stats.append(dict(frame=i, segments=len(pts) // 2, points=len(pts),
                          contours=len(polys), dropped=dropped, clamped=clamped,
                          travel=round(travel_cost(polys), 1),
                          travel_unordered=round(raw_travel, 1)))
        if i % 50 == 0:
            print(f"  frame {i:5d}  segs={stats[-1]['segments']:4d}", flush=True)

    if not frames_pts:
        sys.exit("no frames decoded")

    jsonpath = os.path.join(args.out, "strands.json")
    write_json(jsonpath, frames_pts)
    if args.emit_bin:
        binpath = os.path.join(args.out, "strands.bin")
        write_binary(binpath, frames_pts, args.grid, args.fps)
        write_hex(os.path.join(args.out, "strands.hex"), binpath)
    if not args.no_preview:
        render_preview(os.path.join(args.out, "preview.mp4"),
                       frames_pts, args.grid, args.fps, flip_y=flip_y)

    segs = [s["segments"] for s in stats]
    clamped = [s for s in stats if s["clamped"]]
    worst = sorted(stats, key=lambda s: -s["segments"])[:10]
    saved = sum(s["travel_unordered"] - s["travel"] for s in stats)
    lines = [
        f"frames            {len(stats)}  @ {args.fps} fps  "
        f"({len(stats)/args.fps:.1f}s)",
        f"segments/frame    min {min(segs)}  mean {sum(segs)/len(segs):.1f}  max {max(segs)}",
        f"budget            {args.budget} segments = {2*args.budget} points",
        f"origin            {args.origin}",
        f"over budget       {len(clamped)} frames ({100*len(clamped)/len(stats):.1f}%) "
        f"- detail truncated",
        f"json size         {os.path.getsize(jsonpath)} bytes",
        f"travel saved      {saved:.0f} units total by reordering",
        "",
        "heaviest frames:",
    ] + [f"  {s['frame']:5d}  segs={s['segments']:4d}  pts={s['points']:4d}"
         f"  contours={s['contours']:3d}  dropped={s['dropped']}"
         f"  clamped={s['clamped']}" for s in worst]
    report = "\n".join(lines)
    with open(os.path.join(args.out, "report.txt"), "w") as f:
        f.write(report + "\n")
    print("\n" + report)


if __name__ == "__main__":
    main()