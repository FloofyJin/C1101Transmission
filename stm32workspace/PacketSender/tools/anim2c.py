#!/usr/bin/env python3
"""
anim2c.py -- turn a vector-animation JSON file into C arrays for the STM32.

WHY THIS IS A HOST-SIDE STEP AND NOT RUNTIME PARSING
----------------------------------------------------
The STM32F410RB has 128 KB flash, 32 KB RAM, no SD card and no filesystem
(see PacketSender.ioc -- the only peripherals are SPI and USART2). There is
no file to fopen() at runtime. On top of that, a JSON parser plus a decode
buffer would spend a good slice of the 32 KB RAM to re-derive, every boot,
something that is completely known at build time.

So the JSON is converted here, on your PC, into `const` tables that live in
flash and are read in place. cc1101_send_frame() takes a `const` pointer, so
the points never need copying into RAM at all.

INPUT FORMAT
------------
A list of frames; each frame a list of [x, y] pairs:

    [
      [ [0,0],[0,255],[255,0],[255,255] ],     <- frame 0
      [ [1,0],[1,255],[255,0],[255,255] ]      <- frame 1
    ]

Points are SPAN ENDPOINTS, not an outline: the FPGA's ScanoutEngine derives
blanking from segment parity, so each drawn row is a PAIR of points
(xL,y) then (xR,y). That is why every frame must hold an EVEN number of
points -- an odd count would split a span across a packet boundary.

USAGE
-----
    python tools/anim2c.py Core/Src/demoAnimation.json --fps 20 --kbps 38.4

Writes Core/Inc/animation_data.h and Core/Src/animation_data.c.
"""

import argparse
import json
import os
import sys

# ---- limits that come from the hardware, not from taste ----------------
CC_MAX_POINTS   = 28     # points per radio packet (RX FIFO bound, must be even)
CC_FRAME_POINTS = 1024   # PointRam depth per bank on the Zybo
COORD_MAX       = 255    # x and y are one byte each

# ---- airtime model -----------------------------------------------------
# On-air bytes per packet = preamble + sync(4) + len(1) + payload + CRC(2).
# Payload = 3 header bytes (start_index, count) + 2 bytes per point.
#
# The preamble is NOT fixed: MDMCFG1.NUM_PREAMBLE sets it, and it matters here
# because a longer preamble buys the receiver's AGC more settling time at the
# cost of airtime. Pass --preamble to match whatever MDMCFG1 says, or the budget
# will be optimistic. Sync is 4 bytes because SYNC_MODE=011 sends the 2-byte
# sync word twice.
#
# The per-packet overhead covers SPI transfer, STX and the synth recalibration
# between packets; 0.65 ms is what reconciles this model with the measured
# 15876 points/s at 500 kbps.
SYNC_BYTES       = 4
LEN_BYTES        = 1
CRC_BYTES        = 2
PKT_HDR_BYTES    = 3
PKT_OVERHEAD_MS  = 0.65

# MDMCFG1.NUM_PREAMBLE setting -> preamble length in bytes (datasheet p.78)
PREAMBLE_CHOICES = [2, 3, 4, 6, 8, 12, 16, 24]


def packet_airtime_ms(n_points, kbps, preamble=4):
    """Milliseconds to put one packet of n_points on air, including overhead."""
    payload = PKT_HDR_BYTES + 2 * n_points
    fixed = preamble + SYNC_BYTES + LEN_BYTES + CRC_BYTES
    bits = 8 * (fixed + payload)
    return bits / (kbps * 1000.0) * 1000.0 + PKT_OVERHEAD_MS


def frame_airtime_ms(n_points, kbps, preamble=4):
    """Milliseconds for a whole frame, which send_frame splits into packets."""
    if n_points == 0:
        return 0.0
    full, rest = divmod(n_points, CC_MAX_POINTS)
    total = full * packet_airtime_ms(CC_MAX_POINTS, kbps, preamble)
    if rest:
        total += packet_airtime_ms(rest, kbps, preamble)
    return total


def load_frames(path):
    with open(path, "r", encoding="utf-8") as fh:
        try:
            data = json.load(fh)
        except json.JSONDecodeError as exc:
            sys.exit("error: {} is not valid JSON: {}".format(path, exc))

    if not isinstance(data, list):
        sys.exit("error: top level must be a list of frames")
    if not data:
        sys.exit("error: animation has no frames")

    frames = []
    problems = []

    for fi, frame in enumerate(data):
        if not isinstance(frame, list):
            problems.append("frame {}: expected a list of points".format(fi))
            continue

        pts = []
        for pi, point in enumerate(frame):
            if not isinstance(point, list) or len(point) != 2:
                problems.append("frame {} point {}: expected [x, y]".format(fi, pi))
                continue
            x, y = point
            if not isinstance(x, int) or not isinstance(y, int):
                problems.append(
                    "frame {} point {}: x and y must be integers".format(fi, pi))
                continue
            if not (0 <= x <= COORD_MAX) or not (0 <= y <= COORD_MAX):
                problems.append(
                    "frame {} point {}: ({},{}) outside 0..{}".format(
                        fi, pi, x, y, COORD_MAX))
                continue
            pts.append((x, y))

        # Even count is a hard requirement -- points are span endpoints.
        if len(pts) % 2 != 0:
            problems.append(
                "frame {}: {} points is ODD; spans need endpoint pairs".format(
                    fi, len(pts)))
        if len(pts) > CC_FRAME_POINTS:
            problems.append(
                "frame {}: {} points exceeds PointRam depth ({})".format(
                    fi, len(pts), CC_FRAME_POINTS))
        frames.append(pts)

    if problems:
        for p in problems[:20]:
            print("  " + p, file=sys.stderr)
        if len(problems) > 20:
            print("  ... and {} more".format(len(problems) - 20), file=sys.stderr)
        sys.exit("error: {} problem(s) in {}".format(len(problems), path))

    return frames


def emit(frames, out_dir, name, fps, kbps, src_path):
    pool = []            # flat list of every point, all frames concatenated
    index = []           # (offset, count) per frame
    for pts in frames:
        index.append((len(pool), len(pts)))
        pool.extend(pts)

    inc_dir = os.path.join(out_dir, "Inc")
    src_dir = os.path.join(out_dir, "Src")
    os.makedirs(inc_dir, exist_ok=True)
    os.makedirs(src_dir, exist_ok=True)

    guard = "INC_" + name.upper() + "_DATA_H_"
    src_base = os.path.basename(src_path)
    banner = (
        "/* GENERATED by tools/anim2c.py from {} -- DO NOT EDIT.\n"
        " * Regenerate with:\n"
        " *   python tools/anim2c.py {} --fps {} --kbps {}\n"
        " */\n".format(src_base, src_path, fps, kbps))

    # ---- header ----
    h_path = os.path.join(inc_dir, name + "_data.h")
    with open(h_path, "w", encoding="utf-8") as fh:
        fh.write(banner)
        fh.write("#ifndef {}\n#define {}\n\n".format(guard, guard))
        fh.write('#include "animation.h"\n\n')
        fh.write("extern const anim_clip_t {}_clip;\n\n".format(name))
        fh.write("#endif /* {} */\n".format(guard))

    # ---- source ----
    c_path = os.path.join(src_dir, name + "_data.c")
    with open(c_path, "w", encoding="utf-8") as fh:
        fh.write(banner)
        fh.write('#include "{}_data.h"\n\n'.format(name))

        fh.write("/* Every point of every frame, back to back. {} points. */\n".format(
            len(pool)))
        fh.write("static const cc1101_point_t {}_points[{}] = {{\n".format(
            name, len(pool)))
        for fi, (off, cnt) in enumerate(index):
            fh.write("    /* frame {}: {} points ({} spans) */\n".format(
                fi, cnt, cnt // 2))
            for i in range(off, off + cnt, 8):
                chunk = pool[i:min(i + 8, off + cnt)]
                cells = ", ".join(
                    "{{{:3d},{:3d}}}".format(x, y) for x, y in chunk)
                fh.write("    " + cells + ",\n")
        fh.write("};\n\n")

        fh.write("/* Where each frame starts in the pool, and how long it is. */\n")
        fh.write("static const anim_frame_t {}_frames[{}] = {{\n".format(
            name, len(index)))
        for fi, (off, cnt) in enumerate(index):
            fh.write("    {{ {:5d}, {:4d} }},   /* frame {} */\n".format(off, cnt, fi))
        fh.write("};\n\n")

        fh.write("const anim_clip_t {}_clip = {{\n".format(name))
        fh.write("    .points   = {}_points,\n".format(name))
        fh.write("    .frames   = {}_frames,\n".format(name))
        fh.write("    .n_frames = {},\n".format(len(index)))
        fh.write("};\n")

    return h_path, c_path, pool, index


def report(pool, index, fps, kbps, h_path, c_path, preamble=4):
    period_ms = 1000.0 / fps
    counts = [c for _, c in index]
    worst = max(counts)
    worst_i = counts.index(worst)
    worst_ms = frame_airtime_ms(worst, kbps, preamble)

    flash = len(pool) * 2 + len(index) * 4
    budget_pts = 0
    while frame_airtime_ms(budget_pts + 2, kbps, preamble) <= period_ms:
        budget_pts += 2

    print("wrote " + h_path)
    print("wrote " + c_path)
    print()
    print("  frames          {}".format(len(index)))
    print("  points total    {}".format(len(pool)))
    print("  flash used      {} B ({:.1f} KB)".format(flash, flash / 1024.0))
    print("  playback        {} fps -> {:.1f} ms per frame".format(fps, period_ms))
    print("  link rate       {} kbps".format(kbps))
    print("  preamble        {} bytes -> {:.0f} us of AGC settling".format(
        preamble, preamble * 8 / (kbps * 1000.0) * 1e6))
    print("  fits in budget  {} points ({} spans) per frame".format(
        budget_pts, budget_pts // 2))
    print("  largest frame   #{}: {} points -> {:.1f} ms".format(
        worst_i, worst, worst_ms))
    print("  clip duration   {:.2f} s".format(len(index) / fps))
    print()

    over = [(i, c, frame_airtime_ms(c, kbps, preamble))
            for i, c in enumerate(counts)
            if frame_airtime_ms(c, kbps, preamble) > period_ms]
    if over:
        print("  WARNING: {} frame(s) cannot be sent in {:.1f} ms.".format(
            len(over), period_ms))
        print("           Those frames run late and the animation will stutter.")
        for i, c, ms in over[:5]:
            print("             frame {}: {} points needs {:.1f} ms".format(i, c, ms))
        if len(over) > 5:
            print("             ... and {} more".format(len(over) - 5))
        print("           Fix by: lowering --fps, raising the link rate, or")
        print("           simplifying to <= {} points per frame.".format(budget_pts))
    else:
        print("  OK: every frame fits inside the {:.1f} ms budget.".format(period_ms))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json", help="input animation JSON")
    ap.add_argument("-o", "--out-dir", default="Core",
                    help="project dir holding Inc/ and Src/ (default: Core)")
    ap.add_argument("-n", "--name", default="animation",
                    help="base name for the generated files (default: animation)")
    ap.add_argument("--fps", type=float, default=20.0,
                    help="playback rate used for the budget check (default: 20)")
    ap.add_argument("--preamble", type=int, default=4,
                    choices=PREAMBLE_CHOICES,
                    help="preamble bytes; must match MDMCFG1.NUM_PREAMBLE "
                         "(default: 4)")
    ap.add_argument("--kbps", type=float, default=38.4,
                    help="over-the-air bit rate (default: 38.4, the current "
                         "MDMCFG4/3 setting; use 500 after the M15 move)")
    args = ap.parse_args()

    frames = load_frames(args.json)
    h_path, c_path, pool, index = emit(
        frames, args.out_dir, args.name, args.fps, args.kbps, args.json)
    report(pool, index, args.fps, args.kbps, h_path, c_path, args.preamble)


if __name__ == "__main__":
    main()
