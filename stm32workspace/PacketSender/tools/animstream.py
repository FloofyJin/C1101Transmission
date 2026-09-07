#!/usr/bin/env python3
"""
animstream.py -- push an animation to the STM32 over the ST-Link virtual COM
port, one frame at a time, instead of compiling it into flash.

WHY THIS EXISTS
---------------
anim2c.py bakes a clip into `const` tables. That costs no RAM and needs no PC,
but 128 KB of flash holds only about 300 frames at 160 points -- roughly 15
seconds at 20 fps. Anything longer has to arrive at runtime, and the board's
only spare input is the UART already wired to the ST-Link.

THE PC DOES NOT SET THE FRAME RATE
----------------------------------
This is the part worth understanding. The board decides when each frame goes
on air (animation.c's deadline accumulator). This script only keeps the
board's buffer from running dry. It sends whenever the board says there is
room -- which is why there is no --fps option here.

If the PC paced the sending instead, every USB scheduling hiccup and every
Windows timer wobble would land directly on the oscilloscope. Buffer a few
frames ahead and let the board's crystal do the timing.

FLOW CONTROL
------------
The board reports the sequence number of the last frame it CONSUMED:

    AA 01 <seq>

and this script sends while  (last_sent - last_consumed) & 0xFF < SLOTS.

An absolute position rather than a credit count, so a lost CONTROL message
costs one stall of at most a heartbeat period instead of desynchronising the
window for the rest of the session.

A dropped DATA frame is the harder case, and it deadlocked a real run: a frame
the board throws away is never consumed, so last_consumed cannot advance past
it, and SLOTS consecutive drops stall the link permanently. The board now
advances the window itself when it rejects a frame for LENGTH, and STALL_TIMEOUT
below covers the CRC case, where the sequence byte cannot be trusted.

BLANK FRAMES
------------
A frame with zero points is legal and means "nothing to draw". The board
consumes it, skips the radio, and the picture holds for that period. Bad Apple
has 44 of them, all in runs -- rejecting zero was what triggered the deadlock
above.

USAGE
-----
    pip install pyserial
    python tools/animstream.py Core/Src/badapple.json --port COM5

    (--port is optional if exactly one ST-Link/USB serial port is present.)
"""

import argparse
import os
import struct
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("error: pyserial is not installed.  pip install pyserial")

# Reuse anim2c's loader so both paths enforce the same rules -- even point
# counts, 0..255 coordinates. A frame this script accepts and anim2c rejects
# would be a trap waiting for whoever switches modes.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import anim2c  # noqa: E402

# ---- must match anim_stream.h --------------------------------------------
MAGIC       = b"\xA5\x5A"
TAG         = 0xAA
MSG_WINDOW  = 0x01
MSG_ERROR   = 0x02
MSG_HELLO   = 0x03
CTL1        = 0x5B          # second magic byte of a PC->board control message
CTL_RESET   = 0x01
SLOTS       = 4
MAX_POINTS  = 180

# How long the window may sit still before the sender assumes the outstanding
# frames were dropped and writes them off. Comfortably longer than the board's
# 250 ms heartbeat, so an ordinary lost control message is recovered by the
# next heartbeat and never reaches this path.
STALL_TIMEOUT = 1.0

ERR_NAMES = {0x01: "CRC", 0x02: "LENGTH", 0x03: "BUFFER FULL"}


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def session_reset_bytes(max_points):
    """
    Bytes that put the board's receiver back to a known state, whatever it was
    doing when the last sender died.

    The padding is the load-bearing part. Kill a sender mid-frame and the board
    is stranded in its payload state, still counting down bytes it will never
    get; hand it three bytes of reset and it swallows them as payload and stays
    stuck. So first push enough zeros to finish off the largest possible
    truncated frame -- the parser then fails that frame's CRC, returns to
    hunting for magic, and the reset messages behind the padding are seen
    cleanly. Zeros are safe filler because 0x00 is not the magic byte.

    Sent three times because it is cheap (9 bytes) and because losing the
    handshake costs the whole session.
    """
    pad = b"\x00" * (2 * max_points + 4)          # payload + CRC, worst case
    return pad + bytes([MAGIC[0], CTL1, CTL_RESET]) * 3


def encode_frame(seq, pts):
    body = bytes([seq]) + struct.pack("<H", len(pts))
    body += b"".join(bytes((x, y)) for x, y in pts)
    return MAGIC + body + struct.pack("<H", crc16_ccitt(body))


def limit_spans(pts, max_points):
    """
    Thin a frame down to max_points by dropping whole SPANS at even intervals.

    Truncating the tail instead would lop off one side of the picture, because
    scanline fill emits spans in row order. Keeping an evenly spaced subset
    preserves the silhouette and just coarsens it -- the same thing as
    rendering the shape at fewer scanlines.

    Spans, never individual points: a point is half of a span, and splitting a
    pair makes the FPGA draw a line from one shape's edge to another's.
    """
    if len(pts) <= max_points:
        return pts
    spans = [pts[i:i + 2] for i in range(0, len(pts), 2)]
    keep = max_points // 2
    picked = [spans[i * len(spans) // keep] for i in range(keep)]
    return [p for span in picked for p in span]


def autodetect_port():
    ports = list(serial.tools.list_ports.comports())
    likely = [p for p in ports
              if "STLink" in (p.description or "") or "ST-Link" in (p.description or "")
              or "STMicroelectronics" in (p.manufacturer or "")]
    if len(likely) == 1:
        return likely[0].device
    if len(ports) == 1:
        return ports[0].device
    listing = "\n".join("    {}  {}".format(p.device, p.description) for p in ports)
    sys.exit("error: could not pick a port automatically. Use --port.\n"
             "  available:\n" + (listing if listing else "    (none)"))


class Link(object):
    """Reader half: control messages update the window, everything else is log
    text from the board's printf and is echoed to the console."""

    def __init__(self, ser, quiet):
        self.ser = ser
        self.quiet = quiet
        self.last_consumed = 0xFF
        self.hello = threading.Event()
        self.errors = {}
        self.lock = threading.Lock()
        self.stop = False
        self._text = bytearray()
        self._ctl = bytearray()

    def run(self):
        while not self.stop:
            data = self.ser.read(256) or b""
            if not data:
                continue
            for b in data:
                self._feed(b)

    def _feed(self, b):
        if self._ctl:
            self._ctl.append(b)
            if len(self._ctl) == 3:
                self._control(self._ctl[1], self._ctl[2])
                self._ctl = bytearray()
            return

        if b == TAG:
            # 0xAA can never come from printf -- its output is all ASCII -- so
            # this is unambiguous without escaping the log stream.
            self._flush_text()
            self._ctl = bytearray([b])
            return

        if b in (0x0A, 0x0D):
            self._flush_text()
        else:
            self._text.append(b)

    def _flush_text(self):
        if self._text and not self.quiet:
            try:
                print("  [board] " + self._text.decode("ascii", "replace"))
            except Exception:
                pass
        self._text = bytearray()

    def _control(self, kind, arg):
        with self.lock:
            if kind == MSG_WINDOW:
                self.last_consumed = arg
            elif kind == MSG_HELLO:
                self.last_consumed = 0xFF
                self.hello.set()
            elif kind == MSG_ERROR:
                name = ERR_NAMES.get(arg, "0x%02X" % arg)
                self.errors[name] = self.errors.get(name, 0) + 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json", help="animation JSON, same format as anim2c.py")
    ap.add_argument("--port", help="serial port (default: autodetect ST-Link)")
    ap.add_argument("--baud", type=int, default=230400,
                    help="must match huart2.Init.BaudRate (default: 230400)")
    ap.add_argument("--max-points", type=int, default=MAX_POINTS,
                    help="cap per frame; must match ANIM_STREAM_MAX_POINTS "
                         "(default: %d)" % MAX_POINTS)
    ap.add_argument("--slots", type=int, default=SLOTS,
                    help="must match ANIM_STREAM_SLOTS (default: %d)" % SLOTS)
    ap.add_argument("--loop", action="store_true", help="repeat forever")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress the board's printf output")
    args = ap.parse_args()

    frames = anim2c.load_frames(args.json)

    thinned = 0
    out = []
    for f in frames:
        g = limit_spans(f, args.max_points)
        if len(g) != len(f):
            thinned += 1
        out.append(g)
    frames = out

    port = args.port or autodetect_port()
    ser = serial.Serial(port, args.baud, timeout=0.05, write_timeout=2.0)

    link = Link(ser, args.quiet)
    reader = threading.Thread(target=link.run)
    reader.daemon = True
    reader.start()

    print("port      {} @ {} baud".format(port, args.baud))
    print("clip      {} frames, {} points max".format(
        len(frames), max(len(f) for f in frames)))
    if thinned:
        print("thinned   {} frame(s) reduced to {} points".format(
            thinned, args.max_points))
    print("window    {} frames in flight".format(args.slots))
    print()

    # ---- session handshake ----
    #
    # The board outlives this script, so on a restart it is still holding the
    # previous session's sequence number, slots and half-parsed frame. Clear
    # the OS buffers (they may hold seconds of the board's log output from
    # before we opened the port), then ask the board to do the same.
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(session_reset_bytes(args.max_points))
    ser.flush()

    print("resetting the board's receiver...")
    if link.hello.wait(timeout=3.0):
        print("  board acknowledged")
        print("")
    else:
        # Not fatal: an older build has no reset handler, and the stall timeout
        # below will drag the window back into line within a second or two.
        print("  no acknowledgement -- older firmware? continuing anyway")
        print("")

    seq_next = 0
    idx = 0
    sent = 0
    t0 = time.time()
    t_report = t0
    t_window = t0            # last time the board's window actually moved
    prev_consumed = None
    stalls = 0
    resyncs = 0

    try:
        while True:
            now = time.time()

            if idx >= len(frames):
                if not args.loop:
                    break
                idx = 0

            with link.lock:
                consumed = link.last_consumed
            last_sent = (seq_next - 1) & 0xFF
            in_flight = (last_sent - consumed) & 0xFF

            if in_flight >= args.slots:
                # Board is full. Sleeping beats spinning: at 20 fps a slot
                # opens every 50 ms, and a 1 ms sleep keeps latency far below
                # that while leaving the CPU alone.
                stalls += 1
                time.sleep(0.001)

                # ---- stall timeout ----
                #
                # A frame the board DROPS is never consumed, so last_consumed
                # can never advance past it and the window stays full forever.
                # The board self-heals a length rejection by advancing the
                # window itself, but it deliberately will not do that for a
                # CRC failure -- the sequence byte is inside the CRC's
                # coverage, so it may be the corrupted byte, and trusting it
                # could throw the window hundreds of frames out.
                #
                # So the sender decides instead. If the window has not moved
                # for STALL_TIMEOUT while frames are outstanding, those frames
                # are gone: write them off and carry on. Costs a few dropped
                # frames rather than the rest of the clip.
                if now - t_window >= STALL_TIMEOUT:
                    with link.lock:
                        link.last_consumed = last_sent
                    resyncs += 1
                    t_window = now
                    print("  window stalled {:.1f}s at seq {} -- assuming {} "
                          "frame(s) lost, resyncing".format(
                              STALL_TIMEOUT, consumed, in_flight))
            else:
                ser.write(encode_frame(seq_next, frames[idx]))
                seq_next = (seq_next + 1) & 0xFF
                idx += 1
                sent += 1

            if consumed != prev_consumed:
                prev_consumed = consumed
                t_window = now

            if now - t_report >= 1.0:
                fps = sent / (now - t0)
                with link.lock:
                    errs = dict(link.errors)
                msg = "sent={:6d}  {:5.1f} fps avg  in_flight={}".format(
                    sent, fps, in_flight)
                if resyncs:
                    msg += "  resyncs={}".format(resyncs)
                if errs:
                    msg += "  errors=" + ", ".join(
                        "{} x{}".format(k, v) for k, v in sorted(errs.items()))
                print(msg)
                t_report = now

        # Let the tail drain before closing, or the last few frames die in the
        # driver buffer when the port closes.
        time.sleep(0.5 * args.slots / 10.0 + 0.3)
        print("\ndone: {} frames in {:.1f} s".format(sent, time.time() - t0))

    except KeyboardInterrupt:
        print("\ninterrupted after {} frames".format(sent))
    finally:
        link.stop = True
        ser.close()


if __name__ == "__main__":
    main()
