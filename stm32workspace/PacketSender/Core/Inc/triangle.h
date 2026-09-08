/*
 * triangle.h -- a filled triangle that rotates, as an animation source.
 *
 * WHAT IT IS FOR
 * --------------
 * The bring-up pattern. It needs no PC, no JSON, no flash tables and no radio
 * traffic in the inbound direction, so when the scope shows nothing this is
 * the mode that says whether the fault is upstream of the transmitter at all.
 * It also moves continuously, which a static test pattern does not: a frozen
 * triangle and a triangle that is not being updated look identical, but a
 * stopped rotation is obvious.
 *
 * IT IS AN anim_source_fn, NOT A LOOP OF ITS OWN
 * ----------------------------------------------
 * triangle_next/triangle_done plug into animation_init_source() exactly the
 * way anim_stream_next/anim_stream_release do. That is deliberate: the frame
 * rate, the deadline accumulator and the overrun guard are then identical in
 * all three modes, and main() has one tick call instead of three code paths
 * with three different notions of timing.
 *
 * Previously this pattern ran in its own branch of the main loop with a
 * HAL_Delay() for pacing, which meant the spin rate was whatever the radio
 * happened to manage that second.
 *
 * NOT A 3-POINT OUTLINE
 * ---------------------
 * ScanoutEngine derives blanking from segment PARITY -- even segments are
 * spans, odd are connectors -- so a 3-corner polygon gets two of its three
 * sides blanked. The pipeline draws scanline fill, so the triangle has to
 * arrive as fill: one span per row, two points each.
 *
 *          apex (CX, Y_TOP)
 *               /\
 *              /  \          row i:   xL(i) ......... xR(i)   all at y(i)
 *             /    \
 *            /______\        base 2*TRI_B wide
 *
 * ROTATION IS SAFE HERE
 * ---------------------
 * Rotating span endpoints is legitimate because blanking comes from segment
 * parity, not from y being constant: a rotated span is still segment 2i (even
 * -> drawn) and its connector still 2i+1 (odd -> blanked). Rigid rotation also
 * preserves path length, so the DAC time budget does not change with angle.
 *
 * The size is set by the ROTATION CIRCLE, not the screen. Every point must
 * stay inside 0..255 at EVERY angle or the coordinate wraps and draws a line
 * across the image. With the centroid at (128,128) the budget is a radius of
 * 128; TRI_B=100 / TRI_H=173 puts the base corners at 115.4 and the apex at
 * 115.3 -- near-equal, the best aspect ratio for filling a circle. Swept over
 * all angles the extremes are 12.4 and 243.6.
 */

#ifndef INC_TRIANGLE_H_
#define INC_TRIANGLE_H_

#include "animation.h"
#include "cc1101.h"
#include <stdint.h>

/*
 * Row pitch is TRI_H / (TRI_ROWS - 1). Keep it near the FPGA's SPACING (4
 * coordinate units): the design rule is SPACING ~= row pitch ~= beam spot, and
 * a finer pitch than SPACING spends DAC time without making the fill look any
 * more solid.
 *
 *   64 rows over 173 units -> 2.7 units/row, 128 points, 5 packets/frame
 */
#define TRI_ROWS       64            /* spans; 2 points each                  */
#define TRI_B          100           /* half base width, from the centroid    */
#define TRI_H          173           /* apex-to-base height                   */

#define TRI_POINTS     (2 * TRI_ROWS)   /* EVEN, and <= CC_FRAME_POINTS       */

#define ROT_STEPS      64            /* angles per revolution; power of two   */
#define ROT_CX         128           /* rotation centre                       */
#define ROT_CY         128

/*
 * Angle steps per frame; negative spins the other way.
 *
 * One revolution takes ROT_STEPS / ROT_ADVANCE frames, so at 20 fps this is
 * 3.2 s per turn. Raise it for a faster spin -- do NOT try to speed it up by
 * raising the frame rate, which changes the radio's budget as well.
 */
#define ROT_ADVANCE    1

/* Build the base shape. Call once, before the first triangle_next(). */
void triangle_init(void);

/* ---- anim_source_fn / anim_done_fn: pass these to animation_init_source ----
 *
 * next() renders the current angle and hands back a buffer valid until done().
 * done() advances the angle, so the rotation steps once per frame TRANSMITTED
 * rather than once per call -- a frame that fails to send is retried at the
 * same angle instead of silently skipping one. */
const cc1101_point_t *triangle_next(void *ctx, uint16_t *n_points);
void                  triangle_done(void *ctx);

/* Current angle, for the status print. */
uint8_t triangle_angle(void);

#endif /* INC_TRIANGLE_H_ */
