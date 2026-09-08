/*
 * triangle.c
 *
 *  Created on: Sep 6, 2026
 *      Author: Nymph
 *
 * See triangle.h for the geometry, the blanking rules and why this is an
 * animation source rather than a loop of its own.
 */

#include "triangle.h"

/* The shape about its own centroid, built once. Signed, and NOT clamped to
   8 bits -- these are offsets, not coordinates. */
static int16_t base_x[TRI_POINTS], base_y[TRI_POINTS];

/* One frame's worth of screen coordinates, rewritten each rotation step. */
static cc1101_point_t tri[TRI_POINTS];

static uint8_t angle;

/*
 * Fill the base shape with one span per row, bottom to top.
 *
 * Rows alternate direction (SERPENTINE) so the row-to-row connector is a short
 * hop along the shape's edge instead of a jump back across the figure. Every
 * connector is blanked either way, but a short one costs far less DAC time --
 * the beam still has to physically travel it.
 *
 * Point ORDER carries the blanking, so it is not cosmetic:
 *
 *   points 2i, 2i+1   -> segment 2i   EVEN -> span      -> drawn
 *   points 2i+1, 2i+2 -> segment 2i+1 ODD  -> connector -> blanked
 *
 * TRI_POINTS is even, so the wrap segment (last point back to point 0) is odd
 * and gets blanked too. That matters: the wrap runs from the apex all the way
 * down to the bottom-left corner.
 */
void triangle_init(void)
{
    const int span = TRI_ROWS - 1;    /* so row `span` lands exactly on the apex */

    for (int i = 0; i < TRI_ROWS; i++) {
        /* Centroid-relative: the base sits at -H/3 and the apex at +2H/3, which
           is what puts the rotation centre at the centroid rather than at the
           base. Rotating about anything else makes the triangle wobble. */
        int y    = -(TRI_H / 3) + (i * TRI_H) / span;
        int half = (TRI_B * (span - i)) / span;      /* full at base, 0 at apex */
        int xl   = -half;
        int xr   =  half;

        if (i & 1) { int t = xl; xl = xr; xr = t; }   /* odd rows run right-to-left */

        base_x[2*i    ] = (int16_t)xl;  base_y[2*i    ] = (int16_t)y;
        base_x[2*i + 1] = (int16_t)xr;  base_y[2*i + 1] = (int16_t)y;
    }

    angle = 0;
}

/*
 * Q15 sine, one revolution in ROT_STEPS steps. A table rather than sinf() so
 * this pulls in no libm and the arithmetic is exactly reproducible.
 * cos(i) = sin(i + ROT_STEPS/4).
 */
static const int16_t sin_q15[ROT_STEPS] = {
         0,   3212,   6393,   9512,  12539,  15446,  18204,  20787,
     23170,  25329,  27245,  28898,  30273,  31356,  32137,  32609,
     32767,  32609,  32137,  31356,  30273,  28898,  27245,  25329,
     23170,  20787,  18204,  15446,  12539,   9512,   6393,   3212,
         0,  -3212,  -6393,  -9512, -12539, -15446, -18204, -20787,
    -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
    -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329,
    -23170, -20787, -18204, -15446, -12539,  -9512,  -6393,  -3212,
};

/*
 * Rotate the base shape by `a` steps and drop it on the screen centre.
 *
 * Point ORDER is untouched, which is the whole reason this is safe: the
 * span/connector parity that drives Z blanking survives rotation unchanged.
 *
 * Products peak at 116 * 32767, well inside int32. The clamp should never
 * fire -- the geometry is sized to the rotation circle -- but a coordinate
 * that wrapped past 255 would draw a bright line clean across the image, so
 * it is cheap insurance against a future resize that forgets the constraint.
 */
static void rotate_into(cc1101_point_t *out, uint8_t a)
{
    const int32_t c = sin_q15[(a + ROT_STEPS/4) & (ROT_STEPS - 1)];
    const int32_t s = sin_q15[ a                & (ROT_STEPS - 1)];

    for (int i = 0; i < TRI_POINTS; i++) {
        int32_t bx = base_x[i], by = base_y[i];
        int32_t x = ((bx * c - by * s) >> 15) + ROT_CX;
        int32_t y = ((bx * s + by * c) >> 15) + ROT_CY;

        if (x <   0) x =   0;
        if (x > 255) x = 255;
        if (y <   0) y =   0;
        if (y > 255) y = 255;

        out[i].x = (uint8_t)x;
        out[i].y = (uint8_t)y;
    }
}

const cc1101_point_t *triangle_next(void *ctx, uint16_t *n_points)
{
    (void)ctx;
    rotate_into(tri, angle);
    if (n_points != NULL) *n_points = TRI_POINTS;
    return tri;                 /* never NULL: this source cannot starve */
}

void triangle_done(void *ctx)
{
    (void)ctx;
    angle = (uint8_t)((angle + ROT_ADVANCE) & (ROT_STEPS - 1));
}

uint8_t triangle_angle(void)
{
    return angle;
}
