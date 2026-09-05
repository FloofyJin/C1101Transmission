/*
 * animation.c
 *
 *  Created on: Sep 4, 2026
 *      Author: Nymph
 */

/*
 * See animation.h for what this is and why the pacing lives on the transmit
 * side.
 */

#include "animation.h"

/* Clamp so a zero or absurd fps cannot produce a zero period (which would
   busy-send forever) or a division by zero. */
#define ANIM_FPS_MIN   1u
#define ANIM_FPS_MAX   1000u

static uint32_t period_from_fps(uint16_t fps)
{
    uint32_t f = (uint32_t)fps;
    if (f < ANIM_FPS_MIN) f = ANIM_FPS_MIN;
    if (f > ANIM_FPS_MAX) f = ANIM_FPS_MAX;
    /* Rounded rather than truncated: 30 fps is 33 ms, not 33.333 truncated
       to something that drifts a whole frame every few seconds. */
    return (1000u + f / 2u) / f;
}

void animation_init(animation_t *a, const anim_clip_t *clip, uint16_t fps)
{
    if (a == NULL) return;

    a->clip          = clip;
    a->period_ms     = period_from_fps(fps);
    a->index         = 0;
    a->running       = true;
    a->frames_sent   = 0;
    a->frames_failed = 0;
    a->frames_late   = 0;
    a->loops         = 0;

    /* Due immediately, so the first tick sends frame 0 without a period of
       dead air at startup. */
    a->next_deadline = HAL_GetTick();
}

void animation_stop(animation_t *a)
{
    if (a != NULL) a->running = false;
}

void animation_start(animation_t *a)
{
    if (a == NULL || a->running) return;
    a->running = true;
    /* Resume from now. Carrying the old deadline forward would make the clip
       sprint through however many frames "should" have played while paused. */
    a->next_deadline = HAL_GetTick();
}

void animation_rewind(animation_t *a)
{
    if (a == NULL) return;
    a->index         = 0;
    a->next_deadline = HAL_GetTick();
}

void animation_set_fps(animation_t *a, uint16_t fps)
{
    if (a == NULL) return;
    a->period_ms = period_from_fps(fps);
}

uint16_t animation_next_points(const animation_t *a)
{
    if (a == NULL || a->clip == NULL || a->clip->n_frames == 0) return 0;
    return a->clip->frames[a->index].n_points;
}

bool animation_tick(animation_t *a)
{
    if (a == NULL || a->clip == NULL || !a->running) return false;
    if (a->clip->n_frames == 0) return false;

    uint32_t now = HAL_GetTick();

    /*
     * Signed difference, not `now < deadline`. HAL_GetTick() wraps every ~49
     * days; an unsigned compare would stall playback for the rest of time at
     * the wrap. The signed form only cares about the distance between the two.
     */
    if ((int32_t)(now - a->next_deadline) < 0) return false;   /* not due yet */

    const anim_frame_t *f = &a->clip->frames[a->index];

    /*
     * An empty frame cannot be transmitted: with no points there is no packet,
     * so no end-of-frame flag, so the receiver never swaps banks. Skip the
     * radio but still consume the time slot -- the display holds the previous
     * frame for one period, which is the closest thing to "nothing changed"
     * this protocol can express.
     */
    bool ok = true;
    if (f->n_points > 0) {
        ok = cc1101_send_frame(&a->clip->points[f->offset], f->n_points);
    }

    if (ok) a->frames_sent++;
    else    a->frames_failed++;

    /*
     * ---- the actual pacing ----
     *
     * Advance an ABSOLUTE schedule. Writing
     *
     *     next_deadline = HAL_GetTick() + period_ms;
     *
     * instead would make every frame take (its own transmit time + period),
     * so dense frames would still run long and the whole point would be lost.
     * Accumulating onto the previous deadline is what makes the cadence rigid:
     * a frame that took 2 ms and one that took 40 ms both end up 50 ms apart.
     */
    a->next_deadline += a->period_ms;

    /*
     * ---- overrun guard ----
     *
     * If a frame carried more data than fits in one period, the deadline we
     * just computed is already in the past. Left alone, the accumulator falls
     * further behind on every such frame and then SPRINTS through the sparse
     * ones trying to catch up -- visibly worse than the problem this module
     * exists to solve. So resynchronise to now and count it, which turns a
     * compounding drift into one isolated late frame.
     *
     * frames_late climbing steadily means the clip is too dense for the rate:
     * lower the fps, raise the link rate, or simplify frames. anim2c.py flags
     * this at conversion time so it should not be a surprise at runtime.
     */
    now = HAL_GetTick();
    if ((int32_t)(now - a->next_deadline) >= 0) {
        a->frames_late++;
        a->next_deadline = now + a->period_ms;
    }

    a->index++;
    if (a->index >= a->clip->n_frames) {
        a->index = 0;
        a->loops++;
    }

    return true;
}
