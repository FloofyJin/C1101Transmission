/*
 * tools/hosttest/cc1101.h -- STUB, for the PC build of streamtest.c only.
 *
 * anim_stream.h and animation.h both include "cc1101.h", which on the target
 * pulls in main.h and the whole STM32 HAL. Compiling that on a PC is not
 * possible and not the point: the parser and the pacing are plain C and are
 * exactly the parts worth testing before touching hardware.
 *
 * A -I path is NOT enough to shadow it: for a quoted #include, GCC searches
 * the directory of the INCLUDING file first, so Core/Inc/anim_stream.h would
 * always find Core/Inc/cc1101.h next to itself. Instead this file is force
 * included with -include and claims the real header's guard macro, so the
 * later #include "cc1101.h" resolves to a file that is already "included" and
 * expands to nothing.
 *
 * It must stay in sync with the real cc1101.h for the handful of things used
 * here -- cc1101_point_t above all, because the parser writes x then y into
 * it by byte offset.
 */
#ifndef INC_CC1101_H_
#define INC_CC1101_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint8_t x;
    uint8_t y;
} cc1101_point_t;

/* Opaque on the host -- anim_stream_init() is compiled out of this build. */
typedef struct UART_HandleTypeDef_stub UART_HandleTypeDef;

#define CC_MAX_POINTS   28
#define CC_FRAME_POINTS 1024

/* Provided by streamtest.c. */
uint32_t HAL_GetTick(void);
bool     cc1101_send_frame(const cc1101_point_t *pts, uint16_t n_points);

#endif /* INC_CC1101_H_ */
