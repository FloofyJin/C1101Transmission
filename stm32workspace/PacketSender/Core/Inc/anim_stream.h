/*
 * anim_stream.h -- receive animation frames over UART instead of baking them
 *                  into flash.
 *
 * WHY
 * ---
 * anim2c.py compiles a clip into `const` tables. That works and costs no RAM,
 * but the F410RB has 128 KB of flash and a frame is 2 bytes per point, so a
 * 160-point clip runs out at roughly 300 frames -- 15 seconds at 20 fps. Bad
 * Apple is 3m39s. The clip has to arrive at runtime.
 *
 * The board has no SD card and no filesystem, but it does have a UART wired to
 * the ST-Link's USB virtual COM port. So the PC becomes the storage: it holds
 * the whole clip and pushes one frame at a time, just ahead of playback.
 *
 * WHAT THIS IS NOT
 * ----------------
 * It is not a replacement for the pacing in animation.c. The 20 fps cadence is
 * still decided on this board by animation_tick()'s deadline accumulator. The
 * PC only has to keep the buffer non-empty; it does NOT set the frame rate.
 * Letting the PC's send timing drive playback would put USB scheduling jitter
 * straight onto the screen.
 *
 * HOW THE BYTES GET IN
 * --------------------
 * DMA, in circular mode, straight into the ring -- no interrupt per byte. That
 * matters more here than it looks: SystemClock_Config() runs the core from HSI
 * with no PLL, so this is a 16 MHz machine. At 230400 baud a per-byte RX
 * interrupt would burn a double-digit percentage of the CPU for nothing, and
 * it would do it during the SPI burst that is feeding the radio.
 *
 * The DMA writes; we read behind it. The write position is derived from the
 * stream's remaining-transfer counter, so there is no shared variable between
 * hardware and software and therefore nothing to make atomic.
 *
 * WIRE FORMAT, PC -> BOARD
 * ------------------------
 *   A5 5A | seq(1) | n_points(2, LE) | x0 y0 x1 y1 ... | crc16(2, LE)
 *          \___________________ CRC covers this ______________/
 *
 *   n_points must be EVEN and <= ANIM_STREAM_MAX_POINTS. Even because points
 *   are span ENDPOINTS -- the same rule anim2c.py enforces at build time.
 *   CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
 *
 *   ZERO IS A VALID LENGTH and means "blank frame, nothing to draw". Such a
 *   frame occupies its period like any other but transmits nothing, so the
 *   display holds. Bad Apple has 44 of them.
 *
 * WIRE FORMAT, BOARD -> PC
 * ------------------------
 * The same UART carries printf() logs, so control messages have to be
 * distinguishable from log text. They start with 0xAA, which printf can never
 * emit -- every byte it produces is ASCII, below 0x80. The PC treats anything
 * that is not a 0xAA triple as text to print.
 *
 *   AA 01 <seq>   flow control: "the last frame I consumed was <seq>"
 *   AA 02 <code>  a frame was dropped; see ANIM_STREAM_ERR_*
 *   AA 03 00      hello -- the board (re)started, reset your sequence numbers
 *
 * FLOW CONTROL, AND WHY IT IS ABSOLUTE
 * ------------------------------------
 * "AA 01 <seq>" reports the last frame CONSUMED, not a count of free slots.
 * The PC computes  in_flight = (seq_sent - seq_consumed) & 0xFF  and sends
 * while that is under ANIM_STREAM_SLOTS.
 *
 * Reporting a credit COUNT would be incremental, so a single dropped byte
 * would desynchronise the window permanently -- the PC would either stall
 * forever or overrun the ring for the rest of the session. An absolute
 * sequence number recovers from a lost CONTROL message on its own: the next
 * one carries the complete truth. The heartbeat below covers the case where
 * the lost message was the last one.
 *
 * IT DOES NOT, BY ITSELF, RECOVER FROM A DROPPED DATA FRAME.
 * A frame this end throws away is never consumed, so last_consumed cannot
 * advance past it, and ANIM_STREAM_SLOTS consecutive drops stall the link for
 * good. Found the hard way on a run of 13 blank frames. Two defences:
 *
 *   - a frame rejected for LENGTH advances the window anyway. Its sequence
 *     number was parsed before the length field, so it is trustworthy even
 *     though the length was not.
 *   - a frame rejected for CRC does NOT, because the sequence byte is inside
 *     the CRC's coverage and may itself be the corrupted one. Advancing to a
 *     bogus sequence number could push the window hundreds of frames out and
 *     wedge the link harder than the drop did. That case is left to the
 *     sender's stall timeout -- see animstream.py.
 *
 * USAGE
 * -----
 *     anim_stream_init(&huart2);
 *     animation_init_source(&anim, anim_stream_next, anim_stream_release,
 *                           NULL, 20);
 *     while (1) {
 *         anim_stream_poll();      // drain DMA ring, parse frames
 *         animation_tick(&anim);   // paced send, pulls from the slots
 *     }
 */

#ifndef INC_ANIM_STREAM_H_
#define INC_ANIM_STREAM_H_

#include "cc1101.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * Points per frame. 180 is MEASURED, not derived: the airtime model in
 * anim2c.py predicts ~428 at 250 kbps / 20 fps, but that model assumes an
 * 8 MHz SPI clock. The real machine runs SPI at 16 MHz / 4 = 4 MHz, so the
 * per-packet SPI cost is about double, and the honest ceiling lands near 180.
 *
 * Raising this costs RAM linearly (2 B per point per slot) and, more
 * importantly, spends airtime the frame period may not have. Check
 * frames_late before you touch it.
 */
#ifndef ANIM_STREAM_MAX_POINTS
#define ANIM_STREAM_MAX_POINTS   180
#endif

/*
 * Decoded frames held ahead of playback. This is the whole reason the
 * animation does not stutter when Windows deschedules the sender for 40 ms.
 *
 * Four is deliberate: it is 200 ms of cushion at 20 fps, which covers ordinary
 * USB and OS scheduling hiccups, while keeping the end-to-end latency low
 * enough that a live-generated clip would still feel responsive. Deeper
 * buffers hide jitter better and make every problem harder to see.
 */
#ifndef ANIM_STREAM_SLOTS
#define ANIM_STREAM_SLOTS        4
#endif

/*
 * DMA landing zone, in bytes. MUST be a power of two -- the read position is
 * advanced with a mask, not a modulo, because this core has no hardware
 * divider worth using in a per-byte loop.
 *
 * 1024 holds about 2.7 frames. It only has to cover the gap between two calls
 * to anim_stream_poll(); the SLOTS above are what absorb real jitter. It is
 * generous on purpose because animation_tick() blocks for the duration of a
 * frame's airtime, and nothing is draining the ring while it does.
 */
#ifndef ANIM_STREAM_RING
#define ANIM_STREAM_RING         1024
#endif

/* How often to repeat the flow-control message even when nothing was
   consumed. Without this, a single lost control message would leave the PC
   waiting for a window update that never comes, and playback would starve. */
#ifndef ANIM_STREAM_HEARTBEAT_MS
#define ANIM_STREAM_HEARTBEAT_MS 250
#endif

/* ---- control message tags (board -> PC), see the header comment ---- */
#define ANIM_STREAM_TAG          0xAA
#define ANIM_STREAM_MSG_WINDOW   0x01
#define ANIM_STREAM_MSG_ERROR    0x02
#define ANIM_STREAM_MSG_HELLO    0x03

/* ---- reasons a frame was thrown away ---- */
#define ANIM_STREAM_ERR_CRC      0x01   /* payload corrupt on the wire           */
#define ANIM_STREAM_ERR_LEN      0x02   /* n_points odd, zero, or too large      */
#define ANIM_STREAM_ERR_FULL     0x03   /* all slots busy; PC ignored the window */

/* ---- frame magic (PC -> board) ---- */
#define ANIM_STREAM_MAGIC0       0xA5
#define ANIM_STREAM_MAGIC1       0x5A

/*
 * ---- session control (PC -> board): A5 5B <kind> ----
 *
 * Shares the first magic byte with a data frame, so the parser's existing
 * hunt state finds it with no extra scanning; only the second byte differs.
 *
 * This exists because the board outlives the sender. Stop animstream.py and
 * restart it and the board still holds the previous session's sequence number,
 * its slots still hold that session's frames, and its parser is almost
 * certainly stranded mid-payload. The restarted sender begins at sequence 0,
 * so (last_sent - last_consumed) comes out around 119 -- far over the window --
 * and it stalls. Before this message existed the only cure was resetting the
 * board.
 */
#define ANIM_STREAM_CTL1         0x5B
#define ANIM_STREAM_CTL_RESET    0x01

/*
 * Counters. All monotonic, all safe to read at any time. Read them once a
 * second from the status print -- the ratios are what diagnose the link:
 *
 *   crc_errors climbing      -> baud rate too high, or a bad cable
 *   len_errors climbing      -> the PC and this board disagree on MAX_POINTS
 *   overflows  climbing      -> the PC is ignoring the flow-control window
 *   resyncs    climbing      -> bytes are being lost mid-frame
 */
typedef struct {
    uint32_t frames_ok;      /* complete frames parsed and stored             */
    uint32_t crc_errors;
    uint32_t len_errors;
    uint32_t overflows;      /* frame arrived with no free slot; dropped      */
    uint32_t resyncs;        /* magic byte not where it was expected          */
    uint32_t bytes;          /* total bytes pulled out of the DMA ring        */
    uint8_t  last_seq;       /* sequence number of the most recent good frame */
    uint8_t  depth;          /* slots currently filled, 0..ANIM_STREAM_SLOTS  */
} anim_stream_stats_t;

/*
 * Start the DMA receiver and announce ourselves to the PC. Safe to call once,
 * after MX_USART2_UART_Init(). The handle is used only for TRANSMITTING
 * control messages -- receive bypasses the HAL entirely and talks to the DMA
 * controller directly, so HAL's UART state machine is never in the RX path.
 */
void anim_stream_init(UART_HandleTypeDef *huart);

/*
 * Move bytes from the DMA ring into decoded frame slots. Call as often as you
 * can from the main loop; it is bounded work and never blocks.
 *
 * It parses at most one ring's worth of bytes per call, so it cannot spin
 * forever if the PC floods the link.
 */
void anim_stream_poll(void);

/*
 * Frame source for animation.c. Returns the oldest decoded frame, or NULL when
 * none is ready -- which animation_tick() treats as "hold the current picture"
 * rather than an error, because that is what the display does anyway.
 *
 * The pointer stays valid until anim_stream_release().
 */
const cc1101_point_t *anim_stream_next(void *ctx, uint16_t *n_points);

/* Give back the frame returned by anim_stream_next() and tell the PC there is
   room for another. Call exactly once per successful next(). */
void anim_stream_release(void *ctx);

/* Snapshot of the counters above. */
const anim_stream_stats_t *anim_stream_get_stats(void);

#endif /* INC_ANIM_STREAM_H_ */
