/*
 * cc1101.h
 *
 *  Created on: Aug 7, 2026
 *      Author: Nymph
 */

/*
 * cc1101.h -- CC1101 TRANSMITTER for the DD_RF vector-display link
 *
 * This is the Phase-2 direction reversal: the STM32 transmits and the Zybo PL
 * receives, renders into PointRam, and scans it out to an MCP4922 DAC.
 * Mirrors TxSeq.sv step for step, because it is the same chip solving the same
 * problem:
 *
 *   SIDLE -> SFTX -> burst-write FIFO (len + payload) -> read TXBYTES
 *         -> STX -> read MARCSTATE -> wait GDO2 rise -> wait GDO2 fall -> SIDLE
 *
 * The register table in cc1101.c MUST match the Zybo's ConfigSeq.sv ROM exactly.
 * Frequency, data rate, deviation, sync word and packet control all have to
 * agree or the radios sit next to each other hearing nothing. Read-back verify
 * does NOT catch a mismatch -- it proves each chip stored what it was sent, not
 * that the two chips agree.
 *
 * ---------------------------------------------------------------------------
 * TWO PAYLOAD FORMATS, and why both exist
 * ---------------------------------------------------------------------------
 *
 * 1. cc1101_send_test_packet()  ->  { 0xAA, 0x55, seq }
 *
 *    Byte-identical to what TxSeq.sv transmits today. The Zybo's *existing,
 *    unmodified* bitstream will light LD3 on this. Use it for M13: it proves
 *    the STM32 transmit path and the reversed link direction against a
 *    known-good receiver, with zero FPGA changes in flight.
 *
 * 2. cc1101_send_points() / cc1101_send_frame()  ->  point packets
 *
 *    The real Phase-2 format. RxSeq.sv currently hardcodes EXP_LEN=3,
 *    {0xAA,0x55} and clamps at RX_MAX=16, so it will NOT parse these until the
 *    M14 RTL work lands. Expect silence, not a wrong answer.
 *
 * Get (1) working first. Debugging a new transmitter and a new receiver
 * simultaneously is how these projects stall.
 *
 * ---------------------------------------------------------------------------
 * POINT PACKET FORMAT
 * ---------------------------------------------------------------------------
 *
 *   payload = 59 bytes, carried in ONE radio packet
 *   +---------------+-------+----+----+----+----+-----+
 *   |  start_index  | count | x0 | y0 | x1 | y1 | ... |
 *   |      2 B      |  1 B  |       2 B per point     |
 *   +---------------+-------+----+----+----+----+-----+
 *
 * 28 points per packet means 28 XY PAIRS, not 28 bytes. The chip adds preamble,
 * sync, length and CRC itself -- never write those.
 *
 * Why 28 and not 30: the RECEIVE FIFO binds, not the transmit one. See the
 * CC_MAX_POINTS block below.
 *
 * count[7] marks the LAST packet of a frame, which tells the receiver to swap
 * display buffers. Without it the display would show a frame that is part old
 * and part new, with a tear line sweeping down the image.
 *
 * Points are SPAN ENDPOINTS -- scanline fill sends (xL,y) then (xR,y) for each
 * horizontal run of the shape, so n_points is always even.
 *
 * Two properties worth protecting:
 *   - Idempotent.   Sending the same packet twice changes nothing.
 *   - Self-healing. A lost packet leaves a few stale coordinates; the next pass
 *                   over that index range repairs them. No ACKs, no retries.
 *
 * Blanking is NOT transmitted. x and y fill both payload bytes exactly, and the
 * FPGA derives blanking from the scanline structure -- it needs both endpoints
 * of a segment, which the receiver never has at once anyway.
 */
#ifndef INC_CC1101_H_
#define INC_CC1101_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- pin mapping: must match spi_hall.c's HAL_SPI_MspInit ---- */
#define CC_CS_PORT    GPIOB
#define CC_CS_PIN     GPIO_PIN_6      /* D10 */
#define CC_GDO2_PORT  GPIOB
#define CC_GDO2_PIN   GPIO_PIN_8      /* D7  */
#define CC_MISO_PORT  GPIOA
#define CC_MISO_PIN   GPIO_PIN_6      /* read as GPIO for the CHIP_RDYn wait */

/* ===================================================================
 * CC1101 chip constants -- the C counterpart of the Zybo's cc1101_pkg.sv.
 * Same names, same values. These describe the CHIP, not this driver, so
 * they live here rather than in cc1101.c.
 * =================================================================== */

/* ---- SPI header byte fields ----
 *   bit 7   R/W    0 = write, 1 = read
 *   bit 6   burst  0 = single byte, 1 = keep going
 *   bits5-0 address
 */
#define CC_WRITE        0x00
#define CC_READ         0x80
#define CC_BURST        0x40

/* ---- command strobes: single-byte fire-and-forget commands ---- */
#define SRES            0x30    /* reset          */
#define SRX             0x34    /* enter RX       */
#define STX             0x35    /* enter TX       */
#define SIDLE           0x36    /* go idle        */
#define SFRX            0x3A    /* flush RX FIFO  */
#define SFTX            0x3B    /* flush TX FIFO  */
#define SNOP            0x3D    /* no-op (just fetch the status byte) */

/* ---- status register addresses ----
 * These COLLIDE with the strobes above on purpose: same address, and the burst
 * bit is what makes the access a read instead of an action. 0x3B is SFTX as a
 * strobe and RXBYTES as a status read; 0x3A is SFRX and TXBYTES. Reach them
 * only through cc1101_read_status(), which sets CC_BURST for you -- issue one
 * as a plain read and you flush a FIFO you meant to inspect.
 */
#define PARTNUM_ADDR    0x30
#define VERSION_ADDR    0x31
#define MARCSTATE_ADDR  0x35    /* chip's own state machine -- best TX/RX debug */
#define TXBYTES_ADDR    0x3A
#define RXBYTES_ADDR    0x3B

/* ---- FIFO burst headers ---- */
#define TXFIFO_BURST    0x7F    /* R/W=0, burst=1, addr=0x3F */
#define RXFIFO_BURST    0xFF    /* R/W=1, burst=1, addr=0x3F */

#define PATABLE_ADDR    0x3E

/* ---- MARCSTATE values worth recognising (bits [4:0]) ---- */
#define MARC_IDLE       0x01
#define MARC_RX         0x0D
#define MARC_TX         0x13

/* ---- tunables ---- */

/*
 * TX output power. 0x12 is roughly -30 dBm, the minimum.
 *
 * On the RFreceiver this register was inert -- PATABLE is transmit power only.
 * Here it is real. Two radios 10 cm apart at full power overload the receiver's
 * front end, which is the single most common reason a two-CC1101 bench setup
 * mysteriously fails. Turn it up only once the boards are metres apart.
 */
#ifndef CC_PATABLE_VALUE
#define CC_PATABLE_VALUE  0x12
#endif

/* Give up waiting on GDO2. A missing GDO2 must not wedge the sender. */
#ifndef CC_TX_TIMEOUT_MS
#define CC_TX_TIMEOUT_MS  200
#endif

/* ---- packet geometry ---- */
/*
 * The RX FIFO is the binding limit, NOT the TX FIFO. Both are 64 bytes, but the
 * receiving chip appends two status bytes (RSSI, LQI|CRC_OK) into its own FIFO
 * -- datasheet 15.3.3 -- so:
 *
 *     TX:  1 len + payload            <= 64  ->  payload <= 63
 *     RX:  1 len + payload + 2 status <= 64  ->  payload <= 61   <- binds
 *
 * And the point count must be EVEN, because scanline fill sends points in
 * PAIRS. An odd count splits a span across a packet boundary; lose that packet
 * and one endpoint comes from the new frame while its partner is two frames
 * stale, which draws a line straight across the image. An even count means a
 * lost packet costs a whole span -- one missing row, nearly invisible.
 */
#define CC_PAYLOAD_MAX   61     /* RX FIFO 64 - 1 len - 2 status             */
#define CC_PKT_HDR       3      /* start_index (2 B) + count (1 B)           */
#define CC_MAX_POINTS    28     /* EVEN, and <= (61 - 3) / 2                 */
#define CC_FRAME_POINTS  1024   /* PointRam depth per bank on the Zybo side  */

/* count byte: bits[6:0] = point count, bit 7 = END OF FRAME.
 * The count never exceeds 28, so bit 7 is free. The last packet of a frame
 * sets it; the receiver swaps display buffers at its next scanout wrap.
 * If that packet is lost, no swap happens and the previous frame stays up one
 * extra period -- a far better failure than showing half a frame. */
#define CC_EOF_FLAG      0x80

typedef struct {
    uint8_t x;
    uint8_t y;
} cc1101_point_t;

/*
 * Diagnostics from the last transmit. Blind transmit is unfixable when it
 * fails -- you need to know WHICH step broke. Kept in a struct rather than
 * printf'd inline: __io_putchar blocks on the UART, and printing from inside
 * the transmit path would stall the CPU for milliseconds at a time.
 */
typedef struct {
    uint8_t txbytes;    /* TXBYTES after the FIFO write. Expect 1 + payload_len.
                           0x00     -> the burst write never landed
                           > 1+len  -> the FIFO was not empty before the write
                           bit 7    -> TX FIFO underflow                      */
    uint8_t marcstate;  /* MARCSTATE after STX. Expect 0x13 (TX).
                           0x01     -> STX did not take, chip never left IDLE
                           0x08-0x0C-> caught mid-calibration, fine           */
    bool    sync_seen;  /* GDO2 rose  -> sync word went on air                */
    bool    sent_ok;    /* rose AND fell -> whole packet was transmitted      */
    bool    timed_out;  /* GDO2 never moved                                   */
    bool    drained;    /* rise was missed but TXBYTES hit 0 -- see cc1101.c  */
} cc1101_tx_diag_t;

/* ------------------------------------------------------------------ */

/* Reset, configure, verify by read-back, park in IDLE. Must be called before
   ANY other function here -- it is what hands the driver the SPI handle.
   Returns false if a register read back wrong. */
bool cc1101_init(SPI_HandleTypeDef *hspi);

/* Chip liveness. PARTNUM must read 0x00 (that IS the correct value); VERSION
   is the real check -- non-zero with mixed 1s and 0s proves MISO can drive
   both rails. Call it AFTER cc1101_init, never before. */
void cc1101_selftest(uint8_t *partnum, uint8_t *version);

/* M13: { 0xAA, 0x55, seq }. Byte-identical to TxSeq.sv, so the Zybo's existing
   bitstream receives it unmodified. seq increments per ATTEMPT, so a packet
   lost on air still consumes a number and the receiver can see the gap. */
bool cc1101_send_test_packet(void);

/* Phase 2: write `count` points into the receiver's PointRam starting at
   `start_index`. count must be 1..CC_MAX_POINTS. */
bool cc1101_send_points(uint16_t start_index,
                        const cc1101_point_t *pts,
                        uint8_t count,
                        bool end_of_frame);

/* Split n_points into ceil(n/28) packets and send them all, marking the last
   one end-of-frame so the display swaps buffers. Returns false if any packet
   failed; the rest are still attempted, because a partial frame is repaired by
   the next pass. n_points must be EVEN -- points are span endpoints. */
bool cc1101_send_frame(const cc1101_point_t *pts, uint16_t n_points);

/* Diagnostics from the most recent transmit. */
const cc1101_tx_diag_t *cc1101_last_diag(void);

/* Burst-bit status register read (0x30-0x3D). Exposed for bring-up. */
uint8_t cc1101_read_status(uint8_t addr);

#endif /* INC_CC1101_H_ */
