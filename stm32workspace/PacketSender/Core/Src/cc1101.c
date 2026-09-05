/*
 * cc1101.c
 *
 *  Created on: Aug 7, 2026
 *      Author: Nymph
 */

/*
 * cc1101.c -- see cc1101.h
 *
 * Transmit mission, mirroring TxSeq.sv:
 *
 *   SIDLE -> SFTX -> burst-write FIFO (len + payload) -> read TXBYTES
 *         -> STX -> read MARCSTATE -> wait GDO2 rise -> wait GDO2 fall -> SIDLE
 *
 * The two register reads are pure diagnostics and are the point of the module.
 */
#include "cc1101.h"
#include <string.h>

/* Chip constants (strobes, status addresses, FIFO headers, MARCSTATE values)
   live in cc1101.h -- they describe the chip, not this driver. */

static SPI_HandleTypeDef *spi;
static cc1101_tx_diag_t   diag;
static uint8_t            seq;      /* for cc1101_send_test_packet */

/*
 * Ported verbatim from ConfigSeq.sv's CFG_FLAT (via RFreceiver/cc1101.c).
 * Changing any of these without changing the Zybo side breaks the link
 * silently -- both radios still transmit, they just stop understanding each
 * other.
 *
 * 433.92 MHz GFSK at 250 kbps, 26 MHz crystal.
 *
 * ---- history, because it explains the register choices ----
 * This began as a 38.4 kbps GFSK baseline. Jumping straight to 500 kbps MSK
 * FAILED on hardware -- most packets were lost and the display teleported. The
 * cause was not the rate (500 kBaud is a supported, TI-characterised point) but
 * that the rate moved while DEVIATN, the AGC block and the preamble did not.
 *
 * 250 kbps GFSK was chosen instead because TI publishes a COMPLETE
 * characterisation for it at 433 MHz: -95 dBm, 127 kHz deviation, 540 kHz
 * channel filter. The datasheet revision history even records TI correcting
 * the 250 kBaud reference from MSK to GFSK -- this is their intended point.
 *
 * Five registers are therefore matched to published figures, not guessed, and
 * MUST match ConfigSeq.sv exactly:
 *
 *   MDMCFG2 0x13  GFSK, unchanged -- the modulation that already worked
 *   MDMCFG4 0x2D  DRATE_E=13, channel filter 541.7 kHz (TI's 540 kHz)
 *   MDMCFG3 0x3B  DRATE_M=59  -> 249,939 bps
 *   DEVIATN 0x62  +/-127.0 kHz -> exactly TI's published deviation
 *   MDMCFG1 0x62  16 preamble bytes -> 512 us of AGC settling
 *
 * ---- still NOT verified at this rate ----
 * FSCTRL1 is interpolated, not published. FOCCFG, BSCFG, AGCCTRL2/1/0, FREND1
 * and TEST2/TEST1 are empirical AGC and loop-filter values still carrying their
 * 38.4 kbps settings. Get them from SmartRF Studio -- do not hand-derive them.
 *
 * The bench link tolerates that because two radios 10 cm apart sit tens of dB
 * above the sensitivity limit, which hides a mistuned AGC completely.
 */
static const uint8_t cc_cfg[][2] = {
    {0x00, 0x06},  /* IOCFG2   GDO2: asserts on sync, deasserts at end of packet */
    {0x02, 0x2E},  /* IOCFG0   3-state (GDO0 unused on these modules)            */
    {0x03, 0x47},  /* FIFOTHR                                                    */
    {0x04, 0xD3},  /* SYNC1  ) sync word 0xD391 -- must match                    */
    {0x05, 0x91},  /* SYNC0  )                                                   */
    {0x06, 0xFF},  /* PKTLEN                                                     */
    {0x07, 0x04},  /* PKTCTRL1 append RSSI/LQI/CRC_OK                            */
    {0x08, 0x05},  /* PKTCTRL0 variable length + CRC                             */
    {0x0B, 0x0C},  /* FSCTRL1  IF 304.7 kHz. NOT a datasheet figure -- TI gives
                      only 152 kHz @ 38.4k and 355 kHz @ 500k, so this is
                      interpolated. First thing to check against SmartRF.       */
    {0x0C, 0x00},  /* FSCTRL0                                                    */
    {0x0D, 0x10},  /* FREQ2  ) 433.92 MHz @ 26 MHz xtal -- BAND SPECIFIC         */
    {0x0E, 0xB0},  /* FREQ1  )                                                   */
    {0x0F, 0x71},  /* FREQ0  )                                                   */
    {0x10, 0x2D},  /* MDMCFG4) 249,939 bps, 541.7 kHz channel BW
                      (CHANBW_E=0, CHANBW_M=2, DRATE_E=13)                      */
    {0x11, 0x3B},  /* MDMCFG3) DRATE_M=59; the exponent lives in MDMCFG4[3:0]    */
    {0x12, 0x13},  /* MDMCFG2  GFSK, 30/32 sync-word detection                   */
    {0x13, 0x62},  /* MDMCFG1  16 preamble bytes = 512 us of AGC settling, no
                      FEC. Was 4 bytes: fine at 38.4k (833 us), only 128 us at
                      250k, which the earlier note called marginal.             */
    {0x14, 0xF8},  /* MDMCFG0                                                    */
    {0x15, 0x62},  /* DEVIATN  +/-127.0 kHz, exactly TI's published 250 kBaud
                      GFSK figure (DEVIATION_E=6, DEVIATION_M=2). Was 0x35 =
                      20.6 kHz, correct only for 38.4 kbps.                     */
    {0x18, 0x18},  /* MCSM0    FS_AUTOCAL on IDLE->RX/TX -- do not omit          */
    {0x19, 0x16},  /* FOCCFG                                                     */
    {0x1A, 0x6C},  /* BSCFG                                                      */
    {0x1B, 0x43},  /* AGCCTRL2                                                   */
    {0x1C, 0x40},  /* AGCCTRL1                                                   */
    {0x1D, 0x91},  /* AGCCTRL0                                                   */
    {0x20, 0xFB},  /* WORCTRL                                                    */
    {0x21, 0x56},  /* FREND1                                                     */
    {0x22, 0x10},  /* FREND0                                                     */
    {0x23, 0xE9},  /* FSCAL3                                                     */
    {0x24, 0x2A},  /* FSCAL2                                                     */
    {0x25, 0x00},  /* FSCAL1                                                     */
    {0x26, 0x1F},  /* FSCAL0                                                     */
    {0x29, 0x59},  /* FSTEST                                                     */
    {0x2C, 0x81},  /* TEST2                                                      */
    {0x2D, 0x35},  /* TEST1                                                      */
    {0x2E, 0x09},  /* TEST0                                                      */
};

/* ------------------------------------------------------------------ */

static inline void cs_low (void) { HAL_GPIO_WritePin(CC_CS_PORT, CC_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(CC_CS_PORT, CC_CS_PIN, GPIO_PIN_SET);   }

static uint8_t xfer(uint8_t out)
{
    uint8_t in = 0;
    HAL_SPI_TransmitReceive(spi, &out, &in, 1, HAL_MAX_DELAY);
    return in;
}

/*
 * Wait for CHIP_RDYn (status byte bit 7) to go low after CSn falls.
 *
 * The chip drives it onto SO the instant CSn drops -- before any clock edge --
 * so this is just reading the MISO pin. IDR reflects the pad state even while
 * the pin is in alternate-function mode, which is what makes this work without
 * reconfiguring the pin. This replaces a guessed delay with the chip actually
 * telling us its crystal is running.
 */
static void wait_ready(void)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(CC_MISO_PORT, CC_MISO_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 10) break;   /* don't wedge on a dead chip */
    }
}

static void cc_write_reg(uint8_t addr, uint8_t val)
{
    cs_low();
    wait_ready();
    xfer(CC_WRITE | addr);
    xfer(val);
    cs_high();
}

static uint8_t cc_read_reg(uint8_t addr)
{
    uint8_t v;
    cs_low();
    wait_ready();
    xfer(CC_READ | addr);       /* returns the status byte -- discarded */
    v = xfer(0x00);             /* dummy byte clocks the register out   */
    cs_high();
    return v;
}

uint8_t cc1101_read_status(uint8_t addr)
{
    uint8_t v;
    cs_low();
    wait_ready();
    xfer(CC_READ | CC_BURST | addr);
    v = xfer(0x00);
    cs_high();
    return v;
}

/* A strobe only executes when CSn RISES, so it must not be held low. */
static void cc_strobe(uint8_t cmd)
{
    cs_low();
    wait_ready();
    xfer(cmd);
    cs_high();
}

/*
 * Burst-write the TX FIFO: header, then the length byte, then the payload, all
 * inside ONE CSn-low window. Lifting CSn between the header and the data is
 * the classic off-by-one-byte failure -- it shows up as the chip's status byte
 * arriving where register data should be.
 */
static void cc_write_fifo(const uint8_t *payload, uint8_t len)
{
    cs_low();
    wait_ready();
    xfer(TXFIFO_BURST);
    xfer(len);                                  /* variable-length mode */
    for (uint8_t i = 0; i < len; i++)
        xfer(payload[i]);
    cs_high();
}

/* ------------------------------------------------------------------ */

bool cc1101_init(SPI_HandleTypeDef *hspi)
{
    bool ok = true;

    spi = hspi;
    seq = 0;
    memset(&diag, 0, sizeof(diag));

    /* Manual power-on reset ritual (datasheet section 19.1). */
    cs_high(); HAL_Delay(1);
    cs_low();  HAL_Delay(1);
    cs_high(); HAL_Delay(1);

    cc_strobe(SRES);
    HAL_Delay(5);

    for (unsigned i = 0; i < sizeof(cc_cfg)/sizeof(cc_cfg[0]); i++)
        cc_write_reg(cc_cfg[i][0], cc_cfg[i][1]);

    cc_write_reg(PATABLE_ADDR, CC_PATABLE_VALUE);

    /*
     * Read-back verify. Proves the writes STUCK (SPI correctness), NOT that the
     * values are correct for RF -- a wrong-but-valid value passes this and the
     * link still won't work. PATABLE is skipped: it is not a plain register.
     */
    for (unsigned i = 0; i < sizeof(cc_cfg)/sizeof(cc_cfg[0]); i++) {
        if (cc_read_reg(cc_cfg[i][0]) != cc_cfg[i][1])
            ok = false;
    }

    cc_strobe(SIDLE);
    cc_strobe(SFTX);        /* only valid in IDLE or TXFIFO_UNDERFLOW */
    return ok;
}

void cc1101_selftest(uint8_t *partnum, uint8_t *version)
{
    if (partnum) *partnum = cc1101_read_status(PARTNUM_ADDR);
    if (version) *version = cc1101_read_status(VERSION_ADDR);
}

const cc1101_tx_diag_t *cc1101_last_diag(void)
{
    return &diag;
}

/*
 * Wait for the chip's state machine to actually reach IDLE.
 *
 * SFTX is only accepted in IDLE or TXFIFO_UNDERFLOW, and strobing SIDLE does
 * not mean the chip is there yet -- it is still winding down TX. Flush too
 * early and it is silently ignored, so the next payload lands on top of
 * whatever the last packet left behind. Only reachable now that send_frame
 * transmits its packets back to back; one packet every 200 ms never hit it.
 */
static bool wait_idle(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        if ((cc1101_read_status(MARCSTATE_ADDR) & 0x1F) == MARC_IDLE) return true;
        if (HAL_GetTick() - t0 > ms) return false;
    }
}

/*
 * The transmit mission. Blocking: returns once the packet is on the air or the
 * GDO2 wait gave up.
 */
static bool tx_raw(const uint8_t *payload, uint8_t len)
{
    if (len == 0 || len > CC_PAYLOAD_MAX) return false;

    memset(&diag, 0, sizeof(diag));

    cc_strobe(SIDLE);
    wait_idle(10);              /* SFTX is a no-op unless we are IDLE */
    cc_strobe(SFTX);            /* a leftover byte corrupts this packet */

    cc_write_fifo(payload, len);

    /* Did the bytes actually reach the FIFO? Expect 1 + len. */
    diag.txbytes = cc1101_read_status(TXBYTES_ADDR);

    cc_strobe(STX);

    /* Did the chip actually leave IDLE? Expect 0x13 (TX). */
    diag.marcstate = cc1101_read_status(MARCSTATE_ADDR);

    /*
     * GDO2 with IOCFG2 = 0x06 rises when the sync word has gone out and falls
     * at the end of the packet, so rise-then-fall is the chip telling you the
     * whole frame was transmitted.
     *
     * The rise can be MISSED, and at the current rate it usually will be.
     * Sync goes out after preamble + sync ~= 6 bytes, which was 1.25 ms at
     * 38.4 kbps but is only ~96 us at 500 kbps -- roughly 13x shorter, and
     * far too brief to catch reliably between two GPIO polls.
     *
     * A missed rise is NOT a failed transmit, so fall back to TXBYTES: if the
     * FIFO drained, the packet went out and we simply did not see the pulse.
     * This fallback is what stops the link from appearing to "fail" at 500
     * kbps purely because it got faster. Trust `sent_ok || drained`, never
     * `sync_seen` alone.
     */
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(CC_GDO2_PORT, CC_GDO2_PIN) == GPIO_PIN_RESET) {
        if ((cc1101_read_status(TXBYTES_ADDR) & 0x7F) == 0) {
            diag.drained = true;
            break;
        }
        if (HAL_GetTick() - t0 > CC_TX_TIMEOUT_MS) {
            diag.timed_out = true;
            cc_strobe(SIDLE);
            cc_strobe(SFTX);
            return false;
        }
    }

    if (!diag.drained) {
        diag.sync_seen = true;

        /* Wait for end of packet. */
        t0 = HAL_GetTick();
        while (HAL_GPIO_ReadPin(CC_GDO2_PORT, CC_GDO2_PIN) == GPIO_PIN_SET) {
            if (HAL_GetTick() - t0 > CC_TX_TIMEOUT_MS) {
                diag.timed_out = true;
                cc_strobe(SIDLE);
                cc_strobe(SFTX);
                return false;
            }
        }
        diag.sent_ok = true;
    }

    cc_strobe(SIDLE);
    return true;
}

bool cc1101_send_points(uint16_t start_index, const cc1101_point_t *pts,
                        uint8_t count, bool end_of_frame)
{
    uint8_t payload[CC_PKT_HDR + 2 * CC_MAX_POINTS];

    if (pts == NULL || count == 0 || count > CC_MAX_POINTS) return false;

    /* Must be even: points are span endpoints, and splitting a span across a
       packet boundary means a lost packet leaves a dangling endpoint, which
       draws a line clean across the image. */
    if (count & 1u) return false;

    /* Never address past the end of the receiver's PointRam. */
    if ((uint32_t)start_index + count > CC_FRAME_POINTS) return false;

    payload[0] = (uint8_t)(start_index >> 8);      /* big-endian, matching RxSeq */
    payload[1] = (uint8_t)(start_index & 0xFF);
    payload[2] = (uint8_t)(count | (end_of_frame ? CC_EOF_FLAG : 0));

    for (uint8_t i = 0; i < count; i++) {
        payload[CC_PKT_HDR + 2*i    ] = pts[i].x;
        payload[CC_PKT_HDR + 2*i + 1] = pts[i].y;
    }

    return tx_raw(payload, (uint8_t)(CC_PKT_HDR + 2 * count));
}

bool cc1101_send_frame(const cc1101_point_t *pts, uint16_t n_points)
{
    bool all_ok = true;

    if (pts == NULL || n_points == 0)        return false;
    if (n_points & 1u)                       return false;   /* spans are pairs */
    if (n_points > CC_FRAME_POINTS)          return false;

    for (uint16_t i = 0; i < n_points; i += CC_MAX_POINTS) {
        uint16_t left  = (uint16_t)(n_points - i);
        uint8_t  chunk = (uint8_t)((left > CC_MAX_POINTS) ? CC_MAX_POINTS : left);
        bool     last  = (uint16_t)(i + chunk) >= n_points;

        /* Keep going after a failure. The format is self-healing: a dropped
           packet leaves a few stale coordinates that the next pass repairs,
           so abandoning the rest of the frame only makes the gap bigger.
           The exception is the LAST packet -- if that one is lost the display
           simply does not swap, and the previous frame stays up one period. */
        if (!cc1101_send_points(i, &pts[i], chunk, last))
            all_ok = false;
    }
    return all_ok;
}
