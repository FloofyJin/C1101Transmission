/*
 * cc1101.c
 *
 *  Created on: Aug 3, 2026
 *      Author: Nymph
 */

/*
 * cc1101.c -- see cc1101.h
 *
 * Mirrors the receive sequence in the Zybo's RxSeq.sv, because it is the same
 * chip solving the same problem:
 *
 *   SRES -> write config -> SFRX -> SRX -> wait GDO2 rise -> wait GDO2 fall
 *        -> read RXBYTES (twice, must agree) -> burst-read FIFO -> check CRC
 *        -> SFRX -> re-arm
 */
#include "cc1101.h"
#include <string.h>

/* ---- SPI header byte fields ---- */
#define CC_WRITE        0x00
#define CC_READ         0x80
#define CC_BURST        0x40

/* ---- command strobes ---- */
#define SRES            0x30
#define SRX             0x34
#define SIDLE           0x36
#define SFRX            0x3A

/* ---- status registers (need the burst bit to reach) ---- */
#define RXBYTES_ADDR    0x3B
#define MARCSTATE_ADDR  0x35
#define PARTNUM_ADDR    0x30
#define VERSION_ADDR    0x31

#define RXFIFO_BURST    0xFF
#define PATABLE_ADDR    0x3E

#define RX_MAX          16      /* clamp: never clock out more than this */

static SPI_HandleTypeDef *spi;

/*
 * Ported verbatim from ConfigSeq.sv's CFG_FLAT. Changing any of these without
 * changing the Zybo side breaks the link silently -- the radios still transmit,
 * they just stop understanding each other.
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
    {0x0B, 0x06},  /* FSCTRL1                                                    */
    {0x0C, 0x00},  /* FSCTRL0                                                    */
    {0x0D, 0x10},  /* FREQ2  ) 433.92 MHz @ 26 MHz xtal -- BAND SPECIFIC         */
    {0x0E, 0xB0},  /* FREQ1  )                                                   */
    {0x0F, 0x71},  /* FREQ0  )                                                   */
    {0x10, 0xCA},  /* MDMCFG4) 38.4 kbps, 101.6 kHz channel BW                   */
    {0x11, 0x83},  /* MDMCFG3)                                                   */
    {0x12, 0x13},  /* MDMCFG2  GFSK, 30/32 sync-word detection                   */
    {0x13, 0x22},  /* MDMCFG1  4-byte preamble, no FEC                           */
    {0x14, 0xF8},  /* MDMCFG0                                                    */
    {0x15, 0x35},  /* DEVIATN  +/-20.6 kHz                                       */
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

/*
 * Status registers 0x30-0x3D live at the same addresses as the command strobes.
 * The burst bit is what makes the access a read instead of an action.
 */
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

static void cc_read_burst(uint8_t hdr, uint8_t *buf, uint8_t n)
{
    cs_low();
    wait_ready();
    xfer(hdr);                          /* status byte -- discarded */
    for (uint8_t i = 0; i < n; i++)
        buf[i] = xfer(0x00);
    cs_high();
}

/* ------------------------------------------------------------------ */

void cc1101_init(SPI_HandleTypeDef *hspi)
{
    spi = hspi;

    /* Manual power-on reset ritual (datasheet section 19.1). */
    cs_high(); HAL_Delay(1);
    cs_low();  HAL_Delay(1);
    cs_high(); HAL_Delay(1);

    cc_strobe(SRES);
    HAL_Delay(5);

    for (unsigned i = 0; i < sizeof(cc_cfg)/sizeof(cc_cfg[0]); i++)
        cc_write_reg(cc_cfg[i][0], cc_cfg[i][1]);

    /* PATABLE: RX-only here, so the value barely matters -- kept equal to the
       Zybo's for symmetry. Note the Zybo transmits at ~-30 dBm. */
    cc_write_reg(PATABLE_ADDR, 0x12);

    cc_strobe(SIDLE);
    cc_strobe(SFRX);        /* only valid in IDLE or RXFIFO_OVERFLOW */
    cc_strobe(SRX);
}

/*
 * Non-blocking. Call repeatedly from the main loop; returns true once a full
 * packet has been pulled out of the FIFO.
 */
bool cc1101_rx_poll(cc1101_pkt_t *pkt)
{
    if (HAL_GPIO_ReadPin(CC_GDO2_PORT, CC_GDO2_PIN) == GPIO_PIN_RESET)
        return false;                       /* no sync word yet */

    /* GDO2 high = sync detected. It falls at end of packet. */
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(CC_GDO2_PORT, CC_GDO2_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 200) {     /* stuck high -> give up */
            cc_strobe(SIDLE); cc_strobe(SFRX); cc_strobe(SRX);
            return false;
        }
    }

    /*
     * Read RXBYTES until two consecutive reads agree. A status read caught
     * mid-update returns garbage, and this value decides how many bytes we
     * clock out -- get it wrong and the whole burst desynchronises.
     */
    uint8_t a = cc1101_read_status(RXBYTES_ADDR);
    uint8_t b = cc1101_read_status(RXBYTES_ADDR);
    for (int i = 0; i < 8 && a != b; i++) { a = b; b = cc1101_read_status(RXBYTES_ADDR); }

    if (b & 0x80) {                          /* RXFIFO_OVERFLOW */
        cc_strobe(SIDLE); cc_strobe(SFRX); cc_strobe(SRX);
        return false;
    }

    uint8_t n = b & 0x7F;
    if (n == 0 || n > RX_MAX) {
        cc_strobe(SIDLE); cc_strobe(SFRX); cc_strobe(SRX);
        return false;
    }

    uint8_t buf[RX_MAX];
    cc_read_burst(RXFIFO_BURST, buf, n);

    /* FIFO layout: [len][payload...][RSSI][LQI | CRC_OK<<7] */
    memset(pkt, 0, sizeof(*pkt));
    pkt->len      = buf[0];
    pkt->rssi_raw = buf[n - 2];
    pkt->lqi      = buf[n - 1] & 0x7F;
    pkt->crc_ok   = (buf[n - 1] & 0x80) != 0;

    uint8_t payload_n = n - 3;              /* minus len byte and 2 status */
    if (payload_n > sizeof(pkt->payload)) payload_n = sizeof(pkt->payload);
    for (uint8_t i = 0; i < payload_n; i++) pkt->payload[i] = buf[1 + i];

    /* Always flush before going back to RX -- a leftover partial packet
       corrupts the next good one. */
    cc_strobe(SIDLE);
    cc_strobe(SFRX);
    cc_strobe(SRX);
    return true;
}

/* Datasheet section 17.3: 2's complement, 1/2 dB steps, 74 dB offset at 433 MHz. */
int cc1101_rssi_dbm(uint8_t rssi_raw)
{
    int d = rssi_raw;
    if (d >= 128) d -= 256;
    return (d / 2) - 74;
}
