/*
 * cc1101.h
 *
 *  Created on: Aug 3, 2026
 *      Author: Nymph
 */

/*
 * cc1101.h -- CC1101 receiver for the DD_RF link
 *
 * Receives the packets transmitted by the Zybo Z7-10 PL design:
 *
 *     payload = { 0xAA, 0x55, seq }
 *
 * The two markers let you tell real data from noise at a glance; seq increments
 * once per transmit attempt, so gaps in it are lost packets.
 *
 * The register table in cc1101.c MUST match the Zybo's ConfigSeq ROM exactly.
 * Frequency, data rate, deviation, sync word and packet control all have to
 * agree or the radios will sit next to each other hearing nothing. This is the
 * single most common reason a two-CC1101 bench setup fails -- do not swap in a
 * library's default config.
 */
#ifndef CC1101_H
#define CC1101_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- pin mapping: edit to match your .ioc ---- */
#define CC_CS_PORT    GPIOB
#define CC_CS_PIN     GPIO_PIN_6      /* D10 */
#define CC_GDO2_PORT  GPIOA
#define CC_GDO2_PIN   GPIO_PIN_8      /* D7  */
#define CC_MISO_PORT  GPIOA
#define CC_MISO_PIN   GPIO_PIN_6      /* read as GPIO for the CHIP_RDYn wait */

/* ---- what a received packet looks like ---- */
typedef struct {
    uint8_t len;        /* length byte: payload bytes, not counting itself */
    uint8_t payload[8];
    uint8_t rssi_raw;   /* appended status byte 1 */
    uint8_t lqi;        /* appended status byte 2, bits 6:0 */
    bool    crc_ok;     /* appended status byte 2, bit 7 */
} cc1101_pkt_t;

void    cc1101_init(SPI_HandleTypeDef *hspi);
bool    cc1101_rx_poll(cc1101_pkt_t *pkt);   /* true when a packet was read */
uint8_t cc1101_read_status(uint8_t addr);    /* burst-bit status register read */
int     cc1101_rssi_dbm(uint8_t rssi_raw);

#endif /* CC1101_H */
