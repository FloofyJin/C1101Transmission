/*
 * anim_stream.c -- UART frame receiver. See anim_stream.h for the protocol and
 * the reasoning behind it.
 *
 * The file is split in two on purpose:
 *
 *   - the PARSER is pure C over a byte at a time, with no reference to any
 *     peripheral. That makes it testable on a PC (see tools/streamtest.c),
 *     which matters because a framing bug on the board looks identical to a
 *     wiring problem, a baud mismatch, or a radio fault.
 *   - the TRANSPORT is register-level DMA, compiled out of the host test.
 */

#include "anim_stream.h"
#include <string.h>

#ifndef ANIM_STREAM_HOST_TEST
#include "main.h"
#endif

#if (ANIM_STREAM_RING & (ANIM_STREAM_RING - 1)) != 0
#error "ANIM_STREAM_RING must be a power of two -- the read index uses a mask"
#endif

#define RING_MASK   (ANIM_STREAM_RING - 1)

/* ------------------------------------------------------------------ */
/* state                                                              */
/* ------------------------------------------------------------------ */

/*
 * Decoded frames, oldest-first. `rd`/`wr` chase each other and `count` breaks
 * the full/empty tie -- cheaper and clearer than sacrificing a slot to keep
 * the two indices distinct, and there is no concurrency here to protect
 * against: everything runs from the main loop.
 */
static cc1101_point_t s_slot[ANIM_STREAM_SLOTS][ANIM_STREAM_MAX_POINTS];
static uint16_t       s_slot_n[ANIM_STREAM_SLOTS];
static uint8_t        s_slot_seq[ANIM_STREAM_SLOTS];
static uint8_t        s_rd, s_wr, s_count;

/* Parser. */
typedef enum {
    P_MAGIC0 = 0, P_MAGIC1, P_SEQ, P_LEN_LO, P_LEN_HI,
    P_PAYLOAD, P_CRC_LO, P_CRC_HI, P_CTL
} parse_state_t;

static parse_state_t   s_state;
static uint16_t        s_need;      /* payload bytes still expected           */
static uint16_t        s_got;       /* payload bytes taken so far             */
static uint16_t        s_npoints;
static uint8_t         s_seq;
static uint16_t        s_crc;       /* running CRC over seq..payload          */
static uint16_t        s_crc_rx;    /* CRC as received                        */
static cc1101_point_t *s_dst;       /* NULL = parse but discard (no free slot) */

static anim_stream_stats_t s_stats;

/* ------------------------------------------------------------------ */
/* CRC-16/CCITT-FALSE                                                  */
/* ------------------------------------------------------------------ */
/*
 * Nibble-at-a-time: 32 bytes of table instead of 512. Flash pressure is the
 * entire reason this module exists, so spending half a kilobyte of it on a
 * CRC table would be a poor trade -- and at 16 MHz the byte-wise table would
 * save around 200 us per second, which is nothing.
 */
static const uint16_t s_crc_tab[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
};

static inline uint16_t crc16_step(uint16_t crc, uint8_t b)
{
    crc = (uint16_t)((crc << 4) ^ s_crc_tab[((crc >> 12) ^ (b >> 4)) & 0x0Fu]);
    crc = (uint16_t)((crc << 4) ^ s_crc_tab[((crc >> 12) ^ (b     )) & 0x0Fu]);
    return crc;
}

/* ------------------------------------------------------------------ */
/* transport                                                           */
/* ------------------------------------------------------------------ */

#ifndef ANIM_STREAM_HOST_TEST

static uint8_t             s_ring[ANIM_STREAM_RING];
static uint16_t            s_ring_rd;
static UART_HandleTypeDef *s_huart;
static uint32_t            s_last_ctl_ms;

/*
 * Where the DMA has written up to. NDTR counts DOWN, and in circular mode it
 * reloads itself, so this is always a valid index and never needs a lock:
 * the hardware owns NDTR, we own s_ring_rd, and neither writes the other's.
 */
static inline uint16_t dma_head(void)
{
    uint16_t left = (uint16_t)(DMA1_Stream5->NDTR & 0xFFFFu);
    return (uint16_t)((ANIM_STREAM_RING - left) & RING_MASK);
}

/*
 * Control message: tag, kind, argument. Blocking, but only 3 bytes -- 130 us
 * at 230400 -- and it is never called from inside the radio transmit path.
 */
static void send_ctl(uint8_t kind, uint8_t arg)
{
    uint8_t b[3] = { ANIM_STREAM_TAG, kind, arg };
    if (s_huart != NULL) {
        HAL_UART_Transmit(s_huart, b, sizeof b, 10);
        s_last_ctl_ms = HAL_GetTick();
    }
}

void anim_stream_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    memset(&s_stats, 0, sizeof s_stats);
    s_rd = s_wr = s_count = 0;
    s_state = P_MAGIC0;
    s_ring_rd = 0;

    /*
     * USART2_RX is DMA1, stream 5, channel 4 (RM0401 table 27). This is done
     * with registers rather than HAL_UART_Receive_DMA() deliberately: the HAL
     * path arms transfer-complete callbacks, drives huart2's RxState, and --
     * the part that actually bites -- aborts the transfer on an overrun error.
     * A receiver that switches itself off when the line gets noisy is worse
     * than useless. Circular DMA straight off the peripheral cannot overrun,
     * because it drains DR within a bit time and never stops.
     */
    __HAL_RCC_DMA1_CLK_ENABLE();

    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) { /* disable is not instant */ }

    /* Clear every stale flag for stream 5, or the first transfer inherits
       them and the peripheral refuses to start. */
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5
                | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

    DMA1_Stream5->PAR  = (uint32_t)&(USART2->DR);
    DMA1_Stream5->M0AR = (uint32_t)s_ring;
    DMA1_Stream5->NDTR = ANIM_STREAM_RING;
    DMA1_Stream5->FCR  = 0x00000021u;   /* direct mode, FIFO off -- reset value */

    /* Byte-wide both sides (PSIZE/MSIZE = 0), peripheral-to-memory (DIR = 0),
       memory pointer increments, wrap forever. No interrupt is enabled: we
       poll NDTR instead, so nothing here can preempt the radio transmit. */
    DMA1_Stream5->CR = (4u << DMA_SxCR_CHSEL_Pos)
                     | DMA_SxCR_MINC
                     | DMA_SxCR_CIRC;
    DMA1_Stream5->CR |= DMA_SxCR_EN;

    /* Clear a pending ORE left over from before DMA was armed: the read of SR
       followed by DR is the documented clearing sequence, and skipping it
       leaves RXNE stuck so the DMA never sees a single byte. */
    (void)USART2->SR;
    (void)USART2->DR;

    USART2->CR3 |= USART_CR3_DMAR;

    /* Tell the PC we are here and that its sequence numbers are stale. 0xFF as
       "last consumed" makes the first frame the PC sends (seq 0) land exactly
       one past it, so the window arithmetic needs no special case at startup. */
    s_stats.last_seq = 0xFF;
    send_ctl(ANIM_STREAM_MSG_HELLO, 0);
    send_ctl(ANIM_STREAM_MSG_WINDOW, 0xFF);
}

#else  /* host test: no peripherals, bytes are pushed in by the test */

static void send_ctl(uint8_t kind, uint8_t arg) { (void)kind; (void)arg; }

void anim_stream_test_reset(void)
{
    memset(&s_stats, 0, sizeof s_stats);
    s_rd = s_wr = s_count = 0;
    s_state = P_MAGIC0;
    s_stats.last_seq = 0xFF;
}

#endif /* ANIM_STREAM_HOST_TEST */

/* ------------------------------------------------------------------ */
/* parser                                                              */
/* ------------------------------------------------------------------ */

/*
 * Drop everything from the previous session and announce a clean slate.
 *
 * Called on an A5 5B 01 from the sender, and equivalent to a board reset as
 * far as this module is concerned. Counters are zeroed too: leaving them would
 * make the new session's statistics read as a continuation of the old one, and
 * the first thing anyone does with a fresh stream is check whether the error
 * counts are moving.
 *
 * last_seq goes to 0xFF rather than 0 so that the sender's first frame -- which
 * is sequence 0 -- lands exactly one past it and the window arithmetic needs no
 * special case, the same convention anim_stream_init() uses.
 */
static void stream_reset(void)
{
    s_rd = s_wr = s_count = 0;
    s_dst = NULL;
    memset(&s_stats, 0, sizeof s_stats);
    s_stats.last_seq = 0xFF;

    send_ctl(ANIM_STREAM_MSG_HELLO, 0);
    send_ctl(ANIM_STREAM_MSG_WINDOW, 0xFF);
}

/*
 * One byte. Never blocks, never allocates, and every failure path returns to
 * P_MAGIC0 so a corrupt frame costs exactly one frame and not the session.
 */
static void parse_byte(uint8_t b)
{
    switch (s_state) {

    case P_MAGIC0:
        /*
         * Hunting. Bytes that are not 0xA5 are silently dropped, not counted:
         * at startup the PC's terminal may still be echoing, and counting
         * those as errors would make the statistics useless from the first
         * second onward.
         */
        if (b == ANIM_STREAM_MAGIC0) s_state = P_MAGIC1;
        break;

    case P_MAGIC1:
        if (b == ANIM_STREAM_MAGIC1) {
            s_state = P_SEQ;
        } else if (b == ANIM_STREAM_CTL1) {
            s_state = P_CTL;
        } else {
            s_stats.resyncs++;
            /* 0xA5 0xA5 is a legitimate start followed by a repeat, so re-test
               this byte as a first magic rather than dropping it. */
            s_state = (b == ANIM_STREAM_MAGIC0) ? P_MAGIC1 : P_MAGIC0;
        }
        break;

    case P_CTL:
        /*
         * Deliberately NOT CRC-protected. A control message is three bytes and
         * idempotent, and the sender repeats it; adding a checksum would buy
         * nothing but another way for the handshake itself to fail. A spurious
         * A5 5B 01 inside corrupted data would cost one session reset, which
         * is recoverable, whereas a handshake that will not go through is not.
         */
        if (b == ANIM_STREAM_CTL_RESET) stream_reset();
        s_state = P_MAGIC0;
        break;

    case P_SEQ:
        s_seq   = b;
        s_crc   = crc16_step(0xFFFFu, b);
        s_state = P_LEN_LO;
        break;

    case P_LEN_LO:
        s_npoints = b;
        s_crc     = crc16_step(s_crc, b);
        s_state   = P_LEN_HI;
        break;

    case P_LEN_HI:
        s_npoints |= (uint16_t)b << 8;
        s_crc      = crc16_step(s_crc, b);

        /*
         * Validate BEFORE trusting the length. A corrupt length would
         * otherwise make the parser swallow up to 65535 bytes of what is
         * probably the next few frames -- one bad byte turning into a second
         * of dead air. Rejecting here costs one frame instead.
         *
         * ZERO IS LEGAL. An all-black frame has no spans, and Bad Apple has 44
         * of them; rejecting zero made a run of blank frames look like a burst
         * of protocol errors. A zero-point frame is carried through the slots
         * like any other so it still consumes its period -- see animation.c,
         * which skips the radio but holds the schedule.
         */
        if ((s_npoints & 1u) != 0 || s_npoints > ANIM_STREAM_MAX_POINTS) {
            s_stats.len_errors++;
            /*
             * Advance the window past the frame we are throwing away.
             *
             * Without this the PC waits forever for a frame that no longer
             * exists: its in-flight count is (last_sent - last_consumed), and
             * a frame that is dropped is never consumed, so ANIM_STREAM_SLOTS
             * consecutive drops stall the link permanently. The sequence
             * number is safe to use here -- it was parsed before the length
             * and is not implicated in the length being wrong.
             */
            s_stats.last_seq = s_seq;
            send_ctl(ANIM_STREAM_MSG_WINDOW, s_seq);
            send_ctl(ANIM_STREAM_MSG_ERROR, ANIM_STREAM_ERR_LEN);
            s_state = P_MAGIC0;
            break;
        }

        /*
         * Decode straight into the destination slot -- no staging buffer, so
         * no second copy and no extra 360 bytes of RAM. If the CRC turns out
         * bad the slot simply is not committed, and it was free anyway.
         */
        if (s_count < ANIM_STREAM_SLOTS) {
            s_dst = s_slot[s_wr];
        } else {
            /* No room. Keep parsing so the stream stays in sync -- bailing to
               P_MAGIC0 here would hunt for magic inside coordinate data and
               almost certainly find a false one. */
            s_dst = NULL;
            s_stats.overflows++;
            send_ctl(ANIM_STREAM_MSG_ERROR, ANIM_STREAM_ERR_FULL);
        }

        s_need  = (uint16_t)(s_npoints * 2u);
        s_got   = 0;
        /* A blank frame has no payload bytes at all, so the CRC follows the
           length immediately. Entering P_PAYLOAD with nothing to collect would
           wait for a byte that never comes and swallow the CRC as data. */
        s_state = (s_need == 0) ? P_CRC_LO : P_PAYLOAD;
        break;

    case P_PAYLOAD:
        s_crc = crc16_step(s_crc, b);
        if (s_dst != NULL) {
            /* Even byte -> x, odd byte -> y. cc1101_point_t is two uint8_t in
               that order, so the frame goes on air exactly as it arrived. */
            if ((s_got & 1u) == 0) s_dst[s_got >> 1].x = b;
            else                   s_dst[s_got >> 1].y = b;
        }
        if (++s_got >= s_need) s_state = P_CRC_LO;
        break;

    case P_CRC_LO:
        s_crc_rx = b;
        s_state  = P_CRC_HI;
        break;

    case P_CRC_HI:
        s_crc_rx |= (uint16_t)b << 8;
        if (s_crc_rx != s_crc) {
            s_stats.crc_errors++;
            send_ctl(ANIM_STREAM_MSG_ERROR, ANIM_STREAM_ERR_CRC);
        } else if (s_dst != NULL) {
            s_slot_n[s_wr]   = s_npoints;
            s_slot_seq[s_wr] = s_seq;
            s_wr = (uint8_t)((s_wr + 1u) % ANIM_STREAM_SLOTS);
            s_count++;
            s_stats.frames_ok++;
        }
        s_dst   = NULL;
        s_state = P_MAGIC0;
        break;

    default:
        s_state = P_MAGIC0;
        break;
    }

    s_stats.depth = s_count;
}

#ifdef ANIM_STREAM_HOST_TEST
void anim_stream_test_feed(const uint8_t *buf, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        parse_byte(buf[i]);
        s_stats.bytes++;
    }
}
#endif

/* ------------------------------------------------------------------ */
/* public                                                              */
/* ------------------------------------------------------------------ */

#ifndef ANIM_STREAM_HOST_TEST
void anim_stream_poll(void)
{
    uint16_t head = dma_head();

    /*
     * Bounded: at most one ring's worth per call. If the PC floods the link
     * this returns to the main loop instead of looping until the DMA laps us,
     * which would starve animation_tick() and stall playback -- the exact
     * failure the buffering is here to prevent.
     */
    uint16_t guard = ANIM_STREAM_RING;
    while (s_ring_rd != head && guard-- != 0) {
        parse_byte(s_ring[s_ring_rd]);
        s_ring_rd = (uint16_t)((s_ring_rd + 1u) & RING_MASK);
        s_stats.bytes++;
    }

    /*
     * Heartbeat. The window message is normally sent when a frame is consumed,
     * but if one of those is lost the PC sits on a stale window and eventually
     * stops sending. Repeating the current position costs 3 bytes four times a
     * second and makes that unrecoverable case merely a hiccup.
     */
    if ((uint32_t)(HAL_GetTick() - s_last_ctl_ms) >= ANIM_STREAM_HEARTBEAT_MS) {
        send_ctl(ANIM_STREAM_MSG_WINDOW, s_stats.last_seq);
    }
}
#endif

const cc1101_point_t *anim_stream_next(void *ctx, uint16_t *n_points)
{
    (void)ctx;
    if (s_count == 0) return NULL;
    if (n_points != NULL) *n_points = s_slot_n[s_rd];
    return s_slot[s_rd];
}

void anim_stream_release(void *ctx)
{
    (void)ctx;
    if (s_count == 0) return;

    s_stats.last_seq = s_slot_seq[s_rd];
    s_rd = (uint8_t)((s_rd + 1u) % ANIM_STREAM_SLOTS);
    s_count--;
    s_stats.depth = s_count;

    /* Opening a slot is the event the PC is waiting on, so report it now
       rather than letting the heartbeat get to it up to 250 ms later. */
    send_ctl(ANIM_STREAM_MSG_WINDOW, s_stats.last_seq);
}

const anim_stream_stats_t *anim_stream_get_stats(void)
{
    return &s_stats;
}
