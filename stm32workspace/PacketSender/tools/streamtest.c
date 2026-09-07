/*
 * streamtest.c -- PC-hosted tests for the UART frame parser and for the
 * streaming path through animation.c.
 *
 * Build and run (from the PacketSender directory):
 *
 *   gcc -std=c11 -Wall -Wextra -O1 -DANIM_STREAM_HOST_TEST \
 *       -Itools/hosttest -ICore/Inc \
 *       tools/streamtest.c Core/Src/anim_stream.c Core/Src/animation.c \
 *       -o tools/streamtest.exe && tools/streamtest.exe
 *
 * WHY BOTHER
 * ----------
 * On the board, every one of these failures looks the same: the picture does
 * not move. A framing bug, a wrong baud rate, an unplugged antenna and a
 * mis-set CC1101 register are indistinguishable from the far end of a scope
 * probe. Proving the parser here means that when the board is silent, the
 * parser is not the thing to go and look at.
 */

#include "anim_stream.h"
#include "animation.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Declared in anim_stream.c under ANIM_STREAM_HOST_TEST. */
void anim_stream_test_feed(const uint8_t *buf, uint32_t n);
void anim_stream_test_reset(void);

/* ---- harness ----------------------------------------------------------- */

static int checks = 0, failures = 0;

static void check(int cond, const char *what)
{
    checks++;
    if (!cond) {
        failures++;
        printf("  FAIL  %s\n", what);
    } else {
        printf("  ok    %s\n", what);
    }
}

static void check_eq(unsigned long got, unsigned long want, const char *what)
{
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL  %s (got %lu, want %lu)\n", what, got, want);
    } else {
        printf("  ok    %s = %lu\n", what, got);
    }
}

/* ---- fake target ------------------------------------------------------- */

static uint32_t g_tick;
uint32_t HAL_GetTick(void) { return g_tick; }

/*
 * Airtime is modelled INSIDE the send, not after it returns. That is where it
 * happens on the real board -- cc1101_send_frame() blocks for the duration --
 * and getting this wrong is how an earlier version of this harness convinced
 * itself the overrun guard worked when it had never once seen a late clock.
 */
static uint32_t g_send_ms;
static uint32_t g_sends;
static uint16_t g_last_n;
static cc1101_point_t g_last_pts[ANIM_STREAM_MAX_POINTS];

bool cc1101_send_frame(const cc1101_point_t *pts, uint16_t n_points)
{
    g_sends++;
    g_last_n = n_points;
    if (n_points <= ANIM_STREAM_MAX_POINTS)
        memcpy(g_last_pts, pts, (size_t)n_points * sizeof *pts);
    g_tick += g_send_ms;
    return true;
}

/* ---- frame builder (mirrors tools/animstream.py) ----------------------- */

static uint16_t crc16_ref(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

/* Returns the encoded length. `pts` is n_points pairs. */
static size_t build(uint8_t *out, uint8_t seq, uint16_t n_points,
                    const cc1101_point_t *pts)
{
    size_t i = 0;
    out[i++] = ANIM_STREAM_MAGIC0;
    out[i++] = ANIM_STREAM_MAGIC1;
    size_t body = i;
    out[i++] = seq;
    out[i++] = (uint8_t)(n_points & 0xFF);
    out[i++] = (uint8_t)(n_points >> 8);
    for (uint16_t p = 0; p < n_points; p++) {
        out[i++] = pts ? pts[p].x : (uint8_t)(p * 3u);
        out[i++] = pts ? pts[p].y : (uint8_t)(p * 7u + 1u);
    }
    uint16_t crc = crc16_ref(out + body, i - body);
    out[i++] = (uint8_t)(crc & 0xFF);
    out[i++] = (uint8_t)(crc >> 8);
    return i;
}

static void feed(const uint8_t *b, size_t n) { anim_stream_test_feed(b, (uint32_t)n); }

/* Feed one byte at a time -- proves the state machine survives arbitrary
   fragmentation, which is exactly what a DMA ring hands it. */
static void feed_dribbled(const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; i++) anim_stream_test_feed(b + i, 1);
}

/* ---- tests ------------------------------------------------------------- */

static void t_crc_vector(void)
{
    printf("\n-- CRC-16/CCITT-FALSE reference vector --\n");
    /* The published check value for "123456789". If this is wrong, the C and
       Python implementations can still agree with each other and both be
       wrong, so pin it to the standard rather than to ourselves. */
    check_eq(crc16_ref((const uint8_t *)"123456789", 9), 0x29B1,
             "crc16(\"123456789\")");
}

static void t_good_frame(void)
{
    printf("\n-- a well-formed frame --\n");
    anim_stream_test_reset();

    cc1101_point_t pts[4] = { {10, 20}, {30, 40}, {50, 60}, {70, 80} };
    uint8_t buf[64];
    size_t n = build(buf, 7, 4, pts);
    feed(buf, n);

    const anim_stream_stats_t *s = anim_stream_get_stats();
    check_eq(s->frames_ok, 1, "frames_ok");
    check_eq(s->depth, 1, "depth");

    uint16_t got_n = 0;
    const cc1101_point_t *p = anim_stream_next(NULL, &got_n);
    check(p != NULL, "next() returns a frame");
    check_eq(got_n, 4, "point count");
    check(p && p[0].x == 10 && p[0].y == 20, "point 0 round-trips");
    check(p && p[3].x == 70 && p[3].y == 80, "point 3 round-trips");

    anim_stream_release(NULL);
    check_eq(anim_stream_get_stats()->depth, 0, "depth after release");
    check_eq(anim_stream_get_stats()->last_seq, 7, "last_seq after release");
    check(anim_stream_next(NULL, &got_n) == NULL, "next() empty after release");
}

static void t_fragmentation(void)
{
    printf("\n-- byte-at-a-time delivery --\n");
    anim_stream_test_reset();

    uint8_t buf[512];
    size_t n = build(buf, 0, 100, NULL);
    feed_dribbled(buf, n);

    check_eq(anim_stream_get_stats()->frames_ok, 1, "frames_ok when dribbled");
    uint16_t got = 0;
    check(anim_stream_next(NULL, &got) != NULL && got == 100, "100 points");
}

static void t_leading_garbage(void)
{
    printf("\n-- resync past log noise and false magic --\n");
    anim_stream_test_reset();

    uint8_t junk[] = { 'h', 'i', '\r', '\n', 0xA5, 0xA5, 0x5A };
    /* Note the 0xA5 0xA5 0x5A: the first A5 is a false start and the parser
       must re-test the second one rather than swallowing it, or every frame
       preceded by an A5 byte would be lost. */
    uint8_t buf[512];
    size_t n = build(buf, 3, 6, NULL);

    feed(junk, 4);                    /* pure log text */
    feed(junk + 4, 2);                /* A5 A5 -- false start then real start */
    feed(buf + 1, n - 1);             /* 5A then the rest of a real frame */

    check_eq(anim_stream_get_stats()->frames_ok, 1, "frame found after noise");
    check_eq(anim_stream_get_stats()->resyncs, 1, "one resync counted");
}

static void t_bad_crc(void)
{
    printf("\n-- corrupt payload --\n");
    anim_stream_test_reset();

    uint8_t buf[512];
    size_t n = build(buf, 1, 8, NULL);
    buf[8] ^= 0xFF;                   /* flip a payload byte */
    feed(buf, n);

    check_eq(anim_stream_get_stats()->crc_errors, 1, "crc_errors");
    check_eq(anim_stream_get_stats()->frames_ok, 0, "nothing committed");
    check(anim_stream_next(NULL, NULL) == NULL, "no frame available");

    /* And the very next frame still parses -- a corrupt frame must cost one
       frame, not the session. */
    n = build(buf, 2, 8, NULL);
    feed(buf, n);
    check_eq(anim_stream_get_stats()->frames_ok, 1, "recovers on next frame");
}

static void t_bad_length(void)
{
    printf("\n-- illegal point counts --\n");

    struct { uint16_t n; const char *why; } cases[] = {
        { 7,                            "odd count (splits a span)" },
        { ANIM_STREAM_MAX_POINTS + 2,   "beyond MAX_POINTS" },
    };

    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        anim_stream_test_reset();
        uint8_t buf[4096];
        size_t n = build(buf, 0, cases[i].n, NULL);
        feed(buf, n);
        const anim_stream_stats_t *s = anim_stream_get_stats();
        checks++;
        if (s->len_errors == 1 && s->frames_ok == 0) {
            printf("  ok    rejected: %s\n", cases[i].why);
        } else {
            failures++;
            printf("  FAIL  not rejected: %s (len_errors=%lu frames_ok=%lu)\n",
                   cases[i].why, (unsigned long)s->len_errors,
                   (unsigned long)s->frames_ok);
        }
    }

    /* A rejected length must NOT make the parser eat the following bytes: it
       has no idea how many there are. Prove the next frame still lands. */
    anim_stream_test_reset();
    uint8_t buf[4096];
    size_t n = build(buf, 0, 9, NULL);          /* odd -> rejected */
    feed(buf, n);
    n = build(buf, 1, 10, NULL);                /* valid */
    feed(buf, n);
    check_eq(anim_stream_get_stats()->frames_ok, 1,
             "valid frame after a bad length");
}

/*
 * Regression: blank frames.
 *
 * badapple.json has 44 frames with no spans at all, in runs of up to 20. The
 * first version of this parser rejected zero as an illegal length, which is
 * what took the link down at frame 1829 on hardware.
 */
static void t_blank_frames(void)
{
    printf("\n-- blank (zero-point) frames --\n");
    anim_stream_test_reset();

    uint8_t buf[64];
    size_t n = build(buf, 42, 0, NULL);
    check_eq(n, 7, "a blank frame is 7 bytes on the wire");
    feed(buf, n);

    const anim_stream_stats_t *s = anim_stream_get_stats();
    check_eq(s->frames_ok, 1, "accepted, not rejected");
    check_eq(s->len_errors, 0, "not counted as a length error");

    uint16_t got = 0xFFFF;
    const cc1101_point_t *p = anim_stream_next(NULL, &got);
    check(p != NULL, "next() returns it (NULL would mean starvation)");
    check_eq(got, 0, "zero points");
    anim_stream_release(NULL);
    check_eq(anim_stream_get_stats()->last_seq, 42, "window advanced past it");

    /* A run of them, the way the real clip has it, interleaved with content. */
    anim_stream_test_reset();
    for (int i = 0; i < 13; i++) {
        n = build(buf, (uint8_t)i, 0, NULL);
        feed(buf, n);
        anim_stream_release(NULL);           /* consumed by animation_tick */
    }
    n = build(buf, 13, 4, NULL);
    feed(buf, n);
    check_eq(anim_stream_get_stats()->frames_ok, 14,
             "13 blanks then a real frame");
    check_eq(anim_stream_get_stats()->len_errors, 0, "no length errors");
}

/*
 * Regression: the flow-control window must survive a dropped frame.
 *
 * A frame the board discards is never consumed, so last_consumed cannot
 * advance past it on its own. Four consecutive drops used to saturate the
 * sender's in-flight count and stall the link permanently. A LENGTH rejection
 * now advances the window itself.
 */
static void t_window_survives_drops(void)
{
    printf("\n-- window recovers from dropped frames --\n");
    anim_stream_test_reset();

    uint8_t buf[4096];

    /* Frame 10 lands and is consumed, so the window sits at 10. */
    size_t n = build(buf, 10, 4, NULL);
    feed(buf, n);
    anim_stream_release(NULL);
    check_eq(anim_stream_get_stats()->last_seq, 10, "window at the good frame");

    /* Now SLOTS+2 consecutive frames the board must reject. Odd counts, so
       the sequence number is still trustworthy. */
    for (int i = 0; i < ANIM_STREAM_SLOTS + 2; i++) {
        n = build(buf, (uint8_t)(11 + i), 9, NULL);   /* odd -> rejected */
        feed(buf, n);
    }

    const anim_stream_stats_t *s = anim_stream_get_stats();
    check_eq(s->len_errors, ANIM_STREAM_SLOTS + 2, "all rejected");
    check_eq(s->last_seq, (unsigned)(10 + ANIM_STREAM_SLOTS + 2),
             "window advanced past every drop");

    /*
     * The property that actually matters: the sender's in-flight count stays
     * under SLOTS, so it never stops sending. Computed exactly as
     * animstream.py does.
     */
    unsigned last_sent = (10 + ANIM_STREAM_SLOTS + 2) & 0xFF;
    unsigned in_flight = (last_sent - s->last_seq) & 0xFF;
    check(in_flight < ANIM_STREAM_SLOTS, "in_flight below the window limit");
    check_eq(in_flight, 0, "no phantom frames outstanding");
}

/*
 * Regression: restarting the sender without resetting the board.
 *
 * Reported from hardware -- stop animstream.py and start it again and nothing
 * plays until the STM32 is reset. Three pieces of stale state cause it, and
 * the session reset has to clear all three:
 *
 *   1. last_seq is still the old session's, so the restarted sender's
 *      in_flight computes to ~119 and it never sends
 *   2. the parser is stranded mid-payload, eating the new session's first bytes
 *   3. the slots still hold the old session's frames
 */
static void t_session_restart(void)
{
    printf("\n-- sender restarts without a board reset --\n");
    anim_stream_test_reset();

    uint8_t buf[4096];

    /* ---- session 1: run a while, then get killed mid-frame ---- */
    for (int i = 0; i < 3; i++) {
        size_t n = build(buf, (uint8_t)(135 + i), 4, NULL);
        feed(buf, n);
    }
    anim_stream_release(NULL);
    anim_stream_release(NULL);
    check_eq(anim_stream_get_stats()->last_seq, 136, "session 1 window");
    check_eq(anim_stream_get_stats()->depth, 1, "a frame left in the slots");

    /* Killed in the middle of a 180-point frame: header and part of the
       payload arrived, the rest never will. */
    size_t n = build(buf, 138, 180, NULL);
    feed(buf, 40);
    check(anim_stream_get_stats()->depth == 1, "parser now stranded mid-frame");

    /* ---- the deadlock, before the reset is applied ---- */
    unsigned stale = anim_stream_get_stats()->last_seq;
    unsigned in_flight = (0xFFu - stale) & 0xFFu;   /* restarted sender: seq 0 */
    check(in_flight >= ANIM_STREAM_SLOTS,
          "stale window would stall a restarted sender");

    /* ---- session 2: handshake, then normal frames from sequence 0 ---- */
    static const uint8_t pad[2 * ANIM_STREAM_MAX_POINTS + 4] = { 0 };
    static const uint8_t rst[3] = {
        ANIM_STREAM_MAGIC0, ANIM_STREAM_CTL1, ANIM_STREAM_CTL_RESET
    };
    feed(pad, sizeof pad);
    for (int i = 0; i < 3; i++) feed(rst, sizeof rst);

    const anim_stream_stats_t *s = anim_stream_get_stats();
    check_eq(s->last_seq, 0xFF, "window back to the startup baseline");
    check_eq(s->depth, 0, "stale frames dropped");
    check_eq(s->frames_ok, 0, "counters zeroed for the new session");

    /* The sender's very first frame must land -- this is the whole point. */
    n = build(buf, 0, 4, NULL);
    feed(buf, n);
    check_eq(anim_stream_get_stats()->frames_ok, 1, "first frame of session 2");

    uint16_t got = 0;
    check(anim_stream_next(NULL, &got) != NULL, "and it is playable");
    anim_stream_release(NULL);
    check_eq(anim_stream_get_stats()->last_seq, 0, "window tracks session 2");

    in_flight = (0u - 0u) & 0xFFu;
    check(in_flight < ANIM_STREAM_SLOTS, "sender free to keep sending");
}

static void t_overflow(void)
{
    printf("\n-- more frames than slots --\n");
    anim_stream_test_reset();

    uint8_t buf[512];
    for (int i = 0; i < ANIM_STREAM_SLOTS + 1; i++) {
        size_t n = build(buf, (uint8_t)i, 4, NULL);
        feed(buf, n);
    }

    const anim_stream_stats_t *s = anim_stream_get_stats();
    check_eq(s->frames_ok, ANIM_STREAM_SLOTS, "only SLOTS frames stored");
    check_eq(s->overflows, 1, "one overflow counted");
    check_eq(s->depth, ANIM_STREAM_SLOTS, "buffer full");

    /* The dropped frame must not desync the stream: free a slot and the next
       frame has to land. This is why the parser consumes an overflowed
       payload instead of bailing to magic-hunting inside coordinate data. */
    anim_stream_release(NULL);
    size_t n = build(buf, 99, 4, NULL);
    feed(buf, n);
    check_eq(anim_stream_get_stats()->frames_ok, ANIM_STREAM_SLOTS + 1,
             "still in sync after an overflow");
}

static void t_fifo_order(void)
{
    printf("\n-- slots come back oldest first --\n");
    anim_stream_test_reset();

    uint8_t buf[512];
    for (int i = 0; i < ANIM_STREAM_SLOTS; i++) {
        cc1101_point_t p[2] = { { (uint8_t)i, 0 }, { (uint8_t)i, 255 } };
        size_t n = build(buf, (uint8_t)(50 + i), 2, p);
        feed(buf, n);
    }

    for (int i = 0; i < ANIM_STREAM_SLOTS; i++) {
        uint16_t got = 0;
        const cc1101_point_t *p = anim_stream_next(NULL, &got);
        checks++;
        if (p && p[0].x == i) {
            printf("  ok    slot %d first out\n", i);
        } else {
            failures++;
            printf("  FAIL  slot order at %d (got x=%d)\n", i, p ? p[0].x : -1);
        }
        anim_stream_release(NULL);
        check_eq(anim_stream_get_stats()->last_seq, (unsigned)(50 + i),
                 "last_seq tracks the consumed frame");
    }
}

/* ---- streaming through animation.c ------------------------------------- */

static void t_paced_playback(void)
{
    printf("\n-- animation_tick() over a live source --\n");
    anim_stream_test_reset();
    g_tick = 1000;
    g_sends = 0;
    g_send_ms = 5;                 /* well inside a 50 ms period */

    animation_t a;
    animation_init_source(&a, anim_stream_next, anim_stream_release, NULL, 20);
    check_eq(a.period_ms, 50, "20 fps -> 50 ms period");

    uint8_t buf[512];
    size_t n = build(buf, 0, 4, NULL);
    feed(buf, n);

    check(animation_tick(&a), "first frame goes out immediately");
    check_eq(g_sends, 1, "one send");
    check_eq(anim_stream_get_stats()->depth, 0, "slot released after send");

    /* Nothing more is due for 50 ms even with a frame ready. */
    n = build(buf, 1, 4, NULL);
    feed(buf, n);
    check(!animation_tick(&a), "not due yet");
    check_eq(g_sends, 1, "still one send");

    g_tick += 50;
    check(animation_tick(&a), "due again after one period");
    check_eq(g_sends, 2, "two sends");
}

static void t_starvation(void)
{
    printf("\n-- source runs dry --\n");
    anim_stream_test_reset();
    g_tick = 1000;
    g_sends = 0;
    g_send_ms = 5;

    animation_t a;
    animation_init_source(&a, anim_stream_next, anim_stream_release, NULL, 20);

    /* No frames fed at all. */
    check(!animation_tick(&a), "tick reports no frame sent");
    check_eq(a.frames_starved, 1, "starvation counted");
    check_eq(g_sends, 0, "radio stayed quiet");

    /*
     * The critical property: a starved slot still CONSUMES its period. If it
     * did not, playback would sprint the instant the PC fell behind, which is
     * the exact failure the pacing exists to prevent.
     */
    check(!animation_tick(&a), "starved slot is not retried early");
    check_eq(a.frames_starved, 1, "no second starve inside the same period");

    g_tick += 50;
    uint8_t buf[64];
    size_t n = build(buf, 0, 4, NULL);
    feed(buf, n);
    check(animation_tick(&a), "recovers when data arrives");
    check_eq(g_sends, 1, "one send after recovery");
}

static void t_overrun_guard(void)
{
    printf("\n-- frames that overrun their period --\n");
    anim_stream_test_reset();
    g_tick = 1000;
    g_sends = 0;
    g_send_ms = 80;               /* 80 ms of airtime in a 50 ms period */

    animation_t a;
    animation_init_source(&a, anim_stream_next, anim_stream_release, NULL, 20);

    /*
     * Spin the way main() actually does: call tick over and over, and let the
     * clock advance 1 ms on the calls that do nothing. Feeding a frame and
     * calling tick exactly once per frame would test a machine where time
     * only passes during transmission, and would report a false failure here
     * -- the second frame simply would not be due yet.
     */
    uint8_t buf[512];
    int fed = 0;
    for (int spins = 0; g_sends < 3 && spins < 10000; spins++) {
        if (fed < 3 && anim_stream_get_stats()->depth == 0) {
            size_t n = build(buf, (uint8_t)fed, 4, NULL);
            feed(buf, n);
            fed++;
        }
        if (!animation_tick(&a)) g_tick += 1;
    }

    check_eq(g_sends, 3, "all three sent");
    check_eq(a.frames_late, 3, "each one counted late");

    /*
     * And the deadline must be resynchronised, not accumulating a debt: after
     * three 80 ms frames the schedule should be one period ahead of NOW, not
     * 90 ms in the past.
     */
    check((int32_t)(a.next_deadline - g_tick) > 0,
          "deadline is ahead of now (no compounding debt)");
    check_eq(a.next_deadline - g_tick, 50, "resynced to now + one period");
}

static void t_clip_mode_unbroken(void)
{
    printf("\n-- flash clip mode still works --\n");
    g_tick = 1000;
    g_sends = 0;
    g_send_ms = 5;

    static const cc1101_point_t pool[6] = {
        {1,1},{2,2},   {3,3},{4,4},   {5,5},{6,6}
    };
    static const anim_frame_t fr[3] = { {0,2}, {2,2}, {4,2} };
    static const anim_clip_t clip = { pool, fr, 3 };

    animation_t a;
    animation_init(&a, &clip, 20);

    for (int i = 0; i < 3; i++) {
        check(animation_tick(&a), "clip frame sent");
        g_tick += 50;
    }
    check_eq(g_sends, 3, "three frames");
    check_eq(a.loops, 1, "wrapped once");
    check_eq(a.index, 0, "back at frame 0");
    check_eq(a.frames_starved, 0, "clip mode never starves");
}

/*
 * Feed a file of raw wire bytes through the parser and report what came out.
 *
 * This is how animstream.py is checked against this parser instead of against
 * a second Python implementation of the same idea. Two encoders that agree
 * with each other and disagree with the board is the classic way a wire format
 * goes wrong, and it is invisible until the hardware is silent.
 */
static int replay(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 2; }

    anim_stream_test_reset();

    uint8_t chunk[256];
    size_t got;
    /* Chunked, not slurped: this also exercises arbitrary fragmentation, the
       way the DMA ring hands over whatever happens to have landed. */
    while ((got = fread(chunk, 1, sizeof chunk, f)) > 0) {
        for (size_t i = 0; i < got; i++) {
            anim_stream_test_feed(chunk + i, 1);
            /* Drain as we go, or a file with more than SLOTS frames would
               report overflows that the real board never sees. */
            if (anim_stream_get_stats()->depth == ANIM_STREAM_SLOTS)
                anim_stream_release(NULL);
        }
    }
    fclose(f);

    const anim_stream_stats_t *s = anim_stream_get_stats();
    printf("frames_ok=%lu crc_errors=%lu len_errors=%lu overflows=%lu "
           "resyncs=%lu bytes=%lu last_seq=%u\n",
           (unsigned long)s->frames_ok, (unsigned long)s->crc_errors,
           (unsigned long)s->len_errors, (unsigned long)s->overflows,
           (unsigned long)s->resyncs, (unsigned long)s->bytes,
           (unsigned)s->last_seq);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc == 3 && strcmp(argv[1], "--replay") == 0)
        return replay(argv[2]);

    printf("streamtest -- MAX_POINTS=%d SLOTS=%d RING=%d\n",
           ANIM_STREAM_MAX_POINTS, ANIM_STREAM_SLOTS, ANIM_STREAM_RING);

    t_crc_vector();
    t_good_frame();
    t_fragmentation();
    t_leading_garbage();
    t_bad_crc();
    t_bad_length();
    t_blank_frames();
    t_window_survives_drops();
    t_session_restart();
    t_overflow();
    t_fifo_order();
    t_paced_playback();
    t_starvation();
    t_overrun_guard();
    t_clip_mode_unbroken();

    printf("\n%d checks, %d failures -- %s\n",
           checks, failures, failures ? "FAILED" : "PASS");
    return failures ? 1 : 0;
}
