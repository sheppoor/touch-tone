/*
 * Touch Tone — Pebble Time 2 (Emery) DTMF keypad watchapp.
 *
 * Tapping a key plays the North American DTMF tone pair while the finger is
 * held. The buttons add three more telephone sounds: Select holds a dial tone,
 * Up holds a looping busy signal, and Down plays the Special Information Tone
 * (SIT) error sequence once. Only one sound plays at a time (single voice).
 */

#include <pebble.h>
#include "touch_tone_audio.h"

/* ------------------------------------------------------------------ */
/* Tunable settings                                                    */
/* ------------------------------------------------------------------ */
/* Draw the sub-letters (ABC, DEF, …) ABOVE the main digit when true,   */
/* BELOW it when false. The lone digit/symbol keys (1, *, #) follow the  */
/* same digit baseline so every digit in a row stays aligned.           */
#define ALPHAS_ABOVE  false

/* ------------------------------------------------------------------ */
/* Layout (Section 3). The grid is sized and centered from the actual   */
/* screen bounds at window load — Emery is larger than the plan's 180². */
/* ------------------------------------------------------------------ */
#define COLS          3
#define ROWS          4
#define KEY_COUNT    12
#define H_GAP         6   /* horizontal gap between keys (and side margin) */
#define V_GAP         6   /* vertical gap between keys (and top/bottom)     */

/* Computed in compute_layout() from the root layer bounds. */
static int s_key_w      = 48;
static int s_key_h      = 38;
static int s_col_stride = 54;  /* s_key_w + H_GAP */
static int s_row_stride = 42;  /* s_key_h + V_GAP */
static int s_left_margin = 12;
static int s_top_margin  = 8;

#define DIAL_STREAM_KEY 12  /* sentinel: stream is playing the dial tone */

static void compute_layout(GRect bounds) {
  int w = bounds.size.w;
  int h = bounds.size.h;
  /* Fill the screen: 3 cols / 4 rows with H_GAP/V_GAP between keys and an
   * equal outer margin on every side (COLS+1 gaps wide, ROWS+1 gaps tall). */
  s_key_w = (w - (COLS + 1) * H_GAP) / COLS;
  s_key_h = (h - (ROWS + 1) * V_GAP) / ROWS;
  s_col_stride = s_key_w + H_GAP;
  s_row_stride = s_key_h + V_GAP;
  int grid_w = COLS * s_key_w + (COLS - 1) * H_GAP;
  int grid_h = ROWS * s_key_h + (ROWS - 1) * V_GAP;
  s_left_margin = (w - grid_w) / 2;  /* center horizontally */
  s_top_margin  = (h - grid_h) / 2;  /* center vertically   */
}

/* ------------------------------------------------------------------ */
/* Colors (Section 4). GColorFromRGB is not a constant expression, so   */
/* these are macros evaluated at draw time rather than static consts.   */
/* Validate against the 64-color e-paper palette on device.             */
/* ------------------------------------------------------------------ */
#define COLOR_BACKGROUND   GColorBlack
#define COLOR_KEY_BORDER   GColorFromRGB(0x8B, 0x73, 0x55)  /* warm brown  */
#define COLOR_KEY_FACE     GColorFromRGB(0xD4, 0xC5, 0xA9)  /* warm beige  */
#define COLOR_KEY_TEXT     GColorBlack
#define COLOR_STATUS_TEXT  GColorWhite

/* ------------------------------------------------------------------ */
/* Key labels (Section 4)                                              */
/* ------------------------------------------------------------------ */
typedef struct {
  const char *main_label;
  const char *sub_label;  /* NULL if none */
} KeyLabel;

static const KeyLabel KEY_LABELS[KEY_COUNT] = {
  {"1", NULL},  {"2", "ABC"},  {"3", "DEF"},
  {"4", "GHI"}, {"5", "JKL"},  {"6", "MNO"},
  {"7", "PRS"}, {"8", "TUV"},  {"9", "WXY"},
  {"*", NULL},  {"0", "OPER"}, {"#", NULL},
};

/* ------------------------------------------------------------------ */
/* State (Section 6)                                                   */
/* ------------------------------------------------------------------ */
static Window *s_window = NULL;
static Layer  *s_canvas = NULL;

static int      s_active_key = -1;        /* -1 = none, 0-11 = active key  */
static uint32_t s_tone_start_ms = 0;      /* now_ms() when tone started     */
static AppTimer *s_min_duration_timer = NULL;
static bool     s_stop_pending = false;   /* finger lifted before 50ms floor*/
static bool     s_dial_playing = false;   /* true while Enter is held        */
static bool     s_touch_enabled = false;
static bool     s_app_in_focus = true;    /* false while a notification/overlay covers us */
/* Note: the SDK exposes no way to detect a muted speaker (no speaker_is_muted(),
 * no volume getter), so a "muted" status indicator is left unimplemented;
 * quiet_time_is_active() is unrelated — it governs notifications/vibration, not
 * the app's speaker output. Apps cannot query or override the system volume. */

static bool     s_busy_playing = false;   /* true while Up (busy signal) is held */
static bool     s_sit_playing = false;    /* true while the SIT sequence is playing */
static AppTimer *s_sit_timer = NULL;      /* fires when the whole SIT has played    */
static bool     s_sit_closed = false;     /* guard: stream_close() called once      */

/* Shared single-stream playback layer state. */
typedef enum {
  STREAM_NONE = 0,
  STREAM_TONE,   /* one fixed buffer looped (the dial tone)                      */
  STREAM_BUSY,   /* alternates tone/silence buffers to make the busy cadence    */
  STREAM_SIT,    /* steps through the SIT tone/gap sequence once, then closes   */
  STREAM_SYNTH,  /* real-time DTMF synthesis from a phase accumulator (keypad)  */
} StreamMode;
static StreamMode      s_stream_mode = STREAM_NONE;
static bool            s_stream_open = false;
static int             s_stream_key = -1;    /* -1, 0-11 (DTMF), or DIAL_STREAM_KEY */
static const int16_t  *s_stream_buffer = NULL;
static AppTimer       *s_refill_timer = NULL;
static uint32_t        s_stream_start_ms = 0; /* now_ms() of first write       */
static uint32_t        s_bytes_written = 0;   /* total bytes handed to speaker  */

#define MIN_TONE_MS 50

/* Busy signal cadence: 0.5s ON, 0.5s OFF. At 8 kHz that is 4000 samples per
 * phase = exactly 5 buffers of TONE_BUFFER_SAMPLES (800), so the phase always
 * lands on a buffer boundary. */
#define BUSY_PHASE_SAMPLES 4000

/* SIT sequence (ITU-T E.180): three ascending pure tones with 30ms gaps. Phase
 * lengths are NOT buffer-aligned (the gaps are shorter than one buffer), so the
 * SIT feed writes exact per-phase sample counts rather than whole buffers. */
#define SIT_TONE1_SAMPLES   2640   /*  950 Hz, 330 ms */
#define SIT_GAP_SAMPLES      240   /* silence,  30 ms */
#define SIT_TONE2_SAMPLES   2640   /* 1400 Hz, 330 ms */
#define SIT_TONE3_SAMPLES   3040   /* 1800 Hz, 380 ms (long variant) */
#define SIT_TOTAL_SAMPLES   (SIT_TONE1_SAMPLES + SIT_GAP_SAMPLES + \
                             SIT_TONE2_SAMPLES + SIT_GAP_SAMPLES + SIT_TONE3_SAMPLES)  /* 8800 */
#define SIT_TOTAL_MS        (SIT_TOTAL_SAMPLES / 8)   /* 8000 samples/s => /8 per ms => 1100 ms */
#define SIT_DRAIN_MARGIN_MS 250   /* keep s_sit_playing set until audio fully drains */

typedef struct {
  const int16_t *buffer;   /* tone or silence buffer for this phase */
  uint32_t       samples;  /* phase duration in samples            */
} SitPhase;

static const SitPhase SIT_SEQUENCE[] = {
  { g_sit_950,      SIT_TONE1_SAMPLES },
  { g_busy_silence, SIT_GAP_SAMPLES   },
  { g_sit_1400,     SIT_TONE2_SAMPLES },
  { g_busy_silence, SIT_GAP_SAMPLES   },
  { g_sit_1800,     SIT_TONE3_SAMPLES },
};
#define SIT_PHASE_COUNT 5

/* PCM stream feed parameters (8 kHz, 16-bit signed mono). */
#define BYTES_PER_SAMPLE   2
#define BUFFER_BYTES       (TONE_BUFFER_SAMPLES * BYTES_PER_SAMPLE)
#define BYTES_PER_MS       16   /* 8000 samples/s * 2 bytes / 1000 ms          */
#define REFILL_INTERVAL_MS 50   /* how often we top up the queue (< lead)       */
#define TARGET_LEAD_MS     50   /* audio queued ahead of playback. This is also  */
                                /* the onset latency you hear at the start of a  */
                                /* sound (key tone or dial tone). Lower = snappier */
                                /* start, but less cushion against underruns.     */

/* Fixed playback volume (0-100). The Up/Down volume control was removed. */
#define SPEAKER_VOLUME     66

/* ------------------------------------------------------------------ */
/* Real-time DTMF synthesis (keypad tones only).                       */
/*                                                                     */
/* The 12 keypad tones are synthesized continuously from two phase     */
/* accumulators rather than looping a pre-computed buffer. Because the  */
/* phase persists across every refill, there are no loop boundaries —   */
/* and so none of the ~10 Hz pulsing a repeated non-harmonic 800-sample */
/* buffer produced. (Dial tone, busy, and SIT still use buffers.)       */
/* ------------------------------------------------------------------ */
#define SYNTH_SAMPLES 1024   /* samples generated per refill chunk (~128ms)      */

typedef struct {
  uint32_t phase_acc;   /* 32-bit phase, wraps naturally on overflow             */
  uint32_t phase_inc;   /* per-sample phase step for this frequency              */
} ToneOscillator;

static ToneOscillator s_osc[2];                 /* [0] = row freq, [1] = col freq */
static int16_t        s_synth_buf[SYNTH_SAMPLES];

/* DTMF row/column frequencies by key index (0-11). */
static const uint16_t DTMF_FREQS[12][2] = {
  {697, 1209}, {697, 1336}, {697, 1477},   /* 1 2 3 */
  {770, 1209}, {770, 1336}, {770, 1477},   /* 4 5 6 */
  {852, 1209}, {852, 1336}, {852, 1477},   /* 7 8 9 */
  {941, 1209}, {941, 1336}, {941, 1477},   /* * 0 # */
};
/* Per-key phase increments, computed once at startup (see init below). */
static uint32_t s_dtmf_phase_inc[12][2];

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint32_t now_ms(void) {
  time_t secs = 0;
  uint16_t ms = time_ms(&secs, NULL);
  return (uint32_t)secs * 1000U + ms;
}

/* ------------------------------------------------------------------ */
/* Audio: single PCM stream. The dial tone, busy signal, and SIT loop   */
/* pre-computed buffers; the DTMF keypad tones are synthesized live.    */
/* We keep the stream fed for as long as a sound should play, then stop. */
/* ------------------------------------------------------------------ */

/* Compute each key's two phase increments once: inc = freq/8000 * 2^32.
 * Double math, run a single time at startup — never in the sample loop. */
static void init_dtmf_phase_increments(void) {
  for (int k = 0; k < 12; k++) {
    for (int t = 0; t < 2; t++) {
      s_dtmf_phase_inc[k][t] =
          (uint32_t)((double)DTMF_FREQS[k][t] / 8000.0 * 4294967296.0);
    }
  }
}

/* Generate SYNTH_SAMPLES of the active two-tone DTMF mix into s_synth_buf by
 * advancing both phase accumulators. The accumulators are NOT reset here — they
 * persist across refills, which is exactly what makes the waveform continuous
 * (no loop boundary). They are zeroed only at key-down, in synth_start(). */
static void synth_fill_buffer(void) {
  for (int i = 0; i < SYNTH_SAMPLES; i++) {
    s_osc[0].phase_acc += s_osc[0].phase_inc;
    s_osc[1].phase_acc += s_osc[1].phase_inc;
    int32_t sample = (int32_t)SINE_TABLE[s_osc[0].phase_acc >> 24]
                   + (int32_t)SINE_TABLE[s_osc[1].phase_acc >> 24];
    s_synth_buf[i] = (int16_t)(sample >> 1);  /* /2: two full-scale sines summed */
  }
}

/* Which buffer to write next. STREAM_TONE always feeds its fixed buffer.
 * STREAM_BUSY derives the cadence phase from the write position: the first
 * BUSY_PHASE_SAMPLES of every 2*BUSY_PHASE_SAMPLES are the tone, the rest are
 * silence — which produces the 0.5s-on / 0.5s-off US busy signal. Because the
 * phase length is a whole multiple of the buffer length, each write lands
 * entirely within one phase. */
static const int16_t *feed_buffer(void) {
  if (s_stream_mode == STREAM_BUSY) {
    uint32_t samples = s_bytes_written / BYTES_PER_SAMPLE;
    uint32_t phase = (samples / BUSY_PHASE_SAMPLES) & 1u;  /* 0 = ON, 1 = OFF */
    return phase ? g_busy_silence : g_busy_tone_on;
  }
  return s_stream_buffer;
}

/* Top up the speaker's internal queue with whole buffer loops, keeping a
 * TARGET_LEAD_MS cushion ahead of estimated playback. Writing proactively (vs.
 * waiting for a drained/finished callback, which fires only once and too late)
 * is what keeps a held tone — and the busy signal — sounding. */
static void feed_stream(void) {
  if (!s_stream_open) return;
  uint32_t consumed = (now_ms() - s_stream_start_ms) * BYTES_PER_MS;
  uint32_t target = consumed + (uint32_t)TARGET_LEAD_MS * BYTES_PER_MS;
  int guard = 0;
  while (s_bytes_written < target && guard++ < 64) {
    uint32_t n;
    if (s_stream_mode == STREAM_SYNTH) {
      synth_fill_buffer();                 /* fresh, phase-continuous samples */
      uint32_t want = (uint32_t)SYNTH_SAMPLES * BYTES_PER_SAMPLE;
      n = speaker_stream_write(s_synth_buf, want);
      if (n < want) {
        /* The phase advanced for the whole chunk, but only n bytes were taken.
         * Rewind both accumulators by the dropped samples so the next chunk
         * resumes exactly where playback left off (modular subtraction is exact). */
        uint32_t dropped = (want - n) / BYTES_PER_SAMPLE;
        s_osc[0].phase_acc -= dropped * s_osc[0].phase_inc;
        s_osc[1].phase_acc -= dropped * s_osc[1].phase_inc;
      }
      if (n == 0) break;
      s_bytes_written += n;
      if (n < want) break;
    } else {
      const int16_t *buf = feed_buffer();  /* re-evaluate: busy phase may flip */
      if (!buf) break;
      n = speaker_stream_write(buf, BUFFER_BYTES);
      if (n == 0) break;            /* internal queue full — try again next tick */
      s_bytes_written += n;
      if (n < BUFFER_BYTES) break;  /* partial accept (rare) — keep alignment    */
    }
  }
}

static bool sit_feed(void);  /* fwd decl: returns true once the whole SIT is queued */

static void refill_timer_cb(void *ctx) {
  s_refill_timer = NULL;
  if (!s_stream_open) return;
  if (s_stream_mode == STREAM_SIT) {
    if (sit_feed()) return;  /* sequence fully queued+closed — completion timer finalizes */
  } else {
    feed_stream();
  }
  s_refill_timer = app_timer_register(REFILL_INTERVAL_MS, refill_timer_cb, NULL);
}

/* Tear down any current stream and open a fresh one (single hardware voice).
 * Returns true with the stream open and counters reset; the caller then sets
 * the mode and primes the queue. */
static bool stream_reopen(void) {
  if (s_refill_timer) {
    app_timer_cancel(s_refill_timer);
    s_refill_timer = NULL;
  }
  if (s_stream_open) {
    speaker_stop();  /* immediate: don't drain the old voice's queued audio */
    s_stream_open = false;
  }
  if (!speaker_stream_open(SpeakerPcmFormat_8kHz_16bit, SPEAKER_VOLUME)) {
    s_stream_mode = STREAM_NONE;
    s_stream_buffer = NULL;
    s_stream_key = -1;
    return false;
  }
  s_stream_open = true;
  s_stream_start_ms = now_ms();
  s_bytes_written = 0;
  return true;
}

static void stream_start(const int16_t *buffer, int key) {
  if (!stream_reopen()) return;
  s_stream_mode = STREAM_TONE;
  s_stream_buffer = buffer;
  s_stream_key = key;
  feed_stream();  /* prime the queue */
  s_refill_timer = app_timer_register(REFILL_INTERVAL_MS, refill_timer_cb, NULL);
}

/* Start the busy signal. feed_buffer() supplies the alternating tone/silence
 * buffers from the write position, so there is no fixed s_stream_buffer. */
static void busy_start(void) {
  if (!stream_reopen()) return;
  s_stream_mode = STREAM_BUSY;
  s_stream_buffer = NULL;
  s_stream_key = -1;
  feed_stream();  /* prime the queue */
  s_refill_timer = app_timer_register(REFILL_INTERVAL_MS, refill_timer_cb, NULL);
}

/* Queue the SIT sequence, writing exact per-phase sample counts (the 30ms gaps
 * are shorter than one buffer, so whole-buffer writes won't do). Returns true
 * once all SIT_TOTAL_SAMPLES are queued and the stream has been closed to drain
 * to completion. Robust to partial accepts: the phase is recomputed each write
 * from the absolute write position. */
static bool sit_feed(void) {
  if (!s_stream_open) return true;
  const uint32_t total_bytes = (uint32_t)SIT_TOTAL_SAMPLES * BYTES_PER_SAMPLE;
  uint32_t consumed = (now_ms() - s_stream_start_ms) * BYTES_PER_MS;
  uint32_t target = consumed + (uint32_t)TARGET_LEAD_MS * BYTES_PER_MS;
  if (target > total_bytes) target = total_bytes;  /* never write past the sequence end */

  int guard = 0;
  while (s_bytes_written < target && guard++ < 64) {
    uint32_t samples = s_bytes_written / BYTES_PER_SAMPLE;
    /* Locate the current phase and how many samples remain within it. */
    const int16_t *buf = NULL;
    uint32_t remaining = 0;
    uint32_t pos = 0;
    for (int i = 0; i < SIT_PHASE_COUNT; i++) {
      uint32_t end = pos + SIT_SEQUENCE[i].samples;
      if (samples < end) { buf = SIT_SEQUENCE[i].buffer; remaining = end - samples; break; }
      pos = end;
    }
    if (!buf) break;
    uint32_t chunk = remaining < TONE_BUFFER_SAMPLES ? remaining : TONE_BUFFER_SAMPLES;
    uint32_t want = chunk * BYTES_PER_SAMPLE;
    uint32_t n = speaker_stream_write(buf, want);
    if (n == 0) break;            /* queue full — resume next tick */
    s_bytes_written += n;
    if (n < want) break;          /* partial accept — recompute next tick */
  }

  if (s_bytes_written >= total_bytes && !s_sit_closed) {
    s_sit_closed = true;
    speaker_stream_close();  /* Phase 5: let the queued audio drain, then stop */
    return true;
  }
  return false;
}

/* Finalize the SIT: cancel timers, stop/teardown the stream, clear flags. */
static void sit_finish(void) {
  if (s_refill_timer) { app_timer_cancel(s_refill_timer); s_refill_timer = NULL; }
  if (s_sit_timer)    { app_timer_cancel(s_sit_timer);    s_sit_timer = NULL; }
  if (s_stream_open) { speaker_stop(); s_stream_open = false; }
  s_stream_mode = STREAM_NONE;
  s_stream_buffer = NULL;
  s_stream_key = -1;
  s_bytes_written = 0;
  s_sit_closed = false;
  s_sit_playing = false;
}

static void sit_complete_cb(void *ctx) {
  s_sit_timer = NULL;
  sit_finish();  /* whole sequence has played — return to idle */
}

/* Play the SIT error sequence once. Non-interruptible: s_sit_playing stays set
 * (so all other input is ignored) until sit_complete_cb fires ~SIT_TOTAL_MS later. */
static void sit_start(void) {
  if (!stream_reopen()) return;
  s_stream_mode = STREAM_SIT;
  s_stream_buffer = NULL;
  s_stream_key = -1;
  s_sit_closed = false;
  s_sit_playing = true;
  s_sit_timer = app_timer_register(SIT_TOTAL_MS + SIT_DRAIN_MARGIN_MS, sit_complete_cb, NULL);
  if (!sit_feed()) {  /* prime; keep refilling only if not already fully queued */
    s_refill_timer = app_timer_register(REFILL_INTERVAL_MS, refill_timer_cb, NULL);
  }
}

static void stream_stop(void) {
  if (s_refill_timer) {
    app_timer_cancel(s_refill_timer);
    s_refill_timer = NULL;
  }
  if (s_stream_open) {
    /* speaker_stop() halts immediately; speaker_stream_close() would instead
     * play out the ~400ms still queued, so a held tone would linger audibly
     * after liftoff. Stop, don't drain. */
    speaker_stop();
    s_stream_open = false;
  }
  s_stream_mode = STREAM_NONE;
  s_stream_buffer = NULL;
  s_stream_key = -1;
  s_bytes_written = 0;
}

/* Start a keypad tone via the real-time synth. The phase accumulators are reset
 * to 0 so each key press starts cleanly from zero amplitude (no key-down click);
 * from there feed_stream() keeps generating continuously while the key is held. */
static void play_dtmf(int key_index) {
  if (!stream_reopen()) return;
  s_stream_mode = STREAM_SYNTH;
  s_stream_buffer = NULL;
  s_stream_key = key_index;
  s_osc[0].phase_acc = 0;
  s_osc[1].phase_acc = 0;
  s_osc[0].phase_inc = s_dtmf_phase_inc[key_index][0];
  s_osc[1].phase_inc = s_dtmf_phase_inc[key_index][1];
  feed_stream();  /* prime the queue with synthesized samples */
  s_refill_timer = app_timer_register(REFILL_INTERVAL_MS, refill_timer_cb, NULL);
}

static void play_dial_tone(void) {
  stream_start(g_dial_tone, DIAL_STREAM_KEY);
}

static void stop_audio(void) {
  stream_stop();
}

/* ------------------------------------------------------------------ */
/* Minimum-duration floor timer                                        */
/* ------------------------------------------------------------------ */
static void min_duration_cb(void *ctx) {
  s_min_duration_timer = NULL;
  if (s_stop_pending) {
    s_stop_pending = false;
    stop_audio();
    s_active_key = -1;
  }
}

/* Clear dial-tone bookkeeping (does not touch the stream itself). */
static void cancel_dial_state(void) {
  s_dial_playing = false;
}

/* Clear key-tone bookkeeping (does not touch the stream itself). */
static void cancel_key_state(void) {
  if (s_min_duration_timer) {
    app_timer_cancel(s_min_duration_timer);
    s_min_duration_timer = NULL;
  }
  s_stop_pending = false;
  s_active_key = -1;
}

/* ------------------------------------------------------------------ */
/* Touch input (Section 6)                                             */
/* ------------------------------------------------------------------ */
static int hit_test_key(int16_t x, int16_t y) {
  if (x < s_left_margin || y < s_top_margin) return -1;
  int col = (x - s_left_margin) / s_col_stride;
  int row = (y - s_top_margin) / s_row_stride;
  if (col < 0 || col >= COLS || row < 0 || row >= ROWS) return -1;
  /* Reject touches in the gap between keys. */
  int x_in_key = (x - s_left_margin) % s_col_stride;
  int y_in_key = (y - s_top_margin) % s_row_stride;
  if (x_in_key >= s_key_w || y_in_key >= s_key_h) return -1;
  return row * COLS + col;
}

static void touch_handler(const TouchEvent *event, void *context) {
  /* A notification/overlay covering the app still leaks touch events through to
   * the keypad beneath it. While we are out of focus, ignore touch entirely so a
   * tap on the notification can't trigger an invisible key. */
  if (!s_app_in_focus) return;
  switch (event->type) {
    case TouchEvent_Touchdown: {
      if (s_busy_playing) break;      /* single voice: busy signal owns the speaker */
      if (s_sit_playing) break;       /* single voice: SIT is playing (non-interruptible) */
      if (s_active_key != -1) break;  /* single voice: ignore new touch */
      int key = hit_test_key(event->x, event->y);
      if (key == -1) break;
      cancel_dial_state();  /* a key tone preempts the dial tone (one voice) */
      play_dtmf(key);
      s_active_key = key;
      s_stop_pending = false;
      s_tone_start_ms = now_ms();
      break;
    }
    case TouchEvent_Liftoff: {
      if (s_active_key == -1) break;
      uint32_t elapsed = now_ms() - s_tone_start_ms;
      if (elapsed < MIN_TONE_MS) {
        /* Enforce the 50 ms floor: defer the stop. */
        s_stop_pending = true;
        if (s_min_duration_timer) app_timer_cancel(s_min_duration_timer);
        s_min_duration_timer =
            app_timer_register(MIN_TONE_MS - elapsed, min_duration_cb, NULL);
      } else {
        stop_audio();
        s_active_key = -1;
      }
      break;
    }
    case TouchEvent_PositionUpdate:
      break;  /* ignored — no drag behavior */
    default:
      break;
  }
}

/* ------------------------------------------------------------------ */
/* Button input (Section 6)                                            */
/* ------------------------------------------------------------------ */
/* Enter (Select) plays the dial tone for as long as it is held. Raw click
 * gives separate press/release callbacks. */
static void select_down_handler(ClickRecognizerRef r, void *ctx) {
  if (s_busy_playing) return;  /* single voice: busy signal owns the speaker */
  if (s_sit_playing) return;   /* single voice: SIT is playing (non-interruptible) */
  if (s_dial_playing) return;
  cancel_key_state();  /* preempt any held key tone (single voice) */
  play_dial_tone();
  s_dial_playing = true;
}

static void select_up_handler(ClickRecognizerRef r, void *ctx) {
  if (!s_dial_playing) return;
  s_dial_playing = false;
  stop_audio();
}

/* Up plays the US busy signal for as long as it is held. It is a separate,
 * mutually-exclusive voice: it does not start if a key tone or the dial tone
 * is already sounding, and while it plays, touch and Select are ignored. */
static void up_down_handler(ClickRecognizerRef r, void *ctx) {
  if (s_busy_playing) return;
  if (s_sit_playing) return;       /* SIT is playing (non-interruptible)            */
  if (s_active_key != -1) return;  /* a DTMF key is held — don't start (one voice) */
  if (s_dial_playing) return;      /* the dial tone is held — don't start          */
  busy_start();
  s_busy_playing = true;
}

static void up_up_handler(ClickRecognizerRef r, void *ctx) {
  if (!s_busy_playing) return;
  s_busy_playing = false;
  stop_audio();  /* speaker_stop() — stops immediately on release */
}

/* Down plays the SIT error sequence once. A single click triggers it; while it
 * plays (s_sit_playing) every input — including another Down press — is ignored,
 * so the sequence always runs to completion. */
static void down_sit_handler(ClickRecognizerRef r, void *ctx) {
  if (s_sit_playing) return;
  if (s_busy_playing) return;       /* busy signal is sounding — ignore (one voice) */
  if (s_active_key != -1) return;   /* a DTMF key tone is sounding — ignore         */
  if (s_dial_playing) return;       /* the dial tone is sounding — ignore           */
  sit_start();
}

static void click_config_provider(void *context) {
  window_raw_click_subscribe(BUTTON_ID_SELECT,
                             select_down_handler, select_up_handler, NULL);
  /* Up = hold-to-play busy signal. Long-click (down+up) gives hold-to-activate;
   * a short delay so a brief tap won't trigger it but a hold feels responsive. */
  window_long_click_subscribe(BUTTON_ID_UP, 100, up_down_handler, up_up_handler);
  /* Down = play the SIT error tone once (single click). */
  window_single_click_subscribe(BUTTON_ID_DOWN, down_sit_handler);
  /* Do NOT subscribe BUTTON_ID_BACK — let PebbleOS handle app exit. */
}

/* ------------------------------------------------------------------ */
/* Rendering (Section 4)                                               */
/* ------------------------------------------------------------------ */
/* Within-key text box heights, tuned for the fonts below. */
#define DIGIT_BOX_H 30  /* FONT_KEY_GOTHIC_28_BOLD (digits) */
#define ALPHA_BOX_H 16  /* FONT_KEY_GOTHIC_14 (sub-letters) */

/* '*' and '#' render small in every stock font (the glyphs are compact), and
 * no stock font sits between the digits and an over-large '#', so BOTH symbols
 * are drawn geometrically. This gives exact sizes — digits < '#' < '*' — and a
 * consistent stroke style. DIGIT_CENTER_DY is the digit ink center measured
 * from the digit box top; it places the symbols on the digits' line. */
#define DIGIT_CENTER_DY  18
#define SYMBOL_STROKE     3

#define STAR_RADIUS      10   /* 6-armed asterisk, ~21px tall (largest) */

#define HASH_HALF_H       8   /* '#' ~19px tall — between digits and old size */
#define HASH_HALF_W       8
#define HASH_VX           4   /* vertical bars at cx ± HASH_VX   */
#define HASH_HY           3   /* horizontal bars at cy ± HASH_HY */

/* Draw a 6-armed asterisk centered at (cx, cy): one vertical line plus two
 * diagonals at ±30° from horizontal. Sized larger than the digits and '#'. */
static void draw_asterisk(GContext *ctx, int cx, int cy, int r) {
  graphics_context_set_stroke_color(ctx, COLOR_KEY_TEXT);
  graphics_context_set_stroke_width(ctx, SYMBOL_STROKE);
  graphics_context_set_antialiased(ctx, true);
  int dx = (r * 866) / 1000;  /* r * cos(30°) */
  int dy = (r * 500) / 1000;  /* r * sin(30°) */
  graphics_draw_line(ctx, GPoint(cx, cy - r), GPoint(cx, cy + r));
  graphics_draw_line(ctx, GPoint(cx - dx, cy - dy), GPoint(cx + dx, cy + dy));
  graphics_draw_line(ctx, GPoint(cx - dx, cy + dy), GPoint(cx + dx, cy - dy));
}

/* Draw a '#' centered at (cx, cy): two vertical bars and two horizontal bars.
 * Slightly larger than the digits, smaller than the asterisk. */
static void draw_hash(GContext *ctx, int cx, int cy) {
  graphics_context_set_stroke_color(ctx, COLOR_KEY_TEXT);
  graphics_context_set_stroke_width(ctx, SYMBOL_STROKE);
  graphics_context_set_antialiased(ctx, true);
  graphics_draw_line(ctx, GPoint(cx - HASH_VX, cy - HASH_HALF_H),
                          GPoint(cx - HASH_VX, cy + HASH_HALF_H));
  graphics_draw_line(ctx, GPoint(cx + HASH_VX, cy - HASH_HALF_H),
                          GPoint(cx + HASH_VX, cy + HASH_HALF_H));
  graphics_draw_line(ctx, GPoint(cx - HASH_HALF_W, cy - HASH_HY),
                          GPoint(cx + HASH_HALF_W, cy - HASH_HY));
  graphics_draw_line(ctx, GPoint(cx - HASH_HALF_W, cy + HASH_HY),
                          GPoint(cx + HASH_HALF_W, cy + HASH_HY));
}

static void draw_key(GContext *ctx, int index) {
  int row = index / COLS;
  int col = index % COLS;
  int x = s_left_margin + col * s_col_stride;
  int y = s_top_margin  + row * s_row_stride;

  /* Border (keycap housing). */
  GRect border = GRect(x, y, s_key_w, s_key_h);
  graphics_context_set_fill_color(ctx, COLOR_KEY_BORDER);
  graphics_fill_rect(ctx, border, 0, GCornerNone);

  /* Face, inset 2px on all sides. */
  GRect face = GRect(x + 2, y + 2, s_key_w - 4, s_key_h - 4);
  graphics_context_set_fill_color(ctx, COLOR_KEY_FACE);
  graphics_fill_rect(ctx, face, 0, GCornerNone);

  const KeyLabel *label = &KEY_LABELS[index];
  graphics_context_set_text_color(ctx, COLOR_KEY_TEXT);

  GFont alpha_font = fonts_get_system_font(FONT_KEY_GOTHIC_14);

  /* Center the [digit][alpha] stack vertically in the key, then place the two
   * boxes according to ALPHAS_ABOVE. The digit box lands at the SAME y on every
   * key — including the letterless 1, *, # — so all digits in a row line up. */
  int top_pad = (s_key_h - (DIGIT_BOX_H + ALPHA_BOX_H)) / 2;
  if (top_pad < 0) top_pad = 0;

  int digit_y, alpha_y;
  if (ALPHAS_ABOVE) {
    alpha_y = y + top_pad;
    digit_y = y + top_pad + ALPHA_BOX_H;
  } else {
    digit_y = y + top_pad;
    alpha_y = y + top_pad + DIGIT_BOX_H;
  }

  /* Symbols are drawn (digits < '#' < '*'); digits use the system font. */
  int cx = x + s_key_w / 2;
  int cy = digit_y + DIGIT_CENTER_DY;
  if (label->main_label[0] == '*') {
    draw_asterisk(ctx, cx, cy, STAR_RADIUS);
  } else if (label->main_label[0] == '#') {
    draw_hash(ctx, cx, cy);
  } else {
    graphics_draw_text(ctx, label->main_label,
                       fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD),
                       GRect(x, digit_y, s_key_w, DIGIT_BOX_H),
                       GTextOverflowModeTrailingEllipsis,
                       GTextAlignmentCenter, NULL);
  }

  if (label->sub_label) {
    graphics_draw_text(ctx, label->sub_label, alpha_font,
                       GRect(x, alpha_y, s_key_w, ALPHA_BOX_H),
                       GTextOverflowModeTrailingEllipsis,
                       GTextAlignmentCenter, NULL);
  }
}

static void canvas_update_proc(Layer *layer, GContext *ctx) {
  GRect bounds = layer_get_bounds(layer);

  graphics_context_set_fill_color(ctx, COLOR_BACKGROUND);
  graphics_fill_rect(ctx, bounds, 0, GCornerNone);

  for (int i = 0; i < KEY_COUNT; i++) {
    draw_key(ctx, i);
  }

  /* Optional one-line status message at the bottom (Section 6). The plan's
   * "muted" message is omitted: the SDK has no speaker-mute query. */
  const char *status = NULL;
  if (!s_touch_enabled) {
    status = "Enable touch in Settings";
  }
  if (status) {
    GRect strip = GRect(0, bounds.size.h - 18, bounds.size.w, 18);
    graphics_context_set_fill_color(ctx, COLOR_BACKGROUND);
    graphics_fill_rect(ctx, strip, 0, GCornerNone);
    graphics_context_set_text_color(ctx, COLOR_STATUS_TEXT);
    graphics_draw_text(ctx, status,
                       fonts_get_system_font(FONT_KEY_GOTHIC_14),
                       GRect(0, bounds.size.h - 18, bounds.size.w, 18),
                       GTextOverflowModeTrailingEllipsis,
                       GTextAlignmentCenter, NULL);
  }
}

/* ------------------------------------------------------------------ */
/* App focus (notifications / system overlays)                         */
/* ------------------------------------------------------------------ */
/* When an overlay (e.g. a notification) covers the app, button events are
 * redirected to it by the OS, but touch events still leak through to the keypad
 * underneath. We respond to focus loss by (a) immediately silencing whatever is
 * playing and (b) clearing s_app_in_focus so touch_handler becomes a no-op until
 * focus returns. Audio is NOT auto-restarted on return — the user makes a fresh
 * touch/press. */
static void focus_will_change(bool in_focus) {
  if (in_focus) return;  /* re-enable is handled in focus_did_change */

  /* Overlay is appearing: stop the stream and quiesce every voice. */
  stop_audio();         /* stops playback + cancels the refill timer + resets stream */
  cancel_key_state();   /* cancels the 50ms floor timer, clears s_active_key         */
  cancel_dial_state();  /* clears s_dial_playing                                       */
  s_busy_playing = false;
  if (s_sit_timer) { app_timer_cancel(s_sit_timer); s_sit_timer = NULL; }
  s_sit_playing = false;
  s_sit_closed = false;

  s_app_in_focus = false;
}

static void focus_did_change(bool in_focus) {
  if (in_focus) {
    s_app_in_focus = true;  /* overlay gone — accept touch again on the next tap */
  }
}

/* ------------------------------------------------------------------ */
/* Window lifecycle (Section 6)                                        */
/* ------------------------------------------------------------------ */
static void main_window_load(Window *window) {
  init_dtmf_phase_increments();  /* precompute per-key phase steps for the synth */

  /* Startup cues: a single vibrate, and turn the backlight on. We use
   * light_enable_interaction() rather than light_enable(true) so the backlight
   * simply lights up and then auto-offs per the user's configured timeout —
   * it is not forced to stay on. */
  vibes_short_pulse();
  light_enable_interaction();

  s_touch_enabled = touch_service_is_enabled();

  Layer *root = window_get_root_layer(window);
  GRect bounds = layer_get_bounds(root);
  compute_layout(bounds);  /* size & center the keypad to the real screen */
  s_canvas = layer_create(bounds);
  layer_set_update_proc(s_canvas, canvas_update_proc);
  layer_add_child(root, s_canvas);

  if (s_touch_enabled) {
    touch_service_subscribe(touch_handler, NULL);
  }

  /* Suspend touch (and silence any in-flight audio) while a notification or other
   * system overlay covers the app. */
  app_focus_service_subscribe_handlers((AppFocusHandlers){
    .will_focus = focus_will_change,
    .did_focus  = focus_did_change,
  });
}

static void main_window_unload(Window *window) {
  app_focus_service_unsubscribe();
  speaker_stop();  /* immediate stop — no audio leak after exit */
  if (s_refill_timer) {
    app_timer_cancel(s_refill_timer);
    s_refill_timer = NULL;
  }
  if (s_min_duration_timer) {
    app_timer_cancel(s_min_duration_timer);
    s_min_duration_timer = NULL;
  }
  if (s_sit_timer) {
    app_timer_cancel(s_sit_timer);
    s_sit_timer = NULL;
  }
  if (s_touch_enabled) {
    touch_service_unsubscribe();
  }
  layer_destroy(s_canvas);
  s_canvas = NULL;
}

/* ------------------------------------------------------------------ */
/* App entry point                                                     */
/* ------------------------------------------------------------------ */
int main(void) {
  s_window = window_create();
  window_set_background_color(s_window, COLOR_BACKGROUND);
  window_set_click_config_provider(s_window, click_config_provider);
  window_set_window_handlers(s_window, (WindowHandlers) {
    .load = main_window_load,
    .unload = main_window_unload,
  });
  window_stack_push(s_window, true);

  app_event_loop();

  window_destroy(s_window);
  return 0;
}
