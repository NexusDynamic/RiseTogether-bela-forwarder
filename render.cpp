// RiseTogether Bela forwarder.
//
// The Bela is the hardware hub of the experiment. Everything it sees or emits
// lands on one clock -- the audio frame counter (which bela runs at 48 kHz)
// -- so a latency between two parties is a frame difference, with no
// cross-device clock involved and nothing to correct for.
//
//   Raspberry Pi ---trigger--> [ Bela ] ---forwarded verbatim---> EEG amp
//                                      ---jittered ~1 Hz timer--> EEG amp
//   iPads (Polly, Pia, ...) ---photodiode---> [ Bela ]
//
// The forwarded trigger is a sample-accurate level mirror of the Pi's line,
// delayed by exactly one block: the PRU writes block k's output word during
// block k+1. The timer trigger is deliberately jittered -- a metronomic pulse
// train would beat against the EEG and inject a correlated artefact -- and both
// its schedule and its RNG seed are logged, so the record stays exact.
//
// A device's photodiode edge can be aligned with the final recorded, collated
// data the ipads record when the photodiode trigger was requested, and then
// when the Bela log is aligned with the coordinator device logs (which uses LSL
// to connect to all of the ipads), the entire sequence can be reconstructed
// referenced to the coordinator's LSL monotonic clock.
//
// LSL lives elsewhere in the stack now. The library, the headers and every line
// of inlet code are kept behind ENABLE_LSL, which is 0.

#include "include/font.h"
#include <Bela.h>

#include "include/linux_i2c.c"
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <include/ssd1306.c>
#include <iomanip>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>

// ---------------------------------------------------------------------------
// Compile-time configuration
// ---------------------------------------------------------------------------

#define ENABLE_LSL 0 // 0 = pins and triggers only; 1 restores the inlet
#define USE_OLED_DISPLAY 0

#if ENABLE_LSL
#include <include/lsl_cpp.h>
#endif

#if USE_OLED_DISPLAY
static const int kOledI2cDev = 1;
#endif

// Digital inputs. `device` names the physical source -- an iPad for a
// photodiode, the Pi for the trigger line -- and is what makes a row in
// edges.csv mean something months later. `active_level` is the logic level that
// means "asserted". `refractory_ms` only drives the `accepted` flag in the log;
// no edge is ever discarded because of it (see the burst guard in render()). It
// is a duration, not a frame count, so it means the same thing whatever rate
// the board comes up at -- gRefractoryFrames[] below holds the conversion, done
// once in setup() against the real digital sample rate.
struct InputPin {
    unsigned int pin;
    const char* role; // "photodiode" | "trigger_in"
    const char* device;
    bool active_level;
    double refractory_ms;
};

// Add a device by adding a row. Two things are indexed by table position rather
// than by pin number -- the gPinStatesAtomic bitfield and gRefractoryFrames[]
// -- so there is a hard limit of 16 rows, checked in setup() along with the
// rest of the map.
static const InputPin kInputPins[] = {
    {0, "photodiode", "Pia", true, 1.0}, // active-HIGH comparator
    {1, "photodiode", "Parsnip", true, 1.0},
    {2, "photodiode", "Polly", true, 1.0},
    {3, "photodiode", "Peter", true, 1.0},
    {4, "photodiode", "Padme", true, 1.0},
    {5, "photodiode", "Patrick", true, 1.0},
    {11, "trigger_in", "rpi", true, 0.0}, // no refractory: never mask a trigger
};
static const size_t kNumInputPins = sizeof(kInputPins) / sizeof(kInputPins[0]);
static const size_t kMaxInputPins = 16;

// Which row above carries the Raspberry Pi trigger. setup() checks that this
// index really does name a "trigger_in" row.
static const size_t kTriggerInIndex = 6;

// Outputs to the EEG amp. These must not also appear in kInputPins.
static const unsigned int kTimerOutPin = 12; // local jittered trigger
static const unsigned int kFwdOutPin = 13;   // level mirror of the Pi trigger
static const bool kOutIdleLevel = false;     // idle LOW, pulse HIGH

// Timer trigger. Interval = period + U(-jitter, +jitter), scheduled from the
// previous *scheduled* frame rather than the emitted one, so the mean rate
// stays exactly 1000/kTimerPeriodMs Hz and quantisation never accumulates.
static const double kTimerPeriodMs = 1000.0;
static const double kTimerJitterMs = 200.0;
static const double kTimerPulseMs = 10.0;

// Both outputs are held idle for this long after the first block: the PRU needs
// a moment to settle, and the startup pin scan below wants a quiet bus.
static const double kStartupHoldMs = 250.0;

// If a pin produces more than this many edges inside one refractory window,
// stop emitting rows for it until it has been stable for a full refractory
// period. Suppressed edges are counted and carried on the next emitted row, so
// the record stays lossless in aggregate while staying bounded in the worst
// case.
static const uint32_t kMaxEdgesPerBurst = 64;

#if ENABLE_LSL
static const char* kStreamPrefixFilter = "LSLTest";
static const double kPullTimeoutSec = 0.05; // blocking pull; see below
static const double kTimeCorrIntervalSec = 1.0;
static const double kTimeCorrTimeoutSec = 0.2; // fast after the first call
static const double kTimeCorrFirstTimeoutSec = 5.0;
static const int32_t kInletBufLenSec = 10;
static const int32_t kInletChunkLen = 1; // no chunking: minimum latency
static const bool kInletRecover = true;
#endif

// Fallback only, for the window in which context->digitalSampleRate has not
// been read yet; setup() overwrites gSampleRate with what the board reports.
// Bela runs at 44100 on most capes and 48000 on the CTAG ones, so nothing here
// may assume either.
static const double kFallbackFs = 44100.0;
static const unsigned int kPeriodSize = 16;
static const size_t kMaxChannels = 8;
static const size_t kMaxIdLen = 64;

// Resolved once in setup() from context->digitalSampleRate, then read-only for
// the rest of the run, so no synchronisation is needed: setup() completes
// before render() or any worker thread starts.
static double gSampleRate = kFallbackFs;
static uint64_t gRefractoryFrames[kMaxInputPins];
static unsigned int gSyncPeriodBlocks = 1;

// Timer-trigger geometry, resolved the same way against the reported rate.
static uint64_t gTimerPeriodFrames = 0;
static uint64_t gTimerJitterFrames = 0;
static uint64_t gTimerPulseFrames = 0;
static uint64_t gStartupHoldFrames = 0;

// xorshift32. std::mt19937 allocates and is far too heavy to call from the RT
// thread; this is three shifts and a modulo. The seed is logged, so a session's
// entire trigger schedule can be regenerated offline.
static uint32_t gRngSeed = 0;
static uint32_t gRngState = 0;

static inline int64_t nextJitterFrames() {
    gRngState ^= gRngState << 13;
    gRngState ^= gRngState >> 17;
    gRngState ^= gRngState << 5;
    if (gTimerJitterFrames == 0)
        return 0;
    const uint32_t span = static_cast<uint32_t>(2 * gTimerJitterFrames + 1);
    return static_cast<int64_t>(gRngState % span) -
           static_cast<int64_t>(gTimerJitterFrames);
}

// Queue depths. Drained every 20 ms by the logger thread.
static const size_t kEdgeQueueSize = 16384;
static const size_t kTriggerQueueSize = 4096;
static const size_t kStatusQueueSize = 1024;
static const size_t kSyncQueueSize = 1024;
static const size_t kLslQueueSize = 2048;
static const size_t kCorrQueueSize = 1024;

static const unsigned int kLoggerPollUs = 20000; // 20 ms
static const double kFlushIntervalS = 1.0;       // bound data loss on hard kill
static const unsigned int kDisplayEveryNPolls = 10; // ~200 ms

// ---------------------------------------------------------------------------
// Event records
// ---------------------------------------------------------------------------

struct EdgeEvent {
    uint64_t frame;          // absolute audio frame of the digital sample
    uint64_t edge_index;     // per-pin monotonic counter (counts ALL edges)
    uint64_t dt_frames_prev; // frames since the previous edge on this pin
    uint32_t
        suppressed_count; // edges dropped by the burst guard since last row
    uint32_t pin_index;   // index into kInputPins
    uint8_t state;        // raw logic level
    uint8_t accepted;     // 1 if outside this pin's refractory window

    EdgeEvent()
        : frame(0), edge_index(0), dt_frames_prev(0), suppressed_count(0),
          pin_index(0), state(0), accepted(0) {}
};

// One row per level change on an output pin -- the definitive record of what
// the EEG amp actually saw, on the same frame axis as every input edge. A timer
// rising edge carries the jitter drawn for the *next* interval, and how late
// the pulse was if a dropped block delayed it.
enum TriggerSource {
    TRIG_TIMER = 0,
    TRIG_FORWARD,
};

static const char* triggerSourceName(uint32_t s) {
    return s == TRIG_TIMER ? "timer" : "forward";
}

struct TriggerEvent {
    uint64_t frame;
    uint64_t seq;          // per-source counter; rising and falling both
    int64_t jitter_frames; // signed; timer rising edges only
    int64_t late_frames;   // > 0 only when a block gap delayed the pulse
    uint32_t source;
    uint8_t level;

    TriggerEvent()
        : frame(0), seq(0), jitter_frames(0), late_frames(0),
          source(TRIG_TIMER), level(0) {}
};

enum StatusCode {
    ST_SESSION_START = 0,
    ST_SESSION_END,
    ST_XRUN,
    ST_BLOCK_GAP,
    ST_QUEUE_FULL,
    ST_STREAM_OPEN,
    ST_STREAM_LOST,
    ST_CLOCK_RESET,
    ST_INFO,
};

static const char* statusName(int code) {
    switch (code) {
    case ST_SESSION_START:
        return "SESSION_START";
    case ST_SESSION_END:
        return "SESSION_END";
    case ST_XRUN:
        return "XRUN";
    case ST_BLOCK_GAP:
        return "BLOCK_GAP";
    case ST_QUEUE_FULL:
        return "QUEUE_FULL";
    case ST_STREAM_OPEN:
        return "STREAM_OPEN";
    case ST_STREAM_LOST:
        return "STREAM_LOST";
    case ST_CLOCK_RESET:
        return "CLOCK_RESET";
    default:
        return "INFO";
    }
}

struct StatusEvent {
    uint64_t frame;
    double lsl_clock; // 0.0 when pushed from RT (no clock calls there)
    int64_t detail_num;
    int32_t code;
    char detail_str[96]; // only ever filled from non-RT threads

    StatusEvent() : frame(0), lsl_clock(0.0), detail_num(0), code(ST_INFO) {
        detail_str[0] = '\0';
    }
};

struct SyncEvent {
    uint64_t frame_req;    // frame stored by render() just before scheduling
    uint64_t frame_latest; // latest frame render() has reported, read here
    double lsl_clock;
    double monotonic_clock;

    SyncEvent()
        : frame_req(0), frame_latest(0), lsl_clock(0.0), monotonic_clock(0.0) {}
};

struct LslEvent {
    uint64_t rx_index;
    double lsl_timestamp;     // raw sender clock (post_none)
    double arrival_lsl_clock; // Bela clock, captured immediately after pull
    uint64_t arrival_frame;   // Bela frame axis, +/- one block
    uint32_t samples_available_before; // 0 => clean, backlog-free arrival
    uint32_t n_channels;
    double ch[kMaxChannels];
    char source_id[kMaxIdLen];
    char stream_name[kMaxIdLen];

    LslEvent()
        : rx_index(0), lsl_timestamp(0.0), arrival_lsl_clock(0.0),
          arrival_frame(0), samples_available_before(0), n_channels(0) {
        memset(ch, 0, sizeof(ch));
        source_id[0] = '\0';
        stream_name[0] = '\0';
    }
};

struct TimeCorrEvent {
    double lsl_clock;
    double correction;
    double remote_time;
    double uncertainty;
    uint8_t clock_reset;
    uint8_t ok;
    char source_id[kMaxIdLen];

    TimeCorrEvent()
        : lsl_clock(0.0), correction(0.0), remote_time(0.0), uncertainty(0.0),
          clock_reset(0), ok(0) {
        source_id[0] = '\0';
    }
};

// ---------------------------------------------------------------------------
// Lock-free SPSC queue (one producer, one consumer; logger is sole consumer).
// Safe across the Xenomai/Linux boundary: shared memory + C++ atomics only,
// no blocking primitives, and Bela mlockall()s the process.
// ---------------------------------------------------------------------------

template <typename T, size_t Size> class LockFreeSPSCQueue {
  private:
    alignas(64) std::atomic<size_t> write_pos{0};
    alignas(64) std::atomic<size_t> read_pos{0};
    T buffer[Size];

  public:
    bool push(const T& item) {
        size_t current_write = write_pos.load(std::memory_order_relaxed);
        size_t next_write = (current_write + 1) % Size;
        if (next_write == read_pos.load(std::memory_order_acquire))
            return false;
        buffer[current_write] = item;
        write_pos.store(next_write, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t current_read = read_pos.load(std::memory_order_relaxed);
        if (current_read == write_pos.load(std::memory_order_acquire))
            return false;
        item = buffer[current_read];
        read_pos.store((current_read + 1) % Size, std::memory_order_release);
        return true;
    }

    size_t size_approx() const {
        size_t w = write_pos.load(std::memory_order_relaxed);
        size_t r = read_pos.load(std::memory_order_relaxed);
        return (w >= r) ? (w - r) : (Size - r + w);
    }
};

static LockFreeSPSCQueue<EdgeEvent, kEdgeQueueSize> gEdgeQueue;
static LockFreeSPSCQueue<TriggerEvent, kTriggerQueueSize> gTriggerQueue;
static LockFreeSPSCQueue<StatusEvent, kStatusQueueSize> gStatusQueueRT;
static LockFreeSPSCQueue<SyncEvent, kSyncQueueSize> gSyncQueue;
#if ENABLE_LSL
static LockFreeSPSCQueue<LslEvent, kLslQueueSize> gLslQueue;
static LockFreeSPSCQueue<TimeCorrEvent, kCorrQueueSize> gCorrQueue;
#endif

// Status records originate from several threads (render, LSL, setup/cleanup),
// which would break the single-producer contract above. The RT path keeps its
// own lock-free queue; every non-RT producer uses this mutex-guarded vector,
// which is safe because none of those threads are real-time.
static std::mutex gStatusMutex;
static std::vector<StatusEvent> gStatusPending;

// Push a status record from a non-RT thread.
static void pushStatus(int code, uint64_t frame, double lsl_clock,
                       int64_t detail_num, const char* detail_str) {
    StatusEvent e;
    e.code = code;
    e.frame = frame;
    e.lsl_clock = lsl_clock;
    e.detail_num = detail_num;
    if (detail_str) {
        strncpy(e.detail_str, detail_str, sizeof(e.detail_str) - 1);
        e.detail_str[sizeof(e.detail_str) - 1] = '\0';
    }
    std::lock_guard<std::mutex> lock(gStatusMutex);
    gStatusPending.push_back(e);
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

// Latest frame render() has finished processing. This is the *end* of the most
// recent block: the PRU has already sampled digital input for that whole block
// by the time render() runs, so blockStart + audioFrames is the best estimate
// of "hardware now".
static std::atomic<uint64_t> gElapsedFramesAtomic{0};

// Frame stored by render() immediately before scheduling the clock-sync task.
// The gap between this and gElapsedFramesAtomic when the task actually runs
// bounds the task's scheduling latency -- logged as bracket_frames.
static std::atomic<uint64_t> gSyncRequestFrame{0};

static std::atomic<bool> gRunning{true};       // producers (LSL thread)
static std::atomic<bool> gLoggerRunning{true}; // consumer (logger thread)
static std::atomic<uint16_t> gPinStatesAtomic{0};
static std::atomic<uint64_t> gDroppedEvents{0};

static AuxiliaryTask gClockSyncTask;

// Session paths, fixed at startup.
static std::string gSessionDir;
static std::string gStem;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double monotonicNow() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) +
           1e-9 * static_cast<double>(ts.tv_nsec);
}

static double nowClock() {
#if ENABLE_LSL
    return lsl::local_clock();
#else
    return monotonicNow();
#endif
}

static std::string csvEscape(const std::string& in) {
    bool needs = in.find_first_of(",\"\n\r") != std::string::npos;
    if (!needs)
        return in;
    std::string out = "\"";
    for (char c : in) {
        if (c == '"')
            out += "\"\"";
        else if (c == '\n' || c == '\r')
            out += ' ';
        else
            out += c;
    }
    out += "\"";
    return out;
}

#if ENABLE_LSL
static std::string jsonEscape(const std::string& in) {
    std::string out = "\"";
    for (char c : in) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (c == '\n' || c == '\r' || c == '\t')
            out += ' ';
        else
            out += c;
    }
    out += "\"";
    return out;
}
#endif // ENABLE_LSL

// Ten significant digits after the point: LSL clocks are seconds since an
// arbitrary recent epoch, so this preserves sub-nanosecond resolution.
static std::string fmtClock(double v) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(9) << v;
    return os.str();
}

static bool makeSessionDir() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    time_t t = tv.tv_sec;
    struct tm tmv;
    localtime_r(&t, &tmv);

    char stamp[32];
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmv);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1000, 9999);

    mkdir("./logs", 0755); // fine if it already exists
    gStem = std::string(stamp) + "_" + std::to_string(dis(gen));
    gSessionDir = std::string("./logs/") + gStem;
    if (mkdir(gSessionDir.c_str(), 0755) != 0 && errno != EEXIST) {
        rt_printf("Error: could not create session directory %s\n",
                  gSessionDir.c_str());
        return false;
    }
    return true;
}

static std::string sessionPath(const char* suffix) {
    return gSessionDir + "/" + gStem + suffix;
}

// ---------------------------------------------------------------------------
// Real-time path
// ---------------------------------------------------------------------------

struct PinRT {
    bool prev_state;
    bool initialised;
    uint64_t last_edge_frame;
    uint64_t last_accepted_frame;
    uint64_t edge_index;
    uint32_t burst_count;
    uint32_t suppressed_count;

    PinRT()
        : prev_state(false), initialised(false), last_edge_frame(0),
          last_accepted_frame(0), edge_index(0), burst_count(0),
          suppressed_count(0) {}
};

static PinRT gPinRT[kNumInputPins];

// RT-safe status push. No string formatting, no clock call.
static void pushStatusRT(int code, uint64_t frame, int64_t detail_num) {
    StatusEvent e;
    e.code = code;
    e.frame = frame;
    e.lsl_clock = 0.0;
    e.detail_num = detail_num;
    if (!gStatusQueueRT.push(e)) {
        gDroppedEvents.fetch_add(1, std::memory_order_relaxed);
    }
}

// RT-safe trigger push. render() is the sole producer, which is what preserves
// the queue's single-producer contract.
static void pushTriggerRT(uint32_t source, uint64_t frame, uint64_t seq,
                          bool level, int64_t jitter_frames,
                          int64_t late_frames) {
    TriggerEvent e;
    e.source = source;
    e.frame = frame;
    e.seq = seq;
    e.level = level ? 1 : 0;
    e.jitter_frames = jitter_frames;
    e.late_frames = late_frames;
    if (!gTriggerQueue.push(e)) {
        gDroppedEvents.fetch_add(1, std::memory_order_relaxed);
    }
}

void clockSyncTask(void*) {
    SyncEvent e;
    e.frame_req = gSyncRequestFrame.load(std::memory_order_relaxed);
    // Read the clocks first, then the frame counter, so frame_latest is never
    // earlier than the instant the clocks were read -- keeps the bracket a true
    // upper bound on staleness.
    e.lsl_clock = nowClock();
    e.monotonic_clock = monotonicNow();
    e.frame_latest = gElapsedFramesAtomic.load(std::memory_order_relaxed);
    if (!gSyncQueue.push(e)) {
        gDroppedEvents.fetch_add(1, std::memory_order_relaxed);
    }
}

// Survey of every digital pin over the first 100 ms of running, reported once.
// digitalRead() is only meaningful once the audio thread has sampled a block,
// so this cannot live in setup(); and one block is 16 frames (0.33 ms), which
// is far too short a window to tell an idle level from a signal, and is also
// the block most likely to be garbage if the PRU has not settled. Every pin is
// reported, watched or not, so a signal on a pin nobody is listening to shows
// up as such instead of looking like a dead sensor -- which is the usual
// symptom of a pin map that does not match the wiring.
static const unsigned int kScanFrames = 4800; // 100 ms at 44.1-48 kHz

struct PinScan {
    unsigned int high_frames;
    unsigned int edges;
    bool first_state;
    bool prev_state;
};

static PinScan gScan[16];
static unsigned int gScanFrames = 0;
static unsigned int gScanBlocks = 0;
static bool gScanDone = false;

static void reportDigitalPinScan(BelaContext* context, unsigned int nPins) {
    rt_printf("--- digital pin scan: %u frames (%.1f ms) over %u blocks ---\n",
              gScanFrames, 1000.0 * gScanFrames / context->digitalSampleRate,
              gScanBlocks);
    rt_printf("pin  first  now   high%%  edges  role\n");

    unsigned int totalEdges = 0;
    unsigned int totalHigh = 0;

    for (unsigned int pin = 0; pin < nPins; pin++) {
        const PinScan& c = gScan[pin];
        totalEdges += c.edges;
        totalHigh += c.high_frames;

        char role[48];
        snprintf(role, sizeof(role), "(unwatched)");
        for (size_t p = 0; p < kNumInputPins; p++) {
            if (kInputPins[p].pin == pin) {
                snprintf(role, sizeof(role), "%s / %s", kInputPins[p].role,
                         kInputPins[p].device);
                break;
            }
        }
        // Our own outputs read back through the same word, so the level shown
        // for them is what we are driving, not a measurement of anything.
        if (pin == kFwdOutPin)
            snprintf(role, sizeof(role), "OUT trigger_fwd (we drive this)");
        else if (pin == kTimerOutPin)
            snprintf(role, sizeof(role), "OUT trigger_timer (we drive this)");

        rt_printf("%3u  %-5s  %-5s %5u  %5u  %s\n", pin,
                  c.first_state ? "HIGH" : "LOW", c.prev_state ? "HIGH" : "LOW",
                  (100 * c.high_frames) / gScanFrames, c.edges, role);
    }

    // Every pin reading a flat zero for 100 ms is far more likely to mean the
    // PRU never filled the digital buffer than to mean every input really is
    // grounded
    // -- an unconnected Bela input floats and an idle active-LOW comparator
    // sits HIGH, so a true all-LOW reading takes deliberate wiring.
    if (totalHigh == 0 && totalEdges == 0) {
        rt_printf(
            "*** All %u pins read LOW for the whole window with no edges. "
            "That is the signature of a\n",
            nPins);
        rt_printf("*** digital buffer the PRU never wrote, not of a wiring "
                  "fault. Check for 'PRU interrupt\n");
        rt_printf("*** timeout' or 'McASP error' above before reading anything "
                  "into these levels.\n");
    }
    rt_printf("--- end pin scan ---\n");
}

// Accumulate one block into the scan; prints and latches off once the window is
// full. Called every block until then.
static void updateDigitalPinScan(BelaContext* context) {
    if (gScanDone)
        return;

    const unsigned int nPins =
        context->digitalChannels < 16 ? context->digitalChannels : 16;

    if (nPins == 0 || context->digitalFrames == 0) {
        gScanDone = true;
        rt_printf("*** NO DIGITAL I/O: digitalChannels=%u digitalFrames=%u "
                  "digitalSampleRate=%.0f\n",
                  context->digitalChannels, context->digitalFrames,
                  context->digitalSampleRate);
        rt_printf("*** digitalRead() cannot return anything and no edge will "
                  "ever be logged. Run line needs --use-digital 1.\n");
        return;
    }

    for (unsigned int n = 0; n < context->digitalFrames; n++) {
        for (unsigned int pin = 0; pin < nPins; pin++) {
            const bool state = digitalRead(context, n, pin);
            PinScan& c = gScan[pin];

            if (gScanFrames == 0) {
                c.first_state = state;
                c.prev_state = state;
                c.high_frames = 0;
                c.edges = 0;
            } else if (state != c.prev_state) {
                c.edges++;
                c.prev_state = state;
            }
            if (state)
                c.high_frames++;
        }
        gScanFrames++;
        if (gScanFrames >= kScanFrames)
            break;
    }
    gScanBlocks++;

    if (gScanFrames >= kScanFrames) {
        gScanDone = true;
        reportDigitalPinScan(context, nPins);
    }
}

void render(BelaContext* context, void* userData) {
    const uint64_t blockStart = context->audioFramesElapsed;
    const uint64_t blockEnd = blockStart + context->audioFrames;

    // --- block continuity: a dropped block means digital frames were never
    // read, so an input edge could be missing entirely and an output pulse
    // could have gone out late. Without this you cannot tell "the iPad never
    // flashed" from "the Bela missed it".
    static uint64_t sExpectedFrame = 0;
    static bool sFirstBlock = true;
    static unsigned int sLastUnderrunCount = 0;

    if (sFirstBlock) {
        sFirstBlock = false;
        sLastUnderrunCount = context->underrunCount;
    } else {
        if (blockStart != sExpectedFrame) {
            pushStatusRT(ST_BLOCK_GAP, blockStart,
                         static_cast<int64_t>(blockStart) -
                             static_cast<int64_t>(sExpectedFrame));
        }
        if (context->underrunCount != sLastUnderrunCount) {
            pushStatusRT(ST_XRUN, blockStart,
                         static_cast<int64_t>(context->underrunCount));
            sLastUnderrunCount = context->underrunCount;
        }
    }
    sExpectedFrame = blockEnd;

    updateDigitalPinScan(context);

    // Re-assert pin direction every block. Directions persist, so this is
    // normally a no-op -- but it costs a handful of bit operations and
    // guarantees that a stray setting cannot silently corrupt a reading or
    // leave an output undriven. Direction lives in bits 0-15 and values in bits
    // 16-31, so this cannot disturb what digitalRead() sees.
    for (size_t p = 0; p < kNumInputPins; p++) {
        pinMode(context, 0, kInputPins[p].pin, INPUT);
    }
    pinMode(context, 0, kFwdOutPin, OUTPUT);
    pinMode(context, 0, kTimerOutPin, OUTPUT);

    // Output state. Bela's digital word is bidirectional and the PRU refills it
    // with input readings every block, so an output level does NOT persist by
    // itself -- it has to be re-driven from frame 0 of every block.
    // digitalWrite() covers frame n to the end of the block, so writing the
    // held level at frame 0 and again on each change reproduces the waveform
    // exactly while touching the word only when something happens.
    static bool sFwdLevel = kOutIdleLevel;
    static bool sTimerLevel = kOutIdleLevel;
    static uint64_t sFwdSeq = 0;
    static uint64_t sTimerSeq = 0;

    // Both outputs stay idle until the startup hold expires: the PRU's first
    // blocks are the ones most likely to be garbage, and a garbage transition
    // forwarded to the EEG amp is an event in the recording that never
    // happened. Once armed, sNextOnFrame advances by period + jitter from its
    // own previous value, never from the frame a pulse actually landed on, so
    // quantisation cannot accumulate.
    static bool sOutputsArmed = false;
    static uint64_t sNextOnFrame = 0;
    static uint64_t sOffFrame = 0;

    if (!sOutputsArmed && blockStart >= gStartupHoldFrames) {
        sOutputsArmed = true;
        // Adopt the Pi line's current level rather than waiting for its next
        // edge, so the mirror is correct from the first armed frame even if the
        // trigger line happens to be asserted right now.
        sFwdLevel = gPinRT[kTriggerInIndex].initialised
                        ? gPinRT[kTriggerInIndex].prev_state
                        : kOutIdleLevel;
        sNextOnFrame = blockStart + gTimerPeriodFrames;
        pushStatusRT(ST_INFO, blockStart, static_cast<int64_t>(sNextOnFrame));
    }

    digitalWrite(context, 0, kFwdOutPin, sFwdLevel);
    digitalWrite(context, 0, kTimerOutPin, sTimerLevel);

    uint16_t states = gPinStatesAtomic.load(std::memory_order_relaxed);
    bool statesChanged = false;

    for (unsigned int n = 0; n < context->digitalFrames; n++) {
        const uint64_t frame = blockStart + n;

        for (size_t p = 0; p < kNumInputPins; p++) {
            const bool state = digitalRead(context, n, kInputPins[p].pin);
            PinRT& s = gPinRT[p];

            if (!s.initialised) {
                s.initialised = true;
                s.prev_state = state;
                s.last_edge_frame = frame;
                s.last_accepted_frame = frame;
                if (state)
                    states |= (1 << p);
                else
                    states &= ~(1 << p);
                statesChanged = true;
                continue;
            }

            if (state == s.prev_state)
                continue;

            const uint64_t dt = frame - s.last_edge_frame;
            s.prev_state = state;
            s.last_edge_frame = frame;
            s.edge_index++;

            if (state)
                states |= (1 << p);
            else
                states &= ~(1 << p);
            statesChanged = true;

            // Forward the Pi's line to the EEG amp first. The mirror is the one
            // thing in this loop with a hard latency budget, and it must not
            // sit behind the logging bookkeeping -- nor behind the refractory
            // guard below, which may decide not to emit a row but must never
            // decide not to pass a trigger on.
            if (p == kTriggerInIndex && sOutputsArmed) {
                sFwdLevel = state;
                digitalWrite(context, n, kFwdOutPin, sFwdLevel);
                pushTriggerRT(TRIG_FORWARD, frame, sFwdSeq++, state, 0, 0);
            }

            const bool accepted =
                (frame - s.last_accepted_frame) >= gRefractoryFrames[p];
            if (accepted) {
                s.last_accepted_frame = frame;
                s.burst_count = 0;
            } else {
                s.burst_count++;
            }

            // Bounded chatter guard: keep counting but stop emitting rows until
            // the pin has been quiet for a full refractory period (which is
            // what makes the next edge `accepted` and resets burst_count).
            if (!accepted && s.burst_count > kMaxEdgesPerBurst) {
                s.suppressed_count++;
                continue;
            }

            EdgeEvent e;
            e.frame = frame;
            e.edge_index = s.edge_index;
            e.dt_frames_prev = dt;
            e.suppressed_count = s.suppressed_count;
            e.pin_index = static_cast<uint32_t>(p);
            e.state = state ? 1 : 0;
            e.accepted = accepted ? 1 : 0;

            if (gEdgeQueue.push(e)) {
                s.suppressed_count = 0;
            } else {
                gDroppedEvents.fetch_add(1, std::memory_order_relaxed);
            }
        }

        // --- jittered timer trigger.
        // `>=` rather than `==` so that a dropped block cannot swallow a pulse;
        // the resulting lateness is logged rather than hidden.
        if (sOutputsArmed) {
            if (!sTimerLevel && frame >= sNextOnFrame) {
                const int64_t late = static_cast<int64_t>(frame - sNextOnFrame);
                sTimerLevel = true;
                digitalWrite(context, n, kTimerOutPin, true);
                sOffFrame = frame + gTimerPulseFrames;

                const int64_t jit = nextJitterFrames();
                sNextOnFrame += gTimerPeriodFrames + jit;
                // After a gap long enough to swallow a whole interval, catch up
                // to the grid rather than firing a burst to make up the count.
                if (sNextOnFrame <= frame)
                    sNextOnFrame = frame + gTimerPeriodFrames;

                pushTriggerRT(TRIG_TIMER, frame, sTimerSeq++, true, jit, late);
            } else if (sTimerLevel && frame >= sOffFrame) {
                sTimerLevel = false;
                digitalWrite(context, n, kTimerOutPin, false);
                pushTriggerRT(TRIG_TIMER, frame, sTimerSeq++, false, 0, 0);
            }
        }

        for (unsigned int j = 0; j < context->audioOutChannels; j++) {
            audioWrite(context, n, j, 0.0f);
        }
    }

    if (statesChanged) {
        gPinStatesAtomic.store(states, std::memory_order_relaxed);
    }

    gElapsedFramesAtomic.store(blockEnd, std::memory_order_relaxed);

    static unsigned int renderCount = 0;
    renderCount++;

    // Clock-sync pairs every ~200 ms.
    if (renderCount % gSyncPeriodBlocks == 0) {
        gSyncRequestFrame.store(blockEnd, std::memory_order_relaxed);
        Bela_scheduleAuxiliaryTask(gClockSyncTask);
    }
}

// ---------------------------------------------------------------------------
// LSL thread
//
// A plain std::thread, not a Bela AuxiliaryTask, for two reasons: it can block
// on the network without stalling anything real-time, and file/socket work here
// causes no Xenomai mode switches.
//
// The pull is *blocking* with a short timeout. pull_sample() then returns
// within microseconds of liblsl handing over the sample, so local_clock() on
// the very next line is a genuine arrival timestamp. Polling non-blocking at
// some interval instead would add that whole interval as quantisation error to
// the number we are trying to measure.
// ---------------------------------------------------------------------------

#if ENABLE_LSL

static std::atomic<bool> gStreamConnected{false};
static std::string gConnectedSourceId;
static std::mutex gConnectedMutex;

static void writeStreamXml(const std::string& xml) {
    std::ofstream f(sessionPath("_stream.xml"),
                    std::ios::out | std::ios::trunc);
    if (f.is_open()) {
        f << xml;
        f.close();
    }
}

static void lslThreadFunc() {
    lsl::continuous_resolver* resolver = nullptr;
    lsl::stream_inlet* inlet = nullptr;

    std::string srcId, streamName;
    uint32_t nChan = 0;
    uint64_t rxIndex = 0;
    std::vector<double> buf;
    double nextCorrTime = 0.0;
    bool firstCorrection = true;

    while (gRunning.load(std::memory_order_relaxed)) {
        // ---- discovery
        // -------------------------------------------------------
        if (!inlet) {
            if (!resolver)
                resolver = new lsl::continuous_resolver(5.0);

            std::vector<lsl::stream_info> results = resolver->results();
            for (size_t i = 0; i < results.size() && !inlet; i++) {
                lsl::stream_info& info = results[i];
                if (info.name().empty())
                    continue;
                if (info.name().find(kStreamPrefixFilter) == std::string::npos)
                    continue;

                // Any numeric format is fine: pull_sample(vector<double>) makes
                // liblsl convert, so the Bela side is immune to Flutter-side
                // channel changes.
                if (info.channel_format() == lsl::cf_string ||
                    info.channel_format() == lsl::cf_undefined) {
                    pushStatus(ST_INFO, 0, nowClock(), info.channel_format(),
                               "skipped non-numeric stream");
                    continue;
                }

                try {
                    inlet = new lsl::stream_inlet(
                        info, kInletBufLenSec, kInletChunkLen, kInletRecover);
                    // Raw sender timestamps. The correction is logged
                    // separately as a time series so it can be fitted offline;
                    // never let liblsl silently smooth or shift the thing being
                    // measured.
                    inlet->set_postprocessing(lsl::post_none);
                    inlet->open_stream(2.0);

                    srcId = info.source_id();
                    streamName = info.name();
                    nChan = static_cast<uint32_t>(info.channel_count());
                    buf.assign(nChan, 0.0);

                    // Full header (includes <desc>, so whatever metadata the
                    // Flutter app attaches -- device model, refresh rate, patch
                    // position -- lands in the session directory verbatim).
                    try {
                        writeStreamXml(inlet->info(5.0).as_xml());
                    } catch (std::exception&) {
                        writeStreamXml(info.as_xml());
                    }

                    {
                        std::lock_guard<std::mutex> lock(gConnectedMutex);
                        gConnectedSourceId = srcId;
                    }
                    gStreamConnected.store(true, std::memory_order_relaxed);
                    pushStatus(ST_STREAM_OPEN, gElapsedFramesAtomic.load(),
                               nowClock(), nChan,
                               (streamName + " / " + srcId).c_str());

                    // Stop resolving. Continuous multicast discovery for the
                    // whole session adds background traffic that perturbs the
                    // very latency being measured.
                    delete resolver;
                    resolver = nullptr;

                    // First call blocks until liblsl's background time-sync
                    // thread has an estimate; every later call just reads the
                    // current one.
                    firstCorrection = true;
                    nextCorrTime = 0.0;
                } catch (std::exception& e) {
                    pushStatus(ST_INFO, 0, nowClock(), 0, e.what());
                    delete inlet;
                    inlet = nullptr;
                }
            }

            if (!inlet) {
                usleep(250000);
                continue;
            }
        }

        // ---- pull
        // ------------------------------------------------------------
        try {
            // A non-zero backlog means the sample was already sitting in
            // liblsl's buffer, so the arrival stamp reflects consumer lag
            // rather than transport. Logged per row; filter on it before
            // computing T4.
            const uint32_t availBefore =
                static_cast<uint32_t>(inlet->samples_available());

            const double ts = inlet->pull_sample(buf, kPullTimeoutSec);

            if (ts != 0.0) {
                const double arrival = lsl::local_clock();
                const uint64_t frame =
                    gElapsedFramesAtomic.load(std::memory_order_relaxed);

                LslEvent e;
                e.rx_index = rxIndex++;
                e.lsl_timestamp = ts;
                e.arrival_lsl_clock = arrival;
                e.arrival_frame = frame;
                e.samples_available_before = availBefore;
                e.n_channels = nChan;
                const size_t nCopy =
                    std::min(static_cast<size_t>(nChan), kMaxChannels);
                for (size_t c = 0; c < nCopy; c++)
                    e.ch[c] = buf[c];
                strncpy(e.source_id, srcId.c_str(), kMaxIdLen - 1);
                e.source_id[kMaxIdLen - 1] = '\0';
                strncpy(e.stream_name, streamName.c_str(), kMaxIdLen - 1);
                e.stream_name[kMaxIdLen - 1] = '\0';

                if (!gLslQueue.push(e)) {
                    gDroppedEvents.fetch_add(1, std::memory_order_relaxed);
                }
            }

            // ---- time correction series
            // ---------------------------------------
            const double now = lsl::local_clock();
            if (now >= nextCorrTime) {
                TimeCorrEvent c;
                c.lsl_clock = now;
                strncpy(c.source_id, srcId.c_str(), kMaxIdLen - 1);
                c.source_id[kMaxIdLen - 1] = '\0';
                try {
                    double remote = 0.0, uncertainty = 0.0;
                    c.correction = inlet->time_correction(
                        &remote, &uncertainty,
                        firstCorrection ? kTimeCorrFirstTimeoutSec
                                        : kTimeCorrTimeoutSec);
                    c.remote_time = remote;
                    c.uncertainty = uncertainty;
                    c.clock_reset = inlet->was_clock_reset() ? 1 : 0;
                    c.ok = 1;
                    firstCorrection = false;
                    if (c.clock_reset) {
                        pushStatus(ST_CLOCK_RESET, gElapsedFramesAtomic.load(),
                                   now, 0, srcId.c_str());
                    }
                } catch (lsl::lost_error&) {
                    throw;
                } catch (std::exception&) {
                    c.ok = 0; // timed out; recorded as a gap rather than
                              // interpolated
                }
                // Every measurement is logged. Averaging on-device would
                // destroy the drift information the offline fit needs.
                if (!gCorrQueue.push(c)) {
                    gDroppedEvents.fetch_add(1, std::memory_order_relaxed);
                }
                nextCorrTime = now + kTimeCorrIntervalSec;
            }
        } catch (lsl::lost_error& e) {
            pushStatus(ST_STREAM_LOST, gElapsedFramesAtomic.load(), nowClock(),
                       0, e.what());
            gStreamConnected.store(false, std::memory_order_relaxed);
            try {
                inlet->close_stream();
            } catch (...) {
            }
            delete inlet;
            inlet = nullptr;
        } catch (std::exception& e) {
            pushStatus(ST_INFO, gElapsedFramesAtomic.load(), nowClock(), 0,
                       e.what());
            usleep(100000);
        }
    }

    if (inlet) {
        try {
            inlet->close_stream();
        } catch (...) {
        }
        delete inlet;
    }
    delete resolver;
}

#endif // ENABLE_LSL

// ---------------------------------------------------------------------------
// Logger thread
//
// Sole consumer of every queue. Plain std::thread on a 20 ms poll, so the RT
// path never touches a mutex or condition variable to signal it. Files are
// opened once and flushed on an interval, rather than reopened per write.
// ---------------------------------------------------------------------------

// Live readout only, maintained by the logger thread. The authoritative
// pairing is done offline against the CSVs, where a device that never flashed
// shows up as a gap instead of being silently attributed to the next trigger.
struct LiveStats {
    uint64_t timerPulses = 0; // rising edges only
    uint64_t fwdPulses = 0;   // rising edges only
    uint64_t xruns = 0;
#if ENABLE_LSL
    uint64_t lslSamples = 0;
    double lastOneWayMs = 0.0;
    double lastCorrection = 0.0;
    bool haveCorrection = false;
#endif
    bool haveFwd = false;
    uint64_t lastFwdFrame = 0;
    uint64_t deviceEdges[kMaxInputPins] = {0};
    bool awaitingFlash[kMaxInputPins] = {false};
    double lastLatencyMs[kMaxInputPins];

    LiveStats() {
        for (size_t i = 0; i < kMaxInputPins; i++)
            lastLatencyMs[i] = -1.0;
    }
};

static LiveStats gStats;

static std::ofstream gEdgeFile, gTriggerFile, gStatusFile, gSyncFile;
#if ENABLE_LSL
static std::ofstream gLslFile, gCorrFile;
#endif

static bool openLogFiles() {
    gEdgeFile.open(sessionPath("_edges.csv"), std::ios::out | std::ios::trunc);
    gTriggerFile.open(sessionPath("_triggers.csv"),
                      std::ios::out | std::ios::trunc);
    gStatusFile.open(sessionPath("_status.csv"),
                     std::ios::out | std::ios::trunc);
    gSyncFile.open(sessionPath("_sync.csv"), std::ios::out | std::ios::trunc);
    if (!gEdgeFile.is_open() || !gTriggerFile.is_open() ||
        !gStatusFile.is_open() || !gSyncFile.is_open())
        return false;

    gEdgeFile << "frame,pin,role,device,state,active,dt_frames_prev,accepted,"
                 "suppressed_count,edge_index\n";
    gTriggerFile << "frame,source,seq,level,jitter_frames,late_frames\n";
    gStatusFile << "frame,lsl_clock,event,detail_num,detail\n";
    gSyncFile << "frame_req,frame_latest,bracket_frames,lsl_clock,"
                 "monotonic_clock\n";

#if ENABLE_LSL
    gLslFile.open(sessionPath("_lsl.csv"), std::ios::out | std::ios::trunc);
    gCorrFile.open(sessionPath("_timecorr.csv"),
                   std::ios::out | std::ios::trunc);
    if (!gLslFile.is_open() || !gCorrFile.is_open())
        return false;

    gLslFile
        << "rx_index,source_id,stream_name,lsl_timestamp,arrival_lsl_clock,"
           "arrival_frame,samples_available_before,n_channels";
    for (size_t c = 0; c < kMaxChannels; c++)
        gLslFile << ",ch" << c;
    gLslFile << "\n";

    gCorrFile << "lsl_clock,source_id,correction,remote_time,uncertainty,"
                 "clock_reset,ok\n";
#endif
    return true;
}

static void closeLogFiles() {
    gEdgeFile.flush();
    gEdgeFile.close();
    gTriggerFile.flush();
    gTriggerFile.close();
    gStatusFile.flush();
    gStatusFile.close();
    gSyncFile.flush();
    gSyncFile.close();
#if ENABLE_LSL
    gLslFile.flush();
    gLslFile.close();
    gCorrFile.flush();
    gCorrFile.close();
#endif
}

static void writeStatusRow(const StatusEvent& e) {
    gStatusFile << e.frame << ',';
    if (e.lsl_clock != 0.0)
        gStatusFile << fmtClock(e.lsl_clock);
    gStatusFile << ',' << statusName(e.code) << ',' << e.detail_num << ','
                << csvEscape(e.detail_str) << '\n';
    if (e.code == ST_XRUN || e.code == ST_BLOCK_GAP)
        gStats.xruns++;
}

// Returns true if anything was written.
static bool drainQueues() {
    bool wrote = false;
    // Triggers before edges, so that within one drain pass a forwarded trigger
    // is already the reference for the photodiode edges that followed it.
    TriggerEvent te;
    while (gTriggerQueue.pop(te)) {
        gTriggerFile << te.frame << ',' << triggerSourceName(te.source) << ','
                     << te.seq << ',' << static_cast<int>(te.level) << ','
                     << te.jitter_frames << ',' << te.late_frames << '\n';
        wrote = true;

        if (te.level) {
            if (te.source == TRIG_TIMER) {
                gStats.timerPulses++;
            } else {
                gStats.fwdPulses++;
                gStats.haveFwd = true;
                gStats.lastFwdFrame = te.frame;
                for (size_t i = 0; i < kNumInputPins; i++) {
                    gStats.awaitingFlash[i] =
                        strcmp(kInputPins[i].role, "photodiode") == 0;
                }
            }
        }
    }

    EdgeEvent ee;
    while (gEdgeQueue.pop(ee)) {
        const InputPin& ip = kInputPins[ee.pin_index];
        const bool active = (ee.state != 0) == ip.active_level;

        gEdgeFile << ee.frame << ',' << ip.pin << ',' << ip.role << ','
                  << csvEscape(ip.device) << ',' << static_cast<int>(ee.state)
                  << ',' << (active ? 1 : 0) << ',' << ee.dt_frames_prev << ','
                  << static_cast<int>(ee.accepted) << ',' << ee.suppressed_count
                  << ',' << ee.edge_index << '\n';
        wrote = true;

        // Count every accepted flash; time only the first one after each
        // forwarded trigger, which is the one that measures display latency.
        if (ee.accepted && active && strcmp(ip.role, "photodiode") == 0) {
            gStats.deviceEdges[ee.pin_index]++;
            if (gStats.haveFwd && gStats.awaitingFlash[ee.pin_index]) {
                gStats.lastLatencyMs[ee.pin_index] =
                    1000.0 *
                    static_cast<double>(ee.frame - gStats.lastFwdFrame) /
                    gSampleRate;
                gStats.awaitingFlash[ee.pin_index] = false;
            }
        }
    }

    StatusEvent se;
    while (gStatusQueueRT.pop(se)) {
        writeStatusRow(se);
        wrote = true;
    }
    {
        std::vector<StatusEvent> pending;
        {
            std::lock_guard<std::mutex> lock(gStatusMutex);
            pending.swap(gStatusPending);
        }
        for (size_t i = 0; i < pending.size(); i++) {
            writeStatusRow(pending[i]);
            wrote = true;
        }
    }

    SyncEvent sy;
    while (gSyncQueue.pop(sy)) {
        gSyncFile << sy.frame_req << ',' << sy.frame_latest << ','
                  << (sy.frame_latest - sy.frame_req) << ','
                  << fmtClock(sy.lsl_clock) << ','
                  << fmtClock(sy.monotonic_clock) << '\n';
        wrote = true;
    }

#if ENABLE_LSL
    LslEvent le;
    while (gLslQueue.pop(le)) {
        gLslFile << le.rx_index << ',' << csvEscape(le.source_id) << ','
                 << csvEscape(le.stream_name) << ','
                 << fmtClock(le.lsl_timestamp) << ','
                 << fmtClock(le.arrival_lsl_clock) << ',' << le.arrival_frame
                 << ',' << le.samples_available_before << ',' << le.n_channels;
        for (size_t c = 0; c < kMaxChannels; c++) {
            gLslFile << ',';
            if (c < le.n_channels)
                gLslFile << fmtClock(le.ch[c]);
        }
        gLslFile << '\n';
        wrote = true;

        gStats.lslSamples++;
        if (gStats.haveCorrection) {
            gStats.lastOneWayMs =
                1000.0 * (le.arrival_lsl_clock -
                          (le.lsl_timestamp + gStats.lastCorrection));
        }
    }

    TimeCorrEvent ce;
    while (gCorrQueue.pop(ce)) {
        gCorrFile << fmtClock(ce.lsl_clock) << ',' << csvEscape(ce.source_id)
                  << ',' << fmtClock(ce.correction) << ','
                  << fmtClock(ce.remote_time) << ',' << fmtClock(ce.uncertainty)
                  << ',' << static_cast<int>(ce.clock_reset) << ','
                  << static_cast<int>(ce.ok) << '\n';
        wrote = true;
        if (ce.ok) {
            gStats.lastCorrection = ce.correction;
            gStats.haveCorrection = true;
        }
    }
#endif
    return wrote;
}

static void flushLogFiles() {
    gEdgeFile.flush();
    gTriggerFile.flush();
    gStatusFile.flush();
    gSyncFile.flush();
#if ENABLE_LSL
    gLslFile.flush();
    gCorrFile.flush();
#endif
}

// ---------------------------------------------------------------------------
// OLED. Driven from the logger thread: the ssd1306 driver does I2C ioctls, so
// it must never run on the RT path (the old code ran it from an AuxiliaryTask,
// which forced a Xenomai mode switch on every update).
// ---------------------------------------------------------------------------

static void updateDisplay() {
#if USE_OLED_DISPLAY
    char line[32];
    const uint16_t states = gPinStatesAtomic.load(std::memory_order_relaxed);

    snprintf(line, sizeof(line), "T:%llu F:%llu",
             (unsigned long long)gStats.timerPulses,
             (unsigned long long)gStats.fwdPulses);
    ssd1306_oled_clear_line(1);
    ssd1306_oled_set_XY(0, 1);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, line);

    // One character per input, upper-case initial of the device name, '*' while
    // asserted -- enough to see at a glance that everything is wired up.
    std::string pins = "P:";
    for (size_t i = 0; i < kNumInputPins; i++) {
        const bool level = ((states >> i) & 1) != 0;
        const bool active = level == kInputPins[i].active_level;
        pins += ' ';
        pins += static_cast<char>(toupper(kInputPins[i].device[0]));
        pins += active ? '*' : '-';
    }
    ssd1306_oled_clear_line(2);
    ssd1306_oled_set_XY(0, 2);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)pins.c_str());

    // One line per photodiode device: last display latency against the most
    // recent forwarded trigger, and how many flashes it has produced.
    unsigned int row = 3;
    for (size_t i = 0; i < kNumInputPins && row <= 5; i++) {
        if (strcmp(kInputPins[i].role, "photodiode") != 0)
            continue;
        if (gStats.lastLatencyMs[i] >= 0.0) {
            snprintf(line, sizeof(line), "%.5s %.1fms n=%llu",
                     kInputPins[i].device, gStats.lastLatencyMs[i],
                     (unsigned long long)gStats.deviceEdges[i]);
        } else {
            snprintf(line, sizeof(line), "%.5s --", kInputPins[i].device);
        }
        ssd1306_oled_clear_line(row);
        ssd1306_oled_set_XY(0, row);
        ssd1306_oled_write_line(SSD1306_FONT_NORMAL, line);
        row++;
    }

    snprintf(
        line, sizeof(line), "!X:%llu D:%llu", (unsigned long long)gStats.xruns,
        (unsigned long long)gDroppedEvents.load(std::memory_order_relaxed));
    ssd1306_oled_clear_line(6);
    ssd1306_oled_set_XY(0, 6);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, line);
#endif
}

static void loggerThreadFunc() {
    double lastFlush = monotonicNow();
    unsigned int polls = 0;

    while (true) {
        const bool running = gLoggerRunning.load(std::memory_order_relaxed);
        const bool wrote = drainQueues();

        const double now = monotonicNow();
        if (now - lastFlush >= kFlushIntervalS) {
            flushLogFiles();
            lastFlush = now;
        }

        if (++polls % kDisplayEveryNPolls == 0)
            updateDisplay();

        // Shutting down: keep draining until a full pass produces nothing.
        if (!running && !wrote)
            break;

        usleep(kLoggerPollUs);
    }

    flushLogFiles();
}

// ---------------------------------------------------------------------------
// Session metadata
//
// Written at startup rather than at shutdown so it survives a hard kill (Bela
// projects are often just stopped). The stream's full XML header lands
// separately in <stem>_stream.xml once the inlet opens.
// ---------------------------------------------------------------------------

static void writeMetaJson(BelaContext* context) {
    std::ofstream f(sessionPath("_meta.json"), std::ios::out | std::ios::trunc);
    if (!f.is_open()) {
        rt_printf("Warning: could not write meta.json\n");
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, nullptr);
    const double wallStart =
        static_cast<double>(tv.tv_sec) + 1e-6 * static_cast<double>(tv.tv_usec);

    char iso[64];
    time_t t = tv.tv_sec;
    struct tm tmv;
    gmtime_r(&t, &tmv);
    strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &tmv);

    f << std::fixed << std::setprecision(9);
    f << "{\n";
    f << "  \"session\": \"" << gStem << "\",\n";
    f << "  \"schema_version\": 2,\n";
    f << "  \"lsl_enabled\": " << (ENABLE_LSL ? "true" : "false") << ",\n";

    // Pins the LSL epoch to wall time. The Bela has no battery-backed RTC, so
    // wall time may be wrong if it has not NTP-synced -- all data timestamps
    // are frames or local_clock(), so this is for orientation only.
    f << "  \"start_wall_unix\": " << wallStart << ",\n";
    f << "  \"start_wall_iso_utc\": \"" << iso << "\",\n";
    f << "  \"start_lsl_clock\": " << nowClock() << ",\n";
    f << "  \"start_monotonic_clock\": " << monotonicNow() << ",\n";

    f << "  \"audio_sample_rate\": " << context->audioSampleRate << ",\n";
    f << "  \"digital_sample_rate\": " << context->digitalSampleRate << ",\n";
    f << "  \"audio_frames_per_block\": " << context->audioFrames << ",\n";
    f << "  \"digital_frames_per_block\": " << context->digitalFrames << ",\n";
    // The rate every frame->seconds conversion in this run was done with. It
    // tracks digital_sample_rate except in the degenerate case where the board
    // reported nothing and the fallback was used.
    f << "  \"resolved_sample_rate\": " << gSampleRate << ",\n";
    f << "  \"sync_period_blocks\": " << gSyncPeriodBlocks << ",\n";
    f << "  \"project_name\": \"" << context->projectName << "\",\n";

#if ENABLE_LSL
    f << "  \"liblsl_library_version\": " << lsl::library_version() << ",\n";
    f << "  \"liblsl_protocol_version\": " << lsl::protocol_version() << ",\n";
    f << "  \"liblsl_header_version\": " << LIBLSL_COMPILE_HEADER_VERSION
      << ",\n";
    f << "  \"liblsl_library_info\": " << jsonEscape(lsl::library_info())
      << ",\n";
    f << "  \"stream_prefix_filter\": \"" << kStreamPrefixFilter << "\",\n";
    f << "  \"inlet_max_buflen_sec\": " << kInletBufLenSec << ",\n";
    f << "  \"inlet_max_chunklen\": " << kInletChunkLen << ",\n";
    f << "  \"inlet_recover\": " << (kInletRecover ? "true" : "false") << ",\n";
    f << "  \"inlet_postprocessing\": \"post_none\",\n";
    f << "  \"pull_timeout_sec\": " << kPullTimeoutSec << ",\n";
    f << "  \"time_correction_interval_sec\": " << kTimeCorrIntervalSec
      << ",\n";
    f << "  \"stream_xml_file\": \"" << gStem << "_stream.xml\",\n";
#endif

    f << "  \"max_edges_per_burst\": " << kMaxEdgesPerBurst << ",\n";

    // The PRU writes block k's output word during block k+1, so every output
    // edge reaches the pin this many frames after the frame it is logged at.
    // Constant, and the same on both output pins, so it cancels out of any
    // comparison between the two -- but not out of a comparison with an input.
    f << "  \"output_block_delay_frames\": " << context->digitalFrames << ",\n";
    f << "  \"startup_hold_frames\": " << gStartupHoldFrames << ",\n";
    f << "  \"timer_period_ms\": " << kTimerPeriodMs << ",\n";
    f << "  \"timer_jitter_ms\": " << kTimerJitterMs << ",\n";
    f << "  \"timer_pulse_ms\": " << kTimerPulseMs << ",\n";
    f << "  \"timer_period_frames\": " << gTimerPeriodFrames << ",\n";
    f << "  \"timer_jitter_frames\": " << gTimerJitterFrames << ",\n";
    f << "  \"timer_pulse_frames\": " << gTimerPulseFrames << ",\n";
    // The whole trigger schedule is reproducible offline from this seed.
    f << "  \"timer_rng\": \"xorshift32\",\n";
    f << "  \"timer_rng_seed\": " << gRngSeed << ",\n";

    f << "  \"output_pins\": [\n";
    f << "    {\"pin\": " << kFwdOutPin << ", \"role\": \"trigger_fwd\""
      << ", \"mirrors_device\": \"" << kInputPins[kTriggerInIndex].device
      << "\", \"mirrors_pin\": " << kInputPins[kTriggerInIndex].pin
      << ", \"idle_level\": " << (kOutIdleLevel ? 1 : 0) << "},\n";
    f << "    {\"pin\": " << kTimerOutPin << ", \"role\": \"trigger_timer\""
      << ", \"idle_level\": " << (kOutIdleLevel ? 1 : 0) << "}\n";
    f << "  ],\n";

    f << "  \"input_pins\": [\n";
    for (size_t i = 0; i < kNumInputPins; i++) {
        f << "    {\"index\": " << i << ", \"pin\": " << kInputPins[i].pin
          << ", \"role\": \"" << kInputPins[i].role << "\", \"device\": \""
          << kInputPins[i].device
          << "\", \"active_level\": " << (kInputPins[i].active_level ? 1 : 0)
          << ", \"refractory_ms\": " << kInputPins[i].refractory_ms
          << ", \"refractory_frames\": " << gRefractoryFrames[i] << "}";
        if (i + 1 < kNumInputPins)
            f << ",";
        f << "\n";
    }
    f << "  ],\n";

    f << "  \"files\": {\n";
    f << "    \"edges\": \"" << gStem << "_edges.csv\",\n";
    f << "    \"triggers\": \"" << gStem << "_triggers.csv\",\n";
    f << "    \"status\": \"" << gStem << "_status.csv\",\n";
    f << "    \"sync\": \"" << gStem << "_sync.csv\"";
#if ENABLE_LSL
    f << ",\n    \"lsl\": \"" << gStem << "_lsl.csv\",\n";
    f << "    \"timecorr\": \"" << gStem << "_timecorr.csv\"";
#endif
    f << "\n  }\n";
    f << "}\n";
    f.close();
}

// ---------------------------------------------------------------------------
// Bela entry points
// ---------------------------------------------------------------------------

static std::thread gLoggerThread;
#if ENABLE_LSL
static std::thread gLslThread;
#endif

bool setup(BelaContext* context, void* userData) {
#if ENABLE_LSL
    // These three are independent counters, not a matched set:
    // library_version() tracks releases, protocol_version() has been pinned at
    // 110 since liblsl 1.10, and LIBLSL_COMPILE_HEADER_VERSION is a header
    // feature level. Report them into the session metadata and leave the
    // judgement to the analysis.
    rt_printf("liblsl %d.%d (wire protocol %d, headers %d)\n",
              lsl::library_version() / 100, lsl::library_version() % 100,
              lsl::protocol_version(), LIBLSL_COMPILE_HEADER_VERSION);
#else
    rt_printf("Forwarder mode; LSL compiled out (ENABLE_LSL=0).\n");
#endif

    if (!makeSessionDir())
        return false;
    rt_printf("Session: %s\n", gSessionDir.c_str());

    // Everything this program does is a digital edge -- read, forwarded or
    // emitted -- so a run without digital I/O produces empty logs, drives
    // nothing, and nothing says why.
    // The usual cause is the run line: Bela parses --use-digital with atoi(),
    // so the plausible-looking "--use-digital yes" evaluates to 0 and silently
    // turns the pins off (as does "--digital-channels 0").
    if (context->digitalChannels == 0 || context->digitalFrames == 0) {
        rt_printf(
            "Error: no digital I/O (channels=%u frames=%u rate=%.0f Hz).\n",
            context->digitalChannels, context->digitalFrames,
            context->digitalSampleRate);
        rt_printf("       The run line needs --use-digital 1 and "
                  "--digital-channels 16; note that these take a NUMBER, and "
                  "\"yes\" parses as 0.\n");
        return false;
    }

    // Resolve everything rate-dependent from what the board actually reports.
    // A CTAG cape brings the McASP up at 48 kHz where a plain Bela cape runs at
    // 44.1 kHz, and a frame count baked in at one rate is silently wrong at the
    // other.
    gSampleRate = context->digitalSampleRate;
    if (!(gSampleRate > 0.0)) {
        rt_printf("Warning: digitalSampleRate is %.0f; assuming %.0f Hz.\n",
                  context->digitalSampleRate, kFallbackFs);
        gSampleRate = kFallbackFs;
    }

    // Clock-sync pairs every ~200 ms, expressed in render() blocks.
    gSyncPeriodBlocks = static_cast<unsigned int>(
        (gSampleRate / static_cast<double>(context->audioFrames)) / 5.0 + 0.5);
    if (gSyncPeriodBlocks < 1)
        gSyncPeriodBlocks = 1;

    rt_printf("Digital rate %.0f Hz; clock-sync every %u blocks (%.0f ms).\n",
              gSampleRate, gSyncPeriodBlocks,
              1000.0 * gSyncPeriodBlocks * context->audioFrames / gSampleRate);

    // Validate the pin map before anything touches it. A duplicated or
    // out-of-range pin gives a session that looks fine and is quietly wrong,
    // which is the worst failure mode there is for a recording rig.
    if (kNumInputPins > kMaxInputPins) {
        rt_printf("Error: %zu input pins, but the state bitfield and "
                  "gRefractoryFrames[] hold at most %zu.\n",
                  kNumInputPins, kMaxInputPins);
        return false;
    }
    if (kTriggerInIndex >= kNumInputPins ||
        strcmp(kInputPins[kTriggerInIndex].role, "trigger_in") != 0) {
        rt_printf("Error: kTriggerInIndex (%zu) does not name a trigger_in row "
                  "in kInputPins.\n",
                  kTriggerInIndex);
        return false;
    }
    {
        bool used[kMaxInputPins] = {false};
        const unsigned int outPins[2] = {kFwdOutPin, kTimerOutPin};
        const unsigned int nUsable = context->digitalChannels < kMaxInputPins
                                         ? context->digitalChannels
                                         : kMaxInputPins;
        for (size_t i = 0; i < kNumInputPins; i++) {
            const unsigned int pin = kInputPins[i].pin;
            if (pin >= nUsable) {
                rt_printf("Error: input pin %u (%s/%s) is outside the %u "
                          "digital channels this run has.\n",
                          pin, kInputPins[i].role, kInputPins[i].device,
                          nUsable);
                return false;
            }
            if (used[pin]) {
                rt_printf("Error: pin %u appears more than once in the pin "
                          "map.\n",
                          pin);
                return false;
            }
            used[pin] = true;
        }
        for (size_t i = 0; i < 2; i++) {
            if (outPins[i] >= nUsable) {
                rt_printf("Error: output pin %u is outside the %u digital "
                          "channels this run has.\n",
                          outPins[i], nUsable);
                return false;
            }
            if (used[outPins[i]]) {
                rt_printf("Error: pin %u is both an input and an output.\n",
                          outPins[i]);
                return false;
            }
            used[outPins[i]] = true;
        }
    }

    for (size_t i = 0; i < kNumInputPins; i++) {
        pinMode(context, 0, kInputPins[i].pin, INPUT);
        gRefractoryFrames[i] = static_cast<uint64_t>(
            kInputPins[i].refractory_ms * gSampleRate / 1000.0 + 0.5);
        rt_printf("  in  pin %2u  %-11s %-8s active=%s  refractory=%.3f ms "
                  "(%llu frames)\n",
                  kInputPins[i].pin, kInputPins[i].role, kInputPins[i].device,
                  kInputPins[i].active_level ? "HIGH" : "LOW",
                  kInputPins[i].refractory_ms,
                  static_cast<unsigned long long>(gRefractoryFrames[i]));
    }

    pinMode(context, 0, kFwdOutPin, OUTPUT);
    pinMode(context, 0, kTimerOutPin, OUTPUT);

    // Timer geometry, resolved against the rate the board actually reports, the
    // same way the refractory windows are.
    gTimerPeriodFrames =
        static_cast<uint64_t>(kTimerPeriodMs * gSampleRate / 1000.0 + 0.5);
    gTimerJitterFrames =
        static_cast<uint64_t>(kTimerJitterMs * gSampleRate / 1000.0 + 0.5);
    gTimerPulseFrames =
        static_cast<uint64_t>(kTimerPulseMs * gSampleRate / 1000.0 + 0.5);
    gStartupHoldFrames =
        static_cast<uint64_t>(kStartupHoldMs * gSampleRate / 1000.0 + 0.5);

    if (gTimerPulseFrames == 0) {
        rt_printf("Error: timer pulse of %.3f ms is shorter than one frame at "
                  "%.0f Hz.\n",
                  kTimerPulseMs, gSampleRate);
        return false;
    }
    // The shortest interval the jitter can draw must still hold a whole pulse
    // and a gap after it, or two pulses merge and the EEG records one long
    // event where there should be two.
    if (gTimerJitterFrames >= gTimerPeriodFrames ||
        gTimerPulseFrames >= gTimerPeriodFrames - gTimerJitterFrames) {
        rt_printf(
            "Error: a %.1f ms pulse does not fit in the shortest interval "
            "(%.1f ms = period %.1f - jitter %.1f).\n",
            kTimerPulseMs, kTimerPeriodMs - kTimerJitterMs, kTimerPeriodMs,
            kTimerJitterMs);
        return false;
    }

    // Seeded from the OS once, then advanced only from render().
    {
        std::random_device rd;
        gRngSeed = static_cast<uint32_t>(rd());
        if (gRngSeed == 0)
            gRngSeed = 0x9e3779b9u; // xorshift32 is stuck at zero
        gRngState = gRngSeed;
    }

    rt_printf("  out pin %2u  trigger_fwd    mirrors %s (pin %u), one block "
              "(%u frames) behind\n",
              kFwdOutPin, kInputPins[kTriggerInIndex].device,
              kInputPins[kTriggerInIndex].pin, context->digitalFrames);
    rt_printf(
        "  out pin %2u  trigger_timer  %.0f ms +/- %.0f ms, %.1f ms pulse "
        "(%llu +/- %llu, %llu frames)\n",
        kTimerOutPin, kTimerPeriodMs, kTimerJitterMs, kTimerPulseMs,
        static_cast<unsigned long long>(gTimerPeriodFrames),
        static_cast<unsigned long long>(gTimerJitterFrames),
        static_cast<unsigned long long>(gTimerPulseFrames));
    rt_printf("  jitter seed %u; outputs idle for the first %.0f ms (%llu "
              "frames).\n",
              gRngSeed, kStartupHoldMs,
              static_cast<unsigned long long>(gStartupHoldFrames));

#if USE_OLED_DISPLAY
    ssd1306_init(kOledI2cDev);
    ssd1306_oled_default_config(64, 128);
    ssd1306_oled_clear_screen();
    ssd1306_oled_set_XY(0, 0);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"RiseTogether fwd");
#endif

    if (!openLogFiles()) {
        rt_printf("Error: could not open log files in %s\n",
                  gSessionDir.c_str());
        return false;
    }
    writeMetaJson(context);

    if ((gClockSyncTask =
             Bela_createAuxiliaryTask(&clockSyncTask, 85, "clock-sync")) == 0)
        return false;

    pushStatus(ST_SESSION_START, 0, nowClock(), 0, gStem.c_str());

    gRunning.store(true, std::memory_order_relaxed);
    gLoggerRunning.store(true, std::memory_order_relaxed);
    gLoggerThread = std::thread(loggerThreadFunc);
#if ENABLE_LSL
    gLslThread = std::thread(lslThreadFunc);
#endif

    rt_printf("Setup complete. %zu inputs, 2 outputs, %.0f Hz digital.\n",
              kNumInputPins, context->digitalSampleRate);
    return true;
}

void cleanup(BelaContext* context, void* userData) {
    rt_printf("Stopping... flushing logs.\n");

    pushStatus(ST_SESSION_END, gElapsedFramesAtomic.load(), nowClock(),
               static_cast<int64_t>(gDroppedEvents.load()), gStem.c_str());

    // render() has already stopped by the time cleanup() runs, so retiring the
    // LSL thread and joining it leaves the logger as the only live thread with
    // nothing more able to arrive behind it.
    gRunning.store(false, std::memory_order_relaxed);
#if ENABLE_LSL
    if (gLslThread.joinable())
        gLslThread.join();
#endif
    gLoggerRunning.store(false, std::memory_order_relaxed);
    if (gLoggerThread.joinable())
        gLoggerThread.join();

    closeLogFiles();

#if USE_OLED_DISPLAY
    ssd1306_oled_clear_screen();
    ssd1306_oled_set_XY(0, 0);
    ssd1306_oled_write_line(SSD1306_FONT_NORMAL, (char*)"Session complete");
    ssd1306_end();
#endif

    const uint64_t dropped = gDroppedEvents.load(std::memory_order_relaxed);
    rt_printf("Done. Session %s, dropped events: %llu\n", gSessionDir.c_str(),
              (unsigned long long)dropped);
    if (dropped > 0) {
        rt_printf("WARNING: %llu events were dropped -- the data has gaps.\n",
                  (unsigned long long)dropped);
    }
}
