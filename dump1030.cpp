// dump1030 - SSR-Abfrage-Detektor (1030 MHz)
// Build: make [WITH_RTLSDR=1] [WITH_PLUTO=0]

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <limits>
#include <memory>
#include <poll.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#ifndef WITH_RTLSDR
#define WITH_RTLSDR 1
#endif
#ifndef WITH_PLUTO
#define WITH_PLUTO 0
#endif
#if WITH_RTLSDR
#include <rtl-sdr.h>
#endif
#if WITH_PLUTO
#include <iio.h>
#endif

// ---------- Konstanten ----------
static constexpr double PI           = 3.14159265358979323846;
static const uint32_t DEFAULT_FREQ   = 1030000000;
static const uint32_t DEFAULT_RATE   = 2500000;
static const uint32_t MIN_RATE       = 2000000;
static const uint32_t MAX_RATE       = 30000000;
static const uint32_t DEFAULT_LEN    = 262144;   // Bytes (I/Q)
static const uint32_t MIN_LEN        = 16384;
static const int      AUTO_GAIN      = -100;
static const int      MAX_GAIN       = 999999;
// Modellraster und P6-Fenster in physikalischer Zeit.
static const double   DPSK_MODEL_RATE = 20'000'000.0;
static constexpr double BD_PREFIX_US = 8.0;
static constexpr double BD_DF_START_US = 3.6;
static constexpr double BD_WINDOW_END_US = 42.0;

// Zeitpunkte in us relativ zur Vorderflanke von P1 (ICAO Annex 10)
namespace us {
    constexpr double PW        = 0.8;   // Breite P1/P2/P3
    constexpr double P2        = 2.0;
    constexpr double P3_A      = 8.0;
    constexpr double P3_C      = 21.0;
    constexpr double P4_GAP    = 2.0;   // P3 -> P4
    constexpr double P4_SHORT  = 0.8;
    constexpr double P4_LONG   = 1.6;
    constexpr double P6        = 3.5;   // Beginn P6
    constexpr double SPR       = 4.75;  // nominaler Sync Phase Reversal
    constexpr double SPR_PRIOR = 4.57;  // gemessener Mittelwert
    constexpr double SPR_MIN   = 3.90;
    constexpr double SPR_MAX   = 4.95;   // Fehl-Locks verhindern
    constexpr double CHIP      = 0.25;  // 4 Mbit/s DPSK
    constexpr double P6_END56  = 3.5 + 16.25;
    constexpr double P6_END112 = 3.5 + 30.25;
}

// Detektionscodes (identisch fuer --order und JSONL-Ausgabe)
enum MsgCode : uint8_t {
    MSG_NONE        = 0,
    MSG_S           = 3,   // Mode S (P1+P2)
    MSG_A           = 11,  // Mode A, kein P4
    MSG_C           = 12,  // Mode C, kein P4
    MSG_A_ACAC      = 21,  // Mode A, kurzer P4 (A/C-only All-Call)
    MSG_C_ACAC      = 22,  // Mode C, kurzer P4
    MSG_A_ACSAC     = 31,  // Mode A, langer P4 (A/C/S All-Call)
    MSG_C_ACSAC     = 32   // Mode C, langer P4
};

// Ratios als Fixed-Point Q8 (r * 256): a/b > r  <=>  a*256 > rq*b
static inline int toQ8(double r) { return (int)lround(r * 256.0); }

struct Thresholds {
    int diff         = 10;   // Mindestabstand Puls/Rauschen (fern)
    int diffclose    = 5;    // Mindestabstand Puls/Rauschen (direkt daneben)
    int ratio        = toQ8(0.25);
    int ratioclose   = toQ8(0.75);
    int ratiop4      = toQ8(0.50);
    int ratioclosep4 = toQ8(1.50);
    int min_peak     = 0;    // mpa
    int max_nf       = 255;  // mnf
    int max_nfc      = 255;  // mnfc
};

struct Counters {
    uint64_t s = 0, a = 0, c = 0;
    uint64_t a_acac = 0, c_acac = 0;
    uint64_t a_acsac = 0, c_acsac = 0;
    uint64_t total() const { return s + a + c + a_acac + c_acac + a_acsac + c_acsac; }
    void add(const Counters &o) {
        s += o.s; a += o.a; c += o.c;
        a_acac += o.a_acac; c_acac += o.c_acac;
        a_acsac += o.a_acsac; c_acsac += o.c_acsac;
    }
    void count(MsgCode m) {
        switch (m) {
            case MSG_S:       s++;       break;
            case MSG_A:       a++;       break;
            case MSG_C:       c++;       break;
            case MSG_A_ACAC:  a_acac++;  break;
            case MSG_C_ACAC:  c_acac++;  break;
            case MSG_A_ACSAC: a_acsac++; break;
            case MSG_C_ACSAC: c_acsac++; break;
            default: break;
        }
    }
};

// Laufende Summen fuer --blmode (statt Vektoren)
struct Average {
    uint64_t sum = 0, n = 0;
    void add(uint8_t v) { sum += v; n++; }
    bool valid() const { return n > 2; }
    unsigned value() const { return n ? (unsigned)(sum / n) : 0; }
};

enum class InputFormat : uint8_t {
    CU8,
    SC16
};

struct SampleBlock {
    InputFormat format = InputFormat::CU8;
    double sample_rate = 0.0;
    std::vector<uint8_t> raw;
    std::vector<uint8_t> magnitude;

    size_t sample_count() const {
        return raw.size() / (format == InputFormat::CU8 ? 2u : 4u);
    }

    std::complex<double> iq_at(size_t sample, double sc16_scale) const {
        if (format == InputFormat::CU8) {
            const uint8_t* p = raw.data() + 2 * sample;
            return {
                static_cast<double>(p[0]) / 127.5 - 1.0,
                static_cast<double>(p[1]) / 127.5 - 1.0
            };
        }
        int16_t i, q;
        const size_t offset = 4 * sample;
        memcpy(&i, raw.data() + offset, sizeof(i));
        memcpy(&q, raw.data() + offset + sizeof(i), sizeof(q));
        return {
            static_cast<double>(i) / sc16_scale,
            static_cast<double>(q) / sc16_scale
        };
    }
};

enum class ReadStatus : uint8_t {
    DATA,
    EOF_REACHED,
    ERROR
};

class SampleSource {
public:
    virtual ~SampleSource() = default;
    virtual bool open() = 0;
    virtual ReadStatus read_block(SampleBlock& out) = 0;
    virtual double sample_rate() const = 0;
    virtual bool live() const = 0;
    virtual void cancel() {}
    virtual void close() = 0;
};

// ---------- Timing (zur Laufzeit aus der Samplerate berechnet) ----------
struct Timing {
    double sps = 2.5;          // Samples pro us
    int pw = 2;                // Pulsbreite (0,8 us) in Samples
    int p2 = 5, p3_a = 20, p3_c = 52;
    int p4_gap = 5, p4_short = 2, p4_long = 4;
    int skip_s56 = 50, skip_s112 = 85;
    int min_skip_s = 0;        // Fallback ohne Decoder
    int edge_step = 1;         // 0,4-us Abstand zu Pulsflanken
    int edge_trim = 0;         // 0,1-us Toleranz an Pulsflanken
    int mode_step = 1;         // sichere Stelle vor P6

    int s(double t_us) const { return (int)lround(t_us * sps); }
    int ceil_s(double t_us) const { return (int)std::ceil(t_us * sps); }
    int floor_s(double t_us) const { return (int)std::floor(t_us * sps); }
    double f(double t_us) const { return t_us * sps; }

    void init(uint32_t rate) {
        sps        = rate / 1e6;
        pw         = std::max(2, s(us::PW));
        p2         = s(us::P2);
        p3_a       = s(us::P3_A);
        p3_c       = s(us::P3_C);
        p4_gap     = s(us::P4_GAP);
        p4_short   = std::max(2, s(us::P4_SHORT));
        p4_long    = std::max(p4_short + 1, s(us::P4_LONG));
        skip_s56   = s(us::P6_END56);
        skip_s112  = s(us::P6_END112);
        min_skip_s = s(20.0);
        edge_step  = std::max(1, floor_s(0.4));
        edge_trim  = std::max(0, floor_s(0.1));
        mode_step  = std::max(1, floor_s(0.2));
    }
    int maxPulseIndex() const {
        return p3_c + pw + p4_gap + p4_long + edge_step + 1;
    }
    int lookaheadSamples() const {
        return ceil_s(BD_PREFIX_US)
            + std::max(maxPulseIndex(), ceil_s(BD_WINDOW_END_US) + 4);
    }
};

// ---------- Qualitaetsmetrik eines demodulierten Uplinks ----------
struct UplinkResult {
    bool    valid = false;
    int     nbits = 0;           // 56 oder 112
    uint8_t bytes[14] = {0};
    float   margin[112] = {0};   // Phasen-/Pfadkonfidenz je Bit
    double  spr_us = 0.0;        // gelockter SPR-Zeitpunkt
    double  spr_score = 0.0;     // 0..1, Schaerfe des Locks
    double  mean_margin = 0.0;   // mittlere Bitkonfidenz
    double  min_margin = 0.0;    // schlechteste Bitentscheidung
    double  decoder_metric = 0.0;// mittlere Viterbi-Modellabweichung
    double  path_metric_gap = -1.0;
    double  freq_off_hz = 0.0;   // geschaetzter Frequenzoffset
    uint8_t quality = 0;         // 0..255, kombiniert
    uint32_t addr = 0;
    int     addrCnt = 0;
    bool    has_addr = false;
    bool    mode5 = false;
    bool    viterbi = false;
    double  p6_cov = 0.0;        // Anteil Samples >= 0.4*P1 im P6-Fenster
    double  p6_min_ratio = 0.0;  // min(mag) / P1 im P6-Fenster
    double  p6_snr_db = 0.0;     // P6-Median relativ zum lokalen Rauschpegel
    double  pre_snr_db = 0.0;    // P1 relativ zum lokalen Rauschpegel
    double  p6_p1_ratio = 0.0;   // P6-Median relativ zur P1-Amplitude
    double  pre_rho = 0.0;       // normierte Praeambel-/SPR-Korrelation (0..1)
    const char *reject_reason = nullptr;
};

// ---------- Globaler Zustand ----------
struct State {
    // Synchronisation (Double-Buffer: Reader schreibt, Main verarbeitet)
    pthread_t       reader_thread{};
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
    SampleBlock     iq_buf;          // vom Reader befuellt
    bool            data_ready = false;
    bool            reader_done = false;
    bool            reader_error = false;
    uint64_t        dropped = 0;     // verworfene Puffer (Main zu langsam)

    // Verarbeitung
    SampleBlock     iq_work;         // Arbeitskopie
    SampleBlock     iq_hist;         // Overlap + neue Samples im Eingangsformat
    std::vector<uint8_t>  mag;       // Overlap + neue Samples (Magnitude)
    uint32_t              overlap = 0;
    uint32_t              prefix_keep = 0; // bereits gescannte Praefix-Historie am Pufferanfang
    uint32_t              lookahead = 0;
    uint64_t              sample_base = 0; // absoluter Index von mag[0]
    std::vector<uint8_t>  lut16;     // 65536 Eintraege, Index = I | Q<<8
    std::vector<uint8_t>  order;     // Reihenfolge erkannter Codes

    // Optionen
    Thresholds th;
    Timing   tm;
    InputFormat input_format = InputFormat::CU8;
    uint32_t data_len   = DEFAULT_LEN;
    uint32_t freq       = DEFAULT_FREQ;
    uint32_t samplerate = DEFAULT_RATE;
    int      dev_index  = 0;
    int      gain       = MAX_GAIN;
    int      ppm        = 0;
    bool     agc        = false;
    bool     print_order = false;
    bool     print_detected = false;
    bool     print_all  = false;
    bool     baseline   = false;
    bool     continuous = false;
    bool     uplink_decode = false;
    bool     p6_diagnostics = false;
    int      p6_min_quality = 180;
    double   p6_min_snr_db = 6.0;
    double   p6_min_pre_snr_db = 3.0;
    double   p6_max_p1_ratio = 4.0;
    double   p6_min_coverage = 0.65;
    double   p6_min_rho = 0.50;
    double   p6_min_lock = 0.50;
    int      p6_timing_radius = 1;
    const char *filename = nullptr;
    double sc16_scale = 2048.0;
    std::string input_name;
    std::string pluto_uri = "ip:192.168.2.1";
    uint32_t pluto_bandwidth = 0;
    double pluto_gain = 52.0;
    bool pluto_gain_set = false;
    bool use_pluto = false;
    bool input_format_set = false;
    std::unique_ptr<SampleSource> source;

    // Statistik
    Counters cur, cum;
    Average  avg_pulse, avg_nf, avg_nfc;
    double   last_burst_us = -1.0;
    uint64_t uplink_ok = 0;

    // RTL-SDR
#if WITH_RTLSDR
    rtlsdr_dev_t *dev = nullptr;
#endif

    // Ausgabedirectory fuer JSONL und Roh-I/Q
    std::string out_dir;
    bool        save_iq  = false;
    FILE*       jsonl_fp = nullptr;
    FILE*       iq_fp    = nullptr;
};

static State G;
static_assert(std::atomic<bool>::is_always_lock_free,
              "g_exit muss lock-free sein (Signal-Handler)");
static std::atomic<bool> g_exit{false};
static std::atomic<int> g_signal{0};

static void sigHandler(int signal_number) {
    g_signal.store(signal_number);
    g_exit = true;   // nur async-signal-safe Operationen
}

static void closeOutputs();

// Nur aus dem Main-Thread vor Start des Reader-Threads aufrufen.
[[noreturn]] static void die(const char *msg) {
    fprintf(stderr, "Fehler: %s\n", msg);
#if WITH_RTLSDR
    if (G.dev) { rtlsdr_close(G.dev);     G.dev = nullptr; }
#endif
    if (G.source) G.source->close();
    closeOutputs();
    exit(1);
}

// ---------- JSONL/Roh-I/Q-Ausgabe ----------

static bool ensureDir(const std::string& path) {
    if (path.empty()) return false;
    std::string cur;
    for (size_t i = 0; i < path.size(); ++i) {
        cur += path[i];
        if (path[i] == '/' || i + 1 == path.size()) {
            if (cur == "/") continue;
            if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) return false;
        }
    }
    return true;
}

static bool initOutputs() {
    if (G.out_dir.empty()) return true;
    if (!ensureDir(G.out_dir)) {
        fprintf(stderr, "Fehler: Kann Ausgabeverzeichnis '%s' nicht anlegen: %s\n",
                G.out_dir.c_str(), strerror(errno));
        return false;
    }

    time_t now = time(nullptr);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_now);

    std::string jsonl_path = G.out_dir + "/events_" + ts + ".jsonl";
    G.jsonl_fp = fopen(jsonl_path.c_str(), "w");
    if (!G.jsonl_fp) {
        fprintf(stderr, "Fehler: Kann '%s' nicht oeffnen: %s\n",
                jsonl_path.c_str(), strerror(errno));
        return false;
    }

    if (G.save_iq) {
        const char* extension =
            G.input_format == InputFormat::CU8 ? ".cu8" : ".sc16";
        std::string iq_path = G.out_dir + "/raw_" + ts + extension;
        G.iq_fp = fopen(iq_path.c_str(), "wb");
        if (!G.iq_fp) {
            fprintf(stderr, "Fehler: Kann '%s' nicht oeffnen: %s\n",
                    iq_path.c_str(), strerror(errno));
            fclose(G.jsonl_fp);
            G.jsonl_fp = nullptr;
            return false;
        }
    }
    return true;
}

static void closeOutputs() {
    if (G.jsonl_fp) { fflush(G.jsonl_fp); fclose(G.jsonl_fp); G.jsonl_fp = nullptr; }
    if (G.iq_fp)    { fflush(G.iq_fp);    fclose(G.iq_fp);    G.iq_fp    = nullptr; }
}

static double monotonicSec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static inline uint32_t ufOf(const uint8_t *b);
static inline bool hasAircraftAddress(uint32_t uf, int nbits);
static uint32_t apToAddressBytes(const uint8_t* bytes, int nbits);
static int formatP6ProtocolJson(const UplinkResult& R, char *out, size_t size);

static void writeJsonlP6(uint64_t smp, int p1, int p2, const UplinkResult& R) {
    if (!G.jsonl_fp || !R.valid) return;

    char hex[32] = {0};
    int nbytes = R.nbits / 8;
    for (int b = 0; b < nbytes; ++b)
        snprintf(hex + b * 2, 3, "%02X", R.bytes[b]);

    char aa_json[16];
    if (!R.mode5 && R.has_addr && R.addrCnt >= 2)
        snprintf(aa_json, sizeof(aa_json), "\"%06X\"", R.addr);
    else
        snprintf(aa_json, sizeof(aa_json), "null");

    double snr_db = (p2 > 0 && p1 > 0) ? 20.0 * log10((double)p1 / (double)p2) : 0.0;

    const char *status = R.mode5 ? "mode5"
                       : (R.has_addr && R.addrCnt >= 2) ? "repeated" : "candidate";
    double data_min_margin = 1.0;
    double ap_min_margin = 1.0;
    for (int b = 0; b < R.nbits; ++b) {
        double &target = (b < R.nbits - 24)
                       ? data_min_margin : ap_min_margin;
        target = std::min(target, (double)R.margin[b]);
    }
    char protocol_json[384];
    formatP6ProtocolJson(R, protocol_json, sizeof(protocol_json));

    fprintf(G.jsonl_fp,
        "{\"type\":\"P6\",\"smp\":%llu,\"mono\":%.6f,\"sample_rate\":%u,"
        "\"uf\":%d,\"mode5\":%s,\"nbits\":%d,"
        "\"p1\":%d,\"p2\":%d,\"snr_db\":%.2f,\"df_hz\":%.1f,"
        "\"spr_us\":%.3f,\"lock\":%.3f,\"cov\":%.3f,\"p6min\":%.3f,"
        "\"p6snr_db\":%.2f,\"pre_snr_db\":%.2f,\"p6_p1_ratio\":%.2f,"
        "\"rho\":%.3f,\"q\":%u,\"viterbi_metric\":%.6f,"
        "\"mean_margin\":%.4f,\"min_margin\":%.4f,"
        "\"data_min_margin\":%.4f,\"ap_min_margin\":%.4f,"
        "\"path_metric_gap\":%.6f%s,\"status\":\"%s\","
        "\"aa\":%s,\"aa_cnt\":%u,\"hex\":\"%s\"}\n",
        (unsigned long long)smp, monotonicSec(), G.samplerate,
        (int)ufOf(R.bytes), R.mode5 ? "true" : "false", R.nbits,
        p1, p2, snr_db, R.freq_off_hz,
        R.spr_us, R.spr_score, R.p6_cov, R.p6_min_ratio, R.p6_snr_db,
        R.pre_snr_db, R.p6_p1_ratio, R.pre_rho, (unsigned)R.quality,
        R.decoder_metric, R.mean_margin, R.min_margin, data_min_margin,
        ap_min_margin, R.path_metric_gap, protocol_json,
        status, aa_json,
        (!R.mode5 && R.has_addr)
            ? (unsigned)R.addrCnt : 0u, hex);
}

static void writeJsonlSsr(const char* subtype, uint64_t smp, int p1, int p3, int p4, int code) {
    if (!G.jsonl_fp) return;
    fprintf(G.jsonl_fp,
        "{\"type\":\"SSR\",\"subtype\":\"%s\",\"smp\":%llu,\"mono\":%.6f,"
        "\"sample_rate\":%u,"
        "\"p1\":%d,\"p3\":%d,\"p4\":%d,\"code\":%d}\n",
        subtype, (unsigned long long)smp, monotonicSec(), G.samplerate,
        p1, p3, p4, code);
}

static void writeJsonlP6Reject(uint64_t smp, int p1, int p2,
                               const UplinkResult& R) {
    if (!G.jsonl_fp || !G.p6_diagnostics || R.valid) return;

    char hex[32] = {0};
    int nbytes = R.nbits / 8;
    for (int b = 0; b < nbytes && b < 14; ++b)
        snprintf(hex + b * 2, 3, "%02X", R.bytes[b]);

    double snr_db = (p2 > 0 && p1 > 0)
                  ? 20.0 * log10((double)p1 / (double)p2) : 0.0;
    int uf = R.nbits > 0 ? (int)ufOf(R.bytes) : -1;
    fprintf(G.jsonl_fp,
        "{\"type\":\"P6_REJECTED\",\"smp\":%llu,\"mono\":%.6f,"
        "\"sample_rate\":%u,"
        "\"reason\":\"%s\",\"uf\":%d,\"nbits\":%d,\"p1\":%d,\"p2\":%d,"
        "\"snr_db\":%.2f,\"df_hz\":%.1f,\"spr_us\":%.3f,\"lock\":%.3f,"
        "\"cov\":%.3f,\"p6min\":%.3f,\"p6snr_db\":%.2f,"
        "\"pre_snr_db\":%.2f,\"p6_p1_ratio\":%.2f,\"rho\":%.3f,"
        "\"q\":%u,\"viterbi_metric\":%.6f,\"mean_margin\":%.4f,"
        "\"min_margin\":%.4f,\"hex\":\"%s\"}\n",
        (unsigned long long)smp, monotonicSec(),
        G.samplerate, R.reject_reason ? R.reject_reason : "unknown",
        uf, R.nbits, p1, p2, snr_db, R.freq_off_hz, R.spr_us,
        R.spr_score, R.p6_cov, R.p6_min_ratio, R.p6_snr_db,
        R.pre_snr_db, R.p6_p1_ratio, R.pre_rho, (unsigned)R.quality,
        R.decoder_metric, R.mean_margin, R.min_margin, hex);
}


// ---------- Magnitude + I/Q-Historie ----------

// 16-Bit-LUT: Index = I | (Q << 8), Mittelpunkt 127.5, Skalierung auf 0..255
static void buildLut() {
    G.lut16.resize(65536);
    for (int q = 0; q < 256; q++) {
        for (int i = 0; i < 256; i++) {
            double fi = (i - 127.5) / 127.5;
            double fq = (q - 127.5) / 127.5;
            double mag = std::sqrt(fi * fi + fq * fq) * (255.0 / std::sqrt(2.0));
            if (mag > 255.0) mag = 255.0;
            G.lut16[i | (q << 8)] = (uint8_t)lround(mag);
        }
    }
}

static size_t sampleBytes(InputFormat format) {
    return format == InputFormat::CU8 ? 2u : 4u;
}

static void prepareSampleBlock(SampleBlock& block) {
    const size_t samples = block.sample_count();
    block.magnitude.resize(samples);
    const uint8_t* raw = block.raw.data();
    uint8_t* magnitude = block.magnitude.data();
    if (block.format == InputFormat::CU8) {
        const uint8_t* lut = G.lut16.data();
        for (size_t j = 0; j < samples; ++j)
            magnitude[j] = lut[raw[2 * j] | (raw[2 * j + 1] << 8)];
        return;
    }

    constexpr uint32_t FIXED_SHIFT = 24;
    const uint64_t magnitude_gain = std::max<uint64_t>(
        1,
        static_cast<uint64_t>(llround(
            (255.0 * 8.0 / 11.0 / G.sc16_scale) *
            static_cast<double>(uint64_t{1} << FIXED_SHIFT)
        ))
    );
    for (size_t j = 0; j < samples; ++j) {
        int16_t raw_i, raw_q;
        memcpy(&raw_i, raw + 4 * j, sizeof(raw_i));
        memcpy(&raw_q, raw + 4 * j + 2, sizeof(raw_q));
        const uint32_t abs_i =
            static_cast<uint32_t>(raw_i < 0 ? -static_cast<int32_t>(raw_i)
                                             : raw_i);
        const uint32_t abs_q =
            static_cast<uint32_t>(raw_q < 0 ? -static_cast<int32_t>(raw_q)
                                             : raw_q);
        const uint32_t high = std::max(abs_i, abs_q);
        const uint32_t low = std::min(abs_i, abs_q);
        const uint64_t value =
            ((8ULL * high + 3ULL * low) * magnitude_gain) >>
            (FIXED_SHIFT + 3);
        magnitude[j] = static_cast<uint8_t>(
            std::min<uint64_t>(255, value)
        );
    }
}

class FileSampleSource : public SampleSource {
public:
    FileSampleSource(std::string path, InputFormat format,
                     uint32_t block_bytes, double rate)
        : path_(std::move(path)), format_(format), block_bytes_(block_bytes),
          rate_(rate) {}
    ~FileSampleSource() override { close(); }

    bool open() override {
        if (path_ == "-") {
            fd_ = STDIN_FILENO;
            return true;
        }
        fd_ = ::open(path_.c_str(), O_RDONLY);
        if (fd_ < 0) {
            fprintf(stderr, "Fehler: Datei '%s' nicht lesbar: %s\n",
                    path_.c_str(), strerror(errno));
            return false;
        }
        return true;
    }

    ReadStatus read_block(SampleBlock& out) override {
        out.format = format_;
        out.sample_rate = rate_;
        out.raw.resize(block_bytes_);
        size_t got = 0;
        bool io_error = false;
        while (got < out.raw.size()) {
            if (g_exit) return ReadStatus::EOF_REACHED;
            if (fd_ == STDIN_FILENO) {
                pollfd fd{STDIN_FILENO, POLLIN, 0};
                int ready;
                do {
                    ready = poll(&fd, 1, 100);
                } while (ready < 0 && errno == EINTR && !g_exit);
                if (g_exit) return ReadStatus::EOF_REACHED;
                if (ready < 0) {
                    fprintf(stderr, "Pipe-Abfrage fehlgeschlagen: %s\n",
                            strerror(errno));
                    return ReadStatus::ERROR;
                }
                if (ready == 0) continue;
            }
            const ssize_t n =
                read(fd_, out.raw.data() + got, out.raw.size() - got);
            if (n < 0) {
                if (errno == EINTR) {
                    if (g_exit) return ReadStatus::EOF_REACHED;
                    continue;
                }
                fprintf(stderr, "Lesefehler: %s\n", strerror(errno));
                io_error = true;
                break;
            }
            if (n == 0) break;
            got += static_cast<size_t>(n);
        }
        const size_t unit = sampleBytes(format_);
        if (got % unit != 0) {
            fprintf(stderr,
                    "Eingabe endet mitten in einem I/Q-Sample (%zu Restbytes)\n",
                    got % unit);
            out.raw.resize(0);
            return ReadStatus::ERROR;
        }
        out.raw.resize(got);
        if (got > 0) prepareSampleBlock(out);
        if (io_error) return ReadStatus::ERROR;
        if (got == 0) return ReadStatus::EOF_REACHED;
        return ReadStatus::DATA;
    }

    double sample_rate() const override { return rate_; }
    bool live() const override { return false; }

    void close() override {
        if (fd_ >= 0 && fd_ != STDIN_FILENO) ::close(fd_);
        fd_ = -1;
    }

private:
    std::string path_;
    InputFormat format_;
    uint32_t block_bytes_;
    double rate_;
    int fd_ = -1;
};

// Anzahl gueltiger (echter, nicht Reserve-) Samples im aktuellen Block.
static uint32_t g_valid_samples = 0;

static void ensureCapacity(uint32_t total) {
    size_t need = (size_t)total + G.lookahead;
    if (G.mag.size() < need)          G.mag.resize(need);
    if (G.iq_hist.raw.size() < sampleBytes(G.input_format) * need)
        G.iq_hist.raw.resize(sampleBytes(G.input_format) * need);
}

// Haengt iq_work[0..len) hinter den Overlap und berechnet die Magnitude.
// Rueckgabe: Anzahl gueltiger Samples (Overlap + neue).
static uint32_t computeMagnitude(const SampleBlock& block, bool final) {
    const uint32_t n_new =
        static_cast<uint32_t>(block.sample_count());
    uint32_t total = G.overlap + n_new;
    ensureCapacity(total);

    uint8_t *iq = G.iq_hist.raw.data();
    G.iq_hist.format = G.input_format;
    G.iq_hist.sample_rate = G.samplerate;
    const size_t stride = sampleBytes(G.input_format);
    if (n_new) {
        memcpy(iq + stride * (size_t)G.overlap,
               block.raw.data(), stride * (size_t)n_new);
    }
    uint8_t *m = G.mag.data() + G.overlap;
    if (n_new) memcpy(m, block.magnitude.data(), n_new);

    if (final) {
        std::fill(G.mag.begin() + total,
                  G.mag.begin() + total + G.lookahead, 0);
        std::fill(G.iq_hist.raw.begin() + stride * (size_t)total,
                  G.iq_hist.raw.begin() + stride * ((size_t)total + G.lookahead),
                  0);
    }
    return total;
}

// Bewahrt Praefix-Historie und ungescannten Lookahead synchron in beiden Puffern.
static void keepOverlap(uint32_t total, uint32_t consumed) {
    if (consumed > total) consumed = total;
    const uint32_t prefix_samples =
        static_cast<uint32_t>(G.tm.ceil_s(BD_PREFIX_US));
    uint32_t tail = total - consumed;
    if (tail > G.lookahead) {
        consumed += tail - G.lookahead;
        tail = G.lookahead;
    }
    const uint32_t history = std::min(prefix_samples, consumed);
    const uint32_t start = consumed - history;
    uint32_t keep = total - start;
    if (keep && start > 0) {
        const size_t stride = sampleBytes(G.input_format);
        memmove(G.mag.data(), G.mag.data() + start, keep);
        memmove(G.iq_hist.raw.data(),
                G.iq_hist.raw.data() + stride * (size_t)start,
                stride * (size_t)keep);
    }
    G.sample_base += start;
    G.overlap = keep;
    G.prefix_keep = history;
}

static FILE *G_txt = stdout;

// ---------- Detektion: rate-unabhaengige Bausteine ----------
static inline bool ratioGt(int a, int b, int rq) { return a * 256 > rq * b; }

static inline int pulseRef(const uint8_t *m, int start, int w) {
    const int trim = std::min(G.tm.edge_trim, std::max(0, (w - 1) / 2));
    int r = 255;
    for (int k = trim; k < w - trim; k++)
        r = std::min<int>(r, m[start + k]);
    return r;
}

static inline bool pulseOk(const uint8_t *m, int start, int w) {
    return pulseRef(m, start, w) > G.th.min_peak;
}

static inline bool pauseFar(int v, int ref) {
    const Thresholds &t = G.th;
    return v <= t.max_nf && v + t.diff < ref && !ratioGt(v, ref, t.ratio);
}

static inline bool pauseClose(int v, int ref) {
    const Thresholds &t = G.th;
    return v <= t.max_nfc && v + t.diffclose < ref && !ratioGt(v, ref, t.ratioclose);
}

static bool pauseRange(const uint8_t *m, int from, int to, int ref) {
    if (from > to) return true;
    const int guard = std::min(
        G.tm.edge_trim, std::max(0, (to - from + 1) / 2)
    );
    for (int k = from; k < from + guard; ++k)
        if (!pauseClose(m[k], ref)) return false;
    for (int k = to - guard + 1; k <= to; ++k)
        if (!pauseClose(m[k], ref)) return false;
    if (guard == 0 &&
        (!pauseClose(m[from], ref) || !pauseClose(m[to], ref)))
        return false;
    for (int k = from + guard; k < to - guard + 1; k++)
        if (!pauseFar(m[k], ref)) return false;
    return true;
}

static inline bool pauseBetween(const uint8_t *m, int from, int to,
                                int ref_a, int ref_b) {
    return pauseRange(m, from, to, std::min(ref_a, ref_b));
}

// ---------- Detektion: P1 / Mode S / P3 / P4 ----------
static bool checkP1(const uint8_t *m) {
    const Timing &T = G.tm;
    if (!pulseOk(m, 0, T.pw)) return false;
    int r1 = pulseRef(m, 0, T.pw);
    if (r1 <= G.th.min_peak) return false;
    return pauseFar(m[T.pw + T.edge_step], r1);
}

static bool checkModeS(const uint8_t *m) {
    const Timing &T = G.tm;
    if (!pulseOk(m, T.p2, T.pw)) return false;
    int r1 = pulseRef(m, 0, T.pw);
    int r2 = pulseRef(m, T.p2, T.pw);
    if (!pauseBetween(m, T.pw, T.p2 - 1, r1, r2)) return false;
    int ref = std::min(r1, r2);
    return pauseClose(m[T.p2 + T.pw + T.mode_step], ref);
}

static bool checkP3(const uint8_t *m, int off) {
    const Timing &T = G.tm;
    if (!pulseOk(m, off, T.pw)) return false;
    int r1 = pulseRef(m, 0, T.pw);
    int r3 = pulseRef(m, off, T.pw);
    if (!pauseBetween(m, T.pw, off - 1, r1, r3)) return false;
    return pauseFar(m[off + T.pw + T.edge_step], r3);
}

enum P4Type { P4_NONE = 0, P4_SHORT, P4_LONG };

static inline bool p4Pause(int v, int ref, bool close) {
    const Thresholds &t = G.th;
    return close ? (v <= t.max_nfc && !ratioGt(v, ref, t.ratioclosep4))
                 : (v <= t.max_nf  && !ratioGt(v, ref, t.ratiop4));
}

static P4Type checkP4(const uint8_t *m, int os) {
    const Timing &T = G.tm;
    const uint8_t *p = m + os;

    auto before = [&](int ref) {
        const int pulse_end_guard =
            std::max(1, T.p4_gap - T.pw - T.edge_trim);
        return p4Pause(p[-T.edge_step], ref, true)
            && p4Pause(p[-2 * T.edge_step], ref, false)
            && p4Pause(p[-pulse_end_guard], ref, true);
    };

    if (pulseOk(p, 0, T.p4_long)) {
        int r = pulseRef(p, 0, T.p4_long);
        if (before(r) && p4Pause(p[T.p4_long + T.edge_step], r, false))
            return P4_LONG;
    }
    if (pulseOk(p, 0, T.p4_short)) {
        int r = pulseRef(p, 0, T.p4_short);
        if (before(r) &&
            p4Pause(p[T.p4_short + T.edge_step], r, false))
            return P4_SHORT;
    }
    return P4_NONE;
}

static const char *codeName(MsgCode c) {
    switch (c) {
        case MSG_S:       return "Mode S";
        case MSG_A:       return "Mode A";
        case MSG_C:       return "Mode C";
        case MSG_A_ACAC:  return "Mode A All-Call";
        case MSG_C_ACAC:  return "Mode C All-Call";
        case MSG_A_ACSAC: return "Mode A All-Call (Compatibility Mode)";
        case MSG_C_ACSAC: return "Mode C All-Call (Compatibility Mode)";
        default:          return "?";
    }
}

// ---------- I/Q-Zugriff fuer den P6-Demodulator ----------
using cd = std::complex<double>;

static inline cd iqAt(int k) {
    return G.iq_hist.iq_at(static_cast<size_t>(k), G.sc16_scale);
}

// Catmull-Rom-Interpolation ueber 4 Stuetzstellen (s0-1 .. s0+2).
static inline bool getIQSample(double t, cd &out) {
    int s0 = (int)std::floor(t);
    if (s0 - 1 < 0 || s0 + 2 >= (int)g_valid_samples) return false;
    double u = t - s0, u2 = u * u, u3 = u2 * u;
    double w0 = -0.5 * u3 +       u2 - 0.5 * u;
    double w1 =  1.5 * u3 - 2.5 * u2 + 1.0;
    double w2 = -1.5 * u3 + 2.0 * u2 + 0.5 * u;
    double w3 =  0.5 * u3 - 0.5 * u2;
    out = w0 * iqAt(s0 - 1) + w1 * iqAt(s0) + w2 * iqAt(s0 + 1) + w3 * iqAt(s0 + 2);
    return true;
}

// ---------- Sync Phase Reversal (SPR) Lock ----------
struct WinSum { cd z = 0.0; double mag = 0.0; int n = 0; };

static bool windowSum(int i, double t0_us, double t1_us, double w, WinSum &ws) {
    const Timing &T = G.tm;
    const int steps = std::max(3, (int)lround((t1_us - t0_us) / 0.1) + 1);
    ws = WinSum{};
    for (int k = 0; k < steps; k++) {
        double tu = t0_us + (t1_us - t0_us) * k / (steps - 1);
        double t  = i + T.f(tu);
        cd z;
        if (!getIQSample(t, z)) return false;
        z *= std::polar(1.0, -w * (t - i));
        ws.z   += z;
        ws.mag += std::abs(z);
        ws.n++;
    }
    return ws.mag > 1e-9;
}

static double sprScore(int i, double tc, double w) {
    WinSum a, b;
    if (!windowSum(i, tc - 0.85, tc - 0.05, w, a) ||
        !windowSum(i, tc + 0.05, tc + 0.45, w, b))
        return -1e9;
    double na = std::abs(a.z), nb = std::abs(b.z);
    if (na < 1e-9 || nb < 1e-9) return -1e9;
    double coh_a = na / a.mag;
    double coh_b = nb / b.mag;
    double rev = -(b.z * std::conj(a.z)).real() / (na * nb);
    double dev = (tc - us::SPR_PRIOR) / 0.85;
    return rev * coh_a * coh_b - 0.15 * dev * dev;
}

struct SprLock { bool ok = false; double t_us = us::SPR; double score = 0.0; };

static SprLock lockSpr(int i, double w) {
    SprLock L;
    constexpr double step = 0.05;
    constexpr int n = static_cast<int>((us::SPR_MAX - us::SPR_MIN) / step + 0.5) + 1;
    static_assert(n > 0 && n <= 64, "SPR-Suchraster ausserhalb gueltiger Grenzen");
    std::array<double, n> sc{};
    int best = -1;
    for (int k = 0; k < n; k++) {
        sc[k] = sprScore(i, us::SPR_MIN + k * step, w);
        if (best < 0 || sc[k] > sc[best]) best = k;
    }
    if (best < 0 || sc[best] < -1e8) return L;

    for (int k = 1; k < best; k++) {
        if (sc[k - 1] < -1e8 || sc[k + 1] < -1e8) continue;
        if (sc[k] >= 0.35 && sc[k] >= 0.8 * sc[best] &&
            sc[k] >= sc[k - 1] && sc[k] >= sc[k + 1]) { best = k; break; }
    }

    double t = us::SPR_MIN + best * step;
    if (best > 0 && best < n - 1 && sc[best - 1] > -1e8 && sc[best + 1] > -1e8) {
        double y0 = sc[best - 1], y1 = sc[best], y2 = sc[best + 1];
        double den = y0 - 2.0 * y1 + y2;
        if (den < -1e-12) {
            double d = 0.5 * (y0 - y2) / den;
            if (std::fabs(d) <= 0.5) t += d * step;
        }
    }
    L.t_us  = t;
    L.score = std::max(0.0, std::min(1.0, sc[best]));
    L.ok    = L.score > 0.35;
    return L;
}

// ---------- P6-Laengenschaetzung & Bit-Demodulation ----------
static double meanMag(int i, double t0_us, double t1_us) {
    const Timing &T = G.tm;
    int a = i + (int)std::ceil(T.f(t0_us));
    int b = i + (int)std::floor(T.f(t1_us));
    if (a < 0 || b >= (int)g_valid_samples || b < a) return -1.0;
    double s = 0.0;
    for (int k = a; k <= b; k++) s += G.mag[k];
    return s / (b - a + 1);
}

static int estimateLength(int i, int p1_ref) {
    double ref = meanMag(i, 6.0, 9.0);
    if (ref < 0.0) return 0;
    if (ref < 0.5 * p1_ref) return 0;
    double tail = meanMag(i, 24.0, 27.0);
    if (tail < 0.0) return 56;
    return (tail > 0.5 * ref) ? 112 : 56;
}

// ===================== Viterbi-DPSK (Paket 1) =====================
// Zeitachse wie p6sim: 20-MS/s-Index, P1-Vorderflanke = 160, Datenchip j ab anchor + 5*j

struct DpskChannel {
    std::vector<float> g0;   // Antwort auf Einheitschip j=0 (20 MS/s)
    std::vector<float> kn;   // bekannter Anteil P1/P2/P6/SPR (20 MS/s)
    int   anchor = 265;      // Datenbeginn (ST_ANCHOR)
    float delay  = 0.f;      // Kanal-Laufzeit in 20-MS/s-Samples
    uint32_t input_sample_rate = 0;
    double model_sample_rate = DPSK_MODEL_RATE;

    static float interp(const std::vector<float>& y, double t) {
        if (t < 0 || t >= (double)y.size() - 1) return 0.f;
        int i = (int)t;
        float a = (float)(t - i);
        return y[i] * (1.f - a) + y[i + 1] * a;
    }
    float g(int m, double t) const { return interp(g0, t - 5.0 * m); }
    float knv(double t)      const { return interp(kn, t); }
};

struct ViterbiResult {
    std::vector<uint8_t> bits;
    std::vector<float> margin;
    float metric = 0.f;      // mittlerer quadratischer Fehler pro Sample
    float path_metric_gap = -1.f;
    double residual_w = 0.0;
    double residual_coherence = 0.0;
    int   nsamp  = 0;
};

struct ViterbiSample {
    int w;
    float kv;
    float B[7];
    std::complex<float> z;
};

struct ViterbiWorkspace {
    std::vector<int> by_count;
    std::vector<int> by_offset;
    std::vector<int> by_cursor;
    std::vector<std::pair<int, int>> sample_bucket; // (bucket, k)
    std::vector<ViterbiSample> by_samples;
    std::vector<float> met, nmet, margin, forward, backward;
    std::vector<uint16_t> traceback;
    std::vector<double> phase_x, phase_y, phase_weights;
    std::vector<int8_t> chip;
};

static thread_local ViterbiWorkspace g_viterbi_workspace;

// r: Frame-Fenster (df-korrigiert), r[k] liegt bei Modellzeit off + model_step * k
static void viterbiDpsk(const DpskChannel& ch, const std::complex<float>* r, int nr,
                        double off, double model_step, float phi, float A, int n,
                        ViterbiResult& res)
{
    constexpr int L = 7, S = 1 << (L - 1), H = (L - 1) / 2;
    const int N = n + 2;                                   // + 2 Guard-Chips
    ViterbiWorkspace& ws = g_viterbi_workspace;
    ws.by_count.assign(N, 0);
    ws.sample_bucket.clear();
    ws.sample_bucket.reserve(nr);
    const std::complex<float> der = std::polar(1.f / A, -phi);

    int ns = 0;
    for (int k = 0; k < nr; ++k) {
        double t = off + model_step * k;
        int jc = (int)std::floor((t - ch.delay - ch.anchor) / 5.0);
        int w = jc - H, e = w + L - 1;
        if (e < 0 || w > N - 1) continue;
        int bucket = std::min(e, N - 1);
        ws.by_count[bucket]++;
        ws.sample_bucket.emplace_back(bucket, k);
        ++ns;
    }

    ws.by_offset.resize(N + 1);
    ws.by_offset[0] = 0;
    for (int j = 0; j < N; ++j)
        ws.by_offset[j + 1] = ws.by_offset[j] + ws.by_count[j];
    ws.by_cursor.assign(ws.by_offset.begin(), ws.by_offset.begin() + N);
    ws.by_samples.resize(ns);

    for (const auto& entry : ws.sample_bucket) {
        int bucket = entry.first;
        int k = entry.second;
        double t = off + model_step * k;
        int jc = (int)std::floor((t - ch.delay - ch.anchor) / 5.0);
        int w = jc - H;
        ViterbiSample& s = ws.by_samples[ws.by_cursor[bucket]++];
        s.w = w;
        s.kv = ch.knv(t);
        s.z = r[k] * der;
        for (int i = 0; i < L; ++i) s.B[i] = (w + i >= 0) ? ch.g(w + i, t) : 0.f;
    }

    // Zustand: letzte 6 Chips, Bit 0 = juengster, 1 = +1, 0 = -1
    constexpr int K = 2;
    const float INF = std::numeric_limits<float>::infinity();
    auto& met = ws.met;
    auto& nmet = ws.nmet;
    auto& margin = ws.margin;
    auto& tb = ws.traceback;
    auto& forward = ws.forward;
    auto& backward = ws.backward;
    met.assign(S * K, INF);
    nmet.resize(S * K);
    margin.assign(N, 0.f);
    tb.resize((size_t)N * S * K);                          // Vorgaengerpfad
    forward.assign((size_t)(N + 1) * S, INF);
    met[0] = 0.f;                                          // nach SPR: alle -1
    forward[0] = 0.f;

    const ViterbiSample* samples_ptr = ws.by_samples.data();
    const int* offset_ptr = ws.by_offset.data();

    auto branchMetric = [&](int j, int st, int cb) {
        if (j >= n && cb != (st & 1)) return INF;
        float metric = 0.f;
        const int begin = offset_ptr[j];
        const int end = offset_ptr[j + 1];
        for (int idx = begin; idx < end; ++idx) {
            const ViterbiSample& s = samples_ptr[idx];
            float v = s.kv;
            for (int i = 0; i < L; ++i) {
                int d = j - (s.w + i);
                if (d < 0) continue;
                int bit = (d == 0) ? cb : (st >> (d - 1)) & 1;
                v += bit ? s.B[i] : -s.B[i];
            }
            metric += std::norm(s.z - v);
        }
        return metric;
    };

    for (int j = 0; j < N; ++j) {
        std::fill(nmet.begin(), nmet.end(), INF);
        uint16_t* tbj = &tb[(size_t)j * S * K];
        for (int st = 0; st < S; ++st) {
            for (int rank = 0; rank < K; ++rank) {
                const float prior = met[st * K + rank];
                if (prior == INF) continue;
                for (int cb = 0; cb < 2; ++cb) {
                    const float branch = branchMetric(j, st, cb);
                    if (!std::isfinite(branch)) continue;
                    float m = prior + branch;
                    int nst = ((st << 1) | cb) & (S - 1);
                    int slot = nst * K;
                    int insert = K;
                    for (int rank_out = 0; rank_out < K; ++rank_out) {
                        if (m < nmet[slot + rank_out]) {
                            insert = rank_out;
                            break;
                        }
                    }
                    if (insert < K) {
                        for (int shift = K - 1; shift > insert; --shift) {
                            nmet[slot + shift] = nmet[slot + shift - 1];
                            tbj[slot + shift] = tbj[slot + shift - 1];
                        }
                        nmet[slot + insert] = m;
                        tbj[slot + insert] = (uint16_t)(st * K + rank);
                    }
                }
            }
        }
        for (int st = 0; st < S; ++st) {
            forward[(size_t)(j + 1) * S + st] = nmet[st * K];
        }
        met.swap(nmet);
    }

    backward.assign((size_t)(N + 1) * S, INF);
    for (int st = 0; st < S; ++st) {
        backward[(size_t)N * S + st] = 0.f;
    }
    for (int j = N - 1; j >= 0; --j) {
        for (int st = 0; st < S; ++st) {
            for (int cb = 0; cb < 2; ++cb) {
                const float branch = branchMetric(j, st, cb);
                if (!std::isfinite(branch)) continue;
                const int nst = ((st << 1) | cb) & (S - 1);
                const float candidate =
                    branch + backward[(size_t)(j + 1) * S + nst];
                backward[(size_t)j * S + st] = std::min(
                    backward[(size_t)j * S + st], candidate
                );
            }
        }
    }
    for (int j = 0; j < n; ++j) {
        float metric0 = INF, metric1 = INF;
        for (int st = 0; st < S; ++st) {
            const float prior = forward[(size_t)j * S + st];
            if (!std::isfinite(prior)) continue;
            for (int cb = 0; cb < 2; ++cb) {
                const float branch = branchMetric(j, st, cb);
                if (!std::isfinite(branch)) continue;
                const int nst = ((st << 1) | cb) & (S - 1);
                const float candidate =
                    prior + branch
                    + backward[(size_t)(j + 1) * S + nst];
                if ((cb != (st & 1)) == 0) {
                    metric0 = std::min(metric0, candidate);
                } else {
                    metric1 = std::min(metric1, candidate);
                }
            }
        }
        const float best_metric = std::min(metric0, metric1);
        if (std::isfinite(best_metric)) {
            margin[j] = std::min(
                1.f,
                std::fabs(metric1 - metric0)
                    / std::max(1e-6f, best_metric)
            );
        }
    }

    int best = 0, second = -1;
    for (int path = 1; path < S * K; ++path) {
        if (met[path] < met[best]) {
            second = best;
            best = path;
        } else if (second < 0 || met[path] < met[second]) {
            second = path;
        }
    }

    auto& chip = ws.chip;
    chip.resize(N);
    for (int j = N - 1, path = best; j >= 0; --j) {
        const int st = path / K;
        chip[j] = (st & 1) ? 1 : -1;
        path = tb[(size_t)j * S * K + path];
    }

    res.nsamp  = ns;
    res.metric = met[best] / std::max(1, ns);
    res.path_metric_gap = (second >= 0)
        ? (met[second] - met[best]) / std::max(1, ns)
        : -1.f;
    res.residual_w = 0.0;
    res.residual_coherence = 0.0;

    auto& phase_x = ws.phase_x;
    auto& phase_y = ws.phase_y;
    auto& phase_weights = ws.phase_weights;
    phase_x.clear();
    phase_y.clear();
    phase_weights.clear();
    phase_x.reserve(nr);
    phase_y.reserve(nr);
    phase_weights.reserve(nr);
    for (int k = 0; k < nr; ++k) {
        double t = off + model_step * k;
        int jc = (int)std::floor((t - ch.delay - ch.anchor) / 5.0);
        int w = jc - H, e = w + L - 1;
        if (e < 0 || w > N - 1) continue;
        std::complex<double> residual_model = ch.knv(t);
        for (int i = 0; i < L; ++i) {
            int m = w + i;
            if (m < 0 || m >= N) continue;
            residual_model += static_cast<double>(chip[m]) * ch.g(m, t);
        }
        residual_model *=
            static_cast<double>(A) * std::polar(1.0, (double)phi);
        if (std::abs(residual_model) < 1e-9) continue;
        std::complex<double> phase = std::complex<double>(r[k])
            * std::conj(residual_model);
        phase_x.push_back((double)k);
        phase_y.push_back(std::arg(phase));
        phase_weights.push_back(std::abs(phase));
    }
    if (phase_x.size() >= 4) {
        double weight = 0.0, sx = 0.0, sy = 0.0;
        std::complex<double> lag = 0.0;
        double lag_weight = 0.0;
        for (size_t k = 1; k < phase_y.size(); ++k) {
            const double w = std::sqrt(phase_weights[k - 1] * phase_weights[k]);
            lag += w * std::polar(
                1.0, phase_y[k] - phase_y[k - 1]
            );
            lag_weight += w;
        }
        const double lag_dx = std::max(
            1.0, phase_x.back() - phase_x.front()
        ) / std::max<size_t>(1, phase_x.size() - 1);
        const double predicted_w = lag_weight > 1e-12
            ? std::arg(lag) / lag_dx
            : 0.0;
        const double x0 = phase_x.front();
        std::complex<double> intercept = 0.0;
        for (size_t k = 0; k < phase_y.size(); ++k) {
            intercept += phase_weights[k] * std::polar(
                1.0, phase_y[k] - predicted_w * (phase_x[k] - x0)
            );
        }
        const double phase0 = std::arg(intercept);
        for (size_t k = 0; k < phase_y.size(); ++k) {
            double predicted =
                phase0 + predicted_w * (phase_x[k] - x0);
            double delta = predicted - phase_y[k];
            phase_y[k] += 2.0 * M_PI
                * std::round(delta / (2.0 * M_PI));
        }
        for (size_t k = 0; k < phase_x.size(); ++k) {
            double w = phase_weights[k];
            weight += w;
            sx += w * phase_x[k];
            sy += w * phase_y[k];
        }
        if (weight > 0.0) {
            double x_mean = sx / weight, y_mean = sy / weight;
            double numerator = 0.0, denominator = 0.0, energy = 0.0;
            for (size_t k = 0; k < phase_x.size(); ++k) {
                double dx = phase_x[k] - x_mean;
                double dy = phase_y[k] - y_mean;
                double w = phase_weights[k];
                numerator += w * dx * dy;
                denominator += w * dx * dx;
                energy += w * dy * dy;
            }
            if (denominator > 0.0) {
                res.residual_w = numerator / denominator;
                double explained = numerator * numerator /
                    std::max(1e-12, denominator);
                res.residual_coherence = std::max(
                    0.0, std::min(1.0, explained /
                        std::max(1e-12, energy))
                );
            }
        }
    }
    res.margin.resize(n);
    for (int j = 0; j < n; ++j) res.margin[j] = margin[j];
    res.bits.resize(n);
    int prev = -1;
    for (int j = 0; j < n; ++j) {
        res.bits[j] = (chip[j] != prev);
        prev = chip[j];
    }
}
// =================== Ende Viterbi-DPSK ===================

DpskChannel g_chan;
bool g_haveChan = false;

bool loadDpskChannel(const char* path, DpskChannel& ch)
{
    std::ifstream f(path);
    std::string magic;
    int ng = 0, nk = 0;
    if (!(f >> magic)) {
        fprintf(stderr, "Kanaldatei %s leer\n", path);
        return false;
    }
    bool header_ok = false;
    if (magic == "DPSKCHAN1") {
        ch.input_sample_rate = DEFAULT_RATE;
        ch.model_sample_rate = DPSK_MODEL_RATE;
        header_ok = static_cast<bool>(
            f >> ch.anchor >> ch.delay >> ng >> nk
        );
    } else if (magic == "DPSKCHAN2") {
        double input_rate = 0.0;
        double model_rate = 0.0;
        header_ok = static_cast<bool>(
            f >> input_rate >> model_rate
              >> ch.anchor >> ch.delay >> ng >> nk
        );
        const bool finite_input =
            header_ok && std::isfinite(input_rate) &&
            std::isfinite(model_rate) && input_rate > 0.0 &&
            input_rate <= UINT32_MAX;
        const double rounded_input = finite_input ? lround(input_rate) : 0.0;
        if (finite_input && rounded_input > 0.0 &&
            std::abs(input_rate - rounded_input) <= 1e-3) {
            ch.input_sample_rate = static_cast<uint32_t>(rounded_input);
        } else {
            header_ok = false;
        }
        ch.model_sample_rate = model_rate;
    }
    if (!header_ok || ng <= 0 || nk <= 0 || ng > 100000 || nk > 100000) {
        fprintf(stderr, "Kanaldatei %s ungueltig\n", path);
        return false;
    }
    if (std::abs(ch.model_sample_rate - DPSK_MODEL_RATE) > 1.0) {
        fprintf(stderr,
                "Kanaldatei %s verwendet Modellrate %.0f, erwartet %.0f Hz\n",
                path, ch.model_sample_rate, DPSK_MODEL_RATE);
        return false;
    }
    ch.g0.resize(ng);
    ch.kn.resize(nk);
    for (auto& v : ch.g0) if (!(f >> v)) { fprintf(stderr, "Kanaldatei: g0 zu kurz\n"); return false; }
    for (auto& v : ch.kn) if (!(f >> v)) { fprintf(stderr, "Kanaldatei: kn zu kurz\n"); return false; }
    float e = 0.f;
    for (float v : ch.g0) e += v;
    fprintf(stderr,
            "Kanal geladen: %s  input %u Hz model %.0f Hz "
            "anchor %d delay %.2f g0 %d kn %d sum(g0) %.3f\n",
            path, ch.input_sample_rate, ch.model_sample_rate,
            ch.anchor, ch.delay, ng, nk, e);
    return true;
}

// ===================== Blind-Decode (Paket 3) =====================
static inline double modelTicksPerInputSample() {
    return DPSK_MODEL_RATE / (double)G.samplerate;
}

struct BlindResult {
    ViterbiResult v;
    double c = 0;                    // Modellzeit von r[0]
    float  phi = 0.f, A = 0.f;
    double dfHz = 0;
    double rho = 0.0;
};

// wie rt_frame: Fenster holen, df schaetzen und entfernen
static bool bdFrame(size_t nPairs, long k, int n,
                    std::vector<std::complex<float>>& r, double& dfHz)
{
    const int prefix_samples = G.tm.ceil_s(BD_PREFIX_US);
    const int window_samples =
        G.tm.ceil_s(BD_PREFIX_US + BD_WINDOW_END_US) + 1;
    const long s0 = k - prefix_samples;
    if (s0 + window_samples <= 0 ||
        s0 + window_samples > static_cast<long>(nPairs))
        return false;
    r.assign((size_t)window_samples, std::complex<float>(0.0f, 0.0f));
    const int first = static_cast<int>(std::max<long>(0, -s0));
    for (int j = first; j < window_samples; ++j) {
        const size_t sample = static_cast<size_t>(s0 + j);
        const cd value = iqAt(static_cast<int>(sample));
        r[j] = std::complex<float>(
            static_cast<float>(value.real()),
            static_cast<float>(value.imag())
        );
    }
    int df_start = prefix_samples + G.tm.s(BD_DF_START_US);
    int data_end = prefix_samples + G.tm.s(
        n == 56 ? us::P6_END56 : us::P6_END112
    );
    df_start = std::max(1, std::min(df_start, window_samples - 1));
    data_end = std::max(df_start + 1, std::min(data_end, window_samples));
    const int df_lag = std::max(1, G.tm.s(0.4));
    df_start = std::max(df_start, df_lag);
    std::complex<double> acc = 0;
    for (int j = df_start; j < data_end; ++j) {
        std::complex<double> d =
            std::complex<double>(r[j])
            * std::conj(std::complex<double>(r[j - df_lag]));
        acc += d * d;
    }
    double w = (acc != 0.0) ? std::arg(acc) / (2.0 * df_lag) : 0.0;
    for (int j = 0; j < window_samples; ++j)
        r[j] *= std::complex<float>(std::polar(1.0, -w * j));
    dfHz = w * (double)G.samplerate / (2 * M_PI);
    return true;
}

// wie st_pre_fit
static bool bdPreFit(const DpskChannel& ch, const std::complex<float>* r, int nr,
                     double c, double model_step,
                     std::complex<double>& num, double& den, double& sigEnergy)
{
    double lim = ch.anchor + ch.delay - 10;
    num = 0; den = 0; sigEnergy = 0;
    for (int k = 0; k < nr; ++k) {
        double t = c + model_step * k;
        if (t >= lim) break;
        double kv = ch.knv(t);
        std::complex<double> rk(r[k]);
        num += rk * kv;
        den += kv * kv;
        sigEnergy += std::norm(rk);
    }
    return den > 0 && sigEnergy > 0;
}

// wie st_blind_decode: Grobtiming per Praeambel, Feintiming per Viterbi-Metrik
bool blindDecodeDpsk(const DpskChannel& ch, size_t nPairs,
                     long k, int n, BlindResult& out)
{
    static thread_local std::vector<std::complex<float>> r;
    static thread_local ViterbiResult cand;
    if (!bdFrame(nPairs, k, n, r, out.dfHz)) return false;
    const int sample_count = (int)r.size();
    const double model_step = modelTicksPerInputSample();
    const double frame_origin =
    BD_PREFIX_US * (DPSK_MODEL_RATE / 1e6)
    - G.tm.ceil_s(BD_PREFIX_US) * model_step;
    
    int bestC = 0; double bestS = -1;
    std::complex<double> num; double den, sigE;
    for (int c = -24; c <= 32; ++c)
        if (bdPreFit(ch, r.data(), sample_count, frame_origin + c, model_step,
                     num, den, sigE)) {
            double s = std::norm(num) / den;
            if (s > bestS) { bestS = s; bestC = c; }
        }
    if (bestS < 0) return false;

    bool have = false;
    for (int cc = bestC - G.p6_timing_radius;
         cc <= bestC + G.p6_timing_radius; ++cc) {
        const double off = frame_origin + cc;
        if (!bdPreFit(ch, r.data(), sample_count, off, model_step,
                      num, den, sigE)) continue;
        float phi = (float)std::arg(num), A = (float)(std::abs(num) / den);
        viterbiDpsk(ch, r.data(), sample_count, off, model_step, phi, A, n, cand);
        if (!have || cand.metric < out.v.metric) {
            std::swap(out.v, cand);
            out.c = off; out.phi = phi; out.A = A;
            out.rho = std::norm(num) / (den * sigE);
            have = true;
        }
    }
    if (have) {
        const double residual_w = out.v.residual_w;
        const double initial_metric = out.v.metric;
        const double initial_df_hz = out.dfHz;
        if (std::abs(residual_w) > 1e-9
            && out.v.residual_coherence >= 0.05) {
            for (int j = 0; j < sample_count; ++j) {
                r[j] *= std::complex<float>(
                    std::polar(1.0, -residual_w * j)
                );
            }
            if (bdPreFit(ch, r.data(), sample_count, out.c, model_step,
                         num, den, sigE)) {
                float phi = (float)std::arg(num);
                float A = (float)(std::abs(num) / den);
                viterbiDpsk(ch, r.data(), sample_count, out.c, model_step,
                            phi, A, n, cand);
                if (cand.metric < initial_metric) {
                    std::swap(out.v, cand);
                    out.phi = phi;
                    out.A = A;
                    out.rho = std::norm(num) / (den * sigE);
                    out.dfHz = initial_df_hz
                        + residual_w * (double)G.samplerate / (2.0 * M_PI);
                }
            }
        }
    }
    return have;
}
// =================== Ende Blind-Decode ===================

// ===================== Paket 4: Adresse, Q, Bestaetigung =====================
static uint32_t modesCrc(const uint8_t *b, int nbits_data) {
    uint32_t crc = 0;
    for (int k = 0; k < nbits_data; k++) {
        uint32_t bit = (b[k / 8] >> (7 - k % 8)) & 1;
        uint32_t top = (crc >> 23) & 1;
        crc = (crc << 1) & 0xFFFFFF;
        if (top ^ bit) crc ^= 0xFFF409;
    }
    return crc;
}

// Uplink-Adresscodierung umkehren: c = obere 24 Koeffizienten von A(x)*G(x)
static uint32_t uplinkDecodeAddr(uint32_t c)
{
    constexpr uint32_t G = 0x1FFF409;
    uint32_t a = 0;
    for (int k = 23; k >= 0; --k) {
        uint32_t bit = (c >> k) & 1;
        for (int i = k + 1; i <= 23; ++i)
            if ((a >> i) & 1) bit ^= (G >> (24 + k - i)) & 1;
        a |= bit << k;
    }
    return a;
}

// Selbsttest mit den gemessenen Werten (einmal beim Start aufrufen)
static bool uplinkAddrSelftest()
{
    bool ok = uplinkDecodeAddr(0xAAAC07) == 0xFFFFFF    // Broadcast
           && uplinkDecodeAddr(0x7FFA56) == 0x4004FE    // MUK6977
           && uplinkDecodeAddr(0xCBE372) == 0xAE1458;   // RCH313
    if (!ok) fprintf(stderr, "WARNUNG: Uplink-Adress-Selbsttest fehlgeschlagen\n");
    return ok;
}

// Zieladresse aus AP (UF0, UF16 ...): AP XOR CRC(Daten)
static uint32_t apToAddress(const uint8_t *b, int nbits)
{
    uint32_t ap = 0;
    for (int k = nbits - 24; k < nbits; ++k)
        ap = (ap << 1) | ((b[k / 8] >> (7 - k % 8)) & 1);
    return uplinkDecodeAddr(modesCrc(b, nbits - 24) ^ ap);
}

// Q aus Viterbi-Metrik: 0,0 -> 255, 0,085 -> ~168, >= 0,25 -> 0
static int dpskQuality(float metric)
{
    int q = (int)std::lround(255.0 * (1.0 - metric / 0.25));
    return std::max(0, std::min(255, q));
}

// "bestaetigt ab 2x": gleiche Adresse innerhalb des Fensters erneut gesehen
struct AddrTracker {
    struct E { int count = 0; double last = -1e18; };
    std::unordered_map<uint32_t, E> m;
    double window = 60.0;                    // Sekunden
    int seen(uint32_t addr, double tSec) {   // liefert Anzahl inkl. dieser Sichtung
        E& e = m[addr];
        if (tSec - e.last > window) e.count = 0;
        e.count++; e.last = tSec;
        if (m.size() > 20000) m.clear();     // Schutz bei Dauerbetrieb
        return e.count;
    }
};
static AddrTracker g_addrTrack;
// =================== Ende Paket 4 ===================

static uint32_t bits(const uint8_t *b, int first, int last);
static int ufLength(uint32_t uf);

static inline bool isMode5Format(const uint8_t *b, int nbits) {
    uint32_t uf = ufOf(b);
    if (uf == 3 || uf == 6 || uf == 14) return true;
    if (uf == 24 && nbits == 56)        return true;

    // ICAO Annex 10 Vol. IV (3.1.2.8.2): Bei UF=0 muessen die Spare-Bits
    // 6..8, 10..13 und 23..32 zwingend 0 sein.
    if (uf == 0 && nbits == 56) {
        if (bits(b, 6, 8) != 0 || bits(b, 10, 13) != 0 || bits(b, 23, 32) != 0)
            return true;
    }

    // ICAO Annex 10 Vol. IV (3.1.2.7.4): Isolierte Comm-C-ELM-End-/Abbruchsegmente
    // (RC=2 oder RC=3) ohne aktive Segment-Sequenz.
    if (uf == 24 && nbits == 112) {
        uint32_t rc = bits(b, 3, 4);
        if (rc == 2 || rc == 3) return true;
    }

    return false;
}

static inline bool hasAircraftAddress(uint32_t uf, int nbits) {
    switch (uf) {
        case 0: case 4: case 5:    return nbits == 56;
        case 16: case 20: case 21:
        case 24:                   return nbits == 112;
        default:                   return false;
    }
}

static uint32_t apToAddressBytes(const uint8_t* bytes, int nbits) {
    return apToAddress(bytes, nbits);
}

static inline uint32_t ufOf(const uint8_t *b) {
    uint32_t uf = bits(b, 1, 5);
    return ((uf >> 3) == 3) ? 24 : uf;
}

static double medianMagnitude(int i, double t0_us, double t1_us) {
    const Timing &T = G.tm;
    int a = i + (int)std::ceil(T.f(t0_us));
    int b = i + (int)std::floor(T.f(t1_us));
    if (a < 0 || b >= (int)g_valid_samples || b < a) return -1.0;
    static thread_local std::vector<int> values;
    values.clear();
    values.reserve((size_t)(b - a + 1));
    for (int k = a; k <= b; ++k) values.push_back(G.mag[k]);
    auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

static void evalP6Envelope(int i, int p1_ref, int nbits,
                           double &cov, double &min_ratio, double &snr_db,
                           double &pre_snr_db, double &p6_p1_ratio) {
    const Timing &T = G.tm;
    double t1_us = (nbits == 112) ? 33.0 : 19.0;
    int a = i + (int)std::ceil(T.f(4.0));
    int b = i + (int)std::floor(T.f(t1_us));
    if (a < 0 || b >= (int)g_valid_samples || b < a || p1_ref <= 0) {
        cov = 0.0;
        min_ratio = 0.0;
        snr_db = -999.0;
        pre_snr_db = -999.0;
        p6_p1_ratio = 0.0;
        return;
    }
    int min_v = 255, above = 0, total = b - a + 1;
    double thresh = 0.4 * p1_ref;
    for (int k = a; k <= b; ++k) {
        int v = G.mag[k];
        if (v < min_v) min_v = v;
        if (v >= thresh) above++;
    }
    cov = (double)above / total;
    min_ratio = (double)min_v / p1_ref;

    double signal = medianMagnitude(i, 5.0, t1_us);
    double end_us = (nbits == 112) ? us::P6_END112 : us::P6_END56;
    double noise1 = medianMagnitude(i, 1.1, 1.7);
    double noise2 = medianMagnitude(i, end_us + 0.5, end_us + 1.8);
    double noise = std::max(1.0, std::max(noise1, noise2));
    snr_db = 20.0 * std::log10(std::max(1.0, signal) / noise);
    pre_snr_db = 20.0 * std::log10((double)p1_ref / noise);
    p6_p1_ratio = std::max(1.0, signal) / p1_ref;
}

static bool validUplinkFormat(const uint8_t *b, int nbits) {
    uint32_t uf = ufOf(b);
    if (isMode5Format(b, nbits)) return true;

    if (uf == 0 || uf == 4 || uf == 5 || uf == 11) {
        if (nbits != 56) return false;
    } else if (uf == 16 || uf == 20 || uf == 21 || uf == 24) {
        if (nbits != 112) return false;
    } else {
        return false;
    }

    if (uf == 0 && (bits(b, 6, 8) != 0 || bits(b, 10, 13) != 0 ||
                    bits(b, 23, 32) != 0))
        return false;
    return true;
}

static bool blindDecodeNeeds112(const BlindResult& br) {
    uint8_t bytes[7] = {0};
    for (size_t b = 0; b < br.v.bits.size() && b < 56; ++b) {
        if (br.v.bits[b]) {
            bytes[b / 8] |= (uint8_t)(0x80 >> (b % 8));
        }
    }
    if (isMode5Format(bytes, 56)) return false;
    return ufLength(ufOf(bytes)) == 112;
}

static UplinkResult decodeUplink(int i, int p1_ref, int nbits) {
    UplinkResult R;
    R.nbits = nbits;
    if (nbits == 0) {
        R.reject_reason = "length";
        return R;
    }

    evalP6Envelope(i, p1_ref, nbits, R.p6_cov, R.p6_min_ratio, R.p6_snr_db,
                   R.pre_snr_db, R.p6_p1_ratio);
    if (R.p6_cov < G.p6_min_coverage) {
        R.reject_reason = "p6_coverage";
        return R;
    }
    if (R.p6_p1_ratio > G.p6_max_p1_ratio) {
        R.reject_reason = "p6_p1_ratio";
        return R;
    }

    static thread_local BlindResult blind;
    const size_t nPairs = g_valid_samples;
    bool blind_ready = blindDecodeDpsk(
        g_chan, nPairs, i, nbits, blind
    );
    if (blind_ready && nbits == 56 && blindDecodeNeeds112(blind)) {
        nbits = 112;
        blind_ready = blindDecodeDpsk(
            g_chan, nPairs, i, nbits, blind
        );
    }
    R.nbits = nbits;
    evalP6Envelope(i, p1_ref, nbits, R.p6_cov, R.p6_min_ratio, R.p6_snr_db,
                   R.pre_snr_db, R.p6_p1_ratio);
    if (R.p6_cov < G.p6_min_coverage) {
        R.reject_reason = "p6_coverage";
        return R;
    }
    if (R.p6_snr_db < G.p6_min_snr_db) {
        R.reject_reason = "p6_snr";
        return R;
    }
    if (R.pre_snr_db < G.p6_min_pre_snr_db) {
        R.reject_reason = "preamble_snr";
        return R;
    }
    if (R.p6_p1_ratio > G.p6_max_p1_ratio) {
        R.reject_reason = "p6_p1_ratio";
        return R;
    }

    if (!blind_ready) {
        R.reject_reason = "blind_decode";
        return R;
    }

    const BlindResult& br = blind;
    double w_rad = br.dfHz * (2.0 * PI) / G.samplerate;
    SprLock L = lockSpr(i, w_rad);

    R.spr_us        = L.t_us;
    R.spr_score     = L.score;
    R.freq_off_hz   = br.dfHz;
    R.quality       = (uint8_t)dpskQuality(br.v.metric);
    R.pre_rho       = br.rho;
    R.decoder_metric = br.v.metric;

    memset(R.bytes, 0, sizeof(R.bytes));
    for (int b = 0; b < nbits; ++b) {
        if (br.v.bits[b]) R.bytes[b / 8] |= (uint8_t)(0x80 >> (b % 8));
    }
    R.path_metric_gap = br.v.path_metric_gap;
    double sum_margin = 0.0;
    R.min_margin = 1.0;
    for (int b = 0; b < R.nbits; ++b) {
        double margin = br.v.margin[b];
        R.margin[b] = (float)margin;
        sum_margin += margin;
        R.min_margin = std::min(R.min_margin, margin);
    }
    R.mean_margin = sum_margin / R.nbits;
    R.viterbi = true;

    if (R.pre_rho < G.p6_min_rho) {
        R.reject_reason = "preamble_rho";
        return R;
    }
    if (R.spr_score < G.p6_min_lock) {
        R.reject_reason = "spr_lock";
        return R;
    }
    if (R.quality < G.p6_min_quality) {
        R.reject_reason = "viterbi_quality";
        return R;
    }
    if (!validUplinkFormat(R.bytes, R.nbits)) {
        R.reject_reason = "uplink_format";
        return R;
    }
    R.valid = true;
    G.uplink_ok++;
    return R;
}

// ---------- Mode-S-CRC & UF-Dekodierung ----------
static inline uint32_t bits(const uint8_t *b, int first, int last) {
    uint32_t v = 0;
    for (int k = first; k <= last; k++)
        v = (v << 1) | ((b[(k - 1) / 8] >> (7 - (k - 1) % 8)) & 1);
    return v;
}

static int formatP6ProtocolJson(const UplinkResult& R, char *out, size_t size) {
    const uint8_t *b = R.bytes;
    uint32_t uf = ufOf(b);
    int n = 0;

    switch (uf) {
    case 0:
    case 16:
        n = snprintf(out, size,
                     ",\"rl\":%u,\"aq\":%u",
                     bits(b, 9, 9), bits(b, 14, 14));
        if (uf == 0) {
            n += snprintf(out + std::min<size_t>(n, size - 1),
                          size - std::min<size_t>(n, size - 1),
                          ",\"ds\":%u", bits(b, 15, 22));
        } else if (R.nbits == 112) {
            n += snprintf(out + std::min<size_t>(n, size - 1),
                          size - std::min<size_t>(n, size - 1),
                          ",\"mid\":\"%06X\"", bits(b, 65, 88));
        }
        break;
    case 4:
    case 5:
    case 20:
    case 21: {
        uint32_t di = bits(b, 14, 16);
        n = snprintf(out, size,
                     ",\"pc\":%u,\"rr\":%u,\"di\":%u,\"sd\":%u",
                     bits(b, 6, 8), bits(b, 9, 13), di, bits(b, 17, 32));
        if (di == 0 || di == 1 || di == 7) {
            n += snprintf(out + std::min<size_t>(n, size - 1),
                          size - std::min<size_t>(n, size - 1),
                          ",\"ii\":%u", bits(b, 17, 20));
        } else if (di == 3) {
            n += snprintf(out + std::min<size_t>(n, size - 1),
                          size - std::min<size_t>(n, size - 1),
                          ",\"si\":%u", bits(b, 17, 22));
        }
        break;
    }
    case 11: {
        uint32_t pr = bits(b, 6, 9);
        uint32_t ic = bits(b, 10, 13);
        uint32_t cl = bits(b, 14, 16);
        n = snprintf(out, size, ",\"pr\":%u,\"ic\":%u,\"cl\":%u", pr, ic, cl);
        if (cl == 0) {
            n += snprintf(out + std::min<size_t>(n, size - 1),
                          size - std::min<size_t>(n, size - 1),
                          ",\"ii\":%u", ic);
        } else if (cl <= 4) {
            n += snprintf(out + std::min<size_t>(n, size - 1),
                          size - std::min<size_t>(n, size - 1),
                          ",\"si\":%u", ic + 16 * (cl - 1));
        }
        break;
    }
    case 24:
        n = snprintf(out, size, ",\"rc\":%u,\"nc\":%u",
                     bits(b, 3, 4), bits(b, 5, 8));
        break;
    default:
        out[0] = '\0';
        break;
    }
    return n;
}

static int ufLength(uint32_t uf) {
    switch (uf) {
        case 0: case 4: case 5: case 11:  return 56;
        case 16: case 20: case 21:        return 112;
        default: return (uf >= 24) ? 112 : 0;
    }
}

static void printUfFields(const UplinkResult &R, uint64_t pos = 0) {
    const uint8_t *b = R.bytes;
    uint32_t uf = bits(b, 1, 5);
    if ((uf >> 3) == 3) uf = 24;
    fprintf(G_txt, " UF=%u", uf);
    int exp = ufLength(uf);
    if (!R.mode5 && exp && exp != R.nbits)
        fprintf(G_txt, " [LAENGE? erwartet %d]", exp);
    if (R.mode5)
        fprintf(G_txt, " [Mode5]");
    else if (R.has_addr)
        fprintf(G_txt, " AA=%06X%s", R.addr, R.addrCnt >= 2 ? " *" : " ?");
    switch (uf) {
    case 0: case 16:
        fprintf(G_txt, " RL=%u AQ=%u", bits(b, 9, 9), bits(b, 14, 14));
        if (uf == 0) fprintf(G_txt, " DS=0x%02X", bits(b, 15, 22));
        // --- neu (Paket 5): MID = ICAO des sendenden TCAS ---
        if (uf == 16 && R.nbits == 112)
            fprintf(G_txt, " MID=%06X", bits(b, 65, 88));
        break;
    case 4: case 5: case 20: case 21: {
        uint32_t di = bits(b, 14, 16);
        fprintf(G_txt, " PC=%u RR=%u DI=%u SD=0x%04X",
                bits(b, 6, 8), bits(b, 9, 13), di, bits(b, 17, 32));
        if (di == 0 || di == 1 || di == 7)
            fprintf(G_txt, " II=%u", bits(b, 17, 20));
        else if (di == 3)
            fprintf(G_txt, " SI=%u", bits(b, 17, 22));
        break;
    }
    case 11: {
        uint32_t pr = bits(b, 6, 9), ic = bits(b, 10, 13), cl = bits(b, 14, 16);
        fprintf(G_txt, " PR=%u", pr);
        if (cl == 0)       fprintf(G_txt, " II=%u", ic);
        else if (cl <= 4)  fprintf(G_txt, " SI=%u", ic + 16 * (cl - 1));
        else               fprintf(G_txt, " CL=%u(reserviert)", cl);
        break;
    }
    case 24:
        fprintf(G_txt, " RC=%u NC=%u", bits(b, 3, 4), bits(b, 5, 8));
        break;
    default: break;
    }
    int nd = R.nbits - 24;
    uint32_t ap = bits(b, nd + 1, R.nbits);
    fprintf(G_txt, " APx=%06X", ap ^ modesCrc(b, nd));   // Rohwert (codiert)

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    fprintf(G_txt, " SMP=%llu MONO=%lld.%06ld",
            (unsigned long long)pos, (long long)ts.tv_sec, ts.tv_nsec / 1000L);
}

static const char *beamClass(MsgCode c, int p1, int p2, double &db) {
    db = 20.0 * std::log10((double)std::max(p1, 1) / std::max(p2, 1));
    if (c == MSG_S)  return "MODE-S";
    if (p2 <= G.th.max_nf || db >= 9.0) return "MAIN";
    if (db <= 0.0)   return "SLS";
    return "UNSICHER";
}

static void analyzeBurst(uint64_t pos, MsgCode code, const uint8_t *m,
                         const UplinkResult *R) {
    const Timing &T = G.tm;
    double ts_us = pos * 1e6 / G.samplerate;
    double dt_us = (G.last_burst_us >= 0.0) ? ts_us - G.last_burst_us : 0.0;
    G.last_burst_us = ts_us;

    int p1 = 0, p2 = 0;
    for (int k = 0; k < T.pw; k++) {
        p1 = std::max<int>(p1, m[k]);
        p2 = std::max<int>(p2, m[T.p2 + k]);
    }
    double db;
    const char *beam = beamClass(code, p1, p2, db);

    fprintf(G_txt, "%-38s t=%.1f us dt=%.1f us P1/P2=%d/%d (%.1f dB, %s)",
            codeName(code), ts_us, dt_us, p1, p2, db, beam);

    if (R && R->valid) {
        fprintf(G_txt, "\n  [P6] %d Bit spr=%.2f us lock=%.2f "
                       "margin=%.2f/%.2f df=%+.1f kHz Q=%u "
                       "cov=%.2f p6min=%.2f p6snr=%.1f dB rho=%.2f HEX=",
                R->nbits, R->spr_us, R->spr_score, R->mean_margin,
                R->min_margin, R->freq_off_hz / 1e3, R->quality,
                R->p6_cov, R->p6_min_ratio, R->p6_snr_db, R->pre_rho);
        for (int k = 0; k < R->nbits / 8; k++) fprintf(G_txt, "%02X", R->bytes[k]);
        printUfFields(*R, pos);
    } else if (R && G.uplink_decode) {
        fprintf(G_txt, "\n  [P6] kein Lock (spr=%.2f us score=%.2f)",
                R->spr_us, R->spr_score);
    }
    fprintf(G_txt, "\n");
}

// ---------- Detektion: Hauptschleife ----------
static void printSamples(const uint8_t *m, int n) {
    fprintf(G_txt, "  [Samples]");
    for (int k = 0; k < n; k++) fprintf(G_txt, " %u", m[k]);
    fprintf(G_txt, "\n");
}

static inline void addPulse(const uint8_t *m, int start, int w) {
    for (int k = 0; k < w; k++) G.avg_pulse.add(m[start + k]);
}

static void addPause(const uint8_t *m, int from, int to) {
    if (from > to) return;
    G.avg_nfc.add(m[from]);
    if (to != from) G.avg_nfc.add(m[to]);
    for (int k = from + 1; k < to; k++) G.avg_nf.add(m[k]);
}

static bool handleModeS(const uint8_t *m, uint32_t i, int r1,
                        int &skip, UplinkResult &R) {
    (void)m;
    const Timing &T = G.tm;
    int nbits = estimateLength((int)i, r1);

    if (G.uplink_decode) {
        if (nbits == 0) {
            if (!G.p6_diagnostics) return false;
            R.nbits = 0;
            R.reject_reason = "length";
            int p1 = 0, p2 = 0;
            for (int k = 0; k < T.pw; ++k) {
                p1 = std::max<int>(p1, m[k]);
                p2 = std::max<int>(p2, m[T.p2 + k]);
            }
            writeJsonlP6Reject(G.sample_base + i, p1, p2, R);
            return false;
        }
        R = decodeUplink((int)i, r1, nbits);
    }
    const int decoded_nbits =
        (G.uplink_decode && R.nbits > 0) ? R.nbits : nbits;
    skip = (decoded_nbits == 112) ? T.skip_s112
         : (decoded_nbits == 56)  ? T.skip_s56
         :                  T.min_skip_s;
    return true;
}

static void emitP6(uint64_t pos, int p1, int p2, UplinkResult &R,
                   const uint8_t *samples, int show) {
    const uint32_t uf = ufOf(R.bytes);
    R.mode5 = isMode5Format(R.bytes, R.nbits);
    if (R.mode5) {
        R.has_addr = false;
        R.addr = 0;
        R.addrCnt = 0;
    } else {
        R.has_addr = hasAircraftAddress(uf, R.nbits);
        if (R.has_addr) {
            R.addr = apToAddressBytes(R.bytes, R.nbits);
            R.addrCnt = g_addrTrack.seen(
                R.addr, pos / (double)G.samplerate
            );
        }
    }

    writeJsonlP6(pos, p1, p2, R);
    if (G.print_detected) {
        analyzeBurst(pos, MSG_S, samples, &R);
        printSamples(samples, show);
        fputc('\n', G_txt);
    }
}

static uint32_t detectBlock(uint32_t total, bool final) {
    const Timing &T = G.tm;
    const uint8_t *mag = G.mag.data();
    uint32_t limit = final ? total
                   : (total > G.lookahead ? total - G.lookahead : 0);
    uint32_t i = G.prefix_keep;
    if (i >= limit) return i;

    if (G.print_all) {
        for (uint32_t k = G.overlap; k < total; k++) fprintf(G_txt, " %u", mag[k]);
        fputc('\n', G_txt);
    }

    while (i < limit) {
        const uint8_t *m = mag + i;
        if (!checkP1(m)) { i++; continue; }

        const uint64_t pos = G.sample_base + i;
        const int r1 = pulseRef(m, 0, T.pw);
        MsgCode code = MSG_NONE;
        int skip = 1, show = 0;
        UplinkResult R;
        bool haveR = false;

        // --- Mode S ---
        const bool mode_s_preamble = checkModeS(m);
        if (mode_s_preamble && handleModeS(m, i, r1, skip, R)) {
            code  = MSG_S;
            haveR = G.uplink_decode;
            show  = T.p2 + T.pw + T.edge_step;
            if (G.baseline) {
                addPulse(m, 0, T.pw);
                addPulse(m, T.p2, T.pw);
                addPause(m, T.pw, T.p2 - 1);
            }
        }
        // --- Mode A / C ---
        else {
            const bool p3a = checkP3(m, T.p3_a);
            const bool p3c = checkP3(m, T.p3_c);
            int off = p3a ? T.p3_a : p3c ? T.p3_c : 0;
            if (off) {
                const bool isA = (off == T.p3_a);
                const int  os  = off + T.p4_gap;
                show = os + T.p4_long + T.edge_step;
                switch (checkP4(m, os)) {
                case P4_SHORT:
                    code = isA ? MSG_A_ACAC : MSG_C_ACAC;
                    skip = os + T.p4_short;
                    break;
                case P4_LONG:
                    code = isA ? MSG_A_ACSAC : MSG_C_ACSAC;
                    skip = os + T.p4_long;
                    break;
                default:
                    code = isA ? MSG_A : MSG_C;
                    skip = off + T.pw + T.edge_step;
                    if (G.baseline) {
                        addPulse(m, 0, T.pw);
                        addPulse(m, off, T.pw);
                        addPause(m, T.pw, off - 1);
                        G.avg_nfc.add(m[off + T.pw]);
                        G.avg_nf.add(m[off + T.pw + T.edge_step]);
                    }
                    break;
                }
            }
        }

        if (code != MSG_NONE) {
            G.cur.count(code);
            if (G.print_order) G.order.push_back(code);

            int p1 = 0, p2 = 0;
            for (int k = 0; k < T.pw; k++) {
                p1 = std::max<int>(p1, m[k]);
                p2 = std::max<int>(p2, m[T.p2 + k]);
            }
            if (code == MSG_S && haveR && R.valid) {
                emitP6(pos, p1, p2, R, m, show);
            } else {
                if (code == MSG_S && haveR)
                    writeJsonlP6Reject(pos, p1, p2, R);

                if (code != MSG_S) {
                    int off = (code == MSG_A || code == MSG_A_ACAC || code == MSG_A_ACSAC)
                              ? T.p3_a : T.p3_c;
                    int p3 = 0, p4 = 0;
                    for (int k = 0; k < T.pw; k++)
                        p3 = std::max<int>(p3, m[off + k]);
                    if (code == MSG_A_ACAC || code == MSG_C_ACAC) {
                        for (int k = 0; k < T.p4_short; k++)
                            p4 = std::max<int>(p4, m[off + T.p4_gap + k]);
                    } else if (code == MSG_A_ACSAC || code == MSG_C_ACSAC) {
                        for (int k = 0; k < T.p4_long; k++)
                            p4 = std::max<int>(p4, m[off + T.p4_gap + k]);
                    }
                    writeJsonlSsr(codeName(code), pos, p1, p3, p4, (int)code);
                }

                if (G.print_detected) {
                    analyzeBurst(pos, code, m, haveR ? &R : nullptr);
                    printSamples(m, show);
                    fputc('\n', G_txt);
                }
            }
        }
        i += (uint32_t)std::max(1, skip);
    }

    return std::min(i, total);
}

// ---------- Statistik ----------
static void printCounters(const char *title, const Counters &c) {
    fprintf(G_txt,
        "%s\n"
        "  Nachrichten gesamt:                           %" PRIu64 "\n"
        "  Mode A:                                       %" PRIu64 "\n"
        "  Mode C:                                       %" PRIu64 "\n"
        "  Mode A All-Call:                              %" PRIu64 "\n"
        "  Mode C All-Call:                              %" PRIu64 "\n"
        "  Mode A All-Call (Compatibility Mode):         %" PRIu64 "\n"
        "  Mode C All-Call (Compatibility Mode):         %" PRIu64 "\n"
        "  Mode S:                                       %" PRIu64 "\n\n",
        title, c.total(), c.a, c.c, c.a_acac, c.c_acac,
        c.a_acsac, c.c_acsac, c.s);
}

static void printOrder() {
    if (G.order.empty()) return;
    fprintf(G_txt, "Reihenfolge erkannter Abfragen:\n");
    size_t n = G.order.size();
    for (size_t i = 0; i < n;) {
        size_t j = i + 1;
        while (j < n && G.order[j] == G.order[i]) j++;
        size_t run = j - i;
        if (run > 1)
            fprintf(G_txt, "  %zu x %s\n", run, codeName((MsgCode)G.order[i]));
        else
            fprintf(G_txt, "  %s\n", codeName((MsgCode)G.order[i]));
        i = j;
    }
    fputc('\n', G_txt);
}

static void printBaseline() {
    if (!G.baseline) return;
    fprintf(G_txt, "Baseline-Werte:\n");
    if (G.avg_pulse.valid())
        fprintf(G_txt, "  Empfohlene min. Pulsamplitude (--mpa):  %u\n", G.avg_pulse.value());
    if (G.avg_nf.valid())
        fprintf(G_txt, "  Empfohlener max. Noise Floor (--mnf):   %u\n", G.avg_nf.value());
    if (G.avg_nfc.valid())
        fprintf(G_txt, "  Empfohlener max. Noise Floor nah (--mnfc): %u\n", G.avg_nfc.value());
    fputc('\n', G_txt);
}

static void printStats(uint64_t samples) {
    if (G.cur.total() == 0) {
        if (!G.continuous) fprintf(G_txt, "Keine Nachrichten erkannt.\n");
    } else {
        char title[96];
        snprintf(title, sizeof(title),
                 "Statistik fuer %" PRIu64 " Samples (%.1f ms):", samples,
                 samples * 1000.0 / G.samplerate);
        printCounters(title, G.cur);
        if (G.continuous) {
            G.cum.add(G.cur);
            printCounters("Kumulativ:", G.cum);
        }
    }
    if (G.print_order) printOrder();
    printBaseline();

    G.cur = Counters{};
    G.order.clear();
}

#if WITH_RTLSDR
class RtlSampleSource : public SampleSource {
public:
    ~RtlSampleSource() override { close(); }

    bool open() override {
    int count = (int)rtlsdr_get_device_count();
    if (count <= 0) die("Keine RTL-SDR-Geraete gefunden");
    fprintf(stderr, "%d Geraet(e) gefunden:\n", count);
    for (int j = 0; j < count; j++) {
        char vendor[256] = "", product[256] = "", serial[256] = "";
        rtlsdr_get_device_usb_strings(j, vendor, product, serial);
        fprintf(stderr, "  %d: %s, %s, SN: %s%s\n", j, vendor, product, serial,
                j == G.dev_index ? " (ausgewaehlt)" : "");
    }
    if (G.dev_index < 0 || G.dev_index >= count) die("Ungueltiger Geraeteindex");

    int r = rtlsdr_open(&G.dev, (uint32_t)G.dev_index);
    if (r < 0) {
        fprintf(stderr, "Fehler: rtlsdr_open() lieferte %d\n", r);
        exit(1);
    }

    if (G.gain == AUTO_GAIN) {
        rtlsdr_set_tuner_gain_mode(G.dev, 0);
        fprintf(stderr, "Automatische Tuner-Verstaerkung\n");
    } else {
        rtlsdr_set_tuner_gain_mode(G.dev, 1);
        if (G.gain == MAX_GAIN) {
            int n = rtlsdr_get_tuner_gains(G.dev, nullptr);
            if (n <= 0) die("Keine Gain-Stufen verfuegbar");
            std::vector<int> gains((size_t)n);
            rtlsdr_get_tuner_gains(G.dev, gains.data());
            G.gain = gains.back();
        }
        if (rtlsdr_set_tuner_gain(G.dev, G.gain) < 0)
            fprintf(stderr, "Warnung: Gain %.1f dB nicht setzbar\n", G.gain / 10.0);
    }
    if (G.agc) rtlsdr_set_agc_mode(G.dev, 1);

    if (G.ppm && rtlsdr_set_freq_correction(G.dev, G.ppm) < 0)
        fprintf(stderr, "Warnung: PPM-Korrektur %d nicht setzbar\n", G.ppm);

    if (rtlsdr_set_center_freq(G.dev, G.freq) < 0) die("Frequenz nicht setzbar");
    if (rtlsdr_set_sample_rate(G.dev, G.samplerate) < 0)
        die("Samplerate nicht setzbar");

    uint32_t real = rtlsdr_get_sample_rate(G.dev);
    if (real && real != G.samplerate) {
        double dev = ((double)real - G.samplerate) / G.samplerate * 1e6;
        fprintf(stderr, "Hinweis: Samplerate %u Hz (angefordert %u, %+.1f ppm)\n",
                real, G.samplerate, dev);
        if (std::fabs(dev) > 1000.0) die("Samplerate weicht zu stark ab");
        G.samplerate = real;
    }
    rtlsdr_reset_buffer(G.dev);
    fprintf(stderr, "Gain laut Geraet: %.1f dB, %.3f MS/s, %d ppm\n",
            rtlsdr_get_tuner_gain(G.dev) / 10.0, G.samplerate / 1e6, G.ppm);
        return true;
    }

    ReadStatus read_block(SampleBlock& out) override {
        out.format = InputFormat::CU8;
        out.sample_rate = G.samplerate;
        out.raw.resize(G.data_len);
        int nread = 0;
        const int rc = rtlsdr_read_sync(
            G.dev, out.raw.data(), G.data_len, &nread
        );
        if (rc < 0 || nread <= 0) {
            if (!g_exit && rc < 0)
                fprintf(stderr, "rtlsdr_read_sync() lieferte %d\n", rc);
            return g_exit ? ReadStatus::EOF_REACHED : ReadStatus::ERROR;
        }
        out.raw.resize(static_cast<size_t>(nread) & ~(size_t)1);
        prepareSampleBlock(out);
        return ReadStatus::DATA;
    }

    double sample_rate() const override { return G.samplerate; }
    bool live() const override { return true; }

    void cancel() override {
#if WITH_RTLSDR
        if (G.dev) rtlsdr_cancel_async(G.dev);
#endif
    }

    void close() override {
        if (G.dev) {
            rtlsdr_cancel_async(G.dev);
            rtlsdr_close(G.dev);
            G.dev = nullptr;
        }
    }
};
#endif

#if WITH_PLUTO
class PlutoSampleSource : public SampleSource {
public:
    ~PlutoSampleSource() override { close(); }

    bool open() override {
        context_ = iio_create_context_from_uri(G.pluto_uri.c_str());
        if (!context_) {
            fprintf(stderr, "Pluto: Kontext '%s' nicht erreichbar\n",
                    G.pluto_uri.c_str());
            return false;
        }
        phy_ = iio_context_find_device(context_, "ad9361-phy");
        capture_ = iio_context_find_device(context_, "cf-ad9361-lpc");
        if (!phy_ || !capture_) {
            fprintf(stderr, "Pluto: ad9361-phy/cf-ad9361-lpc nicht gefunden\n");
            close();
            return false;
        }
        struct iio_channel* lo = iio_device_find_channel(
            phy_, "altvoltage0", true
        );
        struct iio_channel* phy_rx = iio_device_find_channel(
            phy_, "voltage0", false
        );
        struct iio_channel* cap_i = iio_device_find_channel(
            capture_, "voltage0", false
        );
        struct iio_channel* cap_q = iio_device_find_channel(
            capture_, "voltage1", false
        );
        if (!lo || !phy_rx || !cap_i || !cap_q) {
            fprintf(stderr, "Pluto: benoetigte I/Q-Kanaele nicht gefunden\n");
            close();
            return false;
        }
        if (iio_channel_attr_write_longlong(lo, "frequency", G.freq) < 0 ||
            iio_channel_attr_write_longlong(
                phy_rx, "sampling_frequency", G.samplerate
            ) < 0 ||
            iio_channel_attr_write(
                phy_rx, "gain_control_mode", "manual"
            ) < 0) {
            fprintf(stderr, "Pluto: Frequenz/Rate/Gain-Modus nicht setzbar\n");
            close();
            return false;
        }
        long long actual_rate = 0;
        if (iio_channel_attr_read_longlong(
                phy_rx, "sampling_frequency", &actual_rate
            ) >= 0 && actual_rate > 0) {
            if (actual_rate < MIN_RATE || actual_rate > MAX_RATE) {
                fprintf(stderr, "Pluto meldet ungueltige Samplerate %lld Hz\n",
                        actual_rate);
                close();
                return false;
            }
            G.samplerate = static_cast<uint32_t>(actual_rate);
        }
        if (G.pluto_bandwidth &&
            iio_channel_attr_write_longlong(
                phy_rx, "rf_bandwidth", G.pluto_bandwidth
            ) < 0) {
            fprintf(stderr, "Pluto: RF-Bandbreite nicht setzbar\n");
            close();
            return false;
        }
        if (G.pluto_gain_set &&
            iio_channel_attr_write_double(
                phy_rx, "hardwaregain", G.pluto_gain
            ) < 0) {
            fprintf(stderr, "Pluto: Hardware-Gain nicht setzbar\n");
            close();
            return false;
        }
        iio_channel_enable(cap_i);
        iio_channel_enable(cap_q);
        const size_t samples = std::max<size_t>(
            1, G.data_len / sampleBytes(InputFormat::SC16)
        );
        buffer_ = iio_device_create_buffer(capture_, samples, false);
        if (!buffer_) {
            fprintf(stderr, "Pluto: IIO-Puffer konnte nicht erzeugt werden\n");
            close();
            return false;
        }
        return true;
    }

    ReadStatus read_block(SampleBlock& out) override {
        const ssize_t bytes = iio_buffer_refill(buffer_);
        if (bytes < 0) {
            fprintf(stderr, "Pluto: Puffer lesen fehlgeschlagen (%zd)\n", bytes);
            return g_exit ? ReadStatus::EOF_REACHED : ReadStatus::ERROR;
        }
        const uint8_t* begin = static_cast<const uint8_t*>(
            iio_buffer_start(buffer_)
        );
        out.format = InputFormat::SC16;
        out.sample_rate = G.samplerate;
        const size_t valid_bytes =
            static_cast<size_t>(bytes) &
            ~(sampleBytes(InputFormat::SC16) - 1);
        out.raw.assign(begin, begin + valid_bytes);
        prepareSampleBlock(out);
        return bytes > 0 ? ReadStatus::DATA : ReadStatus::EOF_REACHED;
    }

    double sample_rate() const override { return G.samplerate; }
    bool live() const override { return true; }

    void cancel() override {
        if (buffer_) iio_buffer_cancel(buffer_);
    }

    void close() override {
        if (buffer_) iio_buffer_destroy(buffer_);
        if (context_) iio_context_destroy(context_);
        buffer_ = nullptr;
        context_ = nullptr;
    }

private:
    struct iio_context* context_ = nullptr;
    struct iio_device* phy_ = nullptr;
    struct iio_device* capture_ = nullptr;
    struct iio_buffer* buffer_ = nullptr;
};
#endif

// ---------- Reader-Thread ----------
static void *readerThread(void *) {
    SampleBlock block;
    block.raw.reserve(G.data_len);
    while (!g_exit) {
        const ReadStatus status = G.source->read_block(block);
        pthread_mutex_lock(&G.mutex);
        if (G.source->live() && status == ReadStatus::DATA && G.data_ready) {
            ++G.dropped;
        } else {
            while (G.data_ready && !g_exit)
                pthread_cond_wait(&G.cond, &G.mutex);
        }
        if (status == ReadStatus::DATA && !g_exit &&
            (!G.source->live() || !G.data_ready)) {
            G.iq_buf.raw.swap(block.raw);
            G.iq_buf.magnitude.swap(block.magnitude);
            G.iq_buf.format = block.format;
            G.iq_buf.sample_rate = block.sample_rate;
            G.data_ready = true;
        } else {
            if (status != ReadStatus::DATA) {
                G.reader_error = status == ReadStatus::ERROR;
                G.reader_done = true;
            }
        }
        pthread_cond_broadcast(&G.cond);
        pthread_mutex_unlock(&G.mutex);
        if (status != ReadStatus::DATA) break;
    }
    return nullptr;
}

// ---------- Kommandozeile ----------
static void showHelp() {
    printf(
"Verwendung: dump1030 [Optionen]\n"
"Eingabe:\n"
"  --device <n>            RTL-SDR-Geraeteindex (Default 0)\n"
"  --input <pfad|->        Datei oder stdin statt RTL-SDR\n"
"  --file <pfad>           Kompatibilitaetsalias fuer --input\n"
"  --format <cu8|sc16>     Eingangsformat fuer Datei/stdin (Default cu8)\n"
"  --sc16-scale <n>        Vollaussteuerung des SC16-Eingangs (Default 2048)\n"
"  --pluto                  PlutoSDR direkt ueber libiio verwenden\n"
"  --pluto-uri <uri>        Pluto-URI (Default ip:192.168.2.1)\n"
"  --pluto-bandwidth <Hz>   Pluto-RX-Bandbreite\n"
"  --pluto-gain <dB>        Pluto-Hardware-Gain, manueller Modus\n"
"  --gain <dB|auto|max>    Tuner-Gain in dB (Default max)\n"
"  --samplerate <r>        2.0..30 MS/s oder Hz (Default 2.5). Bei Datei:\n"
"                          MUSS der Rate der Aufzeichnung entsprechen!\n"
"                          --chan muss zur Trainingsrate passen.\n"
"  --sample-rate <Hz>      Alias fuer --samplerate\n"
"  --freq <Hz>             Empfangsfrequenz (Default 1030 MHz)\n"
"  --ppm <-200..200>       Frequenzkorrektur des Tuners in ppm\n"
"  --agc                   RTL2832-AGC aktivieren\n"
"  --size <bytes>          Blockgroesse, Vielfaches von 16384, min. 16384\n"
"  --continuous            Fortlaufend messen (--size = Intervall)\n"
"Schwellen:\n"
"  --diff <0..255>         Mindestabstand Puls/Pause (fern)\n"
"  --diffclose <0..255>    Mindestabstand Puls/Pause (direkt daneben)\n"
"  --diffratio <r>         Max. Verhaeltnis Pause/Puls (fern)\n"
"  --diffratioclose <r>    Max. Verhaeltnis Pause/Puls (direkt daneben)\n"
"  --diffratiop4 <r>       Wie --diffratio, nur P4-Pruefung\n"
"  --diffratioclosep4 <r>  Wie --diffratioclose, nur P4-Pruefung\n"
"  --mpa <0..255>          Min. Pulsamplitude\n"
"  --mnf <0..255>          Max. Noise Floor (fern)\n"
"  --mnfc <0..255>         Max. Noise Floor (direkt neben Puls)\n"
"Ausgabe:\n"
"  --out-dir <pfad>        Ausgabeverzeichnis fuer JSONL/Roh-I/Q\n"
"  --save-iq               IQ-Daten im Eingangsformat speichern\n"
"  --p6-diagnostics        Abgelehnte P6-Kandidaten als JSONL ausgeben\n"
"  --p6-quality <0..255>   Minimale Viterbi-Qualitaet (Default 180)\n"
"  --p6-coverage <0..1>    Minimale P6-Huellkurvenabdeckung (Default 0.65)\n"
"  --p6-snr <dB>           Minimales P6-SNR (Default 6)\n"
"  --p6-pre-snr <dB>       Minimales Praeambel-SNR (Default 3)\n"
"  --p6-p1-ratio <r>       Maximales P6/P1-Verhaeltnis (Default 4)\n"
"  --p6-rho <0..1>         Minimale Praeambelkorrelation (Default 0.50)\n"
"  --p6-lock <0..1>        Minimale SPR-Lock-Qualitaet (Default 0.50)\n"
"  --p6-timing-radius <n>  Viterbi-Feintiming um Besttreffer (0..8, Default 1)\n"
"  --chan <pfad>           DPSK-Kanaldatei fuer den Uplink-Decoder\n"
"  --msgs                  Erkannte Abfragen mit Samples ausgeben\n"
"  --order                 Reihenfolge der Abfragen ausgeben\n"
"  --blmode                Baseline-Werte fuer mpa/mnf/mnfc ausgeben\n"
"  --print                 Alle Magnitude-Werte ausgeben (langsam!)\n"
"  --uplink-decode         EXPERIMENTELL: Mode-S-P6 (DPSK) demodulieren\n"
"  --help                  Diese Hilfe\n");
}

static const char *needArg(int argc, char **argv, int &i) {
    if (i + 1 >= argc) {
        fprintf(stderr, "Fehler: %s erwartet einen Wert\n", argv[i]);
        exit(1);
    }
    return argv[++i];
}

static long parseLong(const char *opt, const char *s, long lo, long hi) {
    char *end = nullptr;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (errno || end == s || *end || v < lo || v > hi) {
        fprintf(stderr, "Fehler: %s erwartet Ganzzahl %ld..%ld, nicht '%s'\n",
                opt, lo, hi, s);
        exit(1);
    }
    return v;
}

static double parseFloat(const char *opt, const char *s, double lo, double hi) {
    char *end = nullptr;
    errno = 0;
    double v = strtod(s, &end);
    if (errno || end == s || *end || !(v >= lo && v <= hi)) {
        fprintf(stderr, "Fehler: %s erwartet Zahl %.2f..%.2f, nicht '%s'\n",
                opt, lo, hi, s);
        exit(1);
    }
    return v;
}

static void parseArgs(int argc, char **argv) {
    Thresholds &t = G.th;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "--device")) {
            G.dev_index = (int)parseLong(a, needArg(argc, argv, i), 0, 255);
        } else if (!strcmp(a, "--file") || !strcmp(a, "--input")) {
            G.input_name = needArg(argc, argv, i);
            G.filename = G.input_name.c_str();
        } else if (!strcmp(a, "--format")) {
            const char* format = needArg(argc, argv, i);
            if (!strcmp(format, "cu8")) G.input_format = InputFormat::CU8;
            else if (!strcmp(format, "sc16"))
                G.input_format = InputFormat::SC16;
            else die("--format erwartet cu8 oder sc16");
            G.input_format_set = true;
        } else if (!strcmp(a, "--sc16-scale")) {
            G.sc16_scale = parseFloat(
                a, needArg(argc, argv, i), 1.0, 32768.0
            );
        } else if (!strcmp(a, "--pluto")) {
            G.use_pluto = true;
        } else if (!strcmp(a, "--pluto-uri")) {
            G.pluto_uri = needArg(argc, argv, i);
            G.use_pluto = true;
        } else if (!strcmp(a, "--pluto-bandwidth")) {
            G.pluto_bandwidth = static_cast<uint32_t>(parseLong(
                a, needArg(argc, argv, i), 1, 100'000'000
            ));
            G.use_pluto = true;
        } else if (!strcmp(a, "--pluto-gain")) {
            G.pluto_gain = parseFloat(
                a, needArg(argc, argv, i), -10.0, 73.0
            );
            G.pluto_gain_set = true;
            G.use_pluto = true;
        } else if (!strcmp(a, "--chan")) {
            g_haveChan = loadDpskChannel(needArg(argc, argv, i), g_chan);
            if (g_haveChan) uplinkAddrSelftest();
        } else if (!strcmp(a, "--gain")) {
            const char *v = needArg(argc, argv, i);
            if (!strcmp(v, "auto"))     G.gain = AUTO_GAIN;
            else if (!strcmp(v, "max")) G.gain = MAX_GAIN;
            else G.gain = (int)lround(parseFloat(a, v, 0.0, 60.0) * 10.0);
        } else if (!strcmp(a, "--samplerate") ||
                   !strcmp(a, "--sample-rate")) {
            const char *v = needArg(argc, argv, i);
            double r = parseFloat(a, v, 0.0, 31.0e6);
            if (r < 100.0) r *= 1e6;
            uint32_t rate = (uint32_t)lround(r);
            if (rate < MIN_RATE || rate > MAX_RATE) {
                fprintf(stderr, "Fehler: --samplerate muss %.1f..%.1f MS/s sein "
                        "(Pulsbreite 0,8 us braucht >= 2 Samples)\n",
                        MIN_RATE / 1e6, MAX_RATE / 1e6);
                exit(1);
            }
            G.samplerate = rate;
        } else if (!strcmp(a, "--freq")) {
            G.freq = static_cast<uint32_t>(parseLong(
                a, needArg(argc, argv, i), 1, UINT32_MAX
            ));
        } else if (!strcmp(a, "--ppm")) {
            G.ppm = (int)parseLong(a, needArg(argc, argv, i), -200, 200);
        } else if (!strcmp(a, "--uplink-decode")) {
            G.uplink_decode = true;
        } else if (!strcmp(a, "--agc")) {
            G.agc = true;
        } else if (!strcmp(a, "--size")) {
            long v = parseLong(a, needArg(argc, argv, i), MIN_LEN, 64L * 1024 * 1024);
            G.data_len = (uint32_t)(v - v % MIN_LEN);
        } else if (!strcmp(a, "--continuous")) {
            G.continuous = true;
        } else if (!strcmp(a, "--diff")) {
            t.diff = (int)parseLong(a, needArg(argc, argv, i), 0, 255);
        } else if (!strcmp(a, "--diffclose")) {
            t.diffclose = (int)parseLong(a, needArg(argc, argv, i), 0, 255);
        } else if (!strcmp(a, "--diffratio")) {
            t.ratio = toQ8(parseFloat(a, needArg(argc, argv, i), 0.0, 4.0));
        } else if (!strcmp(a, "--diffratioclose")) {
            t.ratioclose = toQ8(parseFloat(a, needArg(argc, argv, i), 0.0, 4.0));
        } else if (!strcmp(a, "--diffratiop4")) {
            t.ratiop4 = toQ8(parseFloat(a, needArg(argc, argv, i), 0.0, 4.0));
        } else if (!strcmp(a, "--diffratioclosep4")) {
            t.ratioclosep4 = toQ8(parseFloat(a, needArg(argc, argv, i), 0.0, 4.0));
        } else if (!strcmp(a, "--mpa")) {
            t.min_peak = (int)parseLong(a, needArg(argc, argv, i), 0, 255);
        } else if (!strcmp(a, "--mnf")) {
            t.max_nf = (int)parseLong(a, needArg(argc, argv, i), 0, 255);
        } else if (!strcmp(a, "--mnfc")) {
            t.max_nfc = (int)parseLong(a, needArg(argc, argv, i), 0, 255);
        } else if (!strcmp(a, "--msgs")) {
            G.print_detected = true;
        } else if (!strcmp(a, "--order")) {
            G.print_order = true;
        } else if (!strcmp(a, "--blmode")) {
            G.baseline = true;
        } else if (!strcmp(a, "--print")) {
            G.print_all = true;
        } else if (!strcmp(a, "--dl")) {
            die("--dl (1090 MHz) wird nicht unterstuetzt");
        } else if (!strcmp(a, "--out-dir")) {
            G.out_dir = needArg(argc, argv, i);
        } else if (!strcmp(argv[i], "--save-iq")) {
            G.save_iq = true;
        } else if (!strcmp(a, "--p6-diagnostics")) {
            G.p6_diagnostics = true;
        } else if (!strcmp(a, "--p6-quality")) {
            G.p6_min_quality = (int)parseLong(a, needArg(argc, argv, i), 0, 255);
        } else if (!strcmp(a, "--p6-coverage")) {
            G.p6_min_coverage = parseFloat(a, needArg(argc, argv, i), 0.0, 1.0);
        } else if (!strcmp(a, "--p6-snr")) {
            G.p6_min_snr_db = parseFloat(a, needArg(argc, argv, i), -60.0, 120.0);
        } else if (!strcmp(a, "--p6-pre-snr")) {
            G.p6_min_pre_snr_db = parseFloat(a, needArg(argc, argv, i), -60.0, 120.0);
        } else if (!strcmp(a, "--p6-p1-ratio")) {
            G.p6_max_p1_ratio = parseFloat(a, needArg(argc, argv, i), 0.0, 20.0);
        } else if (!strcmp(a, "--p6-rho")) {
            G.p6_min_rho = parseFloat(a, needArg(argc, argv, i), 0.0, 1.0);
        } else if (!strcmp(a, "--p6-lock")) {
            G.p6_min_lock = parseFloat(a, needArg(argc, argv, i), 0.0, 1.0);
        } else if (!strcmp(a, "--p6-timing-radius")) {
            G.p6_timing_radius =
                (int)parseLong(a, needArg(argc, argv, i), 0, 8);
        } else if (!strcmp(a, "--help")) {
            showHelp();
            exit(0);
        } else {
            fprintf(stderr, "Unbekannte Option: %s\n\n", a);
            showHelp();
            exit(1);
        }
    }
    if (G.uplink_decode && !g_haveChan)
        die("--uplink-decode benoetigt --chan");
    if (G.p6_diagnostics && G.out_dir.empty())
        die("--p6-diagnostics benoetigt --out-dir");
    if (G.p6_diagnostics && !G.uplink_decode)
        die("--p6-diagnostics benoetigt --uplink-decode");
    if (G.use_pluto && !G.input_name.empty())
        die("--pluto und --input sind nicht kombinierbar");
    if (G.use_pluto) {
        if (G.input_format_set && G.input_format != InputFormat::SC16)
            die("--pluto benoetigt --format sc16");
        G.input_format = InputFormat::SC16;
    }
}

// ---------- Verarbeitung ----------
static uint32_t fetchBlock(bool &last) {
    uint32_t len = 0;
    pthread_mutex_lock(&G.mutex);
    while (!G.data_ready && !G.reader_done && !g_exit) {
        timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 200000000L;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        pthread_cond_timedwait(&G.cond, &G.mutex, &ts);
    }
    if (G.data_ready) {
        std::swap(G.iq_buf, G.iq_work);
        len = static_cast<uint32_t>(G.iq_work.raw.size());
        G.data_ready = false;
        pthread_cond_broadcast(&G.cond);
    }
    last = G.reader_done && !G.data_ready;
    pthread_mutex_unlock(&G.mutex);
    return len;
}

static uint64_t g_samples = 0;

static void processBlock(uint32_t len, bool last) {
    if (G.iq_fp && len >= sampleBytes(G.input_format)) {
        fwrite(G.iq_work.raw.data(), 1, G.iq_work.raw.size(), G.iq_fp);
    }

    static const SampleBlock empty;
    const SampleBlock& input = len ? G.iq_work : empty;
    uint32_t total = computeMagnitude(input, last);
    g_valid_samples = total;
    uint32_t consumed = detectBlock(total, last);
    keepOverlap(total, consumed);
    g_samples += input.sample_count();
    if (G.continuous) {
        printStats(g_samples);
        if (G.jsonl_fp) fflush(G.jsonl_fp);
        g_samples = 0;
    }
}

// ---------- main ----------
int main(int argc, char **argv) {
    parseArgs(argc, argv);
    if (G.use_pluto) {
#if WITH_PLUTO
        G.source.reset(new PlutoSampleSource());
#else
        die("Pluto-Unterstuetzung ist in diesem Build nicht enthalten; "
            "mit WITH_PLUTO=1 bauen");
#endif
    } else if (!G.input_name.empty()) {
        G.source.reset(new FileSampleSource(
            G.input_name, G.input_format, G.data_len, G.samplerate
        ));
    } else {
#if WITH_RTLSDR
        G.input_format = InputFormat::CU8;
        G.source.reset(new RtlSampleSource());
#else
        die("Kein Eingangs-Backend verfuegbar; --input verwenden oder "
            "mit WITH_RTLSDR=1 bauen");
#endif
    }
    if (!G.source->open()) return 1;
    G.tm.init(G.samplerate);
    if (g_haveChan) {
        const double rate_error = std::abs(
            (double)g_chan.input_sample_rate - (double)G.samplerate
        ) / std::max<double>(1.0, G.samplerate);
        if (rate_error > 0.001) {
            fprintf(stderr,
                    "Kanaldatei wurde bei %u Hz trainiert; "
                    "Eingangsrate ist %u Hz\n",
                    g_chan.input_sample_rate, G.samplerate);
            G.source->close();
            return 1;
        }
    }
    if (!initOutputs()) {
        G.source->close();
        return 1;
    }
    buildLut();

    G.lookahead = (uint32_t)G.tm.lookaheadSamples();
    if (G.uplink_decode)
        fprintf(stderr, "Hinweis: --uplink-decode ist experimentell und nicht "
                        "validiert (Quality-Gates in JSONL beachten)\n");

    const size_t max_samples =
        (size_t)G.data_len / sampleBytes(G.input_format);
    G.mag.resize(max_samples + 2 * (size_t)G.lookahead);
    G.iq_hist.format = G.input_format;
    G.iq_hist.sample_rate = G.samplerate;
    G.iq_hist.raw.resize(
        sampleBytes(G.input_format) *
        (max_samples + 2 * (size_t)G.lookahead)
    );

    struct sigaction sa{};
    sa.sa_handler = sigHandler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    if (pthread_create(&G.reader_thread, nullptr, readerThread, nullptr) != 0) {
        fprintf(stderr, "Fehler: Reader-Thread konnte nicht gestartet werden\n");
        G.source->close();
        closeOutputs();
        return 1;
    }

    const bool single =
        !G.use_pluto && G.input_name.empty() && !G.continuous;
    while (!g_exit) {
        bool last = false;
        uint32_t len = fetchBlock(last);
        if (len == 0) {
            if (last) {
                if (G.overlap > G.prefix_keep) processBlock(0, true);
                break;
            }
            continue;
        }
        processBlock(len, last || single);
        if (single || last) break;
    }

    if (!G.continuous) printStats(g_samples);
    if (G.dropped)
        fprintf(stderr, "Warnung: %" PRIu64 " Puffer verworfen "
                        "(Verarbeitung zu langsam)\n", G.dropped);
    if (G.uplink_decode) {
        fprintf(stderr, "Uplink-Decoder: %" PRIu64 " akzeptiert\n",
                G.uplink_ok);
    }

    // ---------- Aufraeumen ----------
    g_exit = true;
    if (G.source) G.source->cancel();
    pthread_mutex_lock(&G.mutex);
    pthread_cond_broadcast(&G.cond);
    pthread_mutex_unlock(&G.mutex);
    pthread_join(G.reader_thread, nullptr);

    G.source->close();
    closeOutputs();
    pthread_cond_destroy(&G.cond);
    pthread_mutex_destroy(&G.mutex);
    const int signal_number = g_signal.load();
    return signal_number ? 128 + signal_number
                         : (G.reader_error ? 1 : 0);
}
