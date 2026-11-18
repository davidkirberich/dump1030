// dump1030 - SSR-Abfrage-Detektor (1030 MHz)
// Build: g++ -O2 -std=gnu++17 -Wall -Wextra -pthread \
//   dump1030.cpp -lrtlsdr -o dump1030

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <complex>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <rtl-sdr.h>

// ---------- Konstanten ----------
static constexpr double PI           = 3.14159265358979323846;
static const uint32_t DEFAULT_FREQ   = 1030000000;
static const uint32_t DEFAULT_RATE   = 2500000;
static const uint32_t MIN_RATE       = 2400000;
static const uint32_t MAX_RATE       = 3200000;
static const uint32_t DEFAULT_LEN    = 262144;   // Bytes (I/Q)
static const uint32_t MIN_LEN        = 16384;
static const int      AUTO_GAIN      = -100;
static const int      MAX_GAIN       = 999999;
// Gemeinsamer Overlap fuer Magnitude UND I/Q (Samples).
// Laengster Zugriff: Mode S 112 Bit endet 33,75 us nach P1
// -> 108 Samples bei 3,2 MS/s + Stuetzstellen der Interpolation.
static const int      MAX_LOOKAHEAD      = 120;

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
static inline int toQ8(float r) { return (int)lroundf(r * 256.0f); }

struct Thresholds {
    int diff         = 10;   // Mindestabstand Puls/Rauschen (fern)
    int diffclose    = 5;    // Mindestabstand Puls/Rauschen (direkt daneben)
    int ratio        = toQ8(0.25f);
    int ratioclose   = toQ8(0.75f);
    int ratiop4      = toQ8(0.50f);
    int ratioclosep4 = toQ8(1.50f);
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

// ---------- Timing (zur Laufzeit aus der Samplerate berechnet) ----------
struct Timing {
    double sps = 2.5;          // Samples pro us
    int pw = 2;                // Pulsbreite (0,8 us) in Samples
    int p2 = 5, p3_a = 20, p3_c = 52;
    int p4_gap = 5, p4_short = 2, p4_long = 4;
    int skip_s56 = 50, skip_s112 = 85;
    int min_skip_s = 0;        // Fallback ohne Decoder

    int s(double t_us) const { return (int)lround(t_us * sps); }
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
    }
    int maxPulseIndex() const { return p3_c + pw + p4_gap + p4_long + 3; }
};

// ---------- Qualitaetsmetrik eines demodulierten Uplinks ----------
struct UplinkResult {
    bool    valid = false;
    int     nbits = 0;           // 56 oder 112
    uint8_t bytes[14] = {0};
    float   margin[112] = {0};   // |cos dphi| pro Bit
    double  spr_us = 0.0;        // gelockter SPR-Zeitpunkt
    double  spr_score = 0.0;     // 0..1, Schaerfe des Locks
    double  mean_margin = 0.0;   // mittl. |cos dphi| der Bitentscheidungen
    double  min_margin = 0.0;    // schlechteste Bitentscheidung
    int     weak_bits = 0;       // Bits mit |cos dphi| < 0,3
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
    std::vector<uint8_t> iq_buf;     // vom Reader befuellt
    uint32_t        iq_len = 0;      // gueltige Bytes in iq_buf
    bool            data_ready = false;
    bool            reader_done = false;
    uint64_t        dropped = 0;     // verworfene Puffer (Main zu langsam)

    // Verarbeitung
    std::vector<uint8_t>  iq_work;   // Arbeitskopie
    std::vector<uint8_t>  iq_hist;   // Overlap + neue Samples (I/Q)
    std::vector<uint8_t>  mag;       // Overlap + neue Samples (Magnitude)
    uint32_t              overlap = 0;
    uint64_t              sample_base = 0; // absoluter Index von mag[0]
    std::vector<uint8_t>  lut16;     // 65536 Eintraege, Index = I | Q<<8
    std::vector<uint8_t>  order;     // Reihenfolge erkannter Codes

    // Optionen
    Thresholds th;
    Timing   tm;
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
    const char *filename = nullptr;

    // Statistik
    Counters cur, cum;
    Average  avg_pulse, avg_nf, avg_nfc;
    double   last_burst_us = -1.0;
    uint64_t uplink_ok = 0, uplink_weak = 0;

    // RTL-SDR / Datei
    rtlsdr_dev_t *dev = nullptr;
    int fd = -1;

    // Ausgabedirectory für JSONL und .cu8
    std::string out_dir;
    bool        save_iq  = false;
    FILE*       jsonl_fp = nullptr;
    FILE*       iq_fp    = nullptr;
};

static State G;
static_assert(std::atomic<bool>::is_always_lock_free,
              "g_exit muss lock-free sein (Signal-Handler)");
static std::atomic<bool> g_exit{false};

static void sigHandler(int) {
    g_exit = true;   // nur async-signal-safe Operationen
}

[[noreturn]] static void die(const char *msg) {
    fprintf(stderr, "Fehler: %s\n", msg);
    exit(1);
}

// ---------- JSONL/cu8 Ausgabe ----------

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
        std::string iq_path = G.out_dir + "/raw_" + ts + ".cu8";
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

// ---------- JSONL/cu8 Ausgabe ----------

static double monotonicSec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static inline uint32_t ufOf(const uint8_t *b);

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
                       : (R.has_addr && R.addrCnt >= 2) ? "confirmed" : "candidate";

    fprintf(G.jsonl_fp,
        "{\"type\":\"P6\",\"smp\":%llu,\"mono\":%.6f,\"uf\":%d,\"mode5\":%s,\"nbits\":%d,"
        "\"p1\":%d,\"p2\":%d,\"snr_db\":%.2f,\"df_hz\":%.1f,"
        "\"spr_us\":%.3f,\"lock\":%.3f,\"cov\":%.3f,\"p6min\":%.3f,"
        "\"p6snr_db\":%.2f,\"pre_snr_db\":%.2f,\"p6_p1_ratio\":%.2f,"
        "\"rho\":%.3f,\"q\":%u,\"status\":\"%s\","
        "\"aa\":%s,\"aa_cnt\":%u,\"hex\":\"%s\"}\n",
        (unsigned long long)smp, monotonicSec(),
        (int)ufOf(R.bytes), R.mode5 ? "true" : "false", R.nbits,
        p1, p2, snr_db, R.freq_off_hz,
        R.spr_us, R.spr_score, R.p6_cov, R.p6_min_ratio, R.p6_snr_db,
        R.pre_snr_db, R.p6_p1_ratio, R.pre_rho, (unsigned)R.quality, status, aa_json,
        (!R.mode5 && R.has_addr) ? (unsigned)R.addrCnt : 0u, hex);
    fflush(G.jsonl_fp);
}

static void writeJsonlSsr(const char* subtype, uint64_t smp, int p1, int p3, int p4, int code) {
    if (!G.jsonl_fp) return;
    fprintf(G.jsonl_fp,
        "{\"type\":\"SSR\",\"subtype\":\"%s\",\"smp\":%llu,\"mono\":%.6f,"
        "\"p1\":%d,\"p3\":%d,\"p4\":%d,\"code\":%d}\n",
        subtype, (unsigned long long)smp, monotonicSec(), p1, p3, p4, code);
    fflush(G.jsonl_fp);
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
        "\"reason\":\"%s\",\"uf\":%d,\"nbits\":%d,\"p1\":%d,\"p2\":%d,"
        "\"snr_db\":%.2f,\"df_hz\":%.1f,\"spr_us\":%.3f,\"lock\":%.3f,"
        "\"cov\":%.3f,\"p6min\":%.3f,\"p6snr_db\":%.2f,"
        "\"pre_snr_db\":%.2f,\"p6_p1_ratio\":%.2f,\"rho\":%.3f,"
        "\"q\":%u,\"hex\":\"%s\"}\n",
        (unsigned long long)smp, monotonicSec(),
        R.reject_reason ? R.reject_reason : "unknown",
        uf, R.nbits, p1, p2, snr_db, R.freq_off_hz, R.spr_us,
        R.spr_score, R.p6_cov, R.p6_min_ratio, R.p6_snr_db,
        R.pre_snr_db, R.p6_p1_ratio, R.pre_rho, (unsigned)R.quality, hex);
    fflush(G.jsonl_fp);
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

// Anzahl gueltiger (echter, nicht Reserve-) Samples im aktuellen Block.
static uint32_t g_valid_samples = 0;

static void ensureCapacity(uint32_t total) {
    size_t need = (size_t)total + MAX_LOOKAHEAD;
    if (G.mag.size() < need)          G.mag.resize(need);
    if (G.iq_hist.size() < 2 * need)  G.iq_hist.resize(2 * need);
}

// Haengt iq_work[0..len) hinter den Overlap und berechnet die Magnitude.
// Rueckgabe: Anzahl gueltiger Samples (Overlap + neue).
static uint32_t computeMagnitude(uint32_t len) {
    uint32_t n_new = len / 2;                    // ungerades Byte ignorieren
    uint32_t total = G.overlap + n_new;
    ensureCapacity(total);

    uint8_t *iq = G.iq_hist.data();
    if (n_new)
        memcpy(iq + 2 * (size_t)G.overlap, G.iq_work.data(), 2 * (size_t)n_new);

    const uint8_t *src = iq + 2 * (size_t)G.overlap;
    uint8_t *m = G.mag.data() + G.overlap;
    const uint8_t *lut = G.lut16.data();
    for (uint32_t j = 0; j < n_new; j++)
        m[j] = lut[src[2 * j] | (src[2 * j + 1] << 8)];

    // Reserve: Magnitude 0, I/Q = 128 (Naeherung Nullpunkt 127,5).
    std::fill(G.mag.begin() + total, G.mag.begin() + total + MAX_LOOKAHEAD, 0);
    std::fill(G.iq_hist.begin() + 2 * (size_t)total,
              G.iq_hist.begin() + 2 * ((size_t)total + MAX_LOOKAHEAD), 128);
    return total;
}

// Bewahrt die letzten MAX_LOOKAHEAD Samples beider Puffer synchron auf.
static void keepOverlap(uint32_t total, uint32_t consumed) {
    if (consumed > total) consumed = total;
    uint32_t keep = total - consumed;
    if (keep > (uint32_t)MAX_LOOKAHEAD) {
        consumed += keep - MAX_LOOKAHEAD;
        keep = MAX_LOOKAHEAD;
    }
    if (keep) {
        memmove(G.mag.data(), G.mag.data() + consumed, keep);
        memmove(G.iq_hist.data(), G.iq_hist.data() + 2 * (size_t)consumed,
                2 * (size_t)keep);
    }
    G.sample_base += consumed;
    G.overlap = keep;
}

static FILE *G_txt = stdout;

// ---------- Detektion: rate-unabhaengige Bausteine ----------
static inline bool ratioGt(int a, int b, int rq) { return a * 256 > rq * b; }

static inline int pulseRef(const uint8_t *m, int start, int w) {
    int r = 255;
    for (int k = 0; k < w; k++) r = std::min<int>(r, m[start + k]);
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
    if (!pauseClose(m[from], ref) || !pauseClose(m[to], ref)) return false;
    for (int k = from + 1; k < to; k++)
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
    if (m[0] <= G.th.min_peak || m[T.pw - 1] <= G.th.min_peak) return false;
    int r1 = pulseRef(m, 0, T.pw);
    if (r1 <= G.th.min_peak) return false;
    return pauseClose(m[T.pw], r1) && pauseFar(m[T.pw + 1], r1);
}

static bool checkModeS(const uint8_t *m) {
    const Timing &T = G.tm;
    if (!pulseOk(m, T.p2, T.pw)) return false;
    int r1 = pulseRef(m, 0, T.pw);
    int r2 = pulseRef(m, T.p2, T.pw);
    if (!pauseBetween(m, T.pw, T.p2 - 1, r1, r2)) return false;
    int ref = std::min(r1, r2);
    return pauseClose(m[T.p2 + T.pw], ref) && pauseClose(m[T.p2 + T.pw + 1], ref);
}

static bool checkP3(const uint8_t *m, int off) {
    const Timing &T = G.tm;
    if (!pulseOk(m, off, T.pw)) return false;
    int r1 = pulseRef(m, 0, T.pw);
    int r3 = pulseRef(m, off, T.pw);
    if (!pauseBetween(m, T.pw, off - 1, r1, r3)) return false;
    return pauseClose(m[off + T.pw], r3) && pauseFar(m[off + T.pw + 1], r3);
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
        return p4Pause(p[-1], ref, true) && p4Pause(p[-2], ref, false) &&
               p4Pause(p[-3], ref, true);
    };

    if (pulseOk(p, 0, T.p4_long)) {
        int r = pulseRef(p, 0, T.p4_long);
        if (before(r) && p4Pause(p[T.p4_long], r, true))
            return P4_LONG;
    }
    if (pulseOk(p, 0, T.p4_short)) {
        int r = pulseRef(p, 0, T.p4_short);
        if (before(r) &&
            p4Pause(p[T.p4_short], r, true) &&
            p4Pause(p[T.p4_short + 1], r, false))
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
    const uint8_t *p = G.iq_hist.data() + 2 * (size_t)k;
    return cd((double)p[0] - 127.5, (double)p[1] - 127.5);
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

// ---------- Frequenzoffset ----------
struct FreqEst {
    bool   ok = false;
    double w  = 0.0;          // rad pro Sample
    double hz = 0.0;
    double coherence = 0.0;   // 0..1, Guete der Schaetzung
};

static FreqEst estimateFreqOffset(int i) {
    FreqEst fe;
    const Timing &T = G.tm;
    int a = i + (int)std::ceil(T.f(3.6));
    int b = i + (int)std::floor(T.f(4.6));
    if (a < 0 || b >= (int)g_valid_samples || b - a < 2) return fe;

    cd acc = 0.0;
    double norm = 0.0;
    for (int k = a; k < b; k++) {
        cd d = iqAt(k + 1) * std::conj(iqAt(k));
        double ad = std::abs(d);
        if (ad < 1e-9) continue;
        acc  += d * d / ad;
        norm += ad;
    }
    if (norm <= 0.0) return fe;
    fe.coherence = std::abs(acc) / norm;
    fe.w  = std::arg(acc) / 2.0;
    fe.hz = fe.w * G.samplerate / (2.0 * PI);
    fe.ok = fe.coherence > 0.5 && std::fabs(fe.hz) < 150e3;
    if (!fe.ok) { fe.w = 0.0; }
    return fe;
}

static inline bool diffCos(double ta, double tb, double w, double &c) {
    cd za, zb;
    if (!getIQSample(ta, za) || !getIQSample(tb, zb)) return false;
    cd d = zb * std::conj(za) * std::polar(1.0, -w * (tb - ta));
    double n = std::abs(za) * std::abs(zb);
    c = (n > 1e-9) ? d.real() / n : 0.0;
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
    const double step = 0.05;
    const int n = (int)lround((us::SPR_MAX - us::SPR_MIN) / step) + 1;
    std::vector<double> sc((size_t)n);
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

static const double WEAK_MARGIN = 0.3;
static constexpr int MIN_VITERBI_QUALITY = 180;
static constexpr double MIN_P6_SNR_DB = 6.0;
static constexpr double MIN_PREAMBLE_SNR_DB = 3.0;
static constexpr double MAX_P6_TO_P1_RATIO = 4.0;

// ===================== Viterbi-DPSK (Paket 1) =====================
// Zeitachse wie p6sim: 20-MS/s-Index, P1-Vorderflanke = 160, Datenchip j ab anchor + 5*j
#include <vector>
#include <complex>
#include <cmath>
#include <cstdint>
#include <limits>
#include <algorithm>

struct DpskChannel {
    std::vector<float> g0;   // Antwort auf Einheitschip j=0 (20 MS/s)
    std::vector<float> kn;   // bekannter Anteil P1/P2/P6/SPR (20 MS/s)
    int   anchor = 265;      // Datenbeginn (ST_ANCHOR)
    float delay  = 0.f;      // Kanal-Laufzeit in 20-MS/s-Samples

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
    float metric = 0.f;      // mittlerer quadratischer Fehler pro Sample
    int   nsamp  = 0;
};

// r: Frame-Fenster (df-korrigiert), r[k] liegt bei Modellzeit off + 8k
ViterbiResult viterbiDpsk(const DpskChannel& ch, const std::complex<float>* r, int nr,
                          double off, float phi, float A, int n)
{
    constexpr int L = 7, S = 1 << (L - 1), H = (L - 1) / 2;
    const int N = n + 2;                                   // + 2 Guard-Chips
    struct Smp { int w; float kv; float B[L]; std::complex<float> z; };
    std::vector<std::vector<Smp>> by(N);
    const std::complex<float> der = std::polar(1.f / A, -phi);

    int ns = 0;
    for (int k = 0; k < nr; ++k) {
        double t = off + 8.0 * k;
        int jc = (int)std::floor((t - ch.delay - ch.anchor) / 5.0);
        int w = jc - H, e = w + L - 1;
        if (e < 0 || w > N - 1) continue;
        Smp s;
        s.w = w; s.kv = ch.knv(t); s.z = r[k] * der;
        for (int i = 0; i < L; ++i) s.B[i] = (w + i >= 0) ? ch.g(w + i, t) : 0.f;
        by[std::min(e, N - 1)].push_back(s);
        ++ns;
    }

    // Zustand: letzte 6 Chips, Bit 0 = jüngster, 1 = +1, 0 = -1
    const float INF = std::numeric_limits<float>::infinity();
    std::vector<float> met(S, INF), nmet(S);
    std::vector<uint8_t> tb((size_t)N * S);                // Vorgängerzustand
    met[0] = 0.f;                                          // nach SPR: alle -1

    for (int j = 0; j < N; ++j) {
        std::fill(nmet.begin(), nmet.end(), INF);
        uint8_t* tbj = &tb[(size_t)j * S];
        for (int st = 0; st < S; ++st) {
            if (met[st] == INF) continue;
            for (int cb = 0; cb < 2; ++cb) {
                if (j >= n && cb != (st & 1)) continue;    // Guard: keine Umkehr
                float m = met[st];
                for (const Smp& s : by[j]) {
                    float v = s.kv;
                    for (int i = 0; i < L; ++i) {
                        int d = j - (s.w + i);             // 0 = aktueller Chip
                        if (d < 0) continue;
                        int bit = (d == 0) ? cb : (st >> (d - 1)) & 1;
                        v += bit ? s.B[i] : -s.B[i];
                    }
                    m += std::norm(s.z - v);
                }
                int nst = ((st << 1) | cb) & (S - 1);
                if (m < nmet[nst]) { nmet[nst] = m; tbj[nst] = (uint8_t)st; }
            }
        }
        met.swap(nmet);
    }

    int best = 0;
    for (int st = 1; st < S; ++st) if (met[st] < met[best]) best = st;

    std::vector<int8_t> chip(N);
    for (int j = N - 1, st = best; j >= 0; --j) {
        chip[j] = (st & 1) ? 1 : -1;
        st = tb[(size_t)j * S + st];
    }

    ViterbiResult res;
    res.nsamp  = ns;
    res.metric = met[best] / std::max(1, ns);
    res.bits.resize(n);
    int prev = -1;
    for (int j = 0; j < n; ++j) { res.bits[j] = (chip[j] != prev); prev = chip[j]; }
    return res;
}
// =================== Ende Viterbi-DPSK ===================

#include <fstream>
#include <string>

DpskChannel g_chan;
bool g_haveChan = false;

bool loadDpskChannel(const char* path, DpskChannel& ch)
{
    std::ifstream f(path);
    std::string magic;
    int ng = 0, nk = 0;
    if (!(f >> magic >> ch.anchor >> ch.delay >> ng >> nk) || magic != "DPSKCHAN1"
        || ng <= 0 || nk <= 0 || ng > 100000 || nk > 100000) {
        fprintf(stderr, "Kanaldatei %s ungueltig\n", path);
        return false;
    }
    ch.g0.resize(ng);
    ch.kn.resize(nk);
    for (auto& v : ch.g0) if (!(f >> v)) { fprintf(stderr, "Kanaldatei: g0 zu kurz\n"); return false; }
    for (auto& v : ch.kn) if (!(f >> v)) { fprintf(stderr, "Kanaldatei: kn zu kurz\n"); return false; }
    float e = 0.f;
    for (float v : ch.g0) e += v;
    fprintf(stderr, "Kanal geladen: %s  anchor %d  delay %.2f  g0 %d  kn %d  sum(g0) %.3f\n",
            path, ch.anchor, ch.delay, ng, nk, e);
    return true;
}

// ===================== Blind-Decode (Paket 3) =====================
#include <cstdio>
#include <sstream>

constexpr int BD_WIN = 125;          // Fenster ab 20 Samples (8 µs) vor P1

struct BlindResult {
    ViterbiResult v;
    double c = 0;                    // Modellzeit von r[0]
    float  phi = 0.f, A = 0.f;
    double dfHz = 0;
    double rho = 0.0;
};

// wie rt_frame: Fenster holen, df schätzen und entfernen
static bool bdFrame(const uint8_t* iq, size_t nPairs, long k, int n,
                    std::complex<float>* r, double& dfHz)
{
    long s0 = k - 20;
    if (s0 < 0 || (size_t)(s0 + BD_WIN) > nPairs) return false;
    for (int j = 0; j < BD_WIN; ++j)
        r[j] = { iq[2 * (s0 + j)] - 127.5f, iq[2 * (s0 + j) + 1] - 127.5f };
    int end = 20 + (n == 56 ? 49 : 84);
    std::complex<double> acc = 0;
    for (int j = 29; j < end; ++j) {
        std::complex<double> d = std::complex<double>(r[j]) * std::conj(std::complex<double>(r[j - 1]));
        acc += d * d;
    }
    double w = (acc != 0.0) ? std::arg(acc) / 2 : 0.0;
    for (int j = 0; j < BD_WIN; ++j)
        r[j] *= std::complex<float>(std::polar(1.0, -w * j));
    dfHz = w * 2.5e6 / (2 * M_PI);
    return true;
}

// wie st_pre_fit
static bool bdPreFit(const DpskChannel& ch, const std::complex<float>* r, double c,
                     std::complex<double>& num, double& den, double& sigEnergy)
{
    double lim = ch.anchor + ch.delay - 10;
    num = 0; den = 0; sigEnergy = 0;
    for (int k = 0; k < BD_WIN; ++k) {
        double t = c + 8.0 * k;
        if (t >= lim) break;
        double kv = ch.knv(t);
        std::complex<double> rk(r[k]);
        num += rk * kv;
        den += kv * kv;
        sigEnergy += std::norm(rk);
    }
    return den > 0 && sigEnergy > 0;
}

// wie st_blind_decode: Grobtiming per Präambel, Feintiming per Viterbi-Metrik
bool blindDecodeDpsk(const DpskChannel& ch, const uint8_t* iq, size_t nPairs,
                     long k, int n, BlindResult& out)
{
    std::complex<float> r[BD_WIN];
    if (!bdFrame(iq, nPairs, k, n, r, out.dfHz)) return false;

    int bestC = 0; double bestS = -1;
    std::complex<double> num; double den, sigE;
    for (int c = -24; c <= 32; ++c)
        if (bdPreFit(ch, r, c, num, den, sigE)) {
            double s = std::norm(num) / den;
            if (s > bestS) { bestS = s; bestC = c; }
        }
    if (bestS < 0) return false;

    bool have = false;
    for (int cc = bestC - 1; cc <= bestC + 1; ++cc) {
        if (!bdPreFit(ch, r, cc, num, den, sigE)) continue;
        float phi = (float)std::arg(num), A = (float)(std::abs(num) / den);
        ViterbiResult v = viterbiDpsk(ch, r, BD_WIN, cc, phi, A, n);
        if (!have || v.metric < out.v.metric) {
            out.v = v; out.c = cc; out.phi = phi; out.A = A;
            out.rho = std::norm(num) / (den * sigE);
            have = true;
        }
    }
    return have;
}

static std::string bitsHex(const std::vector<uint8_t>& b)
{
    std::string s;
    for (size_t i = 0; i + 3 < b.size(); i += 4)
        s += "0123456789ABCDEF"[(b[i] << 3) | (b[i + 1] << 2) | (b[i + 2] << 1) | b[i + 3]];
    return s;
}

// Vergleich: Frames aus p6sim-Log (Zeit_µs n HEX Metrik) mit C++ dekodieren
int viterbiCompare(const char* rawFile, const char* logFile, const DpskChannel& ch)
{
    std::ifstream fr(rawFile, std::ios::binary);
    std::vector<uint8_t> iq((std::istreambuf_iterator<char>(fr)), std::istreambuf_iterator<char>());
    size_t nPairs = iq.size() / 2;
    std::ifstream fl(logFile);
    std::string line;
    int total = 0, same = 0, shown = 0;
    long bitErr = 0, bitTot = 0;
    while (std::getline(fl, line)) {
        std::istringstream ss(line);
        double ts, mPy; int n; std::string hx;
        if (!(ss >> ts >> n >> hx >> mPy)) continue;
        long k = std::lround(ts * 2.5);
        BlindResult br;
        if (!blindDecodeDpsk(ch, iq.data(), nPairs, k, n, br)) continue;
        std::string hc = bitsHex(br.v.bits);
        ++total;
        int e = 0;
        for (size_t i = 0; i < hx.size() && i < hc.size(); ++i) {
            int d = std::stoi(hx.substr(i, 1), nullptr, 16) ^ std::stoi(hc.substr(i, 1), nullptr, 16);
            e += __builtin_popcount(d);
        }
        bitErr += e; bitTot += n;
        if (e == 0) ++same;
        else if (shown++ < 5)
            printf("  Diff %.1f us: py %s (%.4f)  c++ %s (%.4f)  df %.0f Hz\n",
                   ts, hx.c_str(), mPy, hc.c_str(), br.v.metric, br.dfHz);
    }
    printf("Vergleich: %d Frames, identisch %d (%.2f %%), Bitabweichung %.4f %%\n",
           total, same, total ? 100.0 * same / total : 0.0,
           bitTot ? 100.0 * bitErr / bitTot : 0.0);
    return 0;
}
// =================== Ende Blind-Decode ===================

// ===================== Paket 4: Adresse, Q, Bestaetigung =====================
#include <unordered_map>

// CRC-24 Mode S ueber die ersten n-24 Bits (Bits als 0/1)
static uint32_t crc24Bits(const uint8_t* b, int nData)
{
    uint32_t reg = 0;
    for (int i = 0; i < nData + 24; ++i) {
        reg = (reg << 1) | (i < nData ? b[i] : 0);
        if (reg & (1u << 24)) reg ^= 0x1FFF409;
    }
    return reg & 0xFFFFFF;
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
static uint32_t apToAddress(const std::vector<uint8_t>& b)
{
    int n = (int)b.size();
    uint32_t ap = 0;
    for (int i = n - 24; i < n; ++i) ap = (ap << 1) | b[i];
    return uplinkDecodeAddr(crc24Bits(b.data(), n - 24) ^ ap);
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



static bool demodBits(int i, const SprLock &L, double w, int nbits,
                      UplinkResult &R) {
    const Timing &T = G.tm;
    double sum_margin = 0.0, min_margin = 1.0;
    int weak = 0;
    memset(R.bytes, 0, sizeof(R.bytes));

    for (int b = 0; b < nbits; b++) {
        double tb = L.t_us + 0.5 + b * us::CHIP;
        double ta = i + T.f(tb - 0.125);
        double tc = i + T.f(tb + 0.125);
        double c;
        if (!diffCos(ta, tc, w, c)) return false;
        if (c < 0.0) R.bytes[b / 8] |= (uint8_t)(0x80 >> (b % 8));
        double mgn = std::fabs(c);
        sum_margin += mgn;
        min_margin  = std::min(min_margin, mgn);
        if (mgn < WEAK_MARGIN) weak++;
        R.margin[b] = (float)mgn;
    }
    R.nbits       = nbits;
    R.mean_margin = sum_margin / nbits;
    R.min_margin  = min_margin;
    R.weak_bits   = weak;
    return true;
}

static uint8_t combineQuality(const UplinkResult &R) {
    double frac_ok = 1.0 - (double)R.weak_bits / std::max(1, R.nbits);
    double q = R.spr_score * R.mean_margin * frac_ok;
    q = std::max(0.0, std::min(1.0, q));
    return (uint8_t)lround(q * 255.0);
}

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
    std::vector<uint8_t> b((size_t)nbits);
    for (int k = 0; k < nbits; ++k)
        b[k] = (bytes[k / 8] >> (7 - k % 8)) & 1;
    return apToAddress(b);
}

static inline uint32_t ufOf(const uint8_t *b) {
    uint32_t uf = bits(b, 1, 5);
    return ((uf >> 3) == 3) ? 24 : uf;
}

static bool estimateFineFreq(int i, const SprLock &L, const UplinkResult &R,
                             double &w_out) {
    const Timing &T = G.tm;
    cd acc = 0.0;
    int n = 0;
    for (int b = 0; b < R.nbits; b++) {
        double tb = L.t_us + 0.5 + b * us::CHIP;
        cd za, zb;
        if (!getIQSample(i + T.f(tb - 0.125), za) ||
            !getIQSample(i + T.f(tb + 0.125), zb)) break;
        cd d = zb * std::conj(za);
        if ((R.bytes[b / 8] >> (7 - b % 8)) & 1) d = -d;
        acc += d;
        n++;
    }
    if (n < 32 || std::abs(acc) < 1e-9) return false;
    double w  = std::arg(acc) / T.f(us::CHIP);
    double hz = w * G.samplerate / (2.0 * PI);
    if (std::fabs(hz) > 150e3) return false;
    w_out = w;
    return true;
}

static double   g_pos_m[112] = {0};  static uint32_t g_pos_n[112] = {0};
static double   g_pat_m[4]   = {0};  static uint32_t g_pat_n[4]   = {0};

static inline int bitAt(const uint8_t *b, int k) { return (b[k / 8] >> (7 - k % 8)) & 1; }

static void collectStats(const UplinkResult &R) {
    for (int b = 0; b < R.nbits; b++) {
        g_pos_m[b] += R.margin[b]; g_pos_n[b]++;
        int prev = (b > 0) ? bitAt(R.bytes, b - 1) : 0;
        int p = (prev << 1) | bitAt(R.bytes, b);
        g_pat_m[p] += R.margin[b]; g_pat_n[p]++;
    }
}

static double g_spr_offset_us = 0.0;

static double medianMagnitude(int i, double t0_us, double t1_us) {
    const Timing &T = G.tm;
    int a = i + (int)std::ceil(T.f(t0_us));
    int b = i + (int)std::floor(T.f(t1_us));
    if (a < 0 || b >= (int)g_valid_samples || b < a) return -1.0;
    std::vector<int> values;
    values.reserve((size_t)(b - a + 1));
    for (int k = a; k <= b; ++k) values.push_back(G.mag[k]);
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
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

static UplinkResult decodeUplink(int i, int p1_ref, int nbits) {
    UplinkResult R;
    R.nbits = nbits;
    if (nbits == 0) {
        R.reject_reason = "length";
        return R;
    }
    evalP6Envelope(i, p1_ref, nbits, R.p6_cov, R.p6_min_ratio, R.p6_snr_db,
                   R.pre_snr_db, R.p6_p1_ratio);
    if (R.p6_cov < 0.65) {
        R.reject_reason = "p6_coverage";
        return R;
    }
    if (R.p6_snr_db < MIN_P6_SNR_DB) {
        R.reject_reason = "p6_snr";
        return R;
    }
    if (R.pre_snr_db < MIN_PREAMBLE_SNR_DB) {
        R.reject_reason = "preamble_snr";
        return R;
    }
    if (R.p6_p1_ratio > MAX_P6_TO_P1_RATIO) {
        R.reject_reason = "p6_p1_ratio";
        return R;
    }

    // --- Viterbi-Pfad (Paket 4) ---
    if (g_haveChan) {
        BlindResult br;
        size_t nPairs = G.iq_hist.size() / 2;
        if (blindDecodeDpsk(g_chan, G.iq_hist.data(), nPairs, i, nbits, br)) {
            double w_rad = br.dfHz * (2.0 * PI) / G.samplerate;
            SprLock L = lockSpr(i, w_rad);
            L.t_us += g_spr_offset_us;

            R.nbits       = nbits;
            R.spr_us      = L.t_us;
            R.spr_score   = L.score;
            R.freq_off_hz = br.dfHz;
            R.quality     = (uint8_t)dpskQuality(br.v.metric);
            R.pre_rho     = br.rho;

            memset(R.bytes, 0, sizeof(R.bytes));
            for (int b = 0; b < nbits; ++b) {
                if (br.v.bits[b]) R.bytes[b / 8] |= (uint8_t)(0x80 >> (b % 8));
            }

            if (R.pre_rho < 0.50) {
                R.reject_reason = "preamble_rho";
                return R;
            }
            if (R.spr_score < 0.50) {
                R.reject_reason = "spr_lock";
                return R;
            }
            if (R.quality < MIN_VITERBI_QUALITY) {
                R.reject_reason = "viterbi_quality";
                return R;
            }

            if (!validUplinkFormat(R.bytes, R.nbits)) {
                R.reject_reason = "uplink_format";
                return R;
            }
            R.mean_margin = (double)R.quality / 255.0;
            R.min_margin = R.mean_margin;
            R.weak_bits   = 0;
            R.viterbi     = true;
            collectStats(R);
            R.valid       = true;
            R.reject_reason = nullptr;
            G.uplink_ok++;
        } else {
            R.reject_reason = "blind_decode";
        }
        return R;
    }

    FreqEst fe = estimateFreqOffset(i);
    SprLock L  = lockSpr(i, fe.w);
    L.t_us += g_spr_offset_us;
    R.spr_us      = L.t_us;
    R.spr_score   = L.score;
    R.freq_off_hz = fe.hz;
    if (!L.ok || R.spr_score < 0.50) {
        R.reject_reason = "spr_lock";
        return R;
    }
    if (!demodBits(i, L, fe.w, nbits, R)) {
        R.reject_reason = "demodulation";
        return R;
    }

    if (ufLength(ufOf(R.bytes)) == 112 && R.nbits == 56) {
        UplinkResult R2 = R;
        if (demodBits(i, L, fe.w, 112, R2)) {
            R = R2;
            evalP6Envelope(i, p1_ref, 112, R.p6_cov, R.p6_min_ratio, R.p6_snr_db,
                           R.pre_snr_db, R.p6_p1_ratio);
            if (R.p6_cov < 0.65) {
                R.reject_reason = "p6_coverage";
                return R;
            }
            if (R.p6_snr_db < MIN_P6_SNR_DB) {
                R.reject_reason = "p6_snr";
                return R;
            }
            if (R.pre_snr_db < MIN_PREAMBLE_SNR_DB) {
                R.reject_reason = "preamble_snr";
                return R;
            }
            if (R.p6_p1_ratio > MAX_P6_TO_P1_RATIO) {
                R.reject_reason = "p6_p1_ratio";
                return R;
            }
        }
    }

    if (ufLength(ufOf(R.bytes)) == 0 && L.t_us > us::SPR_PRIOR + 0.25) {
        SprLock L2 = L;
        L2.t_us -= 0.5;
        UplinkResult R2 = R;
        if (demodBits(i, L2, fe.w, nbits, R2) && ufLength(ufOf(R2.bytes)) != 0) {
            L = L2; R = R2; R.spr_us = L.t_us;
        }
    }

    double wf;
    if (estimateFineFreq(i, L, R, wf)) {
        double hz_f = wf * G.samplerate / (2.0 * PI);
        R.freq_off_hz = hz_f;
        if (std::fabs(hz_f - fe.hz) > 5e3) {
            UplinkResult R2 = R;
            if (demodBits(i, L, wf, R.nbits, R2) && R2.mean_margin > R.mean_margin) {
                R = R2;
                R.freq_off_hz = hz_f;
            }
        }
    }

    R.quality = combineQuality(R);
    if (R.quality < 80) {
        R.reject_reason = "legacy_quality";
        return R;
    }
    if (!validUplinkFormat(R.bytes, R.nbits)) {
        R.reject_reason = "uplink_format";
        return R;
    }
    collectStats(R);
    R.valid = true;
    R.reject_reason = nullptr;
    if (R.weak_bits * 4 > R.nbits) G.uplink_weak++;
    else                           G.uplink_ok++;
    return R;
}

// ---------- Mode-S-CRC & UF-Dekodierung ----------
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

static inline uint32_t bits(const uint8_t *b, int first, int last) {
    uint32_t v = 0;
    for (int k = first; k <= last; k++)
        v = (v << 1) | ((b[(k - 1) / 8] >> (7 - (k - 1) % 8)) & 1);
    return v;
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
                       "margin=%.2f/%.2f weak=%d df=%+.1f kHz Q=%u "
                       "cov=%.2f p6min=%.2f p6snr=%.1f dB rho=%.2f HEX=",
                R->nbits, R->spr_us, R->spr_score, R->mean_margin,
                R->min_margin, R->weak_bits, R->freq_off_hz / 1e3, R->quality,
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
    skip = (nbits == 112) ? T.skip_s112
         : (nbits == 56)  ? T.skip_s56
         :                  T.min_skip_s;
    return true;
}

struct PendingP6 {
    uint64_t     pos = 0;
    int          p1 = 0;
    int          p2 = 0;
    UplinkResult R;
    uint8_t      samples[32] = {0};
    int          show = 0;
};

struct EmitterContext {
    std::vector<PendingP6> train;
    uint64_t               last_pos = 0;
};
static EmitterContext g_emitCtx;

static void flushEmitterTrain() {
    if (g_emitCtx.train.empty()) return;

    for (auto &ev : g_emitCtx.train) {
        uint32_t uf = ufOf(ev.R.bytes);
        ev.R.mode5 = isMode5Format(ev.R.bytes, ev.R.nbits);
        if (ev.R.mode5) {
            ev.R.has_addr = false;
            ev.R.addr     = 0;
            ev.R.addrCnt  = 0;
        } else {
            ev.R.has_addr = hasAircraftAddress(uf, ev.R.nbits);
            if (ev.R.has_addr) {
                ev.R.addr    = apToAddressBytes(ev.R.bytes, ev.R.nbits);
                ev.R.addrCnt = g_addrTrack.seen(ev.R.addr, ev.pos / (double)G.samplerate);
            }
        }

        writeJsonlP6(ev.pos, ev.p1, ev.p2, ev.R);
        if (G.print_detected) {
            analyzeBurst(ev.pos, MSG_S, ev.samples, &ev.R);
            printSamples(ev.samples, ev.show);
            fputc('\n', G_txt);
        }
    }
    g_emitCtx.train.clear();
}

static uint32_t detectBlock(uint32_t total, bool final) {
    const Timing &T = G.tm;
    const uint8_t *mag = G.mag.data();
    uint32_t limit = final ? total
                   : (total > (uint32_t)MAX_LOOKAHEAD ? total - MAX_LOOKAHEAD : 0);
    uint32_t i = 0;

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
        if (checkModeS(m) && handleModeS(m, i, r1, skip, R)) {
            code  = MSG_S;
            haveR = G.uplink_decode;
            show  = T.p2 + T.pw + 2;
            if (G.baseline) {
                addPulse(m, 0, T.pw);
                addPulse(m, T.p2, T.pw);
                addPause(m, T.pw, T.p2 - 1);
            }
        }
        // --- Mode A / C ---
        else {
            int off = checkP3(m, T.p3_a) ? T.p3_a
                    : checkP3(m, T.p3_c) ? T.p3_c : 0;
            if (off) {
                const bool isA = (off == T.p3_a);
                const int  os  = off + T.p4_gap;
                show = os + T.p4_long + 1;
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
                    skip = off + T.pw + 2;
                    if (G.baseline) {
                        addPulse(m, 0, T.pw);
                        addPulse(m, off, T.pw);
                        addPause(m, T.pw, off - 1);
                        G.avg_nfc.add(m[off + T.pw]);
                        G.avg_nf.add(m[off + T.pw + 1]);
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
                const uint64_t dwell_gap = (uint64_t)llround(0.055 * G.samplerate);
                if (!g_emitCtx.train.empty() && pos > g_emitCtx.last_pos &&
                    (pos - g_emitCtx.last_pos) > dwell_gap) {
                    flushEmitterTrain();
                }
                PendingP6 ev;
                ev.pos  = pos;
                ev.p1   = p1;
                ev.p2   = p2;
                ev.R    = R;
                ev.show = std::min(show, 32);
                memcpy(ev.samples, m, (size_t)ev.show);
                g_emitCtx.train.push_back(ev);
                g_emitCtx.last_pos = pos;
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

    const uint64_t dwell_gap = (uint64_t)llround(0.055 * G.samplerate);
    uint64_t block_end = G.sample_base + limit;
    if (final || (!g_emitCtx.train.empty() && block_end > g_emitCtx.last_pos &&
                  (block_end - g_emitCtx.last_pos) > dwell_gap)) {
        flushEmitterTrain();
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

static void printStats(uint32_t samples) {
    if (G.cur.total() == 0) {
        if (!G.continuous) fprintf(G_txt, "Keine Nachrichten erkannt.\n");
    } else {
        char title[96];
        snprintf(title, sizeof(title),
                 "Statistik fuer %u Samples (%.1f ms):", samples,
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

// ---------- RTL-SDR ----------
static void initRtlsdr() {
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
}

static void rtlsdrCallback(unsigned char *buf, uint32_t len, void *) {
    if (g_exit) return;
    pthread_mutex_lock(&G.mutex);
    if (G.data_ready) {
        G.dropped++;
    } else {
        if (len > G.data_len) len = G.data_len;
        memcpy(G.iq_buf.data(), buf, len);
        G.iq_len = len;
        G.data_ready = true;
        pthread_cond_signal(&G.cond);
    }
    pthread_mutex_unlock(&G.mutex);
    if (!G.continuous) rtlsdr_cancel_async(G.dev);
}

// ---------- Datei-Eingabe ----------
static void readFile() {
    G.fd = open(G.filename, O_RDONLY);
    if (G.fd < 0) {
        fprintf(stderr, "Fehler: Datei '%s' nicht lesbar: %s\n",
                G.filename, strerror(errno));
        exit(1);
    }
    std::vector<uint8_t> tmp(G.data_len);
    while (!g_exit) {
        size_t got = 0;
        while (got < tmp.size()) {
            ssize_t n = read(G.fd, tmp.data() + got, tmp.size() - got);
            if (n < 0) {
                if (errno == EINTR) continue;
                fprintf(stderr, "Lesefehler: %s\n", strerror(errno));
                break;
            }
            if (n == 0) break;
            got += (size_t)n;
        }
        got &= ~(size_t)1;

        pthread_mutex_lock(&G.mutex);
        while (G.data_ready && !g_exit)
            pthread_cond_wait(&G.cond, &G.mutex);
        if (got > 0) {
            memcpy(G.iq_buf.data(), tmp.data(), got);
            G.iq_len = (uint32_t)got;
            G.data_ready = true;
        }
        bool eof = got < tmp.size();
        if (eof) G.reader_done = true;
        pthread_cond_broadcast(&G.cond);
        pthread_mutex_unlock(&G.mutex);
        if (eof) break;
    }
    close(G.fd);
    G.fd = -1;
}

// ---------- Reader-Thread ----------
static void *readerThread(void *) {
    if (G.filename) {
        readFile();
    } else {
        int r = rtlsdr_read_async(G.dev, rtlsdrCallback, nullptr, 0, G.data_len);
        if (r < 0 && !g_exit)
            fprintf(stderr, "Fehler: rtlsdr_read_async() lieferte %d\n", r);
        pthread_mutex_lock(&G.mutex);
        G.reader_done = true;
        pthread_cond_broadcast(&G.cond);
        pthread_mutex_unlock(&G.mutex);
    }
    return nullptr;
}

// ---------- Kommandozeile ----------
static void showHelp() {
    printf(
"Verwendung: dump1030 [Optionen]\n"
"Eingabe:\n"
"  --device <n>            RTL-SDR-Geraeteindex (Default 0)\n"
"  --file <pfad>           Rohdatei (8-Bit-I/Q) statt RTL-SDR\n"
"  --gain <dB|auto|max>    Tuner-Gain in dB (Default max)\n"
"  --samplerate <r>        2.4..3.2 MS/s oder Hz (Default 2.5). Bei --file:\n"
"                          MUSS der Rate der Aufzeichnung entsprechen!\n"
"                          --chan erfordert exakt 2.5 MS/s.\n"
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
"  --out-dir <pfad>        Ausgabeverzeichnis fuer JSONL/cu8 Dateien\n"
"  --save-iq               IQ-Daten (im cu8 Format) speichern\n"
"  --p6-diagnostics        Abgelehnte P6-Kandidaten als JSONL ausgeben\n"
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

static float parseFloat(const char *opt, const char *s, float lo, float hi) {
    char *end = nullptr;
    errno = 0;
    float v = strtof(s, &end);
    if (errno || end == s || *end || !(v >= lo && v <= hi)) {
        fprintf(stderr, "Fehler: %s erwartet Zahl %.2f..%.2f, nicht '%s'\n",
                opt, lo, hi, s);
        exit(1);
    }
    return v;
}

static void parseArgs(int argc, char **argv) {
    Thresholds &t = G.th;
        const char* vcompareLog = nullptr;
for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "--device")) {
            G.dev_index = (int)parseLong(a, needArg(argc, argv, i), 0, 255);
        } else if (!strcmp(a, "--file")) {
            G.filename = needArg(argc, argv, i);
        } else if (!strcmp(a, "--chan")) {
	    if (g_haveChan) uplinkAddrSelftest();
            g_haveChan = loadDpskChannel(needArg(argc, argv, i), g_chan);
        } else if (!strcmp(a, "--vcompare")) {
            vcompareLog = needArg(argc, argv, i);
        } else if (!strcmp(a, "--spr-offset")) {
            g_spr_offset_us = strtod(needArg(argc, argv, i), nullptr);
        } else if (!strcmp(a, "--gain")) {
            const char *v = needArg(argc, argv, i);
            if (!strcmp(v, "auto"))     G.gain = AUTO_GAIN;
            else if (!strcmp(v, "max")) G.gain = MAX_GAIN;
            else G.gain = (int)lroundf(parseFloat(a, v, 0.0f, 60.0f) * 10.0f);
        } else if (!strcmp(a, "--samplerate")) {
            const char *v = needArg(argc, argv, i);
            double r = parseFloat(a, v, 0.0f, 4.0e6f);
            if (r < 100.0) r *= 1e6;
            uint32_t rate = (uint32_t)lround(r);
            if (rate < MIN_RATE || rate > MAX_RATE) {
                fprintf(stderr, "Fehler: --samplerate muss %.1f..%.1f MS/s sein "
                        "(Pulsbreite 0,8 us braucht >= 2 Samples)\n",
                        MIN_RATE / 1e6, MAX_RATE / 1e6);
                exit(1);
            }
            G.samplerate = rate;
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
            t.ratio = toQ8(parseFloat(a, needArg(argc, argv, i), 0.0f, 4.0f));
        } else if (!strcmp(a, "--diffratioclose")) {
            t.ratioclose = toQ8(parseFloat(a, needArg(argc, argv, i), 0.0f, 4.0f));
        } else if (!strcmp(a, "--diffratiop4")) {
            t.ratiop4 = toQ8(parseFloat(a, needArg(argc, argv, i), 0.0f, 4.0f));
        } else if (!strcmp(a, "--diffratioclosep4")) {
            t.ratioclosep4 = toQ8(parseFloat(a, needArg(argc, argv, i), 0.0f, 4.0f));
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
        } else if (!strcmp(a, "--help")) {
            showHelp();
            exit(0);
        } else {
            fprintf(stderr, "Unbekannte Option: %s\n\n", a);
            showHelp();
            exit(1);
        }
    }

    if (g_haveChan && G.samplerate != DEFAULT_RATE) {
        fprintf(stderr, "--chan benoetigt exakt 2.5 MS/s; angefordert: %u Hz\n",
                G.samplerate);
        std::exit(1);
    }
    if (vcompareLog) {
        if (!g_haveChan || !G.filename) { fprintf(stderr, "--vcompare braucht --file und --chan\n"); std::exit(1); }
        std::exit(viterbiCompare(G.filename, vcompareLog, g_chan));
    }
    if (G.p6_diagnostics && G.out_dir.empty())
        die("--p6-diagnostics benoetigt --out-dir");
    if (G.p6_diagnostics && !G.uplink_decode)
        die("--p6-diagnostics benoetigt --uplink-decode");
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
        len = G.iq_len;
        G.data_ready = false;
        pthread_cond_broadcast(&G.cond);
    }
    last = G.reader_done && !G.data_ready;
    pthread_mutex_unlock(&G.mutex);
    return len;
}

static uint64_t g_samples = 0;

static void processBlock(uint32_t len, bool last) {
    if (G.iq_fp && len >= 2) {
        fwrite(G.iq_work.data(), 1, len & ~(uint32_t)1, G.iq_fp);
    }

    uint32_t total = computeMagnitude(len);
    g_valid_samples = total;
    uint32_t consumed = detectBlock(total, last);
    keepOverlap(total, consumed);
    g_samples += len / 2;
    if (G.continuous) {
        printStats((uint32_t)g_samples);
        g_samples = 0;
    }
}

// ---------- main ----------
int main(int argc, char **argv) {
    parseArgs(argc, argv);
    if (!initOutputs()) return 1;
    buildLut();
    if (!G.filename) initRtlsdr();
    G.tm.init(G.samplerate);

    const int need = std::max(G.tm.maxPulseIndex(), G.tm.skip_s112 + 4);
    if (need > MAX_LOOKAHEAD) {
        fprintf(stderr, "Interner Fehler: MAX_LOOKAHEAD=%d < benoetigt %d\n",
                MAX_LOOKAHEAD, need);
        return 1;
    }
    if (G.uplink_decode)
        fprintf(stderr, "Hinweis: --uplink-decode ist experimentell und nicht "
                        "validiert (Quality-Gates in JSONL beachten)\n");

    G.iq_buf.resize(G.data_len);
    G.iq_work.resize(G.data_len);
    G.mag.resize(G.data_len / 2 + 2 * MAX_LOOKAHEAD);
    G.iq_hist.resize(2 * ((size_t)G.data_len / 2 + 2 * MAX_LOOKAHEAD));

    struct sigaction sa{};
    sa.sa_handler = sigHandler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    if (pthread_create(&G.reader_thread, nullptr, readerThread, nullptr) != 0)
        die("Reader-Thread konnte nicht gestartet werden");

    const bool single = !G.filename && !G.continuous;
    while (!g_exit) {
        bool last = false;
        uint32_t len = fetchBlock(last);
        if (len == 0) {
            if (last) {
                if (G.overlap) processBlock(0, true);
                break;
            }
            continue;
        }
        processBlock(len, last || single);
        if (single || last) break;
    }

    if (!G.continuous) printStats((uint32_t)g_samples);
    if (G.dropped)
        fprintf(stderr, "Warnung: %" PRIu64 " Puffer verworfen "
                        "(Verarbeitung zu langsam)\n", G.dropped);
    if (G.uplink_decode) {
        fprintf(stderr, "Uplink-Decoder: %" PRIu64 " sauber, %" PRIu64
                        " mit >25%% weak bits\n", G.uplink_ok, G.uplink_weak);
        fprintf(stderr, "Margin je Bitposition (Frames, 1-basiert):\n");
        for (int b = 0; b < 112; b++)
            if (g_pos_n[b])
                fprintf(stderr, "  Bit %3d: %.3f (n=%u)%s", b + 1,
                        g_pos_m[b] / g_pos_n[b], g_pos_n[b], (b % 4 == 3) ? "\n" : "");
        static const char *pn[4] = {"0->0", "0->1", "1->0", "1->1"};
        fprintf(stderr, "\nMargin nach Muster (Vorgänger->Bit):\n");
        for (int p = 0; p < 4; p++)
            if (g_pat_n[p])
                fprintf(stderr, "  %s: %.3f (n=%u)\n", pn[p], g_pat_m[p] / g_pat_n[p], g_pat_n[p]);
    }


    // ---------- Aufraeumen ----------
    g_exit = true;
    if (G.dev) rtlsdr_cancel_async(G.dev);
    pthread_mutex_lock(&G.mutex);
    pthread_cond_broadcast(&G.cond);
    pthread_mutex_unlock(&G.mutex);
    pthread_join(G.reader_thread, nullptr);

    if (G.dev) { rtlsdr_close(G.dev); G.dev = nullptr; }
    flushEmitterTrain();
    closeOutputs();
    return 0;
}
