#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <omp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void build_fir_hamming(float *b_rev, int taps, double cutoff) {
    double alpha = 0.5 * (taps - 1);
    double sum = 0.0;
    double *tmp = malloc((size_t)taps * sizeof(double));
    if (!tmp) { perror("malloc"); exit(1); }

    for (int n = 0; n < taps; n++) {
        double x = n - alpha;
        double sinc = (x == 0.0) ? 1.0 : sin(M_PI * cutoff * x) / (M_PI * cutoff * x);
        double win = 0.54 - 0.46 * cos(2.0 * M_PI * n / (taps - 1));
        tmp[n] = cutoff * sinc * win;
        sum += tmp[n];
    }
    for (int n = 0; n < taps; n++) {
        b_rev[taps - 1 - n] = (float)(tmp[n] / sum);
    }
    free(tmp);
}

static inline uint8_t to_u8(float v) {
    int r = (int)lrintf(v);
    if (r < 0) return 0;
    if (r > 255) return 255;
    return (uint8_t)r;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <in.sc16> <out.cu8> [decimation=4] [sc16_scale=2048] [taps=63] [cutoff_ratio=0.8]\n", argv[0]);
        return 1;
    }

    const char *in_path = argv[1];
    const char *out_path = argv[2];
    int decimation = (argc > 3) ? atoi(argv[3]) : 4;
    double sc16_scale = (argc > 4) ? atof(argv[4]) : 2048.0;
    int taps = (argc > 5) ? atoi(argv[5]) : 63;
    double cutoff_ratio = (argc > 6) ? atof(argv[6]) : 0.8;

    if (decimation < 2 || taps < 3 || (taps % 2) == 0 || sc16_scale <= 0.0 || cutoff_ratio <= 0.0 || cutoff_ratio >= 1.0) {
        fprintf(stderr, "Invalid parameters.\n");
        return 1;
    }

    struct stat st;
    if (stat(in_path, &st) != 0) { perror("stat"); return 1; }
    if (st.st_size % 4 != 0) {
        fprintf(stderr, "Input file size is not a multiple of 4 bytes.\n");
        return 1;
    }

    FILE *fin = fopen(in_path, "rb");
    if (!fin) { perror("fopen input"); return 1; }
    FILE *fout = fopen(out_path, "wb");
    if (!fout) { perror("fopen output"); fclose(fin); return 1; }

    posix_fadvise(fileno(fin), 0, 0, POSIX_FADV_SEQUENTIAL);

    float *b_rev = aligned_alloc(64, (size_t)((taps + 15) & ~15) * sizeof(float));
    build_fir_hamming(b_rev, taps, cutoff_ratio / decimation);
    const float gain = (float)(127.5 / sc16_scale);

    // 16M complex samples = 64 MB input per chunk
    const size_t chunk_samples = (16000000ULL / (size_t)decimation) * (size_t)decimation;
    const size_t overlap = (size_t)(taps - 1);

    int16_t *in_buf = calloc((overlap + chunk_samples) * 2, sizeof(int16_t));
    uint8_t *out_buf = malloc(((chunk_samples / (size_t)decimation) + 1) * 2);
    if (!in_buf || !out_buf) { perror("alloc"); return 1; }

    size_t total_in = 0;
    size_t total_out = 0;

    for (;;) {
        size_t n_read = fread(in_buf + overlap * 2, 4, chunk_samples, fin);
        if (n_read == 0) break;

        size_t n_out = (n_read + (size_t)decimation - 1) / (size_t)decimation;

        #pragma omp parallel for schedule(static)
        for (size_t k = 0; k < n_out; k++) {
            const int16_t *p = in_buf + 2 * (k * (size_t)decimation);
            float acc_i = 0.0f;
            float acc_q = 0.0f;
            for (int j = 0; j < taps; j++) {
                float c = b_rev[j];
                acc_i += c * (float)p[2 * j];
                acc_q += c * (float)p[2 * j + 1];
            }
            out_buf[2 * k]     = to_u8(acc_i * gain + 127.5f);
            out_buf[2 * k + 1] = to_u8(acc_q * gain + 127.5f);
        }

        if (fwrite(out_buf, 2, n_out, fout) != n_out) {
            perror("fwrite");
            return 1;
        }

        total_in += n_read;
        total_out += n_out;

        memmove(in_buf, in_buf + n_read * 2, overlap * 2 * sizeof(int16_t));
    }

    fclose(fin);
    fclose(fout);
    free(in_buf);
    free(out_buf);
    free(b_rev);

    printf("input_samples=%zu output_samples=%zu\n", total_in, total_out);
    return 0;
}