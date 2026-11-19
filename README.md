# dump1030

**dump1030** is a C++17 command-line receiver that detects and decodes Secondary Surveillance Radar (SSR) interrogations on 1030 MHz using an RTL-SDR.

It detects Mode A, Mode C, Mode A/C All-Call (including Compatibility Mode), and Mode S interrogations. Mode S P6 (DPSK) uplinks are decoded using a 64-state Viterbi equalizer, recovering the Uplink Format (UF), protocol fields, and the 24-bit target ICAO address from the Address/Parity (AP) field.

## Origin

Fork of [dump1030 by Fuziih](https://github.com/Fuziih/dump1030), which builds on [antirez's dump1090](https://github.com/antirez/dump1090).

Key changes in this fork:

- **C++17 rewrite:** Double-buffered reader thread, Q8 fixed-point ratio checks, and runtime timing tables.
- **64-state Viterbi DPSK decoder:** Equalizes 4 Mbit/s Mode S P6 uplinks at 2.5 MS/s using a blindly learned receiver impulse response (`--chan`). Includes automatic 56/112-bit length detection (with 112-bit retry on long UF formats) and 2-pass residual frequency/phase refinement.
- **Target ICAO address recovery:** Recovers the 24-bit aircraft address (`AA`) from the AP field for UF0, UF4, UF5, UF16, UF20, UF21, and UF24.
- **P6 quality gates & JSONL output:** Filters candidates by envelope coverage, SNR, preamble correlation, SPR lock, Viterbi quality (`Q >= 180`), and UF/length validity. Optional `--p6-diagnostics` logs rejected candidates (`P6_REJECTED`) with the failed gate.
- **Address repetition tracking:** Marks an address with `*` (and sets `status: "repeated"` in JSONL) once seen at least twice within a 60 s window. First sightings remain `status: "candidate"` with `"aa": null`.
- **Analysis & calibration tools:** Includes `p6sim.py` for channel learning and synthetic benchmarks, plus live monitoring and signature scripts.

## Requirements

- Linux
- RTL-SDR capable of streaming stably at 2.5 MS/s on 1030 MHz
- `g++` (C++17) and `librtlsdr-dev`
- Python 3 (standard library only; needed for `p6sim.py` and helper scripts)

## Installation

```bash
apt update && apt install -y build-essential pkg-config librtlsdr-dev
git clone https://github.com/davidkirberich/dump1030.git
cd dump1030
make
chmod +x dump1030 monitor_p6.py radar_signatures.py
```

## Quick Start

1. Record a 30-second raw I/Q capture (~150 MB; adjust `-d` to your device index):

   ```bash
   rtl_sdr -d 1 -f 1030000000 -s 2500000 -g 32 -n 75000000 raw_30s.bin
   ```

2. Learn the receiver channel response (only needed once per SDR stick, gain, and sample rate):

   ```bash
   python3 p6sim.py realtest raw_30s.bin --max-sec 5 --chan-out chan.txt
   ```

3. Run live with Mode S uplink decoding:

   ```bash
   ./dump1030 --device 1 --gain 32 --chan chan.txt --uplink-decode --msgs --continuous
   ```

4. Run live with JSONL logging and the interactive monitor:

   ```bash
   ./dump1030 --device 1 --gain 32 --chan chan.txt \
     --uplink-decode --continuous --out-dir ./captured
   ```

   In a second terminal:

   ```bash
   ./monitor_p6.py 'captured/events_*.jsonl'
   ```

### Output Fields (`--msgs`)

- `AA=` Recovered 24-bit target ICAO address (`*` = repeated within 60 s, `?` = single sighting).
- `Q=` Frame quality score (`0..255`), derived from the normalized Viterbi path metric.
- `APx=` Raw encoded address/parity value (`AP ^ CRC24(data)`).

## Command-Line Options

### Input & Receiver

```text
--device <n>            RTL-SDR device index (default: 0)
--file <path>           Read 8-bit unsigned I/Q samples from file instead of RTL-SDR
--gain <dB|auto|max>    Tuner gain in dB, "auto", or "max" (default: max)
--samplerate <rate>     Sample rate: 2.4..3.2 MS/s or Hz (default: 2.5). With --file,
                        must match the recording. --chan requires exactly 2.5 MS/s.
--ppm <-200..200>       Tuner frequency correction in ppm
--agc                   Enable RTL2832U digital AGC
--size <bytes>          Read buffer size in bytes (min: 16384; rounded down to a
                        multiple of 16384)
--continuous            Run continuously (--size is the block size and reporting interval)
```

### Mode S Uplink Decoding & Output

```text
--uplink-decode         Enable Mode S P6 (DPSK) uplink decoding (requires --chan)
--chan <path>           Load learned receiver channel file
--out-dir <path>        Directory for JSONL event logs (and optional .cu8 captures)
--save-iq               Save raw I/Q stream (.cu8), requires --out-dir
--p6-diagnostics        Log rejected P6 candidates as P6_REJECTED
                        (requires --out-dir and --uplink-decode)
--p6-quality <0..255>   Minimum Viterbi quality score (default: 180)
--p6-coverage <0..1>    Minimum P6 envelope coverage (default: 0.65)
--p6-snr <dB>           Minimum P6 SNR in dB (default: 6.0)
--p6-pre-snr <dB>       Minimum preamble SNR in dB (default: 3.0)
--p6-p1-ratio <r>       Maximum P6/P1 amplitude ratio (default: 4.0)
--p6-rho <0..1>         Minimum preamble correlation (default: 0.50)
--p6-lock <0..1>        Minimum SPR lock score (default: 0.50)
--p6-timing-radius <n>  Viterbi fine-timing search radius in 20 MS/s steps (0..8, default: 1)
--msgs                  Print decoded interrogations with sample details
--order                 Print sequence order of detected interrogation types
--print                 Print all raw magnitude values (very slow)
--help                  Display help message
```

JSONL `P6` records include `status` (`candidate`, `repeated`, or `mode5`), `q`, `viterbi_metric`, `path_metric_gap`, `mean_margin`, `min_margin`, `data_min_margin`, `ap_min_margin`, `p6snr_db`, `pre_snr_db`, `p6_p1_ratio`, and format-specific fields (`ii`/`si`, `pr`, `ic`, `cl`, `pc`, `rr`, `di`, `sd`, `rl`, `aq`, `ds`, `mid`, `rc`, `nc`).

### Pulse Detector Tuning (Legacy Options)

```text
--diff <0..255>               Minimum pulse-to-pause amplitude difference (far)
--diffclose <0..255>          Minimum pulse-to-pause amplitude difference (adjacent)
--diffratio <r>               Maximum pause-to-pulse amplitude ratio (far)
--diffratioclose <r>          Maximum pause-to-pulse amplitude ratio (adjacent)
--diffratiop4 <r>             Like --diffratio, applied to P4 pulse check
--diffratioclosep4 <r>        Like --diffratioclose, applied to P4 pulse check
--mpa <0..255>                Minimum accepted pulse amplitude
--mnf <0..255>                Maximum noise floor amplitude (far)
--mnfc <0..255>               Maximum noise floor amplitude adjacent to pulses
--blmode                      Print baseline values for mpa/mnf/mnfc
```

## Tools

### `p6_meter.py` (1030 MHz Antenna Meter)

Starts a dedicated `dump1030` instance and displays 5-second bar meters (plus session maxima) for Viterbi quality, bit margins, SPR lock, P6 SNR, and SSR activity. Press `r` to reset maxima, `q` to quit.

```bash
python3 p6_meter.py --device 1 --gain 32 --chan chan.txt
```

### `mode-s-meter.py` (1090 MHz Antenna Meter)

Reads raw `dump1090`-compatible output and displays 5-second bars for Mode S frames, ICAO counts, repetition, and DF diversity. Press `r` to reset maxima, `q` to quit.

```bash
python3 mode-s-meter.py --device 0 --gain 35
```

### `monitor_p6.py` (Live ICAO Monitor)

![Screenshot of monitor_p6.py in action](img/monitor_p6.png)

Tails JSONL logs in a curses interface without interfering with `dump1030`. Omits `000000` and `FFFFFF`.

```bash
python3 monitor_p6.py 'captured/events_*.jsonl'
```

Keys: `1` repeated aircraft, `2` II/SI interrogators, `j`/`k` or arrows to navigate, `/` to filter, `c` to clear filter, `Space`/`p` to pause, `:w [path]` to export TSV, `:q` to quit.

### `radar_signatures.py` (Rotation & Burst Analysis)

Estimates recurring radar rotation periods (4.8..5.2 s) and phases directly from 1030 MHz JSONL logs, with harmonic rejection and optional drift windows for moving emitters.

```bash
python3 radar_signatures.py 'captured/events_*.jsonl'
python3 radar_signatures.py 'captured/events_*.jsonl' --channel p6 --json
python3 radar_signatures.py 'captured/events_*.jsonl' --channel p6 --duration-s 3600 --explore
```

Channels: `civil` (default: SSR + P6 UF4/5/11/20/21), `mode5`, `acas` (UF0/16), `p6` (all accepted P6). `--explore` outputs inter-event gaps, 55 ms burst shapes, burst families, and DF/P1 drift statistics without assuming a fixed rotation period.

### `p6sim.py` (Simulator & Channel Estimator)

Generates synthetic P6 recordings, runs Viterbi unit tests, and estimates the receiver channel file (`chan.txt`) from raw I/Q captures.

```bash
python3 p6sim.py gen -o sim.bin [--count --snr --bw --df --len --filter fir|iir ...]
python3 p6sim.py selftest [--part a|b|c|d|e] [--filter fir|iir] [--mem 7]
python3 p6sim.py realtest raw.bin [--out real.log] [--chan-out chan.txt]
```

### `apx2icao.py` (Offline Log & Parity Analyzer)

Post-processes `--msgs` logs across multiple captures using quality-weighted AP clustering (`--ap-tol`), GF(2) parity checks against ICAO allocation blocks or 1090 MHz `aircraft.json` reference data (`--ref`), and per-bit AP error statistics.

```bash
python3 apx2icao.py cap1.log [cap2.log ...] [--ref aircraft.json|dir] \
                    [--blocks blocks.csv] [--min-q 180] [--ap-tol 2] [--top 30]
```

## Why a Viterbi Decoder?

Mode S uplinks use 4 Mbit/s DPSK (0.25 us per chip). At an RTL-SDR sample rate of 2.5 MS/s, the tuner low-pass filter smears adjacent chips into each other (inter-symbol interference). Simple per-chip phase slicing produces ~20..25% bit errors and almost no error-free frames.

`dump1030` solves this with Maximum-Likelihood Sequence Estimation (MLSE):

1. **Blind channel identification:** `p6sim.py` estimates the combined tuner and filter impulse response from real off-air recordings using decision feedback and least-squares fitting.
2. **Per-frame synchronization:** Timing, carrier phase, and amplitude are fitted against the known preamble (`P1`/`P2`, start of `P6`, SPR), while the carrier frequency offset is estimated from squared differential phases and refined in a second pass against the reconstructed P6 model.
3. **Sequence decoding:** A 64-state Viterbi decoder (7-chip channel memory) finds the most likely bit sequence for the measured channel response.

## Known Limitations

- **USB sample drops:** Some RTL-SDR sticks and USB controllers drop samples at 2.5 MS/s; higher sample rates are not supported with `--chan`.
- **Detector sensitivity:** The real-time C++ pulse detector triggers on fewer weak P6 frames than the offline Python detector in `p6sim.py`.
- **No independent uplink CRC:** Mode S uplinks XOR the 24-bit ICAO address into the CRC, so any single bit error produces a different valid-looking address. Repetition (`*`) is therefore used as a confidence indicator, not external verification.
- **Hardware-specific channel file:** Regenerate `chan.txt` whenever you change the SDR stick or gain setting.
- **UF16 payload:** The `MU` field of UF16 ACAS frames is logged as raw hex, not parsed into individual TCAS subfields.

## License

Distributed under the same license as the original [dump1030](https://github.com/Fuziih/dump1030) by Fuziih.