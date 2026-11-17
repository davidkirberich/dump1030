# dump1030

**dump1030** is a C++17 command-line receiver that detects and decodes Secondary Surveillance Radar (SSR) interrogations on 1030 MHz using an RTL-SDR.

It detects Mode A, Mode C, Mode A/C All-Call (including Compatibility Mode), and Mode S interrogations. **Mode S (P6/DPSK) uplinks are fully decoded**, including the Uplink Format (UF), all protocol fields, and the **target aircraft ICAO address** recovered from the Address/Parity (AP) field.

---

## Origin

This project is a fork of [dump1030 by Fuziih](https://github.com/Fuziih/dump1030), which itself builds upon [antirez's dump1090](https://github.com/antirez/dump1090). Many thanks to both authors for laying the groundwork.

### What is new in this fork

- **Modern C++17 architecture:** Complete rewrite featuring lock-free double buffering, Q8 ratio arithmetic, and precomputed timing tables.
- **Mode S P6 demodulator:** Sync Phase Reversal (SPR) lock, automatic frame length estimation (56/112 bits), per-frame carrier frequency offset estimation, and `--ppm` tuner correction.
- **Viterbi / MLSE DPSK decoder:** A 64-state Maximum-Likelihood Sequence Estimator operating on a blindly learned receiver impulse response. Read chapter "**Why a Viterbi Decoder?**" for more information.
- **Target ICAO address recovery:** Extracts the 24-bit aircraft address (`AA`) via $\text{AA} = \text{AP} \oplus \text{CRC}_{24}(\text{data})$ for UF0, UF16, and related formats.
- **Temporal address confirmation:** Marks an aircraft address as confirmed (`*`) once it is observed at least twice with no gap longer than 60 seconds between sightings.
- **Beast binary output:** Streams Mode A/C (`0x55`) and Mode S uplink (`0x56`) frames to a file, `stdout`, or over TCP.
- **Simulation & analysis suite (`p6sim.py`):** Self-contained Python 3 tool (standard library only) for generating synthetic test signals, running decoder benchmarks, and learning the receiver channel response.

---

## Requirements

- Linux system
- RTL-SDR dongle capable of streaming stably at $2.5\text{ MS/s}$ tuned to $1030\text{ MHz}$
- C++17 compiler (`g++`) and `librtlsdr`
- Python3 (only required for `p6sim.py` during initial channel learning or offline testing)

---

## Installation

```bash
apt update && apt install -y build-essential pkg-config librtlsdr-dev
git clone https://github.com/davidkirberich/dump1030.git
cd dump1030
make
chmod +x dump1030
```

*(Note: Ensure the `Makefile` produces the binary named `dump1030`.)*

---

## Quick Start

1. **Record a 30-second raw I/Q capture** ($\approx 150\text{ MB}$):

   ```bash
   rtl_sdr -d 1 -f 1030000000 -s 2500000 -g 42 -n 75000000 raw_30s.bin
   ```

2. **Learn the receiver channel response.** You only need to repeat this step if you change your RTL-SDR stick, gain setting, or sample rate:

   ```bash
   python3 p6sim.py realtest raw_30s.bin --max-sec 5 --chan-out chan.txt
   ```

3. **Run live with full Mode S uplink decoding:**

   ```bash
   ./dump1030 --device 1 --gain 42 --chan chan.txt --uplink-decode --msgs --continuous
   ```

   Or process an existing I/Q recording offline:

   ```bash
   ./dump1030 --file raw_30s.bin --chan chan.txt --uplink-decode --msgs
   ```

### Example Output

```text
[P6] 112 Bit spr=0.00 us lock=0.00 margin=1.00/1.00 weak=0 df=-3.3 kHz Q=227 HEX=8080000032000000407B54F95894 UF=16 AA=AAAC07 * RL=1 AQ=0 APx=AAAC07
```

- **`AA=`** Recovered 24-bit target ICAO address.
  - **`*`** indicates a **confirmed** address (seen at least twice with no gap longer than $60\text{ s}$ between sightings).
  - **`?`** indicates a first sighting (or that the previous sighting occurred more than $60\text{ s}$ ago).
- **`Q=`** Frame quality score ($0\text{–}255$), derived from the normalized Viterbi path metric.

*Fallback behavior:* If `--chan` is omitted, `dump1030` falls back to legacy per-chip phase slicing. Frame length and Uplink Format (`UF`) remain usable, but payload bits and target addresses will not be reliable.

---

## Command-Line Options

### Input & Receiver

```text
--device <n>            RTL-SDR device index (default: 0)
--file <path>           Read 8-bit unsigned I/Q samples from file instead of RTL-SDR
--gain <dB|auto|max>    Tuner gain in dB, "auto", or "max" (default: max)
--samplerate <rate>     Sample rate: 2.4..3.2 MS/s or Hz (default: 2.5). When using --file,
                        this MUST match the recording's sample rate!
--ppm <-200..200>       Tuner frequency correction in ppm
--agc                   Enable RTL2832U digital AGC
--size <bytes>          Read buffer size in bytes (multiple of 16384, min: 16384)
--continuous            Run continuously (--size sets the reporting interval)
```

### Mode S Uplink Decoding

```text
--uplink-decode         Enable Mode S P6 (DPSK) uplink demodulation
--chan <path>           Load learned receiver channel file (enables Viterbi decoder)
--msgs                  Print decoded interrogations with sample details
```

### Beast Binary Output

Messages can be exported in Beast binary format (`0x55` = Mode A/C, `0x56` = Mode S uplink when `--uplink-decode` is enabled):

```text
--beast-out <path|->    Write Beast binary frames to a file or stdout ("-")
--net-beast-port <p>    Start a Beast TCP server on port <p> (standard port: 30005)
```

### Debugging & Verification

```text
--vcompare <log>        Decode frames listed in a p6sim log using C++ and compare
                        bit-by-bit against Python (requires --file and --chan)
--spr-offset <us>       Manually shift the SPR lock point in microseconds (debug only)
--print                 Print all raw magnitude values (very slow)
--order                 Print sequence order of detected interrogation types
--help                  Display help message
```

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

*(See the original `dump1030` documentation for tuning details; not all threshold flags are mutually compatible.)*

---

## Tools

### `p6sim.py` Simulator & Channel Estimator

`p6sim.py` is a companion Python tool for testing and channel calibration:

```bash
python3 p6sim.py gen -o sim.bin [--count --snr --bw --df --len --filter fir|iir ...]
python3 p6sim.py compare sim.truth sim.log
python3 p6sim.py selftest [--part a|b|c|d|e] [--filter fir|iir] [--mem 7]
python3 p6sim.py realtest raw.bin [--out real.log] [--chan-out chan.txt]
```

- **`gen`** Synthesizes realistic P6 DPSK recordings with ground-truth logs.
- **`selftest`** Runs unit tests on the Viterbi decoder (ideal channel, memory depth, model mismatch, channel estimation, and blind decoding).
- **`realtest`** Detects and decodes frames in raw I/Q captures, estimates the receiver channel (`--chan-out`), and prints AP field statistics.

---

### `apx2icao.py` Offline Log & Parity Analyzer

`apx2icao.py` is an offline analysis script (Python 3, standard library only) for post-processing `--msgs` capture logs. While `dump1030` already recovers and confirms target ICAO addresses in real time[cite: 3], `apx2icao.py` provides deeper statistical verification across multiple log files:

```bash
python3 apx2icao.py cap1.log [cap2.log ...] [--ref aircraft.json|dir] \
                    [--blocks blocks.csv] [--min-q 180] [--ap-tol 2] [--top 30]
```

- **Quality-weighted clustering:** Groups identical payloads and merges AP fields within a configurable Hamming tolerance (`--ap-tol`), resolving bit errors via quality-weighted majority voting[cite: 2].
- **GF(2) parity verification:** Tests invertible linear parity hypotheses ($X = \text{AP} \oplus \text{CRC}_{24}(\text{data})$) against known regional ICAO allocation blocks and optional 1090 MHz receiver reference data (`--ref`, e.g., `aircraft.json` or OpenSky exports)[cite: 2].
- **Diagnostics:** Reports TCAS interrogation intervals ($\sim 1\text{ s}$) and per-bit AP error rates across repeated frames[cite: 2].

---

## Known Limitations

- **USB sample drops:** Some RTL-SDR dongles and USB host controllers drop samples even at $2.5\text{ MS/s}$. Higher sample rates are not supported.
- **Detector sensitivity:** The real-time C++ pulse detector currently triggers on fewer weak P6 frames than the offline Python detector in `p6sim.py`.
- **Single sightings (`?`):** Addresses marked `?` may contain residual bit errors from noise bursts or weak signals. Rely on confirmed addresses (`*`). Aircraft interrogated less frequently than once every $60\text{ s}$ will remain marked `?`.
- **Hardware-specific channel file:** Always regenerate `chan.txt` if you swap your SDR dongle, change the tuner gain, or alter the sample rate.
- **UF16 payload:** The `MU` (Message, Uplink) field of UF16 ACAS frames is extracted as raw hex, not parsed into individual TCAS resolution advisory subfields.

---

## Why a Viterbi Decoder?

Mode S uplink data is modulated using Differential Phase Shift Keying (DPSK) at a chip rate of $4\text{ Mchips/s}$ ($0.25\text{ }\mu\text{s}$ per chip). At a standard RTL-SDR sample rate of $2.5\text{ MS/s}$, the tuner's low-pass filter bandwidth is narrow enough that an isolated chip between two phase reversals is heavily smeared across adjacent samples (inter-symbol interference).

Simple chip-by-chip phase slicing fails under these conditions, yielding roughly $20\text{–}25\%$ bit errors and virtually zero error-free frames. **dump1030** overcomes this with Maximum-Likelihood Sequence Estimation (MLSE):

1. **Blind channel identification:** The combined impulse response of the tuner and anti-aliasing filter is learned *blindly* from real off-air recordings using decision feedback and least-squares estimation. This calibration needs to be performed only once per hardware/gain/sample-rate setup.
2. **Per-frame synchronization:** For each detected interrogation, timing, carrier phase, and amplitude are estimated by fitting the known preamble ($P_1/P_2$, start of $P_6$, and SPR). The carrier frequency offset is estimated separately from the DPSK data chips inside $P_6$ by squaring the phase differences of consecutive samples, which removes the data-dependent $180^\circ$ phase reversals.
3. **Sequence decoding:** A 64-state Viterbi decoder (7-chip channel memory) finds the most likely transmitted bit sequence given the measured channel response.

### Validation

- **Synthetic benchmarks:** $100\%$ error-free frames in `p6sim.py`, including blind tests with unknown, non-linear-phase IIR filters.
- **Off-air recordings:**
  - Decoded AP fields match uniform 24-bit address distributions.
  - Aircraft addresses are repeatedly confirmed by *distinct* interrogation payloads resolving to the exact same ICAO address.
  - Periodic $\sim 1\text{ s}$ TCAS interrogation intervals are clearly visible.
- **C++ vs. Python parity:** $100.00\%$ bit-identical output across 1,209 real off-air frames ($0.0000\%$ bit discrepancy).

---

## License

Distributed under the same license as the original [dump1030](https://github.com/Fuziih/dump1030) by Fuziih.