#!/usr/bin/env python3
"""p6sim.py - synthetische P6-Uplinks mit bekannten Bits

Erzeugen:
  p6sim.py gen -o sim.bin [--sample-rate 2500000] [--format cu8|sc16]
                          [--count 2000]
                          [--snr 25] [--bw 1.0] [--df -6.4]
                          [--len 56|112|mix] [--amp 50] [--gap 500]
                          [--drop-ppm 0] [--seed 1]
Vergleichen:
  p6sim.py compare sim.truth sim.log
"""
import argparse
import cmath
import collections
from concurrent.futures import ProcessPoolExecutor
import math
import os
import random
import re
import struct
import sys
import time

import numpy as np

FS_HI = 20e6
FS = 2.5e6
DROP_BLK = 256                                  # I/Q-Paare je Drop (~512 Byte)


def model_ticks_per_input_sample(sample_rate):
    return FS_HI / float(sample_rate)


def hi(t_us):                                   # us -> Index bei 20 MS/s
    return int(round(t_us * FS_HI / 1e6))


def design_lp(fc_hz, ntaps=129):                # gefenstertes Sinc, Hamming
    M = ntaps - 1
    h = []
    for n in range(ntaps):
        k = n - M / 2
        if k == 0:
            s = 2 * fc_hz / FS_HI
        else:
            s = math.sin(2 * math.pi * fc_hz / FS_HI * k) / (math.pi * k)
        h.append(s * (0.54 - 0.46 * math.cos(2 * math.pi * n / M)))
    g = sum(h)
    return [x / g for x in h]


def burst(bits, phi0, phi1, phi2):
    """Komplexe Huellkurve bei 20 MS/s, Index 0 = Vorderflanke P1."""
    p6_len = 1.25 + 0.5 + len(bits) * 0.25 + 0.5
    x = [0j] * hi(3.5 + p6_len)
    for n in range(hi(0.0), hi(0.8)):
        x[n] = cmath.exp(1j * phi1)
    for n in range(hi(2.0), hi(2.8)):
        x[n] = cmath.exp(1j * phi2)
    t_spr = 3.5 + 1.25
    rev = [hi(t_spr)] + [hi(t_spr + 0.5 + b * 0.25)
                         for b, v in enumerate(bits) if v]
    ph, ri = phi0, 0
    for n in range(hi(3.5), len(x)):
        while ri < len(rev) and n >= rev[ri]:
            ph += math.pi
            ri += 1
        x[n] = cmath.exp(1j * ph)
    return x


def gen(a):
    rnd = random.Random(a.seed)
    sample_rate = a.sample_rate
    h = design_lp(a.bw * 1e6)
    half = len(h) // 2
    sigma = a.amp / 10 ** (a.snr / 20)
    slot = int(round(a.gap * 1e-6 * sample_rate))
    off = slot // 3
    w = 2 * math.pi * a.df * 1e3 / sample_rate
    model_step = model_ticks_per_input_sample(sample_rate)
    samples = []
    truth = []
    g = 0
    for f in range(a.count):
        n = 112 if (a.len == '112' or (a.len == 'mix' and rnd.random() < 0.1)) else 56
        head = [1, 0, 0, 0, 0] if n == 112 else [0, 0, 0, 0, 0]
        bits = head + [rnd.getrandbits(1) for _ in range(n - 5)]
        x = burst(bits, *(rnd.uniform(0, 2 * math.pi) for _ in range(3)))
        frac = rnd.random() * model_step
        y = [complex(rnd.gauss(0, sigma), rnd.gauss(0, sigma)) for _ in range(slot)]
        wave = [0j] * len(x)
        for n_hi in range(len(x)):
            for m, hv in enumerate(h):
                source = n_hi + half - m
                if 0 <= source < len(x):
                    wave[n_hi] += hv * x[source]
        for k in range(slot):
            t = (k - off) * model_step - frac
            i0 = int(math.floor(t))
            if i0 < 0 or i0 >= len(wave) - 1:
                continue
            alpha = t - i0
            value = wave[i0] * (1.0 - alpha) + wave[i0 + 1] * alpha
            y[k] += a.amp * value * cmath.exp(1j * w * (g + k))
        samples.extend(y)
        hexs = format(int(''.join(map(str, bits)), 2), '0%dX' % (n // 4))
        truth.append((g + off + frac / model_step, n, hexs))
        g += slot
        if f % 200 == 0:
            print(f"  {f}/{a.count}", file=sys.stderr)

    out = bytearray()
    dropped = 0
    p_drop = a.drop_ppm * 1e-6
    for s in range(0, len(samples), DROP_BLK):
        if p_drop and rnd.random() < p_drop:
            dropped += 1
            continue
        for v in samples[s:s + DROP_BLK]:
            if a.format == 'cu8':
                out.append(min(255, max(0, int(round(127.5 + v.real)))))
                out.append(min(255, max(0, int(round(127.5 + v.imag)))))
            else:
                scale = a.sc16_scale / 127.5
                out.extend(struct.pack(
                    '<hh',
                    max(-32768, min(32767, int(round(v.real * scale)))),
                    max(-32768, min(32767, int(round(v.imag * scale)))),
                ))

    with open(a.o, 'wb') as fo:
        fo.write(out)
    tname = re.sub(r'\.bin$', '', a.o) + '.truth'
    with open(tname, 'w') as ft:
        ft.write(f"# snr={a.snr} bw={a.bw} df={a.df} amp={a.amp} "
                 f"drop_ppm={a.drop_ppm} seed={a.seed} fs={sample_rate} "
                 f"format={a.format}\n")
        for idx, n, hx in truth:
            ft.write(f"{idx:.3f} {n} {hx}\n")
    print(f"{a.o}: {g / sample_rate:.2f} s, "
          f"{a.count} Frames, {dropped} Drop-Bloecke; "
          f"Wahrheit in {tname}")
    if dropped:
        print("  Hinweis: Indizes in .truth sind nicht um Drops korrigiert "
              "(compare ordnet ueber Reihenfolge + Bits zu).")


def tobits(hx):
    return [int(c) for c in format(int(hx, 16), '0%db' % (len(hx) * 4))]


def apstat(lst):
    if not lst:
        return 0.0, 0.0, 0.0
    w = 0
    pr = collections.Counter()
    runs = collections.Counter()
    for b in lst:
        ap = b[32:56]
        w += sum(ap)
        for x, y in zip(ap, ap[1:]):
            pr[(x, y)] += 1
        for r in re.findall('1+', ''.join(map(str, ap))):
            runs[len(r)] += 1
    t, rt = sum(pr.values()), sum(runs.values())
    return w / len(lst), pr[(1, 1)] / t * 100, (runs[1] / rt * 100 if rt else 0.0)


def compare(a):
    T = []
    for line in open(a.truth):
        if line.startswith('#') or not line.strip():
            continue
        idx, n, hx = line.split()
        T.append((float(idx), int(n), hx))
    pat = re.compile(r'\[P6\].*HEX=([0-9A-F]+)')
    D = []
    for line in open(a.log, errors='replace'):
        m = pat.search(line)
        if m:
            D.append(m.group(1))

    j, matched, unmatched = 0, [], 0
    for hx in D:
        db = tobits(hx)
        best = None
        for k in range(j, min(j + 50, len(T))):
            if T[k][1] != len(db):
                continue
            d = sum(x != y for x, y in zip(db, tobits(T[k][2])))
            if best is None or d < best[0]:
                best = (d, k)
        if best and best[0] <= len(db) // 4:
            matched.append((best[1], db))
            j = best[1] + 1
        else:
            unmatched += 1

    print(f"Wahrheit: {len(T)} Frames | dump1030: {len(D)} P6-Zeilen")
    print(f"Zugeordnet: {len(matched)} ({len(matched) / max(1, len(T)) * 100:.1f} %), "
          f"nicht zuordenbar: {unmatched}")
    if not matched:
        return

    nbit = nerr = okf = 0
    patt = collections.defaultdict(lambda: [0, 0])
    pos = collections.defaultdict(lambda: [0, 0])
    for k, db in matched:
        tb = tobits(T[k][2])
        e = [x != y for x, y in zip(db, tb)]
        nbit += len(tb)
        nerr += sum(e)
        okf += not any(e)
        for b in range(len(tb)):
            pos[b][0] += 1
            pos[b][1] += e[b]
            if b:
                p = patt[f"{tb[b - 1]}->{tb[b]}"]
                p[0] += 1
                p[1] += e[b]

    print(f"Fehlerfreie Frames: {okf} ({okf / len(matched) * 100:.1f} %)")
    print(f"Bitfehlerrate: {nerr / nbit * 100:.2f} %")
    print("Fehlerrate nach Muster (Vorgaenger->Bit, Wahrheit):")
    for key in ('0->0', '0->1', '1->0', '1->1'):
        n, e = patt[key]
        if n:
            print(f"  {key}: {e / n * 100:6.2f} %  (n={n})")

    print("Fehlerrate je Bitposition (1-basiert, nur >1 %):")
    bad = [(b, e / n * 100) for b, (n, e) in sorted(pos.items()) if n and e / n > 0.01]
    print("  " + ("  ".join(f"{b + 1}:{r:.1f}%" for b, r in bad) if bad else "keine"))

    t56 = [tobits(T[k][2]) for k, db in matched if len(db) == 56]
    d56 = [db for k, db in matched if len(db) == 56]
    print("AP-Feld (56 Bit)          Gewicht   11-Anteil  Lauflaenge 1")
    for name, (w, p11, r1) in (("Wahrheit", apstat(t56)),
                               ("dump1030 auf Sim.", apstat(d56)),
                               ("echte Aufnahme", (5.81, 3.1, 87.4))):
        print(f"  {name:22s} {w:6.2f}   {p11:6.1f} %   {r1:6.1f} %")


# ===================== SELFTEST (Stufe 1) =====================
ST_PAD = 160
ST_ANCHOR = ST_PAD + 105
ST_NBMAX = 112
ST_LMIN, ST_LMAX = -20, 59


def st_design_fir(bw, fs=20.0, ntaps=129):
    fc = bw / fs
    mid = (ntaps - 1) / 2
    h = []
    for i in range(ntaps):
        x = i - mid
        s = 2 * fc if x == 0 else math.sin(2 * math.pi * fc * x) / (math.pi * x)
        h.append(s * (0.54 - 0.46 * math.cos(2 * math.pi * i / (ntaps - 1))))
    g = sum(h)
    return [v / g for v in h]


def st_design_butter6(bw, fs=20.0):
    N = 6
    wc = 2 * fs * math.tan(math.pi * bw / fs)
    secs = []
    for k in range(N // 2):
        p = wc * cmath.exp(1j * math.pi * (2 * k + 1 + N) / (2 * N))
        z = (1 + p / (2 * fs)) / (1 - p / (2 * fs))
        a1, a2 = -2 * z.real, abs(z) ** 2
        g = (1 + a1 + a2) / 4
        secs.append((g, 2 * g, g, a1, a2))
    return secs


def st_make_filter(kind, bw):
    if kind == 'fir':
        h = st_design_fir(bw)
        mid = (len(h) - 1) // 2

        def f(x):
            n = len(x)
            nz = [(k, v) for k, v in enumerate(x) if v != 0.0]
            y = [0.0] * n
            for k, v in nz:
                for i, hv in enumerate(h):
                    m = k - mid + i
                    if 0 <= m < n:
                        y[m] += hv * v
            return y
        return f, 0.0
    secs = st_design_butter6(bw)

    def f(x):
        y = list(x)
        for b0, b1, b2, a1, a2 in secs:
            z1 = z2 = 0.0
            out = []
            for v in y:
                o = b0 * v + z1
                z1 = b1 * v - a1 * o + z2
                z2 = b2 * v - a2 * o
                out.append(o)
            y = out
        return y
    imp = f([1.0] + [0.0] * 599)
    delay = sum(i * v for i, v in enumerate(imp)) / sum(imp)
    return f, delay


class StModel:
    def __init__(self, kind, bw):
        self.kind, self.bw = kind, bw
        self.ltot = 2 * ST_PAD + int(round(20 * (5.25 + 0.25 * ST_NBMAX + 0.5)))
        self.anchor = ST_ANCHOR
        f, self.delay = st_make_filter(kind, bw)
        x = [0.0] * self.ltot
        for i in range(self.anchor, self.anchor + 5):
            x[i] = 1.0
        self.g0 = f(x)
        x = [0.0] * self.ltot
        for i in range(ST_PAD, ST_PAD + 16):
            x[i] = 1.0
        for i in range(ST_PAD + 40, ST_PAD + 56):
            x[i] = 1.0
        for i in range(self.anchor - 35, self.anchor - 10):
            x[i] = 1.0
        for i in range(self.anchor - 10, self.anchor):
            x[i] = -1.0
        self.kn = f(x)
        self.cache = {}

    def g(self, m, t):
        return st_interp(self.g0, t - 5 * m)

    def knv(self, t):
        return st_interp(self.kn, t)

    def support(self):
        if 'sup' not in self.cache:
            idx = [i for i, v in enumerate(self.g0) if abs(v) > 1e-4]
            self.cache['sup'] = (idx[0], idx[-1])
        return self.cache['sup']


def st_interp(y, t):
    if t < 0 or t >= len(y) - 1:
        return 0.0
    i = int(t)
    a = t - i
    return y[i] * (1 - a) + y[i + 1] * a


_st_models = {}


def st_model(kind, bw):
    key = (kind, round(bw, 4))
    if key not in _st_models:
        _st_models[key] = StModel(kind, bw)
    return _st_models[key]


def st_bits2chips(bits):
    c, chips = -1, []
    for b in list(bits) + [0, 0]:
        if b:
            c = -c
        chips.append(c)
    return chips


def st_frame_samples(model, chips, off, A, phi, snr, rng, sample_rate=FS):
    lo, hi_idx = model.support()
    rot = A * cmath.exp(1j * phi)
    sig = 1.0 / snr
    out, t = [], off
    step = model_ticks_per_input_sample(sample_rate)
    while t < model.ltot - 1:
        v = model.knv(t)
        j0 = max(0, math.ceil((t - hi_idx) / 5))
        j1 = min(len(chips) - 1, math.floor((t - lo) / 5))
        for j in range(j0, j1 + 1):
            v += chips[j] * model.g(j, t)
        out.append(rot * v + complex(rng.gauss(0, sig), rng.gauss(0, sig)))
        t += step
    return out


def st_old(r, off, delay, anchor, n, model_step=8.0):
    def at(t):
        u = (t - off) / model_step
        i = math.floor(u)
        a = u - i
        if i < 1 or i + 2 >= len(r):
            return 0j
        p0, p1, p2, p3 = r[i - 1], r[i], r[i + 1], r[i + 2]
        return p1 + 0.5 * a * (p2 - p0 + a * (2 * p0 - 5 * p1 + 4 * p2 - p3
                                              + a * (3 * (p1 - p2) + p3 - p0)))
    prev = at(anchor + delay - 2.5)
    bits = []
    for b in range(n):
        cur = at(anchor + delay + 5 * b + 2.5)
        bits.append(1 if (cur * prev.conjugate()).real < 0 else 0)
        prev = cur
    return bits


def st_prepare(rx, r, off, phi, A, L, N, model_step=8.0):
    der = cmath.exp(-1j * phi) / A
    out = []
    for k, s in enumerate(r):
        t = off + model_step * k
        jc = math.floor((t - rx.delay - rx.anchor) / 5)
        w = jc - (L - 1) // 2
        e = w + L - 1
        if e < 0 or w > N - 1:
            continue
        Bs = [rx.g(w + i, t) if w + i >= 0 else 0.0 for i in range(L)]
        out.append((min(e, N - 1), w, rx.knv(t), Bs, s * der))
    return out


def st_viterbi(samples, L, n, N, retm=False):
    by = {}
    for e, w, kv, Bs, z in samples:
        by.setdefault(e, []).append((w, kv, Bs, z))
    states = {tuple([-1] * (L - 1)): (0.0, [])}
    for j in range(N):
        base = j - L + 1
        grp = by.get(j, ())
        new = {}
        for st, (met, path) in states.items():
            opts = (1, -1) if j < n else (st[-1],)
            for c in opts:
                h = st + (c,)
                m = met
                for w, kv, Bs, z in grp:
                    v = kv
                    for i, b in enumerate(Bs):
                        idx = w + i
                        if idx <= j:
                            v += h[idx - base] * b
                    d = z - v
                    m += d.real * d.real + d.imag * d.imag
                ns = h[1:]
                if ns not in new or m < new[ns][0]:
                    new[ns] = (m, path + [c])
        states = new
    best = min(states.values(), key=lambda x: x[0])
    chips = best[1]
    bits, prev = [], -1
    for j in range(n):
        bits.append(1 if chips[j] != prev else 0)
        prev = chips[j]
    return (bits, best[0]) if retm else bits


def st_eval(tx, rx, n, Ls, count, snr, dt, dphi, seed, old=True,
            sample_rate=FS):
    rng = random.Random(seed)
    res = {}
    keys = (['old'] if old else []) + list(Ls)
    model_step = model_ticks_per_input_sample(sample_rate)
    for k in keys:
        res[k] = [0, 0]
    N = n + 2
    for _ in range(count):
        bits = [rng.randint(0, 1) for _ in range(n)]
        chips = st_bits2chips(bits)
        off = rng.uniform(0, model_step)
        phi = rng.uniform(-math.pi, math.pi)
        r = st_frame_samples(
            tx, chips, off, 1.0, phi, snr, rng, sample_rate
        )
        decs = {}
        if old:
            decs['old'] = st_old(
                r, off + dt, rx.delay, rx.anchor, n, model_step
            )
        for L in Ls:
            smp = st_prepare(
                rx, r, off + dt, phi + math.radians(dphi),
                1.0, L, N, model_step
            )
            decs[L] = st_viterbi(smp, L, n, N)
        for k, d in decs.items():
            e = sum(1 for a, b in zip(bits, d) if a != b)
            res[k][0] += e
            res[k][1] += (e == 0)
    return res


def st_print(label, res, count, n):
    for k, (be, ok) in res.items():
        name = 'alt     ' if k == 'old' else 'Viterbi%d' % k
        print('  %-28s %s  BER %7.3f %%   fehlerfrei %6.1f %%'
              % (label, name, 100.0 * be / (count * n), 100.0 * ok / count))


def cmd_selftest(args):
    t0 = time.time()
    n, cnt, snr = args.nbits, args.count, args.snr
    tx = st_model(args.filter, args.bw)
    print('selftest: filter=%s bw=%.2f MHz snr=%.1f n=%d count=%d delay=%.2f Samples'
          % (args.filter, args.bw, snr, n, cnt, tx.delay))
    parts = 'abc' if args.part == 'all' else args.part

    if 'a' in parts:
        print('\n[1a] ideal (Filter, frac, Phase, Amplitude bekannt, df=0)')
        res = st_eval(
            tx, tx, n, [args.mem], cnt, snr, 0.0, 0.0, args.seed,
            sample_rate=args.sample_rate
        )
        st_print('ideal', res, cnt, n)
        ok = 100.0 * res[args.mem][1] / cnt
        print('  -> %s (Kriterium: >= 90 %% fehlerfrei)' % ('BESTANDEN' if ok >= 90 else 'NICHT bestanden'))

    if 'b' in parts:
        print('\n[1b] Gedaechtnis (ideal)')
        res = st_eval(
            tx, tx, n, [3, 4, 5, 6, 7], cnt, snr, 0.0, 0.0,
            args.seed, old=False, sample_rate=args.sample_rate
        )
        st_print('ideal', res, cnt, n)

    if 'c' in parts:
        print('\n[1c] Modell-Fehlanpassung, Viterbi%d' % args.mem)
        cases = [('bw x0.9', 0.9, 0.0, 0.0), ('bw x1.1', 1.1, 0.0, 0.0),
                 ('dt -0.025 us', 1.0, -0.5, 0.0), ('dt +0.025 us', 1.0, 0.5, 0.0),
                 ('dt -0.05 us', 1.0, -1.0, 0.0), ('dt +0.05 us', 1.0, 1.0, 0.0),
                 ('phase -5 deg', 1.0, 0.0, -5.0), ('phase +5 deg', 1.0, 0.0, 5.0),
                 ('phase -10 deg', 1.0, 0.0, -10.0), ('phase +10 deg', 1.0, 0.0, 10.0)]
        for label, fb, dt, dp in cases:
            rx = st_model(args.filter, args.bw * fb)
            res = st_eval(
                tx, rx, n, [args.mem], cnt, snr, dt, dp, args.seed,
                old=(fb != 1.0), sample_rate=args.sample_rate
            )
            st_print(label, res, cnt, n)
    if 'd' in parts:
        print('\n[2a] Kanal geschaetzt aus %d Trainingsframes' % args.train)
        h = st_estimate_channel(
            tx, args.train, n, snr, args.seed + 1000, args.sample_rate
        )
        ht = st_true_h(tx)
        err = math.sqrt(sum((a - b) ** 2 for a, b in zip(h, ht)) / sum(b * b for b in ht))
        print('  rel. Kanalfehler %.4f   delay geschaetzt %.2f Samples' % (
            err, sum((i + ST_LMIN) * v for i, v in enumerate(h)) / sum(h)
        ))
        rx = st_model_from_h(tx, h)
        res = st_eval(
            tx, rx, n, [args.mem], cnt, snr, 0.0, 0.0, args.seed,
            old=False, sample_rate=args.sample_rate
        )
        st_print('geschaetzt', res, cnt, n)
    if 'e' in parts:
        print('\n[2b] Blindtest, Start %s bw=%.2f' % (args.start, args.start_bw))
        rng = random.Random(args.seed + 2000)
        model_step = model_ticks_per_input_sample(args.sample_rate)
        frames = []
        for _ in range(max(cnt, args.train)):
            bits = [rng.randint(0, 1) for _ in range(n)]
            off = rng.uniform(0, model_step)
            phi = rng.uniform(-math.pi, math.pi)
            frames.append((
                bits,
                st_frame_samples(
                    tx, st_bits2chips(bits), off, 1.0, phi, snr, rng,
                    args.sample_rate
                )
            ))
        rx = st_model(args.start, args.start_bw)
        N = n + 2
        for rnd in range(args.rounds + 1):
            last = rnd == args.rounds
            use = frames if last else frames[:args.train]
            dec, be, ok = [], 0, 0
            for bits, r in use:
                decoded = st_blind_decode(
                    rx, r, n, N, args.mem, args.sample_rate
                )
                if decoded is None:
                    be += n
                    continue
                m, d, c, phi, A = decoded
                e = sum(1 for a, b in zip(bits, d) if a != b)
                be += e
                ok += (e == 0)
                dec.append((r, c, phi, A, st_bits2chips(d)))
            print('  Runde %d  %4d Frames  BER %7.3f %%  fehlerfrei %6.1f %%'
                  % (rnd, len(use), 100.0 * be / (len(use) * n), 100.0 * ok / len(use)))
            if not last and dec:
                rx = st_model_from_h(
                    tx,
                    st_ls_frames(rx.ltot, rx.anchor, dec, args.sample_rate)
                )
    print('\nLaufzeit %.1f s' % (time.time() - t0))


def st_input(ltot, anchor, chips):
    x = np.zeros(ltot, dtype=np.float64)
    x[ST_PAD:ST_PAD + 16] = 1.0
    x[ST_PAD + 40:ST_PAD + 56] = 1.0
    x[anchor - 35:anchor - 10] = 1.0
    x[anchor - 10:anchor] = -1.0
    for j, c in enumerate(chips):
        s0 = anchor + 5 * j
        s1 = min(ltot, s0 + 5)
        if s0 < ltot:
            x[s0:s1] = float(c)
    return x


def st_estimate_channel(tx, ntrain, n, snr, seed, sample_rate=FS):
    rng = random.Random(seed)
    frames = []
    model_step = model_ticks_per_input_sample(sample_rate)
    for _ in range(ntrain):
        bits = [rng.randint(0, 1) for _ in range(n)]
        chips = st_bits2chips(bits)
        off = rng.uniform(0, model_step)
        phi = rng.uniform(-math.pi, math.pi)
        r = st_frame_samples(tx, chips, off, 1.0, phi, snr, rng, sample_rate)
        frames.append((r, off, phi, 1.0, chips))
    return st_ls_frames(tx.ltot, tx.anchor, frames, sample_rate)


def st_true_h(tx):
    f, _ = st_make_filter(tx.kind, tx.bw)
    P = 200
    x = [0.0] * 600
    x[P] = 1.0
    y = f(x)
    return [y[P + l] for l in range(ST_LMIN, ST_LMAX + 1)]


def st_model_from_h(tx, h):
    rx = object.__new__(StModel)
    rx.kind, rx.bw, rx.ltot, rx.anchor, rx.cache = 'est', tx.bw, tx.ltot, tx.anchor, {}
    h_arr = np.asarray(h, dtype=np.float64)
    lags = np.arange(ST_LMIN, ST_LMAX + 1)

    def conv(x_arr):
        y = np.zeros(len(x_arr), dtype=np.float64)
        nz = np.flatnonzero(x_arr)
        for m in nz:
            xv = x_arr[m]
            ks = m + lags
            valid = (ks >= 0) & (ks < len(y))
            y[ks[valid]] += h_arr[valid] * xv
        return y.tolist()

    x = np.zeros(rx.ltot, dtype=np.float64)
    x[rx.anchor:rx.anchor + 5] = 1.0
    rx.g0 = conv(x)
    rx.kn = conv(st_input(rx.ltot, rx.anchor, []))
    rx.delay = float(np.dot(lags, h_arr) / np.sum(h_arr))
    return rx


def st_pre_fit(rx, r, c, model_step):
    lim = rx.anchor + rx.delay - 10
    num, den = 0j, 0.0
    for k, s in enumerate(r):
        t = c + model_step * k
        if t >= lim:
            break
        kv = rx.knv(t)
        num += s * kv
        den += kv * kv
    return (num, den) if den > 0 else None


def st_blind_decode(rx, r, n, N, L, sample_rate=FS, frame_origin=0.0):
    best_c, best_s = None, -1.0
    model_step = model_ticks_per_input_sample(sample_rate)
    for c in range(-24, 33):
        f = st_pre_fit(rx, r, frame_origin + c, model_step)
        if f and abs(f[0]) ** 2 / f[1] > best_s:
            best_c, best_s = c, abs(f[0]) ** 2 / f[1]
    if best_c is None:
        return None
    out = None
    for cc in (best_c - 1, best_c, best_c + 1):
        off = frame_origin + cc
        f = st_pre_fit(rx, r, off, model_step)
        if not f:
            continue
        num, den = f
        phi, A = cmath.phase(num), abs(num) / den
        smp = st_prepare(rx, r, off, phi, A, L, N, model_step)
        bits, m = st_viterbi(smp, L, n, N, retm=True)
        m /= max(1, len(smp))
        if out is None or m < out[0]:
            out = (m, bits, off, phi, A)
    return out


def st_ls_frames(ltot, anchor, frames, sample_rate=FS):
    """Vektorisierte LS-Kanalschaetzung ueber numpy."""
    L = ST_LMAX - ST_LMIN + 1
    lags = np.arange(ST_LMIN, ST_LMAX + 1, dtype=np.int64)
    M = np.zeros((L, L), dtype=np.float64)
    v = np.zeros(L, dtype=np.float64)
    model_step = model_ticks_per_input_sample(sample_rate)

    for r, off, phi, A, chips in frames:
        x = st_input(ltot, anchor, chips)
        nx = len(x)
        r_arr = np.asarray(r, dtype=np.complex128)
        ks = np.arange(len(r_arr), dtype=np.float64)
        ts = off + model_step * ks
        idx = np.floor(ts).astype(np.int64)
        alpha = (ts - idx)[:, None]

        m0 = idx[:, None] - lags[None, :]
        m1 = m0 + 1
        x0 = np.where((m0 >= 0) & (m0 < nx), x[np.clip(m0, 0, nx - 1)], 0.0)
        x1 = np.where((m1 >= 0) & (m1 < nx), x[np.clip(m1, 0, nx - 1)], 0.0)
        X = (1.0 - alpha) * x0 + alpha * x1

        nz = np.any(X != 0.0, axis=1)
        if not np.any(nz):
            continue
        X = X[nz]
        y = (r_arr[nz] * (cmath.exp(-1j * phi) / A)).real
        M += X.T @ X
        v += X.T @ y

    M[np.diag_indices(L)] += 1e-6
    return np.linalg.solve(M, v).tolist()


# ===================== REALTEST (Stufe 3, vektorisiert + parallel) =====================
def rt_window_params(sample_rate):
    prefix = int(math.ceil(8.0 * sample_rate / 1e6))
    window = int(math.ceil(50.0 * sample_rate / 1e6)) + 1
    return prefix, window


def rt_open_memmap(path, sample_format, max_samples):
    size = os.path.getsize(path)
    if sample_format == 'cu8':
        if size % 2:
            raise SystemExit('CU8-Dateigroesse ist kein Vielfaches von 2 Bytes.')
        total = size // 2
        n = min(total, max_samples) if max_samples > 0 else total
        if n == 0:
            return None
        return np.memmap(path, dtype=np.uint8, mode='r', shape=(n, 2))
    if size % 4:
        raise SystemExit('SC16-Dateigroesse ist kein Vielfaches von 4 Bytes.')
    total = size // 4
    n = min(total, max_samples) if max_samples > 0 else total
    if n == 0:
        return None
    return np.memmap(path, dtype='<i2', mode='r', shape=(n, 2))


def rt_chunk_power(raw_chunk, sample_format, sc16_scale):
    if sample_format == 'cu8':
        f = (raw_chunk.astype(np.float32) - 127.5) * (1.0 / 127.5)
    else:
        f = raw_chunk.astype(np.float32) * (1.0 / float(sc16_scale))
    return f[:, 0] * f[:, 0] + f[:, 1] * f[:, 1]


def rt_detect(raw_mm, sample_format, sc16_scale, thr_fac, sample_rate):
    """Vektorisierter P1/P2/P6-Detektor direkt auf memmap."""
    ns = len(raw_mm)
    probe_n = min(ns, 1_000_000)
    probe_pow = rt_chunk_power(raw_mm[:probe_n:97], sample_format, sc16_scale)
    thr = float(np.median(probe_pow)) * thr_fac

    def samples(us):
        return max(1, int(round(us * sample_rate / 1e6)))

    p1_in = samples(0.4)
    p1_pause1 = samples(1.2)
    p1_pause2 = min(samples(2.0) - 1, samples(1.6))
    p2 = samples(2.0)
    p2_in = samples(2.4)
    p2_tail = samples(3.2)
    p6_start = samples(4.0)
    p6_stop = samples(7.6)
    skip = samples(34.0)
    guard = samples(40.0)
    p6_offs = np.arange(p6_start, p6_stop + 1, dtype=np.int64)

    hits = []
    CH = 4_000_000
    k0 = 0
    nxt = 0
    while k0 < ns - guard:
        k1 = min(ns, k0 + CH + guard)
        m = rt_chunk_power(raw_mm[k0:k1], sample_format, sc16_scale)
        hi_lim = len(m) - p6_stop - 1
        if hi_lim <= p1_in:
            break
        cand = np.flatnonzero(m[p1_in:hi_lim] > thr) + p1_in
        if cand.size:
            p = m[cand]
            ok = (
                (m[cand - p1_in] < thr)
                & (m[cand + p1_in] > thr)
                & (m[cand + p1_pause1] < p * 0.25)
                & (m[cand + p1_pause2] < p * 0.25)
                & (m[cand + p2] > thr)
                & (m[cand + p2_in] > thr)
                & (m[cand + p2_tail] < p * 0.25)
            )
            cand = cand[ok]
            if cand.size:
                p6_ok = np.all(m[cand[:, None] + p6_offs[None, :]] > (thr * 0.5), axis=1)
                for idx in cand[p6_ok]:
                    g_idx = k0 + int(idx)
                    if g_idx >= nxt:
                        hits.append(g_idx)
                        nxt = g_idx + skip
        k0 += CH
        print(
            '  scan %5.1f s  %d Kandidaten' % (min(k0, ns) / sample_rate, len(hits)),
            end='\r'
        )
    print()
    return hits, thr


def rt_frame(raw_mm, sample_format, sc16_scale, k, thr, sample_rate):
    prefix, window = rt_window_params(sample_rate)
    s0 = k - prefix
    seg = raw_mm[s0:s0 + window]
    if sample_format == 'cu8':
        f = (seg.astype(np.float64) - 127.5) / 127.5
    else:
        f = seg.astype(np.float64) / float(sc16_scale)
    r = f[:, 0] + 1j * f[:, 1]

    energy_start = prefix + int(round(23.0 * sample_rate / 1e6))
    energy_stop = min(len(r), prefix + int(round(27.0 * sample_rate / 1e6)) + 1)
    tail = r[energy_start:energy_stop]
    e = float(np.mean(tail.real * tail.real + tail.imag * tail.imag)) if len(tail) else 0.0
    n = 112 if e > thr else 56

    df_lag = max(1, int(round(0.4 * sample_rate / 1e6)))
    df_start = max(df_lag, prefix + int(round(3.6 * sample_rate / 1e6)))
    data_end_us = 19.75 if n == 56 else 33.75
    end = min(len(r), prefix + int(round(data_end_us * sample_rate / 1e6)))
    if end > df_start:
        d = r[df_start:end] * np.conj(r[df_start - df_lag:end - df_lag])
        acc = complex(np.sum(d * d))
    else:
        acc = 0j
    w = cmath.phase(acc) / (2.0 * df_lag) if acc != 0 else 0.0
    rot = np.exp(-1j * w * np.arange(len(r), dtype=np.float64))
    r = r * rot
    return r.tolist(), n, w * sample_rate / (2 * math.pi)


def rt_stats(label, rows):
    aps = [b[-24:] for b in rows]
    if not aps:
        return
    wt = sum(sum(a) for a in aps) / len(aps)
    pc = {'00': 0, '01': 0, '10': 0, '11': 0}
    runs = {}
    for a in aps:
        for i in range(23):
            pc['%d%d' % (a[i], a[i + 1])] += 1
        s = ''.join(map(str, a))
        for run in s.split('0'):
            if run:
                runs[len(run)] = runs.get(len(run), 0) + 1
    tp, tr = sum(pc.values()), sum(runs.values())
    print('  %-14s %5d Frames  AP-Gewicht %5.2f (soll 12)' % (label, len(aps), wt))
    print('    Paare ' + '  '.join('%s %4.1f %%' % (p, 100.0 * c / tp) for p, c in sorted(pc.items()))
          + '   (soll je 25 %)')
    print('    Laeufe ' + '  '.join('%d: %4.1f %%' % (l, 100.0 * runs[l] / tr) for l in sorted(runs)[:5])
          + '   (soll 50/25/12,5)')


_W_RX = None
_W_L = 7
_W_SR = FS
_W_ORIGIN = 0.0


def _rt_init_worker(rx, L, sample_rate, frame_origin):
    global _W_RX, _W_L, _W_SR, _W_ORIGIN
    _W_RX = rx
    _W_L = L
    _W_SR = sample_rate
    _W_ORIGIN = frame_origin


def _rt_decode_worker(item):
    k, r, n = item
    decoded = st_blind_decode(_W_RX, r, n, n + 2, _W_L, _W_SR, _W_ORIGIN)
    if decoded is None:
        return None
    m, d, c, phi, A = decoded
    return (k, n, m, d, c, phi, A, r)


def _rt_decode_batch(rx, batch, L, sample_rate, frame_origin, workers):
    items = [(k, r, n) for k, r, n, _ in batch]
    if workers <= 1 or len(items) < 4:
        _rt_init_worker(rx, L, sample_rate, frame_origin)
        return [res for res in map(_rt_decode_worker, items) if res is not None]
    out = []
    with ProcessPoolExecutor(
        max_workers=workers,
        initializer=_rt_init_worker,
        initargs=(rx, L, sample_rate, frame_origin),
    ) as pool:
        for res in pool.map(_rt_decode_worker, items, chunksize=max(1, len(items) // (workers * 4))):
            if res is not None:
                out.append(res)
    return out


def cmd_realtest(args):
    t0 = time.time()
    sample_rate = args.sample_rate
    max_samples = int(args.max_sec * sample_rate) if args.max_sec > 0 else 0
    raw_mm = rt_open_memmap(args.file, args.format, max_samples)
    if raw_mm is None:
        raise SystemExit('Keine IQ-Daten in der Eingabedatei; Abbruch.')
    hits, thr = rt_detect(raw_mm, args.format, args.sc16_scale, args.thr, sample_rate)

    frames = []
    prefix, window = rt_window_params(sample_rate)
    frame_origin = ST_PAD - prefix * model_ticks_per_input_sample(sample_rate)
    n_total = len(raw_mm)
    for k in hits:
        s0 = k - prefix
        if s0 >= 0 and s0 + window <= n_total:
            r, n, df = rt_frame(raw_mm, args.format, args.sc16_scale, k, thr, sample_rate)
            frames.append((k, r, n, df))
    dfs = sorted(f[3] for f in frames)
    print('Frames %d (112 Bit: %d)  df-Median %.2f kHz'
          % (len(frames), sum(1 for f in frames if f[2] == 112), dfs[len(dfs) // 2] / 1000 if dfs else 0))
    if not frames:
        raise SystemExit('Keine gueltigen P6-Kandidaten; Kanaldatei wurde nicht geschrieben.')

    workers = args.workers if args.workers > 0 else (os.cpu_count() or 1)
    rx = st_model(args.start, args.start_bw)
    L = args.mem

    for rnd in range(args.rounds):
        decoded = _rt_decode_batch(rx, frames[:args.train], L, sample_rate, frame_origin, workers)
        if not decoded:
            raise SystemExit('Keine decodierbaren Trainingsframes; Abbruch.')
        dec = [(m, (r, c, phi, A, st_bits2chips(d))) for _, _, m, d, c, phi, A, r in decoded]
        dec.sort(key=lambda x: x[0])
        good = [x[1] for x in dec[:max(10, len(dec) // 2)]]
        print('  Kanalrunde %d: Metrik-Median %.4f' % (rnd, dec[len(dec) // 2][0]))
        rx = st_model_from_h(rx, st_ls_frames(rx.ltot, rx.anchor, good, sample_rate))

    if args.chan_out:
        with open(args.chan_out, 'w') as fo:
            fo.write(
                'DPSKCHAN2 %d %d %d %.6f %d %d\n' % (
                    int(round(sample_rate)), int(FS_HI), rx.anchor, rx.delay,
                    len(rx.g0), len(rx.kn)
                )
            )
            fo.write(' '.join('%.7g' % v for v in rx.g0) + '\n')
            fo.write(' '.join('%.7g' % v for v in rx.kn) + '\n')
        print('  Kanal geschrieben: %s (delay %.2f Samples)' % (args.chan_out, rx.delay))

    eval_frames = frames[:args.max_decoded] if args.max_decoded > 0 else frames
    decoded_final = _rt_decode_batch(rx, eval_frames, L, sample_rate, frame_origin, workers)
    res = [(k, n, d, m) for k, n, m, d, _, _, _, _ in decoded_final]
    print('  decodiert %d/%d (workers=%d)' % (len(res), len(frames), workers))

    if args.out:
        with open(args.out, 'w') as fo:
            for k, n, d, m in res:
                fo.write(
                    '%.1f %d %0*X %.4f\n' % (
                        k * 1e6 / sample_rate, n, n // 4,
                        int(''.join(map(str, d)), 2), m
                    )
                )
    uf = {}
    for k, n, d, m in res:
        u = int(''.join(map(str, d[:5])), 2)
        uf[u] = uf.get(u, 0) + 1
    print('UF-Verteilung: ' + '  '.join('UF%d:%d' % (u, c) for u, c in sorted(uf.items(), key=lambda x: -x[1])[:6]))
    ms = sorted(x[3] for x in res)
    med = ms[len(ms) // 2] if ms else 0
    print('\nAP-Statistik (letzte 24 Bit):')
    rt_stats('alle', [x[2] for x in res])
    rt_stats('beste 50 %', [x[2] for x in res if x[3] <= med])
    print('\nLaufzeit %.1f s' % (time.time() - t0))


def main():
    ap = argparse.ArgumentParser(description="P6-Simulator fuer dump1030")
    sub = ap.add_subparsers(dest='cmd', required=True)

    g = sub.add_parser('gen', help="Testdatei erzeugen")
    g.add_argument('-o', required=True, help="Ausgabedatei (I/Q)")
    g.add_argument('--count', type=int, default=2000, help="Anzahl Frames")
    g.add_argument('--snr', type=float, default=25.0, help="SNR in dB")
    g.add_argument('--bw', type=float, default=1.0, help="Tiefpass-Grenzfrequenz in MHz")
    g.add_argument('--df', type=float, default=-6.4, help="Frequenzversatz in kHz")
    g.add_argument('--len', choices=('56', '112', 'mix'), default='mix')
    g.add_argument('--amp', type=float, default=50.0, help="Signalamplitude (ADC-Einheiten)")
    g.add_argument('--gap', type=float, default=500.0, help="Abstand der Frames in us")
    g.add_argument('--sample-rate', type=float, default=FS)
    g.add_argument('--format', choices=('cu8', 'sc16'), default='cu8',
                   help="Ausgangsformat; SC16 ist little-endian int16 I/Q")
    g.add_argument('--sc16-scale', type=float, default=2048.0, help="SC16-Vollaussteuerung")
    g.add_argument('--drop-ppm', type=float, default=0.0, help="Sample-Drops")
    g.add_argument('--seed', type=int, default=1)

    c = sub.add_parser('compare', help="dump1030-Log gegen Wahrheit pruefen")
    c.add_argument('truth')
    c.add_argument('log')

    p_st = sub.add_parser('selftest', help="MLSE/Viterbi-Machbarkeitstest")
    p_st.add_argument('--filter', choices=['fir', 'iir'], default='fir')
    p_st.add_argument('--bw', type=float, default=1.0)
    p_st.add_argument('--snr', type=float, default=25.0)
    p_st.add_argument('--sample-rate', type=float, default=FS)
    p_st.add_argument('--nbits', type=int, choices=[56, 112], default=56)
    p_st.add_argument('--count', type=int, default=200)
    p_st.add_argument('--mem', type=int, default=4)
    p_st.add_argument('--part', default='all')
    p_st.add_argument('--seed', type=int, default=1)
    p_st.add_argument('--train', type=int, default=100)
    p_st.add_argument('--start', choices=['fir', 'iir'], default='fir')
    p_st.add_argument('--start-bw', type=float, default=1.0)
    p_st.add_argument('--rounds', type=int, default=3)

    p_rt = sub.add_parser('realtest', help="Echte IQ-Datei dekodieren")
    p_rt.add_argument('file')
    p_rt.add_argument('--max-sec', type=float, default=30.0)
    p_rt.add_argument('--sample-rate', type=float, default=FS)
    p_rt.add_argument('--format', choices=('cu8', 'sc16'), default='cu8',
                      help="Eingangsformat; SC16 ist little-endian int16 I/Q")
    p_rt.add_argument('--sc16-scale', type=float, default=2048.0)
    p_rt.add_argument('--thr', type=float, default=10.0)
    p_rt.add_argument('--train', type=int, default=150)
    p_rt.add_argument('--max-decoded', type=int, default=300,
                      help="after this many decoded frames stop (0 = unlimited)")
    p_rt.add_argument('--rounds', type=int, default=3)
    p_rt.add_argument('--mem', type=int, default=7)
    p_rt.add_argument('--start', choices=['fir', 'iir'], default='fir')
    p_rt.add_argument('--start-bw', type=float, default=1.0)
    p_rt.add_argument('--workers', type=int, default=0,
                      help="Anzahl paralleler Worker-Prozesse (0 = alle Kerne)")
    p_rt.add_argument('--out', default='')
    p_rt.add_argument('--chan-out', default='')

    args = ap.parse_args()
    if hasattr(args, 'sample_rate') and args.sample_rate <= 0:
        ap.error('--sample-rate must be positive')
    if hasattr(args, 'sc16_scale') and args.sc16_scale <= 0:
        ap.error('--sc16-scale must be positive')

    if args.cmd == 'realtest':
        cmd_realtest(args)
    elif args.cmd == 'selftest':
        cmd_selftest(args)
    elif args.cmd == 'gen':
        gen(args)
    else:
        compare(args)


if __name__ == '__main__':
    main()