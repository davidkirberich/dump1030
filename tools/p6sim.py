#!/usr/bin/env python3
import math, cmath, random, time
"""p6sim.py - synthetische P6-Uplinks mit bekannten Bits (nur Standardbibliothek)

Erzeugen:
  p6sim.py gen -o sim.bin [--count 2000] [--snr 25] [--bw 1.0] [--df -6.4]
                          [--len 56|112|mix] [--amp 50] [--gap 500]
                          [--drop-ppm 0] [--seed 1]
Vergleichen:
  p6sim.py compare sim.truth sim.log

Kanal: ideales DPSK @20 MS/s -> FIR-Tiefpass (--bw, MHz einseitig)
       -> Dezimierung 8:1 auf 2,5 MS/s (zufällige Abtastphase je Frame)
       -> Frequenzversatz (--df, kHz) -> Rauschen -> 8-Bit-Quantisierung
       -> optional Sample-Drops (Blöcke à 256 I/Q-Paare)
"""
import argparse, cmath, collections, math, random, re, sys

FS_HI = 20e6
DEC = 8
FS = FS_HI / DEC                                # 2,5 MS/s
DROP_BLK = 256                                  # I/Q-Paare je Drop (≈512 Byte)


def hi(t_us):                                   # µs -> Index bei 20 MS/s
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
    """Komplexe Hüllkurve bei 20 MS/s, Index 0 = Vorderflanke P1.
    P1 0..0,8 µs, P2 2,0..2,8 µs, P6 ab 3,5 µs, SPR 1,25 µs nach P6-Beginn,
    Bit b: Phasenumkehr (=1) bei SPR + 0,5 + b*0,25 µs."""
    p6_len = 1.25 + 0.5 + len(bits) * 0.25 + 0.5          # 16,25 / 30,25 µs
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
    h = design_lp(a.bw * 1e6)
    L, half = len(h), len(h) // 2
    sigma = a.amp / 10 ** (a.snr / 20)          # SNR = Amplitude / sigma je Komponente
    slot = int(a.gap * 1e-6 * FS)
    off = slot // 3
    w = 2 * math.pi * a.df * 1e3 / FS
    samples = []                                # komplexe Samples @2,5 MS/s
    truth = []
    g = 0
    for f in range(a.count):
        n = 112 if (a.len == '112' or (a.len == 'mix' and rnd.random() < 0.1)) else 56
        head = [1, 0, 0, 0, 0] if n == 112 else [0, 0, 0, 0, 0]   # UF16 / UF0
        bits = head + [rnd.getrandbits(1) for _ in range(n - 5)]
        x = burst(bits, *(rnd.uniform(0, 2 * math.pi) for _ in range(3)))
        frac = rnd.randrange(DEC)
        y = [complex(rnd.gauss(0, sigma), rnd.gauss(0, sigma)) for _ in range(slot)]
        for k in range(-(L // DEC) - 2, (len(x) + L) // DEC + 2):
            t = k * DEC - frac
            acc = 0j
            for m in range(L):
                j = t + half - m
                if 0 <= j < len(x):
                    acc += h[m] * x[j]
            if acc and 0 <= off + k < slot:
                y[off + k] += a.amp * acc * cmath.exp(1j * w * (g + off + k))
        samples.extend(y)
        hexs = format(int(''.join(map(str, bits)), 2), '0%dX' % (n // 4))
        truth.append((g + off + frac / DEC, n, hexs))
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
            out.append(min(255, max(0, int(round(127.5 + v.real)))))
            out.append(min(255, max(0, int(round(127.5 + v.imag)))))

    with open(a.o, 'wb') as fo:
        fo.write(out)
    tname = re.sub(r'\.bin$', '', a.o) + '.truth'
    with open(tname, 'w') as ft:
        ft.write(f"# snr={a.snr} bw={a.bw} df={a.df} amp={a.amp} "
                 f"drop_ppm={a.drop_ppm} seed={a.seed} fs={FS}\n")
        for idx, n, hx in truth:
            ft.write(f"{idx:.3f} {n} {hx}\n")
    print(f"{a.o}: {g / FS:.2f} s, {a.count} Frames, {dropped} Drop-Blöcke; "
          f"Wahrheit in {tname}")
    if dropped:
        print("  Hinweis: Indizes in .truth sind nicht um Drops korrigiert "
              "(compare ordnet über Reihenfolge + Bits zu).")


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
    print("Fehlerrate nach Muster (Vorgänger->Bit, Wahrheit):")
    for key in ('0->0', '0->1', '1->0', '1->1'):
        n, e = patt[key]
        if n:
            print(f"  {key}: {e / n * 100:6.2f} %  (n={n})")

    print("Fehlerrate je Bitposition (1-basiert, nur >1 %):")
    bad = [(b, e / n * 100) for b, (n, e) in sorted(pos.items()) if n and e / n > 0.01]
    print("  " + ("  ".join(f"{b + 1}:{r:.1f}%" for b, r in bad) if bad else "keine"))

    t56 = [tobits(T[k][2]) for k, db in matched if len(db) == 56]
    d56 = [db for k, db in matched if len(db) == 56]
    print("AP-Feld (56 Bit)          Gewicht   11-Anteil  Lauflänge 1")
    for name, (w, p11, r1) in (("Wahrheit", apstat(t56)),
                               ("dump1030 auf Sim.", apstat(d56)),
                               ("echte Aufnahme", (5.81, 3.1, 87.4))):
        print(f"  {name:22s} {w:6.2f}   {p11:6.1f} %   {r1:6.1f} %")


# ===================== SELFTEST (Stufe 1) =====================
# Eigenständig: eigenes 20-MS/s-Modell, nutzt keine gen-Funktionen.
# Zeitachse: P1-Vorderflanke = 0, P6 ab 3,5 µs, SPR 4,75 µs,
# Datenchip j beginnt bei 5,25 + 0,25*j µs, Guard 2 Chips (0,5 µs).
ST_PAD    = 160      # 8 µs Rand bei 20 MS/s
ST_ANCHOR = ST_PAD + 105   # 5,25 µs
ST_NBMAX  = 112

def st_design_fir(bw, fs=20.0, ntaps=129):
    fc = bw / fs                      # bw = einseitige Grenzfrequenz in MHz
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
    wc = 2 * fs * math.tan(math.pi * bw / fs)      # Prewarping
    secs = []
    for k in range(N // 2):
        p = wc * cmath.exp(1j * math.pi * (2 * k + 1 + N) / (2 * N))
        z = (1 + p / (2 * fs)) / (1 - p / (2 * fs))  # bilinear
        a1, a2 = -2 * z.real, abs(z) ** 2
        g = (1 + a1 + a2) / 4                         # DC-Verstärkung 1
        secs.append((g, 2 * g, g, a1, a2))
    return secs

def st_make_filter(kind, bw):
    """liefert (f, delay): f filtert reelle Liste bei 20 MS/s, delay in Samples"""
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
        self.g0 = f(x)                                # Antwort auf Einheitschip j=0
        x = [0.0] * self.ltot
        for i in range(ST_PAD, ST_PAD + 16):        x[i] = 1.0   # P1
        for i in range(ST_PAD + 40, ST_PAD + 56):   x[i] = 1.0   # P2
        for i in range(self.anchor - 35, self.anchor - 10): x[i] = 1.0   # P6 vor SPR
        for i in range(self.anchor - 10, self.anchor):      x[i] = -1.0  # SPR..Bit0
        self.kn = f(x)                                # bekannter Anteil
        self.cache = {}

    def g(self, m, t):
        """Antwort von Datenchip m (Wert +1) zur Zeit t (20-MS/s-Index, gebrochen)"""
        return st_interp(self.g0, t - 5 * m)

    def knv(self, t):
        return st_interp(self.kn, t)

    def support(self):
        """Indexbereich von g0 mit |g0| > 1e-4"""
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
    """DPSK: Bit 1 = Phasenumkehr am Chipbeginn; c[-1] = -1 (nach SPR); +2 Guard-Chips"""
    c, chips = -1, []
    for b in list(bits) + [0, 0]:
        if b:
            c = -c
        chips.append(c)
    return chips

def st_frame_samples(model, chips, off, A, phi, snr, rng):
    """Empfangssamples bei 2,5 MS/s: t_k = off + 8k (20-MS/s-Index). df = 0."""
    lo, hi = model.support()
    rot = A * cmath.exp(1j * phi)
    sig = 1.0 / snr
    out, t = [], off
    while t < model.ltot - 1:
        v = model.knv(t)
        j0 = max(0, math.ceil((t - hi) / 5))
        j1 = min(len(chips) - 1, math.floor((t - lo) / 5))
        for j in range(j0, j1 + 1):
            v += chips[j] * model.g(j, t)
        out.append(rot * v + complex(rng.gauss(0, sig), rng.gauss(0, sig)))
        t += 8
    return out

def st_old(r, off, delay, anchor, n):
    """alte Einzelentscheidung: Catmull-Rom in Chipmitten, Vorzeichen von Re(z_b * conj(z_b-1))"""
    def at(t):
        u = (t - off) / 8
        i = math.floor(u)
        a = u - i
        if i < 1 or i + 2 >= len(r):
            return 0j
        p0, p1, p2, p3 = r[i - 1], r[i], r[i + 1], r[i + 2]
        return p1 + 0.5 * a * (p2 - p0 + a * (2 * p0 - 5 * p1 + 4 * p2 - p3
                                              + a * (3 * (p1 - p2) + p3 - p0)))
    prev = at(anchor + delay - 2.5)          # Mitte des Chips vor Bit 0
    bits = []
    for b in range(n):
        cur = at(anchor + delay + 5 * b + 2.5)
        bits.append(1 if (cur * prev.conjugate()).real < 0 else 0)
        prev = cur
    return bits

def st_prepare(rx, r, off, phi, A, L, N):
    """pro Sample: (Schritt, Fensteranfang w, kn, [B_w..B_w+L-1], derotiertes z)"""
    der = cmath.exp(-1j * phi) / A
    out = []
    for k, s in enumerate(r):
        t = off + 8 * k
        jc = math.floor((t - rx.delay - rx.anchor) / 5)
        w = jc - (L - 1) // 2
        e = w + L - 1
        if e < 0 or w > N - 1:
            continue
        Bs = [rx.g(w + i, t) if w + i >= 0 else 0.0 for i in range(L)]
        out.append((min(e, N - 1), w, rx.knv(t), Bs, s * der))
    return out

def st_viterbi(samples, L, n, N, retm=False):
    """MLSE über Chips, Gedächtnis L Chips -> 2^(L-1) Zustände.
       Chips >= n sind Guard (keine Umkehr), Chips >= N gelten als 0."""
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

def st_eval(tx, rx, n, Ls, count, snr, dt, dphi, seed, old=True):
    """dt: Timingfehler des Empfängers in 20-MS/s-Samples (1 = 0,05 µs), dphi in Grad"""
    rng = random.Random(seed)
    res = {}
    keys = (['old'] if old else []) + list(Ls)
    for k in keys:
        res[k] = [0, 0]                       # Bitfehler, fehlerfreie Frames
    N = n + 2
    for _ in range(count):
        bits = [rng.randint(0, 1) for _ in range(n)]
        chips = st_bits2chips(bits)
        off = rng.uniform(0, 8)
        phi = rng.uniform(-math.pi, math.pi)
        r = st_frame_samples(tx, chips, off, 1.0, phi, snr, rng)
        decs = {}
        if old:
            decs['old'] = st_old(r, off + dt, rx.delay, rx.anchor, n)
        for L in Ls:
            smp = st_prepare(rx, r, off + dt, phi + math.radians(dphi), 1.0, L, N)
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
        res = st_eval(tx, tx, n, [args.mem], cnt, snr, 0.0, 0.0, args.seed)
        st_print('ideal', res, cnt, n)
        ok = 100.0 * res[args.mem][1] / cnt
        print('  -> %s (Kriterium: >= 90 %% fehlerfrei)' % ('BESTANDEN' if ok >= 90 else 'NICHT bestanden'))

    if 'b' in parts:
        print('\n[1b] Gedächtnis (ideal)')
        res = st_eval(tx, tx, n, [3, 4, 5, 6, 7], cnt, snr, 0.0, 0.0, args.seed, old=False)
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
            res = st_eval(tx, rx, n, [args.mem], cnt, snr, dt, dp, args.seed, old=(fb != 1.0))
            st_print(label, res, cnt, n)
    if 'd' in parts:
        print('\n[2a] Kanal geschaetzt aus %d Trainingsframes (Bits, frac, Phase bekannt)' % args.train)
        h = st_estimate_channel(tx, args.train, n, snr, args.seed + 1000)
        ht = st_true_h(tx)
        err = math.sqrt(sum((a - b) ** 2 for a, b in zip(h, ht)) / sum(b * b for b in ht))
        print('  rel. Kanalfehler %.4f   delay geschaetzt %.2f Samples' % (err, sum((i + ST_LMIN) * v for i, v in enumerate(h)) / sum(h)))
        rx = st_model_from_h(tx, h)
        res = st_eval(tx, rx, n, [args.mem], cnt, snr, 0.0, 0.0, args.seed, old=False)
        st_print('geschaetzt', res, cnt, n)
    if 'e' in parts:
        print('\n[2b] Blindtest (Bits, frac, Phase, Amplitude, Kanal unbekannt), Start %s bw=%.2f'
              % (args.start, args.start_bw))
        rng = random.Random(args.seed + 2000)
        frames = []
        for _ in range(max(cnt, args.train)):
            bits = [rng.randint(0, 1) for _ in range(n)]
            off = rng.uniform(0, 8)
            phi = rng.uniform(-math.pi, math.pi)
            frames.append((bits, st_frame_samples(tx, st_bits2chips(bits), off, 1.0, phi, snr, rng)))
        rx = st_model(args.start, args.start_bw)
        N = n + 2
        for rnd in range(args.rounds + 1):
            last = rnd == args.rounds
            use = frames if last else frames[:args.train]
            dec, be, ok = [], 0, 0
            for bits, r in use:
                m, d, c, phi, A = st_blind_decode(rx, r, n, N, args.mem)
                e = sum(1 for a, b in zip(bits, d) if a != b)
                be += e
                ok += (e == 0)
                dec.append((r, c, phi, A, st_bits2chips(d)))
            print('  Runde %d  %4d Frames  BER %7.3f %%  fehlerfrei %6.1f %%'
                  % (rnd, len(use), 100.0 * be / (len(use) * n), 100.0 * ok / len(use)))
            if not last:
                rx = st_model_from_h(tx, st_ls_frames(rx.ltot, rx.anchor, dec))
    print('\nLaufzeit %.1f s' % (time.time() - t0))
ST_LMIN, ST_LMAX = -20, 59      # Kanal-Taps bei 20 MS/s: -1,0 .. +2,95 µs

def st_input(ltot, anchor, chips):
    """bekanntes 20-MS/s-Sendesignal: P1, P2, P6, SPR, Datenchips"""
    x = [0.0] * ltot
    for i in range(ST_PAD, ST_PAD + 16):      x[i] = 1.0
    for i in range(ST_PAD + 40, ST_PAD + 56): x[i] = 1.0
    for i in range(anchor - 35, anchor - 10): x[i] = 1.0
    for i in range(anchor - 10, anchor):      x[i] = -1.0
    for j, c in enumerate(chips):
        for i in range(anchor + 5 * j, min(ltot, anchor + 5 * j + 5)):
            x[i] = float(c)
    return x

def st_solve(M, v):
    n = len(v)
    A = [M[i][:] + [v[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(A[r][c]))
        A[c], A[p] = A[p], A[c]
        piv = A[c][c]
        for r in range(c + 1, n):
            f = A[r][c] / piv
            if f:
                for k in range(c, n + 1):
                    A[r][k] -= f * A[c][k]
    h = [0.0] * n
    for c in range(n - 1, -1, -1):
        h[c] = (A[c][n] - sum(A[c][k] * h[k] for k in range(c + 1, n))) / A[c][c]
    return h

def st_estimate_channel(tx, ntrain, n, snr, seed):
    """LS-Schätzung der reellen Kanalantwort h[l] aus ntrain Frames"""
    rng = random.Random(seed)
    L = ST_LMAX - ST_LMIN + 1
    M = [[0.0] * L for _ in range(L)]
    v = [0.0] * L
    for _ in range(ntrain):
        bits = [rng.randint(0, 1) for _ in range(n)]
        chips = st_bits2chips(bits)
        off = rng.uniform(0, 8)
        phi = rng.uniform(-math.pi, math.pi)
        r = st_frame_samples(tx, chips, off, 1.0, phi, snr, rng)
        x = st_input(tx.ltot, tx.anchor, chips)
        nx = len(x)
        der = cmath.exp(-1j * phi)
        for k, s in enumerate(r):
            t = off + 8 * k
            i = int(t)
            a = t - i
            reg = []
            for l in range(ST_LMIN, ST_LMAX + 1):
                m0, m1 = i - l, i + 1 - l
                x0 = x[m0] if 0 <= m0 < nx else 0.0
                x1 = x[m1] if 0 <= m1 < nx else 0.0
                reg.append((1 - a) * x0 + a * x1)
            if not any(reg):
                continue
            y = (s * der).real
            for p in range(L):
                rp = reg[p]
                if rp == 0.0:
                    continue
                v[p] += rp * y
                row = M[p]
                for q in range(p, L):
                    row[q] += rp * reg[q]
    for p in range(L):
        M[p][p] += 1e-6
        for q in range(p):
            M[p][q] = M[q][p]
    return st_solve(M, v)

def st_true_h(tx):
    f, _ = st_make_filter(tx.kind, tx.bw)
    P = 200
    x = [0.0] * 600
    x[P] = 1.0
    y = f(x)
    return [y[P + l] for l in range(ST_LMIN, ST_LMAX + 1)]

def st_model_from_h(tx, h):
    """Empfängermodell aus geschätzter Kanalantwort (gleiche Schnittstelle wie StModel)"""
    rx = object.__new__(StModel)
    rx.kind, rx.bw, rx.ltot, rx.anchor, rx.cache = 'est', tx.bw, tx.ltot, tx.anchor, {}
    def conv(x):
        y = [0.0] * len(x)
        for m, xv in enumerate(x):
            if xv:
                for idx, hv in enumerate(h):
                    k = m + idx + ST_LMIN
                    if 0 <= k < len(y):
                        y[k] += hv * xv
        return y
    x = [0.0] * rx.ltot
    for i in range(rx.anchor, rx.anchor + 5):
        x[i] = 1.0
    rx.g0 = conv(x)
    rx.kn = conv(st_input(rx.ltot, rx.anchor, []))
    rx.delay = sum((idx + ST_LMIN) * hv for idx, hv in enumerate(h)) / sum(h)
    return rx

def st_pre_fit(rx, r, c):
    """Präambel-Fit bei Hypothese c: liefert (num, den) oder None"""
    lim = rx.anchor + rx.delay - 10          # bis 0,5 µs vor den Daten
    num, den = 0j, 0.0
    for k, s in enumerate(r):
        t = c + 8 * k
        if t >= lim:
            break
        kv = rx.knv(t)
        num += s * kv
        den += kv * kv
    return (num, den) if den > 0 else None

def st_blind_decode(rx, r, n, N, L):
    best_c, best_s = None, -1.0
    for c in range(-24, 33):                 # Grobtiming in 0,05-µs-Schritten
        f = st_pre_fit(rx, r, c)
        if f and abs(f[0]) ** 2 / f[1] > best_s:
            best_c, best_s = c, abs(f[0]) ** 2 / f[1]
    out = None
    for cc in (best_c - 1, best_c, best_c + 1):
        f = st_pre_fit(rx, r, cc)
        if not f:
            continue
        num, den = f
        phi, A = cmath.phase(num), abs(num) / den
        smp = st_prepare(rx, r, cc, phi, A, L, N)
        bits, m = st_viterbi(smp, L, n, N, retm=True)
        m /= max(1, len(smp))
        if out is None or m < out[0]:
            out = (m, bits, cc, phi, A)
    return out

def st_ls_frames(ltot, anchor, frames):
    """LS-Kanal aus entschiedenen Frames: (r, off, phi, A, chips)"""
    L = ST_LMAX - ST_LMIN + 1
    M = [[0.0] * L for _ in range(L)]
    v = [0.0] * L
    for r, off, phi, A, chips in frames:
        x = st_input(ltot, anchor, chips)
        nx = len(x)
        der = cmath.exp(-1j * phi) / A
        for k, s in enumerate(r):
            t = off + 8 * k
            i = math.floor(t)
            a = t - i
            reg = []
            for l in range(ST_LMIN, ST_LMAX + 1):
                m0, m1 = i - l, i + 1 - l
                x0 = x[m0] if 0 <= m0 < nx else 0.0
                x1 = x[m1] if 0 <= m1 < nx else 0.0
                reg.append((1 - a) * x0 + a * x1)
            if not any(reg):
                continue
            y = (s * der).real
            for p in range(L):
                rp = reg[p]
                if rp == 0.0:
                    continue
                v[p] += rp * y
                row = M[p]
                for q in range(p, L):
                    row[q] += rp * reg[q]
    for p in range(L):
        M[p][p] += 1e-6
        for q in range(p):
            M[p][q] = M[q][p]
    return st_solve(M, v)

# ===================== REALTEST (Stufe 3) =====================
def rt_detect(data, nmax, thr_fac):
    """P1/P2/P6-Detektor auf uint8-I/Q, liefert (Liste der P1-Samples, Schwelle)"""
    sq = [(i - 127.5) ** 2 for i in range(256)]
    ns = min(len(data) // 2, nmax)
    probe = sorted(sq[data[2 * j]] + sq[data[2 * j + 1]] for j in range(0, min(ns, 1000000), 97))
    thr = probe[len(probe) // 2] * thr_fac
    hits, CH, k0, nxt = [], 1000000, 0, 0
    while k0 < ns - 100:
        k1 = min(ns, k0 + CH + 100)
        m = [sq[data[2 * j]] + sq[data[2 * j + 1]] for j in range(k0, k1)]
        for i in range(1, len(m) - 90):
            if m[i] <= thr or k0 + i < nxt:
                continue
            p = m[i]
            if (m[i - 1] < thr and m[i + 1] > thr and m[i + 3] < p / 4 and m[i + 4] < p / 4
                    and m[i + 5] > thr and m[i + 6] > thr and m[i + 8] < p / 4
                    and all(m[i + q] > thr / 2 for q in range(10, 20))):
                hits.append(k0 + i)
                nxt = k0 + i + 85          # 34 µs Totzeit
        k0 += CH
        print('  scan %5.1f s  %d Kandidaten' % (k0 / 2.5e6, len(hits)), end='\r')
    print()
    return hits, thr

def rt_frame(data, k, thr):
    """Fenster ab 8 µs vor P1, Länge per Energie bei 23-27 µs, df per Frame"""
    s0 = k - 20
    r = [complex(data[2 * j] - 127.5, data[2 * j + 1] - 127.5) for j in range(s0, s0 + 125)]
    e = sum(abs(r[20 + q]) ** 2 for q in range(57, 68)) / 11
    n = 112 if e > thr else 56
    end = 20 + (49 if n == 56 else 84)
    acc = 0j
    for j in range(20 + 9, end):
        d = r[j] * r[j - 1].conjugate()
        acc += d * d
    w = cmath.phase(acc) / 2 if acc != 0 else 0.0
    r = [z * cmath.exp(-1j * w * j) for j, z in enumerate(r)]
    return r, n, w * 2.5e6 / (2 * math.pi)

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

def cmd_realtest(args):
    t0 = time.time()
    data = open(args.file, 'rb').read()
    if not data:
        raise SystemExit('Keine IQ-Daten in der Eingabedatei; Abbruch.')
    hits, thr = rt_detect(data, int(args.max_sec * 2.5e6), args.thr)
    frames = []
    for k in hits:
        if k >= 20 and 2 * (k + 110) <= len(data):
            r, n, df = rt_frame(data, k, thr)
            frames.append((k, r, n, df))
    dfs = sorted(f[3] for f in frames)
    print('Frames %d (112 Bit: %d)  df-Median %.2f kHz'
          % (len(frames), sum(1 for f in frames if f[2] == 112), dfs[len(dfs) // 2] / 1000 if dfs else 0))
    if not frames:
        raise SystemExit(
            'Keine gueltigen P6-Kandidaten; Kanaldatei wurde nicht geschrieben.'
        )
    rx = st_model(args.start, args.start_bw)
    L = args.mem
    for rnd in range(args.rounds):
        dec = []
        for k, r, n, df in frames[:args.train]:
            m, d, c, phi, A = st_blind_decode(rx, r, n, n + 2, L)
            dec.append((m, (r, c, phi, A, st_bits2chips(d))))
        dec.sort(key=lambda x: x[0])
        good = [x[1] for x in dec[:max(10, len(dec) // 2)]]
        print('  Kanalrunde %d: Metrik-Median %.4f' % (rnd, dec[len(dec) // 2][0]))
        rx = st_model_from_h(rx, st_ls_frames(rx.ltot, rx.anchor, good))
    if args.chan_out:
        with open(args.chan_out, 'w') as fo:
            fo.write('DPSKCHAN1 %d %.6f %d %d\n' % (rx.anchor, rx.delay, len(rx.g0), len(rx.kn)))
            fo.write(' '.join('%.7g' % v for v in rx.g0) + '\n')
            fo.write(' '.join('%.7g' % v for v in rx.kn) + '\n')
        print('  Kanal geschrieben: %s (delay %.2f Samples)' % (args.chan_out, rx.delay))
    res = []
    for i, (k, r, n, df) in enumerate(frames):
        m, d, c, phi, A = st_blind_decode(rx, r, n, n + 2, L)
        res.append((k, n, d, m))
        if i % 50 == 0:
            print('  decodiert %d/%d' % (i, len(frames)), end='\r')
    print()
    if args.out:
        with open(args.out, 'w') as fo:
            for k, n, d, m in res:
                fo.write('%.1f %d %0*X %.4f\n' % (k / 2.5, n, n // 4, int(''.join(map(str, d)), 2), m))
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
# =================== Ende REALTEST ===================

# =================== Ende SELFTEST ===================

ap = argparse.ArgumentParser(description="P6-Simulator für dump1030")
sub = ap.add_subparsers(dest='cmd', required=True)

g = sub.add_parser('gen', help="Testdatei erzeugen")
g.add_argument('-o', required=True, help="Ausgabedatei (.bin, uint8 I/Q)")
g.add_argument('--count', type=int, default=2000, help="Anzahl Frames")
g.add_argument('--snr', type=float, default=25.0, help="SNR in dB")
g.add_argument('--bw', type=float, default=1.0, help="Tiefpass-Grenzfrequenz in MHz")
g.add_argument('--df', type=float, default=-6.4, help="Frequenzversatz in kHz")
g.add_argument('--len', choices=('56', '112', 'mix'), default='mix')
g.add_argument('--amp', type=float, default=50.0, help="Signalamplitude (ADC-Einheiten)")
g.add_argument('--gap', type=float, default=500.0, help="Abstand der Frames in µs")
g.add_argument('--drop-ppm', type=float, default=0.0,
               help="Sample-Drops (z.B. 1850 wie gemessen)")
g.add_argument('--seed', type=int, default=1)

c = sub.add_parser('compare', help="dump1030-Log gegen Wahrheit prüfen")
c.add_argument('truth')
c.add_argument('log')

p_st = sub.add_parser('selftest', help="MLSE/Viterbi-Machbarkeitstest")
p_st.add_argument('--filter', choices=['fir', 'iir'], default='fir')
p_st.add_argument('--bw', type=float, default=1.0)
p_st.add_argument('--snr', type=float, default=25.0)
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
p_rt.add_argument('--thr', type=float, default=10.0)
p_rt.add_argument('--train', type=int, default=150)
p_rt.add_argument('--rounds', type=int, default=3)
p_rt.add_argument('--mem', type=int, default=7)
p_rt.add_argument('--start', choices=['fir', 'iir'], default='fir')
p_rt.add_argument('--start-bw', type=float, default=1.0)
p_rt.add_argument('--out', default='')
p_rt.add_argument('--chan-out', default='')

args = ap.parse_args()
cmd_realtest(args) if args.cmd == 'realtest' else (cmd_selftest(args) if args.cmd == 'selftest' else (gen(args) if args.cmd == 'gen' else compare(args)))
