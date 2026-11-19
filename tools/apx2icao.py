#!/usr/bin/env python3
"""
apx2icao.py - Zieladressen aus dump1030-Uplink-Frames (UF0/UF16/...) ableiten.

Ablauf:
1. Logs parsen ("[P6] ... Q=.. HEX=..")
2. Cluster bilden: Datenbits exakt, AP-Feld mit Hamming-Toleranz,
     Mehrheitsentscheid pro Bit (gewichtet mit Q)
3. X = AP xor CRC(Daten) pro Cluster, gleiche X = gleiches Ziel
4. Parity-Hypothesen (lineare, umkehrbare GF(2)-Abbildungen) testen:
     Adresse A = f^-1(X). Bewertung: ICAO-Blöcke, optional Referenztreffer
5. Bericht: beste Hypothese, Zieltabelle, Fehlerstatistik je Bitposition

Aufruf:
  apx2icao.py cap1.log [cap2.log ...] [--ref aircraft.json|verzeichnis]
              [--blocks blocks.csv] [--min-q 180] [--ap-tol 2] [--top 30]
"""
import argparse, json, os, re, statistics, sys
from collections import defaultdict

GEN = 0xFFF409          # Mode-S-Generator ohne x^24
GEN_FULL = 0x1FFF409    # mit x^24

# ---------------------------------------------------------------- Parsing
RE_MODE = re.compile(r"^Mode S\s+t=([\\d.]+) us")
RE_P6 = re.compile(r"\[P6\]\s+(\d+) Bit .*?Q=(\d+) HEX=([0-9A-Fa-f]+)")

def parse_logs(paths, min_q):
    frames, t_last = [], None
    for p in paths:
        with open(p, errors="replace") as fh:
            for line in fh:
                m = RE_MODE.search(line)
                if m:
                    t_last = float(m.group(1)) / 1e6
                    continue
                m = RE_P6.search(line)
                if not m:
                    continue
                nbits, q, hx = int(m.group(1)), int(m.group(2)), m.group(3)
                if len(hx) * 4 != nbits or q < min_q:
                    continue
                frames.append({"t": t_last, "nbits": nbits, "q": q,
                               "val": int(hx, 16), "src": os.path.basename(p)})
    return frames

# ---------------------------------------------------------------- CRC / Felder
def crc(data, nbits):
    """Mode-S-CRC über nbits (MSB zuerst), identisch zu modesCrc() in dump1030."""
    r = 0
    for k in range(nbits - 1, -1, -1):
        bit = (data >> k) & 1
        top = (r >> 23) & 1
        r = (r << 1) & 0xFFFFFF
        if top ^ bit:
            r ^= GEN
    return r

def split(val, nbits):
    nd = nbits - 24
    return val >> 24, val & 0xFFFFFF, nd     # Daten, AP, Datenbits

def uf_of(val, nbits):
    uf = (val >> (nbits - 5)) & 0x1F
    return 24 if (uf >> 3) == 3 else uf

def popcount(x):
    return bin(x).count("1")

# ---------------------------------------------------------------- Clustering
def cluster(frames, ap_tol):
    """Gruppiert nach (Länge, Datenbits exakt); innerhalb AP mit Hamming <= ap_tol.
    Rückgabe: Cluster mit Mehrheits-AP und Fehlerstatistik je Bitposition."""
    groups = defaultdict(list)
    for f in frames:
        data, ap, nd = split(f["val"], f["nbits"])
        groups[(f["nbits"], data)].append((f, ap))

    clusters, bit_err, bit_tot = [], defaultdict(int), defaultdict(int)
    for (nbits, data), members in groups.items():
        members.sort(key=lambda m: -m[0]["q"])
        open_cl = []                                  # [center, [(f, ap)]]
        for f, ap in members:
            for cl in open_cl:
                if popcount(cl[0] ^ ap) <= ap_tol:
                    cl[1].append((f, ap))
                    break
            else:
                open_cl.append([ap, [(f, ap)]])
        for _, mem in open_cl:
            # gewichteter Mehrheitsentscheid pro AP-Bit
            maj = 0
            for b in range(24):
                w1 = sum(f["q"] for f, ap in mem if (ap >> b) & 1)
                w0 = sum(f["q"] for f, ap in mem if not (ap >> b) & 1)
                if w1 > w0:
                    maj |= 1 << b
            if len(mem) >= 2:                         # Fehlerstatistik nur mit Redundanz
                for f, ap in mem:
                    diff = ap ^ maj
                    for b in range(24):
                        pos = nbits - b               # 1-basierte Bitposition
                        bit_tot[pos] += 1
                        if (diff >> b) & 1:
                            bit_err[pos] += 1
            nd = nbits - 24
            clusters.append({
                "nbits": nbits, "data": data, "ap": maj, "n": len(mem),
                "uf": uf_of((data << 24) | maj, nbits),
                "x": maj ^ crc(data, nd),
                "times": sorted(f["t"] for f, _ in mem if f["t"] is not None),
                "qmean": statistics.mean(f["q"] for f, _ in mem),
            })
    return clusters, bit_err, bit_tot

# ---------------------------------------------------------------- Hypothesen
def rev24(a):
    return int(f"{a:024b}"[::-1], 2)

def mulmod_x24(a):          # A * x^24 mod G
    r = a
    for _ in range(24):
        r <<= 1
        if r & 0x1000000:
            r ^= GEN_FULL
    return r & 0xFFFFFF

def polymul(a, b):
    r = 0
    while b:
        if b & 1:
            r ^= a
        a <<= 1
        b >>= 1
    return r

def mul_low(a):  return polymul(a, GEN_FULL) & 0xFFFFFF
def mul_high(a): return (polymul(a, GEN_FULL) >> 24) & 0xFFFFFF

# Vorwärtsabbildungen f: Adresse A -> Überlagerungswert X (= AP xor CRC).
HYPOTHESES = {
    "direkt (X = A)":              lambda a: a,
    "A*x^24 mod G":                mulmod_x24,
    "A*G low24":                   mul_low,
    "A*G high24":                  mul_high,
    "rev(A)":                      rev24,
    "rev(A)*x^24 mod G":           lambda a: mulmod_x24(rev24(a)),
    "rev(A*x^24 mod G)":           lambda a: rev24(mulmod_x24(a)),
    "rev(A)*G low24":              lambda a: mul_low(rev24(a)),
}

def invert_linear(f):
    """Invertiert eine lineare GF(2)-Abbildung auf 24 Bit (Gauß-Elimination)."""
    cols = [f(1 << i) for i in range(24)]
    for a, b in ((0x123456, 0xABCDEF), (0xFFFFFF, 0x0F0F0F)):
        fa = 0
        for i in range(24):
            if (a >> i) & 1: fa ^= cols[i]
        if fa != f(a):
            return None
    rows = []
    for o in range(24):
        r = 0
        for i in range(24):
            if (cols[i] >> o) & 1:
                r |= 1 << i
        rows.append([r, 1 << o])
    for c in range(24):
        piv = next((r for r in range(c, 24) if (rows[r][0] >> c) & 1), None)
        if piv is None:
            return None
        rows[c], rows[piv] = rows[piv], rows[c]
        for r in range(24):
            if r != c and (rows[r][0] >> c) & 1:
                rows[r][0] ^= rows[c][0]
                rows[r][1] ^= rows[c][1]
    def inv(x):
        a = 0
        for c in range(24):
            if popcount(rows[c][1] & x) & 1:
                a |= 1 << c
        return a
    return inv

# ---------------------------------------------------------------- ICAO-Blöcke
DEFAULT_BLOCKS = [
    (0x300000, 0x33FFFF, "Italien"),     (0x340000, 0x37FFFF, "Spanien"),
    (0x380000, 0x3BFFFF, "Frankreich"),  (0x3C0000, 0x3FFFFF, "Deutschland"),
    (0x400000, 0x43FFFF, "Ver. Königreich"), (0x440000, 0x447FFF, "Österreich"),
    (0x448000, 0x44FFFF, "Belgien"),     (0x450000, 0x457FFF, "Bulgarien"),
    (0x458000, 0x45FFFF, "Dänemark"),    (0x460000, 0x467FFF, "Finnland"),
    (0x468000, 0x46FFFF, "Griechenland"),(0x470000, 0x477FFF, "Ungarn"),
    (0x478000, 0x47FFFF, "Norwegen"),    (0x480000, 0x487FFF, "Niederlande"),
    (0x488000, 0x48FFFF, "Polen"),       (0x490000, 0x497FFF, "Portugal"),
    (0x498000, 0x49FFFF, "Tschechien"),  (0x4A0000, 0x4A7FFF, "Rumänien"),
    (0x4A8000, 0x4AFFFF, "Schweden"),    (0x4B0000, 0x4B7FFF, "Schweiz"),
    (0x4B8000, 0x4BFFFF, "Türkei"),      (0x4CA000, 0x4CAFFF, "Irland"),
    (0x500000, 0x5FFFFF, "Europa/GUS (div.)"),
    (0x100000, 0x1FFFFF, "Russland"),    (0x700000, 0x7FFFFF, "Asien (div.)"),
    (0x800000, 0x8FFFFF, "Asien/Nahost (div.)"),
    (0xA00000, 0xAFFFFF, "USA"),         (0xC00000, 0xC3FFFF, "Kanada"),
    (0x0C0000, 0x0FFFFF, "Afrika/Amerika (div.)"),
    (0xE00000, 0xE3FFFF, "Südamerika (div.)"),
]

def load_blocks(path):
    if not path:
        return DEFAULT_BLOCKS
    out = []
    with open(path) as fh:
        for line in fh:
            parts = line.strip().split(",")
            if len(parts) >= 2 and not line.startswith("#"):
                out.append((int(parts[0], 16), int(parts[1], 16),
                            parts[2] if len(parts) > 2 else "?"))
    return out

def block_of(a, blocks):
    for s, e, name in blocks:
        if s <= a <= e:
            return name
    return None

def block_coverage(blocks):
    return sum(e - s + 1 for s, e, _ in blocks) / float(1 << 24)

# ---------------------------------------------------------------- Referenz
def load_ref(path):
    refs, files = set(), []
    if os.path.isdir(path):
        files = [os.path.join(path, f) for f in os.listdir(path) if f.endswith(".json")]
    elif path:
        files = [path]
    def walk(o):
        if isinstance(o, dict):
            for k, v in o.items():
                if k in ("hex", "icao24") and isinstance(v, str):
                    h = v.strip().lstrip("~").lower()
                    if re.fullmatch(r"[0-9a-f]{6}", h):
                        refs.add(int(h, 16))
                else:
                    walk(v)
        elif isinstance(o, list):
            if o and isinstance(o[0], str) and re.fullmatch(r"[0-9a-fA-F]{6}", o[0]):
                refs.add(int(o[0], 16))
            for v in o:
                walk(v)
    for f in files:
        try:
            with open(f) as fh:
                walk(json.load(fh))
        except (OSError, json.JSONDecodeError) as e:
            print(f"Warnung: {f}: {e}", file=sys.stderr)
    return refs

# ---------------------------------------------------------------- Bericht
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--ref", help="aircraft.json, Verzeichnis davon oder OpenSky-JSON")
    ap.add_argument("--blocks", help="CSV mit ICAO-Blöcken (start,end,land)")
    ap.add_argument("--min-q", type=int, default=180, help="Mindestqualität (Default 180)")
    ap.add_argument("--ap-tol", type=int, default=2, help="Hamming-Toleranz im AP-Feld")
    ap.add_argument("--top", type=int, default=30, help="Zeilen in der Zieltabelle")
    ap.add_argument("--uf", default="0,16", help="auszuwertende UFs (Default 0,16)")
    args = ap.parse_args()

    ufs = {int(u) for u in args.uf.split(",")}
    frames = [f for f in parse_logs(args.logs, args.min_q) if uf_of(f["val"], f["nbits"]) in ufs]
    if not frames:
        sys.exit("Keine passenden Frames gefunden (Logformat/--min-q/--uf prüfen).")

    clusters, bit_err, bit_tot = cluster(frames, args.ap_tol)
    blocks = load_blocks(args.blocks)
    refs = load_ref(args.ref) if args.ref else set()

    targets = defaultdict(lambda: {"n": 0, "clusters": 0, "times": [], "ufs": set()})
    for c in clusters:
        t = targets[c["x"]]
        t["n"] += c["n"]; t["clusters"] += 1
        t["times"] += c["times"]; t["ufs"].add(c["uf"])

    n_rep = sum(1 for c in clusters if c["n"] >= 2)
    print("=" * 72)
    print(f"Frames (Q>={args.min_q}, UF {sorted(ufs)}): {len(frames)}")
    print(f"Cluster: {len(clusters)} (davon mit Wiederholung: {n_rep})")
    print(f"Unterschiedliche Überlagerungswerte X (= Ziele): {len(targets)}")
    print("=" * 72)

    print("\n[1] Decoder-Konsistenz (gilt für ALLE Hypothesen gleich)")
    gaps = []
    for x, t in targets.items():
        ts = sorted(t["times"])
        gaps += [b - a for a, b in zip(ts, ts[1:]) if 0.5 < b - a < 2.0]
    multi = sum(1 for t in targets.values() if t["n"] >= 2)
    print(f"  Ziele mit >=2 Frames: {multi} von {len(targets)}")
    if gaps:
        print(f"  Abstände 0,5-2 s: n={len(gaps)}, Median {statistics.median(gaps):.3f} s "
              f"(TCAS-Tracking erwartet ~1 s)")

    weights = {x: t["n"] for x, t in targets.items()}
    total_w = sum(weights.values())
    cov = block_coverage(blocks)
    print(f"\n[2] Parity-Hypothesen (Blockabdeckung der Tabelle: {cov*100:.1f} % "
          f"des Adressraums = Zufallserwartung)")
    print(f"  {'Hypothese':24s} {'in Block':>9s} {'gewichtet':>10s} {'Ref-Treffer':>12s}")
    results = []
    for name, f in HYPOTHESES.items():
        inv = invert_linear(f)
        if inv is None:
            print(f"  {name:24s}  nicht umkehrbar/linear, übersprungen")
            continue
        addrs = {x: inv(x) for x in targets}
        in_blk = sum(1 for a in addrs.values() if block_of(a, blocks))
        w_blk = sum(weights[x] for x, a in addrs.items() if block_of(a, blocks))
        hits = sum(1 for a in addrs.values() if a in refs) if refs else None
        frac, wfrac = in_blk / len(addrs), w_blk / total_w
        results.append((name, inv, frac, wfrac, hits))
        hs = f"{hits:>12d}" if hits is not None else f"{'-':>12s}"
        print(f"  {name:24s} {frac*100:8.1f}% {wfrac*100:9.1f}% {hs}")

    if not results:
        sys.exit("Keine auswertbare Hypothese.")
    key = (lambda r: (r[4] or 0, r[3])) if refs else (lambda r: r[3])
    results.sort(key=key, reverse=True)
    best = results[0]
    second = results[1] if len(results) > 1 else None
    print(f"\n  Beste Hypothese: {best[0]}")
    if not refs:
        margin = best[3] - (second[3] if second else 0)
        verdict = ("deutlich" if best[3] > 2 * cov and margin > 0.2 else
                   "schwach – mehr Daten oder --ref nötig")
        print(f"  Abstand zur Zufallserwartung ({cov*100:.1f} %): {verdict}")

    inv = best[1]
    print(f"\n[3] Ziele (Hypothese: {best[0]}), sortiert nach Häufigkeit")
    print(f"  {'X':>6s}  {'Adresse':>7s}  {'Frames':>6s} {'Cl':>3s}  {'UF':6s} "
          f"{'Block':18s} {'Ref':3s} {'erste..letzte s':>16s}")
    for x, t in sorted(targets.items(), key=lambda kv: -kv[1]["n"])[:args.top]:
        a = inv(x)
        ts = t["times"]
        span = f"{min(ts):7.1f}..{max(ts):7.1f}" if ts else "-"
        ref = "ja" if a in refs else ("-" if refs else "")
        print(f"  {x:06X}  {a:06X}  {t['n']:6d} {t['clusters']:3d}  "
              f"{','.join(map(str, sorted(t['ufs']))):6s} "
              f"{(block_of(a, blocks) or '(frei?)'):18s} {ref:3s} {span:>16s}")

    print("\n[4] AP-Bitfehler je Position (aus Clustern mit Wiederholung)")
    if not bit_tot:
        print("  zu wenig Wiederholungen")
    else:
        for nb in (56, 112):
            pos = [p for p in sorted(bit_tot) if nb - 23 <= p <= nb]
            if not pos or nb == 112 and not any(c["nbits"] == 112 for c in clusters):
                continue
            line = "  ".join(f"{p}:{100*bit_err[p]/bit_tot[p]:.1f}%"
                             for p in pos if bit_tot[p])
            print(f"  {nb} Bit: {line}")
    print()

if __name__ == "__main__":
    main()
