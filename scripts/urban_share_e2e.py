#!/usr/bin/env python3
"""E2E gate for the urban_share annotation: fork vs stock on identical data.

Usage:
    urban_share_e2e.py FORK_ON_STOCK FORK_ON_FORK STOCK_ON_STOCK STOCK_ON_FORK \
        [lon_min lat_min lon_max lat_max] [n_table_requests] [n_route_pairs]

The four positional arguments are base URLs of osrm-routed instances:
fork binaries serving stock-preprocessed data, fork binaries serving
fork-preprocessed data (with the urban side-cars), and a stock v26.6.5
build serving each of the same two directories. The default bbox is
Bremen; pass any bbox matching the extract being served.

Sections:
A: fork routed vs stock routed, both serving stock data -> tables identical
B: fork routed vs stock routed, both serving fork data  -> tables identical
   (cross-serve: stock ignores the side-car files)
C: adding urban_share to the request must not perturb durations/distances;
   urban_shares shape/diagonal/range checks
D: urban_share on stock data -> NoUrbanData (fork) / parse error (stock)
E: /route leg urban_share must equal the /table cell for the same pair
   (pairs whose durations differ are skipped: an equal-weight tie-break
   legitimately picked another path); every 4th pair re-requested with
   steps=true must report the identical leg summary; route on stock data
   -> NoUrbanData

Staleness scenarios (orchestrated outside this script; all verified on
Bremen 2026-07-20): tampering the recorded /urban/connectivity_checksum,
tampering /urban/config_identity or /common/urban_config_identity, a stock
osrm-extract run over a fork directory (rewrites the class mapping in
.osrm.properties, leaving both side-cars orphaned), and a weight-only
re-extract without a re-contract all produce a startup warning and degrade
to NoUrbanData on /table with durations unchanged; /route keeps its
annotation exactly as long as a valid .osrm.urban_config is present.
"""
import json
import random
import sys
import urllib.error
import urllib.request

FORK_ON_STOCK, FORK_ON_FORK, STOCK_ON_STOCK, STOCK_ON_FORK = sys.argv[1:5]
BBOX = tuple(float(v) for v in sys.argv[5:9]) or None
if not BBOX:
    BBOX = (8.63, 53.02, 8.90, 53.19)  # Bremen: lon_min, lat_min, lon_max, lat_max
N_REQUESTS = int(sys.argv[9]) if len(sys.argv) > 9 else 500
N_ROUTE_PAIRS = int(sys.argv[10]) if len(sys.argv) > 10 else 200
random.seed(42)


def get(base, path):
    try:
        with urllib.request.urlopen(base + path, timeout=30) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def random_point():
    return f"{random.uniform(BBOX[0], BBOX[2]):.6f},{random.uniform(BBOX[1], BBOX[3]):.6f}"


def random_coords():
    return ";".join(random_point() for _ in range(random.randint(4, 12)))


mismatches = {"A": 0, "B": 0, "C": 0}
urban_stats = {"cells": 0, "null": 0, "zero": 0, "mid": 0, "high": 0, "bad": 0}

for i in range(N_REQUESTS):
    coords = random_coords()
    path = f"/table/v1/driving/{coords}?annotations=duration,distance"

    _, fork_s1 = get(FORK_ON_STOCK, path)
    _, stock_s1 = get(STOCK_ON_STOCK, path)
    if (fork_s1.get("durations"), fork_s1.get("distances"), fork_s1.get("code")) != (
        stock_s1.get("durations"),
        stock_s1.get("distances"),
        stock_s1.get("code"),
    ):
        mismatches["A"] += 1

    _, fork_f1 = get(FORK_ON_FORK, path)
    _, stock_f1 = get(STOCK_ON_FORK, path)
    if (fork_f1.get("durations"), fork_f1.get("distances"), fork_f1.get("code")) != (
        stock_f1.get("durations"),
        stock_f1.get("distances"),
        stock_f1.get("code"),
    ):
        mismatches["B"] += 1

    path_urban = path + ",urban_share"
    _, fork_f1_urban = get(FORK_ON_FORK, path_urban)
    if (fork_f1_urban.get("durations"), fork_f1_urban.get("distances")) != (
        fork_f1.get("durations"),
        fork_f1.get("distances"),
    ):
        mismatches["C"] += 1

    shares = fork_f1_urban.get("urban_shares")
    durations = fork_f1_urban.get("durations")
    n = len(durations)
    assert shares is not None and len(shares) == n
    for r in range(n):
        assert len(shares[r]) == n
        for c in range(n):
            v = shares[r][c]
            urban_stats["cells"] += 1
            if r == c:
                if v is not None:
                    urban_stats["bad"] += 1
                continue
            if v is None:
                urban_stats["null"] += 1
                if durations[r][c] is not None and durations[r][c] > 0:
                    urban_stats["bad"] += 1  # reachable pair must have a share
            elif not (0.0 <= v <= 1.0):
                urban_stats["bad"] += 1
            elif v == 0:
                urban_stats["zero"] += 1
            elif v >= 0.7:
                urban_stats["high"] += 1
            else:
                urban_stats["mid"] += 1

print(f"A fork-vs-stock on stock data: {mismatches['A']}/{N_REQUESTS} mismatches")
print(f"B fork-vs-stock on fork data:  {mismatches['B']}/{N_REQUESTS} mismatches")
print(f"C urban_share perturbs tables: {mismatches['C']}/{N_REQUESTS} mismatches")
print(f"urban cells: {urban_stats}")

# D: urban_share against data without the side-car
probe = f"{(BBOX[0]+BBOX[2])/2:.4f},{(BBOX[1]+BBOX[3])/2:.4f};{(BBOX[0]+BBOX[2])/2+0.02:.4f},{(BBOX[1]+BBOX[3])/2-0.01:.4f}"
code_d, body_d = get(FORK_ON_STOCK, f"/table/v1/driving/{probe}?annotations=urban_share")
print(f"D fork on stock data urban_share -> http {code_d}, code={body_d.get('code')}")
code_e, body_e = get(STOCK_ON_STOCK, f"/table/v1/driving/{probe}?annotations=urban_share")
print(f"D stock urban_share -> http {code_e}, code={body_e.get('code')}")

# plausibility (informational, not part of the gate): bbox-centre pair vs a
# corner-to-corner long haul that should pick up motorway distance
_, centre = get(FORK_ON_FORK, f"/table/v1/driving/{probe}?annotations=urban_share")
longhaul_coords = f"{BBOX[0]:.4f},{BBOX[3]:.4f};{BBOX[2]:.4f},{BBOX[1]:.4f}"
_, longhaul = get(FORK_ON_FORK, f"/table/v1/driving/{longhaul_coords}?annotations=urban_share")
print(f"centre pair shares: {centre.get('urban_shares')}")
print(f"long-haul shares: {longhaul.get('urban_shares')}")

# E: /route leg urban_share vs the /table cell over the same pair; every 4th
# pair additionally requests steps=true — the leg summary describes the
# untrimmed leg, so turn-by-turn post-processing must not move it
e_checked = e_skipped = e_bad = 0
e_steps_checked = e_steps_bad = 0
for i in range(N_ROUTE_PAIRS):
    a, b = random_point(), random_point()
    _, table = get(FORK_ON_FORK, f"/table/v1/driving/{a};{b}?annotations=duration,urban_share")
    _, route = get(FORK_ON_FORK, f"/route/v1/driving/{a};{b}?annotations=urban_share&overview=false")
    if table.get("code") != "Ok" or route.get("code") != "Ok":
        e_skipped += 1
        continue
    cell = table["urban_shares"][0][1]
    table_duration = table["durations"][0][1]
    leg = route["routes"][0]["legs"][0]
    if cell is None or table_duration is None:
        e_skipped += 1
        continue
    if abs(table_duration - leg["duration"]) > 0.15:
        e_skipped += 1  # different path chosen (equal-weight tie-break)
        continue
    e_checked += 1
    route_share = leg.get("urban_share")
    if route_share is None or abs(route_share - cell) > 0.001 + 1e-9:
        e_bad += 1
        if e_bad <= 5:
            print(f"E MISMATCH {a};{b}: table={cell} route={route_share}")
    if i % 4 == 0:
        _, steps_route = get(
            FORK_ON_FORK,
            f"/route/v1/driving/{a};{b}?annotations=urban_share&overview=false&steps=true",
        )
        if steps_route.get("code") == "Ok":
            e_steps_checked += 1
            steps_share = steps_route["routes"][0]["legs"][0].get("urban_share")
            if steps_share != route_share:
                e_steps_bad += 1
                if e_steps_bad <= 5:
                    print(f"E STEPS MISMATCH {a};{b}: plain={route_share} steps={steps_share}")
print(f"E route-vs-table shares: {e_checked} compared, {e_skipped} skipped, {e_bad} mismatches")
print(f"E steps=true stability: {e_steps_checked} compared, {e_steps_bad} mismatches")

code_f, body_f = get(FORK_ON_STOCK, f"/route/v1/driving/{probe}?annotations=urban_share")
print(f"E fork on stock data route urban_share -> http {code_f}, code={body_f.get('code')}")

ok = (
    mismatches["A"] == 0
    and mismatches["B"] == 0
    and mismatches["C"] == 0
    and urban_stats["bad"] == 0
    and body_d.get("code") == "NoUrbanData"
    and e_bad == 0
    and e_checked >= N_ROUTE_PAIRS // 2
    and e_steps_bad == 0
    and e_steps_checked > 0
    and body_f.get("code") == "NoUrbanData"
)
print("E2E RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
