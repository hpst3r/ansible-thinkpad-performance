#!/usr/bin/env python3
"""Offline sim of tpfand.Controller against a crude laptop thermal model."""
import math, random, sys
sys.path.insert(0, __import__("os").path.join(__import__("os").path.dirname(__import__("os").path.abspath(__file__)), "..", "roles", "thinkpad_performance", "files"))
from tpfand import Controller, DEFAULTS

def run(name, power_fn, secs=600, fault_at=None, seed=1):
    random.seed(seed)
    c = Controller(dict(DEFAULTS))
    rpm_of = {s[0]: s[1] for s in DEFAULTS["steps"]}
    T, rpm, changes, prev, worst, hist = 60.0, 6200.0, 0, None, 0, {}
    for t in range(secs):
        P = power_fn(t)
        # thermal resistance falls with airflow; heatsink time constant ~25 s
        Rth = 4.5 - 2.2 * (rpm / 6200)          # C/W  (fan off ~4.5, full ~2.3)
        T_ss = 35 + P * Rth
        T += (T_ss - T) / 25.0
        tctl = T + random.gauss(0, 2.5)          # spiky sensor
        if fault_at and fault_at <= t < fault_at + 5:
            tctl = None
        idx = c.update(float(t), 1.0, tctl, P)
        lvl = c.steps[idx][0]
        rpm += (rpm_of[lvl] - rpm) * 0.5         # fan spin dynamics ~2 s
        if lvl != prev:
            changes += prev is not None; prev = lvl
        hist[lvl] = hist.get(lvl, 0) + 1
        worst = max(worst, T)
    print(f"{name:28s} changes={changes:3d}  maxT={worst:5.1f}  endT={T:5.1f}  "
          f"time@level={dict(sorted(hist.items(), key=lambda kv: rpm_of[kv[0]]))}")
    return changes, worst

ok = True
ch, w = run("idle 6W", lambda t: 6)
ok &= ch <= 6
ch, w = run("light 12W", lambda t: 12)
ok &= ch <= 10
ch, w = run("steady 25W", lambda t: 25)
ok &= ch <= 8
ch, w = run("sustained 35W", lambda t: 35)
ok &= ch <= 4
ch, w = run("step 8->30W @200s", lambda t: 8 if t < 200 else 30)
ok &= ch <= 10
ch, w = run("bursty 8/30W 20s period", lambda t: 30 if (t // 10) % 2 else 8)
ok &= ch <= 30
ch, w = run("square 5/25W 120s period", lambda t: 25 if (t // 60) % 2 else 5, secs=1200)
ok &= ch <= 45
ch, w = run("sensor fault @300s", lambda t: 15, fault_at=300)
# emergency: spike raw tctl
c = Controller(dict(DEFAULTS)); c.idx = 1
for t in range(5): c.update(float(t), 1.0, 60.0, 8)
idx = c.update(5.0, 1.0, 92.0, 8)
ok &= idx == c.top and c.emerg
for t in range(6, 30): idx = c.update(float(t), 1.0, 60.0, 8)
ok &= not c.emerg
print("emergency latch/release OK" if ok else "FAIL", "| overall", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
