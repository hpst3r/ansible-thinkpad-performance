#!/usr/bin/env python3
"""Regression tests for review findings (no root, fake fan, fake sensors)."""
import os, signal, sys, threading, time, json, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "roles", "thinkpad_performance", "files"))
import tpfand
from tpfand import Controller, DEFAULTS, validate_cfg

fails = []
def check(name, cond, info=""):
    print(("PASS " if cond else "FAIL ") + name + (f"  [{info}]" if info and not cond else ""))
    if not cond: fails.append(name)

class FakeFan:
    def __init__(self, path, fail_on=()):
        self.cmds, self.level, self.fail_on = [], "auto", set(fail_on)
    def read(self):
        lv = {"full-speed": "disengaged"}.get(self.level, self.level)
        return 4000, lv
    def cmd(self, s):
        self.cmds.append(s)
        if s in self.fail_on: raise OSError(5, "EIO")
        if s.startswith("level "): self.level = s[6:]

def run_main(seconds, cfg_over=None, find=lambda n, a: None, fail_on=()):
    holder = {}
    def mkfan(path):
        holder["fan"] = FakeFan(path, fail_on); return holder["fan"]
    tpfand.Fan = mkfan
    tpfand.find_hwmon = find
    tmpd = tempfile.mkdtemp(dir=os.environ.get("TMPDIR"))
    cfg = {"interval_s": 0.1, "status_path": os.path.join(tmpd, "s.json"), "log_every_s": 999}
    cfg.update(cfg_over or {})
    cp = os.path.join(tmpd, "c.json"); json.dump(cfg, open(cp, "w"))
    sys.argv = ["tpfand", cp]
    threading.Timer(seconds, lambda: os.kill(os.getpid(), signal.SIGTERM)).start()
    err = None
    try:
        tpfand.main()
    except BaseException as ex:
        err = ex
    return holder["fan"], err

# 1. no sensor at startup -> full speed, no crash, restore 'level auto' before 'watchdog 0'
fan, err = run_main(1.0)
check("no-sensor: no exception escapes", err is None, repr(err))
check("no-sensor: commands full-speed", "level full-speed" in fan.cmds, fan.cmds[:5])
lv = [c for c in fan.cmds if c.startswith("level")]
check("no-sensor: exactly one level write while running (no churn)", lv.count("level full-speed") == 1, lv)
check("shutdown: 'level auto' precedes final 'watchdog 0'", fan.cmds[-2:] == ["level auto", "watchdog 0"], fan.cmds[-3:])

# 2. 'watchdog 0' failing must not prevent 'level auto'
fan, err = run_main(0.6, fail_on={"watchdog 0"})
check("restore: level auto written even if watchdog 0 fails", "level auto" in fan.cmds and fan.level == "auto", fan.cmds[-3:])

# 3. 'level auto' failing once is retried
class Flaky(FakeFan):
    n = 0
    def cmd(self, s):
        if s == "level auto" and Flaky.n == 0:
            Flaky.n += 1; self.cmds.append(s); raise OSError(5, "EIO")
        super().cmd(s)
tpfand.Fan = lambda p: Flaky(p)
ok = tpfand.restore_auto(f := Flaky("x"))
check("restore: retries level auto after failure", ok and f.level == "auto", f.cmds)

# 4. single-tick sensor glitch holds current step
c = Controller(dict(DEFAULTS))
for t in range(300): c.update(float(t), 1.0, 55.0, 6.0)
before = c.idx
c.update(300.0, 1.0, None, 6.0)
check("glitch: one None sample holds step", c.idx == before, (before, c.idx))
for t in range(301, 305): c.update(float(t), 1.0, None, 6.0)
check("glitch: persistent failure -> full speed", c.idx == c.top, c.idx)

# 5. config validation
bad = [{"ec_watchdog_s": 0}, {"ec_watchdog_s": 200}, {"interval_s": 6}, {"setpoint_c": "70"},
       {"steps": [["1", 2900], ["0", 0]]}, {"min_step": 9}, {"emerg_release_c": 90}]
for b in bad:
    cfg = dict(DEFAULTS); cfg.update(b)
    try:
        validate_cfg(cfg); check(f"validate rejects {b}", False)
    except ValueError:
        check(f"validate rejects {b}", True)
validate_cfg(dict(DEFAULTS)); check("validate accepts defaults", True)

# 6. sensor appears later (late module load) -> picked up via re-resolve
calls = {"n": 0}
real_sensor = os.path.join(tempfile.mkdtemp(dir=os.environ.get("TMPDIR")), "temp1_input")
open(real_sensor, "w").write("60000")
def late(name, attr):
    calls["n"] += 1
    return real_sensor if (name == "k10temp" and time.monotonic() - t0 > 0.3) else None
t0 = time.monotonic()
orig = tpfand.Sensors.__init__
def fast_init(self, retry_s=0.2): orig(self, retry_s)
tpfand.Sensors.__init__ = fast_init
fan, err = run_main(2.0, find=late)
check("late sensor: no exception", err is None, repr(err))
lv = [c for c in fan.cmds if c.startswith("level")]
check("late sensor: leaves full-speed once Tctl appears (60C)", len(lv) >= 2 and lv[1] != "level full-speed", lv)

print("ALL PASS" if not fails else f"{len(fails)} FAIL")
sys.exit(1 if fails else 0)
