#!/usr/bin/env python3
# Run as root (with tpfand stopped and thinkpad_acpi fan_control=1):
#   sudo systemctl stop tpfand && sudo python3 tools/fan-calibrate.py; sudo systemctl start tpfand
# (1) checks which fan commands cause an rpm dip, (2) measures the steady rpm of each EC level
# (each level written once). Use the results for tpfand's "steps" config.
import time, re, json, glob
FAN = "/proc/acpi/ibm/fan"
TCTL = next(h + "/temp1_input" for h in glob.glob("/sys/class/hwmon/hwmon*") if open(h + "/name").read().strip() == "k10temp")
def rpm(): return int(re.search(r"^speed:\s+(\d+)", open(FAN).read(), re.M).group(1))
def tctl(): return round(int(open(TCTL).read()) / 1000)
def cmd(s):
    with open(FAN, "w") as f: f.write(s)
def watch(n, every=None, what=None):
    out = []
    for _ in range(n):
        if every: cmd(what)
        time.sleep(1); out.append(rpm())
    return out
res = {}
try:
    cmd("level full-speed"); print("settle full-speed:", watch(15), flush=True)
    print("re-write 'watchdog 30' 1Hz:", watch(10, 1, "watchdog 30"), flush=True)
    cmd("watchdog 0")
    print("re-write 'level full-speed' 1Hz:", watch(10, 1, "level full-speed"), flush=True)
    print("recover:", watch(8), flush=True)
    cmd("level disengaged"); print("disengaged:", watch(10), flush=True)
    for lvl in ["7", "6", "5", "4", "3", "2", "1", "0", "7", "full-speed"]:
        cmd(f"level {lvl}"); h = watch(14)
        res.setdefault(lvl, []).append(h)
        print(f"level {lvl}: {h} Tctl={tctl()}", flush=True)
        if tctl() >= 94 and lvl in "0123": break
finally:
    cmd("watchdog 0"); cmd("level auto")
print(json.dumps({k: v[-1][-1] for k, v in res.items()}, indent=2))
