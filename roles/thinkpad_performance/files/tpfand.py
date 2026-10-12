#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
"""tpfand — ThinkPad fan controller (PID + power feedforward + quantized EC levels).

Built for ThinkPad T14 Gen 2a (Ryzen 7 PRO 5850U). Requires thinkpad_acpi fan_control=1.

EC facts measured on this machine (2026-10-12):
  * steady rpm per level: 0=0, 1=2900, 2=3100, 3=3600, 4..7=4600, full-speed=6200
  * ANY "level X" write (even re-writing the current level) dips the fan to ~3800 rpm
    for 1-2 s, so the level is written only when it must change.
  * "watchdog N" writes do not disturb the fan, so the driver watchdog is re-armed
    every tick: if this daemon dies/hangs, thinkpad_acpi returns the fan to EC auto.

Control: demand = P + I + D(on measurement) + FF(package power), 0..1, -> target rpm
-> quantizer with hysteresis/dwell over the measured steps. Emergency latch forces
full speed on high Tctl. Sensor failure -> full speed.
"""
import glob, json, os, re, signal, socket, sys, time

DEFAULTS = {
    "interval_s": 1.0,
    "setpoint_c": 70.0,          # target smoothed Tctl
    "temp_tau_s": 6.0,           # EMA time constant for Tctl (Zen Tctl is spiky)
    "kp": 0.05,                  # demand per degC above setpoint
    "ki": 0.004,                 # demand per degC*s
    "i_deadband_c": 1.5,         # no integration within +-this of setpoint (limit-cycle damping)
    "kd": 0.15,                  # demand per degC/s of rise (derivative on measurement)
    "deriv_tau_s": 3.0,          # EMA on the derivative
    "i_min": -0.5, "i_max": 1.0,
    "ff_gain": 0.6,              # feedforward weight from package power
    "ff_p0_w": 8.0,              # power at which FF starts
    "ff_p1_w": 30.0,             # power at which FF is full
    # (EC level command, steady rpm) — measured; ascending
    "steps": [["0", 0], ["1", 2900], ["3", 3600], ["7", 4600], ["full-speed", 6200]],  # level 2 (3100) dropped: 200 rpm gap only adds churn
    "min_step": 1,               # floor index into steps (1 = never fully stop)
    "up_dwell_s": 3.0,           # min time at a step before stepping up again (> 1-2 s level-write dip)
    "down_dwell_s": 20.0,        # demand must stay low this long before stepping down (one step)
    "down_margin_c": 3.0,        # ...and smoothed Tctl must be this far below setpoint (bias high, no limit cycle)
    "hyst_frac": 0.25,           # fraction of the gap below midpoint required for downshift
    "up_hyst_frac": 0.15,        # fraction of the gap above midpoint required for upshift
    "emerg_raw_c": 90.0,         # raw Tctl >= this -> full speed (latched)
    "emerg_ema_c": 85.0,         # smoothed Tctl >= this -> full speed (latched)
    "emerg_release_c": 78.0,     # latch releases after EMA below this ...
    "emerg_release_s": 10.0,     # ... for this long
    "ec_watchdog_s": 10,
    "fan_path": "/proc/acpi/ibm/fan",
    "status_path": "/run/tpfand/status.json",
    "log_every_s": 60,
    "fault_hold_ticks": 3,       # tolerate this many consecutive Tctl read failures before forcing full speed
    "max_tick_errors": 5,        # consecutive unexpected tick errors before exiting (systemd restarts us)
}


def find_hwmon(name, attr):
    for h in sorted(glob.glob("/sys/class/hwmon/hwmon*")):
        try:
            if open(os.path.join(h, "name")).read().strip() == name and os.path.exists(os.path.join(h, attr)):
                return os.path.join(h, attr)
        except OSError:
            pass
    return None


class Controller:
    """Pure control logic: feed (t, tctl_c, power_w) -> desired step index. No I/O."""

    def __init__(self, cfg):
        self.c = cfg
        self.steps = cfg["steps"]
        self.top = len(self.steps) - 1
        self.ema = None
        self.prev_ema = None
        self.dTdt = 0.0
        self.i = 0.0
        self.idx = self.top           # start at full speed; let it come down
        self.last_change = -1e9
        self.low_since = None
        self.emerg = False
        self.emerg_ok_since = None
        self.demand = 1.0
        self.terms = {}
        self.faults = 0

    def _alpha(self, dt, tau):
        return dt / (tau + dt) if tau > 0 else 1.0

    def update(self, t, dt, tctl, power):
        c = self.c
        if tctl is None:                       # sensor failure: hold briefly, then full speed
            self.faults += 1
            self.terms = {"fault": "tctl", "count": self.faults}
            if self.ema is None or self.faults > c["fault_hold_ticks"]:
                if self.idx != self.top:
                    self.idx, self.last_change = self.top, t
                self.demand = 1.0
                self.low_since = None
            return self.idx
        self.faults = 0
        if self.ema is None:
            self.ema = self.prev_ema = tctl
        self.prev_ema = self.ema
        self.ema += self._alpha(dt, c["temp_tau_s"]) * (tctl - self.ema)
        raw_d = (self.ema - self.prev_ema) / dt if dt > 0 else 0.0
        self.dTdt += self._alpha(dt, c["deriv_tau_s"]) * (raw_d - self.dTdt)

        e = self.ema - c["setpoint_c"]
        p = c["kp"] * e
        d = c["kd"] * max(self.dTdt, 0.0)      # only pre-empt rises; let P/I handle falls
        ff = 0.0
        if power is not None and c["ff_p1_w"] > c["ff_p0_w"]:
            ff = c["ff_gain"] * min(max((power - c["ff_p0_w"]) / (c["ff_p1_w"] - c["ff_p0_w"]), 0.0), 1.0)
        unsat = p + self.i + d + ff
        # conditional integration (anti-windup): don't push further into saturation
        if abs(e) > c["i_deadband_c"] and not ((unsat >= 1.0 and e > 0) or (unsat <= 0.0 and e < 0)):
            self.i = min(max(self.i + c["ki"] * e * dt, c["i_min"]), c["i_max"])
        self.demand = min(max(p + self.i + d + ff, 0.0), 1.0)
        self.terms = {"p": round(p, 3), "i": round(self.i, 3), "d": round(d, 3), "ff": round(ff, 3)}

        # emergency latch
        if tctl >= c["emerg_raw_c"] or self.ema >= c["emerg_ema_c"]:
            self.emerg, self.emerg_ok_since = True, None
        elif self.emerg:
            if self.ema < c["emerg_release_c"]:
                self.emerg_ok_since = self.emerg_ok_since if self.emerg_ok_since is not None else t
                if t - self.emerg_ok_since >= c["emerg_release_s"]:
                    self.emerg, self.emerg_ok_since = False, None
                    self.low_since = t          # restart down-dwell from here
                    self.last_change = t
            else:
                self.emerg_ok_since = None
        if self.emerg:
            if self.idx != self.top:
                self.idx, self.last_change = self.top, t
            self.low_since = None
            return self.idx

        target = self.demand * self.steps[self.top][1]
        rpm = [s[1] for s in self.steps]
        lo = c["min_step"]
        # step up: to the highest step whose midpoint-below is <= target (round to nearest)
        want_up = self.idx
        for j in range(self.idx + 1, self.top + 1):
            if target >= (rpm[j - 1] + rpm[j]) / 2 + c["up_hyst_frac"] * (rpm[j] - rpm[j - 1]):
                want_up = j
        if want_up > self.idx:
            self.low_since = None
            if t - self.last_change >= c["up_dwell_s"]:
                self.idx, self.last_change = want_up, t
            return self.idx
        # step down (one step) after sustained low demand, with hysteresis below midpoint
        if self.idx > lo:
            mid = (rpm[self.idx - 1] + rpm[self.idx]) / 2
            thresh = mid - c["hyst_frac"] * (rpm[self.idx] - rpm[self.idx - 1])
            if target < thresh and self.ema < c["setpoint_c"] - c["down_margin_c"]:
                self.low_since = self.low_since if self.low_since is not None else t
                if t - self.low_since >= c["down_dwell_s"] and t - self.last_change >= c["down_dwell_s"]:
                    self.idx, self.last_change, self.low_since = self.idx - 1, t, None
            else:
                self.low_since = None
        if self.idx < lo:
            self.idx, self.last_change = lo, t
        return self.idx


def sd_notify(msg):
    addr = os.environ.get("NOTIFY_SOCKET")
    if not addr:
        return
    if addr.startswith("@"):
        addr = "\0" + addr[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
            s.connect(addr)
            s.sendall(msg.encode())
    except OSError:
        pass


class Fan:
    def __init__(self, path):
        self.path = path

    def read(self):
        txt = open(self.path).read()
        rpm = re.search(r"^speed:\s+(\d+)", txt, re.M)
        lvl = re.search(r"^level:\s+(\S+)", txt, re.M)
        return (int(rpm.group(1)) if rpm else None, lvl.group(1) if lvl else None)

    def cmd(self, s):
        with open(self.path, "w") as f:
            f.write(s)


def norm_level(l):
    # /proc reports e.g. "auto", "full-speed", "disengaged", or a number
    return {"disengaged": "full-speed"}.get(l, l)


def validate_cfg(cfg):
    """Raise ValueError on any config that would break control or the failsafes."""
    num = lambda k: isinstance(cfg[k], (int, float)) and not isinstance(cfg[k], bool)
    for k, v in DEFAULTS.items():
        if isinstance(v, (int, float)) and not num(k):
            raise ValueError(f"{k} must be a number, got {cfg[k]!r}")
    wd = cfg["ec_watchdog_s"]
    if not (isinstance(wd, int) and 1 <= wd <= 120):
        raise ValueError("ec_watchdog_s must be an integer 1..120 (thinkpad_acpi limit; 0 would disable the failsafe)")
    if not (0 < cfg["interval_s"] < min(wd, 10) / 2):
        raise ValueError("interval_s must be > 0 and < min(ec_watchdog_s, WatchdogSec=10)/2")
    steps = cfg["steps"]
    if not (isinstance(steps, list) and len(steps) >= 2 and all(
            isinstance(s, list) and len(s) == 2 and isinstance(s[0], str) and isinstance(s[1], (int, float))
            for s in steps)):
        raise ValueError("steps must be a list of [level_command, rpm] pairs (>= 2)")
    rpms = [s[1] for s in steps]
    if any(b <= a for a, b in zip(rpms, rpms[1:])):
        raise ValueError("steps rpm must be strictly ascending")
    if not (isinstance(cfg["min_step"], int) and 0 <= cfg["min_step"] < len(steps)):
        raise ValueError("min_step must be an integer index into steps")
    if not (cfg["fault_hold_ticks"] >= 0 and cfg["max_tick_errors"] >= 1):
        raise ValueError("fault_hold_ticks >= 0 and max_tick_errors >= 1 required")
    if cfg["emerg_release_c"] >= cfg["emerg_ema_c"]:
        raise ValueError("emerg_release_c must be below emerg_ema_c")


def load_cfg(path):
    cfg = dict(DEFAULTS)
    if path and os.path.exists(path):
        with open(path) as f:
            user = json.load(f)
        unknown = set(user) - set(DEFAULTS)
        if unknown:
            print(f"tpfand: ignoring unknown config keys: {sorted(unknown)}", flush=True)
        cfg.update({k: v for k, v in user.items() if k in DEFAULTS})
    validate_cfg(cfg)
    return cfg


def restore_auto(fan):
    """Return the fan to EC auto. 'level auto' first and on its own; watchdog is moot in auto."""
    ok = False
    for _ in range(2):
        try:
            fan.cmd("level auto")
            ok = True
            break
        except OSError as ex:
            print(f"tpfand: restore 'level auto' failed: {ex}", flush=True)
            time.sleep(0.2)
    try:
        fan.cmd("watchdog 0")
    except OSError:
        pass
    print("tpfand: stopped, fan returned to auto" if ok else
          "tpfand: stopped, FAILED to return fan to auto (driver watchdog/ExecStopPost are the backstop)", flush=True)
    return ok


class Sensors:
    """hwmon paths, re-resolved (rate-limited) when missing or failing — survives late module load / renumbering."""

    def __init__(self, retry_s=10.0):
        self.retry_s = retry_s
        self.tctl_path = self.pwr_path = None
        self.last_try = -1e9
        self.warned_pwr = False
        self.resolve(force=True)

    def resolve(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_try < self.retry_s:
            return
        self.last_try = now
        if self.tctl_path is None:
            self.tctl_path = find_hwmon("k10temp", "temp1_input")
            if self.tctl_path:
                print(f"tpfand: Tctl sensor {self.tctl_path}", flush=True)
        if self.pwr_path is None:
            self.pwr_path = find_hwmon("amdgpu", "power1_input") or find_hwmon("amdgpu", "power1_average")
            if self.pwr_path:
                print(f"tpfand: power sensor {self.pwr_path}", flush=True)
                self.warned_pwr = False
            elif not self.warned_pwr:
                print("tpfand: power sensor not found; feedforward disabled until it appears", flush=True)
                self.warned_pwr = True

    @staticmethod
    def _read(path, scale):
        try:
            return int(open(path).read()) / scale
        except (OSError, ValueError):
            return None

    def read(self):
        tctl = self._read(self.tctl_path, 1000.0) if self.tctl_path else None
        if tctl is None and self.tctl_path:
            self.tctl_path = None          # force re-resolve
        power = self._read(self.pwr_path, 1e6) if self.pwr_path else None
        if power is None and self.pwr_path:
            self.pwr_path = None
        if self.tctl_path is None or self.pwr_path is None:
            self.resolve()
            if tctl is None and self.tctl_path:
                tctl = self._read(self.tctl_path, 1000.0)
        return tctl, power


def fmt(v, nd=1):
    return "None" if v is None else f"{v:.{nd}f}"


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--check-config":
        try:
            load_cfg(sys.argv[2])
        except (ValueError, OSError) as ex:        # json.JSONDecodeError is a ValueError
            print(f"tpfand: invalid config {sys.argv[2]}: {ex}", file=sys.stderr)
            sys.exit(1)
        print(f"tpfand: config {sys.argv[2]} OK")
        sys.exit(0)
    cfg = load_cfg(sys.argv[1] if len(sys.argv) > 1 else "/etc/tpfand.json")
    fan = Fan(cfg["fan_path"])
    sensors = Sensors()
    ctl = Controller(cfg)
    running = [True]

    def stop(*_):
        running[0] = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    print(f"tpfand: start setpoint={cfg['setpoint_c']}C steps={cfg['steps']} min_step={cfg['min_step']}", flush=True)
    applied = None
    last_t = time.monotonic()
    last_log = -1e9
    errors = 0
    warned_unparsed = False
    sd_notify("READY=1")
    try:
        while running[0]:
            now = time.monotonic()
            dt = max(now - last_t, 1e-3)
            last_t = now
            fan_ok = False
            try:
                tctl, power = sensors.read()
                idx = ctl.update(now, dt, tctl, power)
                want = ctl.steps[idx][0]
                rpm, cur = fan.read()
                fan.cmd(f"watchdog {cfg['ec_watchdog_s']}")   # re-arm; does not disturb fan
                if cur is None and applied == want:
                    if not warned_unparsed:
                        print("tpfand: cannot parse current level; not re-applying (driver watchdog is backstop)", flush=True)
                        warned_unparsed = True
                elif applied != want or norm_level(cur) != norm_level(want):
                    fan.cmd(f"level {want}")
                    if applied != want:
                        print(f"tpfand: {applied} -> {want}  Tctl={fmt(tctl)} ema={fmt(ctl.ema)} P={fmt(power)} "
                              f"demand={ctl.demand:.2f} {ctl.terms} emerg={ctl.emerg}", flush=True)
                    else:
                        print(f"tpfand: level drifted to {cur}, re-applied {want}", flush=True)
                    applied = want
                fan_ok = True
                errors = 0
                sd_notify("WATCHDOG=1")                       # tied to successful fan control only
                status = {"t": time.time(), "tctl": tctl, "ema": None if ctl.ema is None else round(ctl.ema, 2),
                          "power_w": power, "rpm": rpm, "level": want, "demand": round(ctl.demand, 3),
                          "terms": ctl.terms, "emergency": ctl.emerg}
                if now - last_log >= cfg["log_every_s"]:
                    print(f"tpfand: status {status}", flush=True)
                    last_log = now
                try:
                    d = os.path.dirname(cfg["status_path"])
                    os.makedirs(d, exist_ok=True)
                    tmp = cfg["status_path"] + ".tmp"
                    with open(tmp, "w") as f:
                        json.dump(status, f)
                    os.replace(tmp, cfg["status_path"])
                except OSError:
                    pass
            except Exception as ex:                           # never let one bad tick kill control silently
                errors += 1
                print(f"tpfand: tick error {errors}/{cfg['max_tick_errors']}: {type(ex).__name__}: {ex}", flush=True)
                if not fan_ok:
                    try:
                        if applied != ctl.steps[-1][0]:
                            fan.cmd(f"level {ctl.steps[-1][0]}")
                            applied = ctl.steps[-1][0]
                    except Exception:
                        pass
                if errors >= cfg["max_tick_errors"]:
                    print("tpfand: too many consecutive errors, exiting for restart", flush=True)
                    break
            time.sleep(max(cfg["interval_s"] - (time.monotonic() - now), 0.05))
    finally:
        restore_auto(fan)


if __name__ == "__main__":
    main()
