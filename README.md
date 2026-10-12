# ansible-thinkpad-performance

Ansible role (and a localhost playbook) that makes an AMD ThinkPad run at its
real thermal limit instead of Lenovo's conservative power and skin-temperature
limits, and replaces the EC fan curve with a PID fan controller.

Built and tested on a **ThinkPad T14 Gen 2a (Ryzen 7 PRO 5850U), Fedora 44**.

## Why

With tuned `latency-performance` (platform profile `performance`, amd-pstate
EPP `performance`, boost on, on AC), the CPU still sat at ~17 W. Nothing in
the OS was limiting it. `ryzenadj --info` showed the SMU's STAPM limit pinned
at 17 W, because Lenovo's skin-temperature tracking (STT, target 50.5 °C)
was pulling it down. The EC fan curve also never exceeds ~4600 rpm, even
though the fan can do ~6200.

| | Stock | With this role |
|---|---|---|
| Sustained package power | 14–18 W | 25–35 W |
| All-core clock (under load) | ~2.2 GHz | ~3.3–3.5 GHz |
| Fan | 4600 rpm max (EC auto) | up to 6200 rpm |
| Limiting factor | STT → STAPM 17 W | Tctl (AMD throttle, 96 °C) |

## What it does

1. **Preflight:** asserts an AMD CPU, kernel lockdown `none` (ryzenadj reaches
   the SMU through `/dev/mem`), and `thinkpad_acpi` present.
2. **tuned:** optionally activates a profile (the playbook uses
   `latency-performance`).
3. **ryzenadj:** builds [FlyGoat/RyzenAdj](https://github.com/FlyGoat/RyzenAdj)
   from a pinned commit and installs it to `/usr/local/bin`. Installs
   `apu-tune` plus `apu-tune.timer`, which re-apply the limits every 15 s,
   because firmware resets them on platform-profile changes, AC events and
   resume. Settings go in `/etc/default/apu-tune`.
4. **Fan:** sets `thinkpad_acpi fan_control=1`. If the running module doesn't
   have it, the role reloads the module and re-applies the tuned profile,
   because a reload resets the platform profile to `balanced` (and resets the
   keyboard backlight). Then it installs and enables `tpfand`.

## tpfand

`tpfand` is a small Python daemon with no dependencies, at
`roles/thinkpad_performance/files/tpfand.py`.

The EC accepts only discrete levels. On the T14 G2a their steady speeds are:

| EC level | rpm |
|---|---|
| 0 | 0 |
| 1 | 2900 |
| 2 | 3100 |
| 3 | 3600 |
| 4–7 | 4600 |
| full-speed | 6200 |

**Every `level` write restarts the EC fan loop, about a 1–2 s dip to ~3800 rpm,
even when it re-writes the current level.** Writing `watchdog N` does not
disturb the fan. tpfand is designed around that:

- **Control:** PID on smoothed Tctl (default setpoint 70 °C), plus feedforward
  from package power (the amdgpu `PPT` sensor), so the fan starts ramping
  before the heat arrives.
- **Quantizer:** maps demand onto the measured steps. Upshifts are quick
  (≥ 3 s dwell). Downshifts are one step at a time, after 20 s of low demand,
  and only while temperature is ≥ 3 °C below setpoint. This biases the fan
  high and stops it hunting between two steps.
- **Emergency latch:** full speed at raw Tctl ≥ 90 °C or smoothed Tctl
  ≥ 85 °C, held until the smoothed value stays below 78 °C for 10 s.
- **Fewer writes:** the level is written only when it changes, or when firmware
  changes it behind the daemon's back (e.g. after resume).

### Failsafes

- The `thinkpad_acpi` watchdog is re-armed every second. If the daemon dies or
  hangs, the kernel returns the fan to EC auto within 10 s.
- systemd runs it with `Type=notify`, `WatchdogSec=10`, `Restart=always` and
  `TimeoutStopSec=5`. `ExecStopPost` sets the fan to `auto`.
- Shutdown writes `level auto` first (with a retry), then `watchdog 0`.
- One failed sensor read holds the current step. After 3 consecutive failures
  it goes to full speed. Missing hwmon sensors are re-discovered.
- Any unexpected error in a tick forces full speed. After 5 consecutive errors
  the daemon exits and systemd restarts it.
- The config is validated at start-up, and also when Ansible installs it
  (`tpfand --check-config`).

### Monitoring

```sh
watch -n1 cat /run/tpfand/status.json   # Tctl, EMA, power, rpm, level, PID terms
journalctl -fu tpfand                    # level transitions + periodic status
sudo ryzenadj --info                     # current SMU limits / values
```

## Usage

```sh
git clone https://github.com/hpst3r/ansible-thinkpad-performance
cd ansible-thinkpad-performance
ansible-playbook playbook.yaml -K
```

The playbook targets `localhost` through `inventory.yaml`, using a local
connection.

## Variables

All variables are in `roles/thinkpad_performance/defaults/main.yaml`.

| Variable | Default | Notes |
|---|---|---|
| `thinkpad_performance_ryzenadj_enabled` | `true` | Build ryzenadj and install apu-tune |
| `thinkpad_performance_ryzenadj_version` | pinned SHA | RyzenAdj commit to build |
| `thinkpad_performance_ryzenadj_build_jobs` | `4` | Bounded build parallelism |
| `thinkpad_performance_stapm_limit_mw` | `35000` | Stock 17000 on T14 G2a |
| `thinkpad_performance_slow_limit_mw` | `35000` | Stock 24000 |
| `thinkpad_performance_fast_limit_mw` | `40000` | Stock 30000 |
| `thinkpad_performance_apu_skin_temp_c` | `105` | Stock 50.5; 105 effectively disables STT |
| `thinkpad_performance_ryzenadj_extra_args` | `""` | e.g. `--tctl-temp=95` |
| `thinkpad_performance_apu_tune_interval_s` | `15` | Re-apply interval |
| `thinkpad_performance_fan_enabled` | `true` | Enable `fan_control=1` |
| `thinkpad_performance_tpfand_enabled` | `true` | Run tpfand |
| `thinkpad_performance_fan_level` | `""` | Fixed level written by apu-tune when tpfand is off |
| `thinkpad_performance_tpfand_config` | `{setpoint_c: 70.0, min_step: 1}` | Overrides tpfand `DEFAULTS` |
| `thinkpad_performance_tuned_profile` | `""` | tuned profile to activate |

Useful `tpfand` keys:
- `setpoint_c`: lower it for more fan and lower temperatures.
- `min_step`: `0` allows the fan to stop.
- `down_dwell_s`: how long load must stay low before stepping down.
- `steps`: `[EC level, rpm]` pairs.

## Other ThinkPads

The power limits and fan steps are specific to the T14 Gen 2a. On another
model, read `sudo ryzenadj --info` first, then measure the fan levels:

```sh
sudo systemctl stop tpfand
sudo python3 tools/fan-calibrate.py
sudo systemctl start tpfand
```

Set `thinkpad_performance_tpfand_config.steps` from the results. Drop levels
that share an rpm, and steps that are very close together.

## Reverting

```sh
sudo systemctl disable --now tpfand apu-tune.timer   # fan returns to EC auto
sudo tuned-adm profile "$(tuned-adm active | sed -n 's/^Current active profile: //p')"
```

The second command re-applies the platform profile, which makes the firmware
restore its own limits. To remove everything, also delete:
- `/etc/modprobe.d/thinkpad_acpi.conf`
- `/etc/default/apu-tune`, `/etc/tpfand.json`
- `/etc/systemd/system/{apu-tune.service,apu-tune.timer,tpfand.service}`
- `/usr/local/{bin/ryzenadj,sbin/apu-tune,sbin/tpfand}`
- `/usr/local/src/RyzenAdj`

## Caveats

- This bypasses Lenovo's skin-temperature limit **on purpose**. The chassis
  gets hot, and the fan is loud under load.
- Requires kernel lockdown `none` (in practice, Secure Boot disabled).
- `dnf install` cancels a pending GNOME Software offline update. It
  reschedules itself.

## Tests

```sh
python3 tests/sim_test.py      # controller vs. a crude thermal model (hunting, step response, latch)
python3 tests/regress_test.py  # main() against a fake fan/sensors (failsafes, config validation)
ansible-lint && ansible-playbook --syntax-check playbook.yaml
```

CI runs all of these.

## License

[AGPL-3.0-or-later](LICENSE)
