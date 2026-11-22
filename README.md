# tftbridge

A Klipper add-on that lets a BigTreeTech TFT35 run in **Touch Screen mode** with Klipper. The TFT
talks Marlin-style G-code over a serial port; `tftbridge` connects that port to Klipper and fills in
the parts of Marlin's behaviour the TFT relies on.

## Credit

This is a fork of **[oldhui-uk/tftbridge](https://github.com/oldhui-uk/tftbridge)** by **K. Hui
(Old Hui)**. The original idea, the hardware connection and the first version of the bridge are his:

- [Using BTT TFT35 with Klipper in touch mode](https://oldhui.wordpress.com/2024/01/28/using-btt-tft35-with-klipper-in-touch-mode/) (hardware connection)
- [Using BTT TFT35 in touch screen mode with a Klipper add-on](https://oldhui.wordpress.com/2024/02/02/using-btt-tft35-in-touch-screen-mode-with-a-klipper-add-on/) (the add-on)

The original is a transparent pass-through. This fork keeps its design (a Klipper `extras` module
bridging the TFT's serial port to Klipper's pseudo-terminal) and adds the behaviour described below.

Small parts were adapted from **[oterek/klipper-btt-tft-bridge](https://github.com/oterek/klipper-btt-tft-bridge)**
by **Piter (garagescunny)**, MIT License: answering bare `M220`/`M221` queries locally (and the
`Feedrate: … FR:` / `Flow:` reply format), and advertising `Cap:BABYSTEPPING:1`. Their copyright and
licence notice is in [NOTICE](NOTICE).

**The changes in this fork were developed with the help of Claude Code (Claude Opus 5.5).** They were
written and tested together with the repository owner on their own printer.

Licence: GPL-3.0, the same as the original (see [LICENSE](LICENSE)); the MIT-licensed portions are
listed in [NOTICE](NOTICE).

## What this fork changes

Tested on a BTT **TFT35 E3 V3.0** with firmware **V27.x** (December 2022), connected to a Raspberry
Pi 5 UART at 250000 baud. The changes are based on the TFT firmware source, tag `Vx.x.27`
(`TFT/src/User/API/parseACK.c`, `Printing.c`, `Gcode/gcode.c`).

| Problem in Touch mode | What the bridge now does |
|---|---|
| Threads never stopped, because reads had no timeout. After every Klipper `RESTART` an extra pair of threads competed for the TFT's port. Lines were lost, and the TFT waited forever for an `ok`; commands typed on the TFT arrived minutes late. | Reads time out (0.2 s). The threads stop and are joined on `klippy:disconnect`, and errors are logged instead of swallowed. |
| The TFT froze while Klipper ran a long command (e.g. a start macro with homing and heat soak), because its status polls queued behind it. | `M105`, `M27`, `M114` and `M115` are answered by the bridge from Klipper's state, so temperatures and progress keep updating. |
| The TFT didn't notice prints started from Mainsail, Fluidd or Mobileraker, so it had no printing screen and no Pause or Stop buttons. | `M115` reports `Cap:SDCARD:1`, so the TFT (with `onboard_sd` on auto) treats Klipper's virtual SD as an onboard SD card. It follows Klipper's own `File opened:`, `M27` progress and `Done printing file`. |
| A pause or cancel from the web didn't reach the TFT (Klipper's `M27` still reports progress while paused). | A 1 s watcher sends `//action:paused` and `//action:cancel` when the print state changes. |
| `M117` messages didn't appear on the TFT. | They're sent as `//action:notification <text>` and shown on its status screen. |
| The TFT's Pause, Resume and Stop buttons send Marlin commands. | `M25` → `PAUSE`. `M24` → `RESUME` when paused; otherwise `M24` is forwarded, to start a file selected on the TFT. `M524` → `CANCEL_PRINT`. So your `PAUSE`/`RESUME`/`CANCEL_PRINT` macros run. |
| The Onboard SD list hung on large libraries. The TFT's receive buffer is 5000 bytes (`CMD_MAX_REV`), and a full `M20` of about 1000 files also overflowed Klipper's pty, which lost the closing `End file list`. | `M20` is answered with the newest files only (`onboard_sd_files`, default 40, capped at 4200 bytes). |
| The TFT cuts file names at the first space. | Spaces are shown as `_`, and `M23` maps the shown name back to the real file. |
| F0 on the TFT never showed the part fan's speed. | The part fan's speed is sent as `M106 P0 S<n>` whenever it changes. |
| The TFT's Speed menu sends a bare `M220` and `M221` to read the values. Klipper treats them as `S100` and **resets** speed and flow. | Bare `M220` and `M221` are answered by the bridge; with `S` they go to Klipper. *(Adapted from oterek/klipper-btt-tft-bridge.)* |
| Babystep (`M290 Z…`) isn't a Klipper command. | It becomes `SET_GCODE_OFFSET Z_ADJUST=… MOVE=1`. `M115` advertises `Cap:BABYSTEPPING:1` *(as in oterek/klipper-btt-tft-bridge)*. |
| Klipper reports unknown-command errors for the TFT's handshake. | `M108`, `M876`, `M155`, `M211` and `M420` are acknowledged with `ok` and do nothing. |

### Touch-mode menus on TFT firmware V28 (added 2026-11-22)

Based on the V28 source (`TFT/src/User/Menu/*.c`, `API/Mainboard_AckHandler.c`). Tested with a local
pty harness (`tests/run_test.py`); on-printer testing is still to do.

| TFT menu | What it sends | What the bridge does |
|---|---|---|
| Tuning → PID | `M303 E0/E-1 … S<t>` | `PID_CALIBRATE HEATER=extruder/heater_bed TARGET=<t>`, then reports `PID Autotune finished` (or `failed`). Save with `SAVE_CONFIG`. |
| Tuning → Tune extruder | `M92` query, then `M92 T0 E<steps>` | Reports steps/mm from `rotation_distance`. The new value is applied live with `SET_EXTRUDER_ROTATION_DISTANCE`, and the bridge shows the `rotation_distance` to put in the config. |
| Tuning → Home offset | `M206` | Reports zeros; changes are refused (not used with Klipper). |
| Bed leveling → ABL | `G28`, `G29`, `M118 P0 ABL Completed` | `BED_MESH_CALIBRATE`; "ABL Completed" reaches the TFT when the mesh is done. `M115` advertises `Cap:AUTOLEVEL:1` so the menu appears. |
| Level corner | `G30 E1 X Y` | Moves the probe over X,Y and runs `PROBE`; replies `Bed X: Y: Z:` (or `Z Probe Past Bed` on an error, so the TFT never hangs). |
| BLTouch | `M280 P0 S10/90/120/160`, `M48` | `BLTOUCH_DEBUG COMMAND=pin_down/pin_up/self_test/reset`; `PROBE_ACCURACY` with its result as `Mean:` / `Standard Deviation:`. |
| Z align | `G34` | `Z_TILT_ADJUST` (enable with `z_steppers_alignment:1` in the TFT's `config.ini`). |
| Mesh editor | `M420 V1 T1`, `M421` | Shows Klipper's active mesh (as Marlin's `Bilinear Leveling Grid`); edits are refused. |
| Machine → Parameter settings | `M503 S0`, then edits | Lists Klipper's live values (steps/mm, max feed rate and acceleration, PID, probe offset, pressure advance, TMC currents). **Read only:** edits are refused, since Klipper keeps them in `printer.cfg`. |
| Babystep (on exit) and Probe offset | `M851 …` | Reports the probe offset (`[bltouch]`/`[probe]` `z_offset`); changes are refused. |
| LED colour | `M150 R U B P` | Calls a `_TFT_LED_COLOR R= G= B=` macro (0–1) that you define in Klipper, if you want it. |
| Custom menu `SCREWS_TILT_CALCULATE` | — | The per-screw results are summarised as one notification, e.g. `Screws: FL base, RL CW 0:15`. |
| Mesh valid, Mesh tuner, Shut down, BLTouch HS mode, case light | `G26`, `G42`, `M81`, `M401 S`, `M355` | Answered with "not supported with Klipper". |

Replies to translated commands are rewritten on their way back. Every line sent to Klipper is matched
to its `ok` in order, and the extra `ok`s of multi-line translations are dropped, so the TFT sees one
per command. TFT firmware V28 stops waiting for an `ok` after 15 s, so long commands (PID, mesh) don't
block the screen, and their results still arrive when Klipper finishes.

## Installation

Install it like other add-ons (for example `klipper-led_effect`): clone the repository and link the
module into Klipper's `extras` folder.

```bash
cd ~
git clone https://github.com/ahanifen/tftbridge.git
ln -sf ~/tftbridge/tftbridge.py ~/klipper/klippy/extras/tftbridge.py
# keep Klipper's git status clean
grep -qx 'klippy/extras/tftbridge.py' ~/klipper/.git/info/exclude || echo 'klippy/extras/tftbridge.py' >> ~/klipper/.git/info/exclude
sudo systemctl restart klipper
```

Klipper loads Python modules only when the service starts, so after updating the bridge restart the
**service**. `RESTART` and `FIRMWARE_RESTART` don't reload modules.

### Klipper configuration

```ini
[tftbridge]
tft_device: /dev/ttyAMA1           # serial port wired to the TFT
tft_baud: 250000                   # must match the TFT's serial port setting
klipper_device: /home/pi/printer_data/comms/klippy.serial
klipper_baud: 250000
# Optional (defaults shown):
#answer_status_locally: True       # answer M105/M27/M114/M115/M20 and the M220/M221 queries in the bridge
#translate_buttons: True           # M25/M24/M524/M23/M290 translations
#host_actions: True                # pause/cancel/notification/fan updates pushed to the TFT
#onboard_sd_files: 40              # newest files listed on the TFT's Onboard SD screen
# tft_timeout / klipper_timeout from the original are accepted and ignored
```

Klipper needs `[virtual_sdcard]`, `[pause_resume]`, `[display_status]` and `[print_stats]`. The part
fan display reads `[fan_generic fan0]`. It reports on the slicer scale, using `part_fan_max` from a
`_USER_SETTINGS` macro if one exists, otherwise 0.45. Adjust `fan_report()` for other setups.

### Moonraker update manager

```ini
[update_manager tftbridge]
type: git_repo
path: ~/tftbridge
origin: https://github.com/ahanifen/tftbridge.git
primary_branch: main
managed_services: klipper
```

### TFT settings

No `config.ini` change was needed on the tested TFT. Its `onboard_sd` setting was on auto, and the
bridge's `M115` reply enables Onboard SD. If Onboard SD doesn't appear, set `onboard_sd:1` in the
TFT's `config.ini`. After installing or restarting the bridge, switch the TFT to Marlin mode and back
(or power-cycle it) so it repeats its `M115` handshake.

## Known limitations

- Restarting the Klipper service briefly drops the bridge. TFT firmware V27 has no acknowledgement
  timeout, so it simply waits; newer TFT firmware may show "ACK timed out" once.
- Settings changed from the TFT's Parameter settings or Probe offset menus are refused; only the
  extruder steps/mm (Tune extruder) is applied, and only until Klipper restarts.
- The TFT's menus are built into its firmware. Items can't be added to Tuning or Leveling without
  rebuilding it; use the Custom menu (`custom_gcode_N` in `config.ini`) for Klipper commands.
- Only the newest files fit in the TFT's Onboard SD list. Older files are still printable from the
  web interface.
- Developed and tested on one printer (Ender 3 Pro, Klipper v0.13, TFT35 E3 V3.0, firmware V27.x).
  Other BTT TFT models and firmware versions are untested.

## Related

[oterek/klipper-btt-tft-bridge](https://github.com/oterek/klipper-btt-tft-bridge) is a separate
rewrite of the original. It talks to Klipper directly in-process instead of through the pty. The
parts of it this fork uses are credited above and in [NOTICE](NOTICE).
