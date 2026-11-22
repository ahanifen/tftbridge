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
| Klipper reports unknown-command errors for the TFT's handshake. | `M92`, `M503`, `M150`, `M108`, `M876`, `M155`, `M211` and `M420` are acknowledged with `ok` and do nothing. |

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
- The probe-offset menu (`M851`) isn't translated.
- Only the newest files fit in the TFT's Onboard SD list. Older files are still printable from the
  web interface.
- Developed and tested on one printer (Ender 3 Pro, Klipper v0.13, TFT35 E3 V3.0, firmware V27.x).
  Other BTT TFT models and firmware versions are untested.

## Related

[oterek/klipper-btt-tft-bridge](https://github.com/oterek/klipper-btt-tft-bridge) is a separate
rewrite of the original. It talks to Klipper directly in-process instead of through the pty. The
parts of it this fork uses are credited above and in [NOTICE](NOTICE).
