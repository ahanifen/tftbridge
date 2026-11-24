#
# BigTreeTech TFT35 bridge
#
# Original author: K. Hui (https://oldhui.wordpress.com/2024/02/02/using-btt-tft35-in-touch-screen-mode-with-a-klipper-add-on/)
# Reworked 2026-11-21 for this printer (TFT35 E3 V3.0, firmware V27.x, Marlin-mode serial over /dev/ttyAMA1):
#   - threads stop cleanly on disconnect/restart (reads time out), so a Klipper RESTART no longer
#     leaves old threads competing for the TFT's port (lost lines -> TFT waiting forever for "ok")
#   - status polls (M105, M27, M114) and M115 are answered here from Klipper's state, so the TFT
#     stays responsive while a long command (e.g. START_PRINT) holds Klipper's gcode queue
#   - M115 reports Cap:SDCARD:1, so the TFT (onboard_sd = auto) treats Klipper's virtual SD as an
#     onboard SD card and follows prints started from Mainsail/Mobileraker ("File opened:",
#     "Done printing file", M27 progress)
#   - TFT buttons are translated: M25 -> PAUSE, M24 -> RESUME (when paused), M524 -> CANCEL_PRINT;
#     Marlin-only commands Klipper doesn't know are acknowledged here
#   - pause/cancel from other clients and M117 messages are pushed to the TFT as host actions
#   - part fan speed is pushed as "M106 P0 S<n>" (slicer scale) so the TFT's F0 tracks it
#   - bare M220/M221 (the TFT's Speed menu queries) are answered here; Klipper would treat them
#     as S100 and reset speed/flow. M290 Z (babystep) -> SET_GCODE_OFFSET Z_ADJUST=.. MOVE=1
#
# Repository: https://github.com/ahanifen/tftbridge (fork of https://github.com/oldhui-uk/tftbridge)
# Licence: GPL-3.0, as the original. Changes made with the help of Claude Code (Claude Opus 5.5).
# Portions adapted from klipper-btt-tft-bridge (https://github.com/oterek/klipper-btt-tft-bridge),
# Copyright (c) 2026 Piter (garagescunny), MIT License - see NOTICE: the local answers to bare
# M220/M221 (and their reply format) and Cap:BABYSTEPPING:1 in M115.
#   - M20 (file list) is answered here with the newest files only: the TFT's receive buffer is
#     5000 bytes (V27 CMD_MAX_REV) and Klipper's full list (~1000 files) overflowed both it and the
#     Klipper pty (lost "End file list" -> TFT stuck loading). The TFT also cuts names at the first
#     space, so spaces are shown as "_" and M23 maps the shown name back to the real file.
# Added 2026-11-22 for TFT firmware V28 (Touch-mode menus that expect Marlin):
#   - Tuning: PID (M303 -> PID_CALIBRATE, "PID Autotune finished/failed"), Tune extruder (M92 query
#     answered from rotation_distance; "M92 E" -> SET_EXTRUDER_ROTATION_DISTANCE, live only)
#   - Leveling (Cap:AUTOLEVEL:1): ABL (G29 + "ABL Completed"), Level corner (G30 -> PROBE, reply
#     "Bed X: Y: Z:"), BLTouch (M280 -> BLTOUCH_DEBUG, M48 -> PROBE_ACCURACY "Mean"/"Standard
#     Deviation"), Z align (G34 -> Z_TILT_ADJUST), Mesh editor (M420 V1 T1 -> Klipper's mesh as a
#     Marlin bilinear grid, view only)
#   - Parameter settings: M503 is answered with Klipper's live values (read only; edits are refused)
#   - M851/M206 answered, M150 -> _TFT_LED_COLOR macro, M81/G26/G42/M421/EEPROM -> "not supported"
#   - SCREWS_TILT_CALCULATE results are summarised as one notification
#   Replies Klipper sends for a translated command are rewritten on the way back (see Pending):
#   each forwarded line is matched to Klipper's "ok" in order, and extra "ok"s of multi-line
#   translations are dropped so the TFT sees exactly one per command.
#
import collections, logging, math, os, re, threading, time
import serial

M20_MAX_BYTES = 4200    # stay well under the TFT's 5000-byte buffer
GCODE_EXTS = ("gcode", "g", "gco")

READ_TIMEOUT = 0.2      # seconds; lets the threads notice a stop request
WATCH_INTERVAL = 1.0    # seconds between print-state checks for host actions
POS_EVERY = 2           # push temperatures (always) and position (while printing) every POS_EVERY watcher ticks
# Marlin commands the TFT sends that Klipper doesn't implement: acknowledge, do nothing
SWALLOW = {"M108", "M876", "M155", "M211", "M420"}
# Marlin settings commands: Klipper keeps these in printer.cfg, so the TFT's editors are read only
READ_ONLY = {"M200", "M201", "M203", "M204", "M205", "M207", "M208", "M209", "M218", "M301", "M304",
             "M900", "M906", "M913", "M914", "M500", "M501", "M502", "M421", "M206"}
NOT_SUPPORTED = {"G26": "Mesh validation pattern (G26)", "G42": "Mesh tuner (G42)",
                 "M81": "Power off: use Mainsail", "M401": "BLTouch HS mode", "M355": "Case light"}
BLTOUCH = {10: "pin_down", 90: "pin_up", 120: "self_test", 160: "reset"}
PROBE_AT = re.compile(r"probe at ([-\d.]+),([-\d.]+) is z=([-\d.]+)")
ACCURACY = re.compile(r"probe accuracy results: .*average ([-\d.]+), median [-\d.]+, standard deviation ([-\d.]+)")
SCREW = re.compile(r"^// (.+?) (?:\(base\) )?: x=[-\d.]+, y=[-\d.]+, z=[-\d.]+(?: : adjust (CW|CCW) (\d+):(\d+))?")

class Pending:
    """One line forwarded to Klipper: how many "ok"s it produces and how its replies are rewritten."""
    def __init__(self, oks=1, filt=None, done=None):
        self.oks = oks          # Klipper "ok"s to expect (multi-line translations send several)
        self.filt = filt        # line -> list of lines for the TFT (None = forward unchanged)
        self.done = done        # (had_error) -> extra lines sent before the final "ok"
        self.error = False

class TftBridge:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.tftDevice = config.get('tft_device')
        self.tftBaud = config.getint('tft_baud')
        self.klipperDevice = config.get('klipper_device')
        self.klipperBaud = config.getint('klipper_baud')
        # tft_timeout / klipper_timeout are accepted for compatibility but no longer used
        config.get('tft_timeout', None)
        config.get('klipper_timeout', None)
        self.answerLocal = config.getboolean('answer_status_locally', True)
        self.translate = config.getboolean('translate_buttons', True)
        self.notify = config.getboolean('host_actions', True)
        self.maxFiles = config.getint('onboard_sd_files', 40, minval=1)
        self.nameMap = {}
        self.tftSerial = None
        self.klipperSerial = None
        self.writeLock = threading.Lock()
        self.stopEvent = threading.Event()
        self.threads = []
        self.seenUnknown = set()
        self.pending = collections.deque()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:disconnect", self.handle_disconnect)

    # ---------- lifecycle ----------
    def handle_ready(self):
        self.heaters = self.printer.lookup_object('heaters', None)
        self.sdcard = self.printer.lookup_object('virtual_sdcard', None)
        self.gcodeMove = self.printer.lookup_object('gcode_move', None)
        self.printStats = self.printer.lookup_object('print_stats', None)
        self.pauseResume = self.printer.lookup_object('pause_resume', None)
        self.displayStatus = self.printer.lookup_object('display_status', None)
        self.partFan = self.printer.lookup_object('fan_generic fan0', None)
        self.settings = self.printer.lookup_object('gcode_macro _USER_SETTINGS', None)
        self.toolhead = self.printer.lookup_object('toolhead', None)
        self.extruder = self.printer.lookup_object('extruder', None)
        self.probe = self.printer.lookup_object('probe', None)
        self.bedMesh = self.printer.lookup_object('bed_mesh', None)
        self.configfile = self.printer.lookup_object('configfile', None)
        self.pending = collections.deque()
        try:
            self.tftSerial = serial.Serial(self.tftDevice, self.tftBaud, timeout=READ_TIMEOUT)
            self.klipperSerial = serial.Serial(self.klipperDevice, self.klipperBaud, timeout=READ_TIMEOUT)
        except Exception:
            logging.exception("tftbridge: could not open %s or %s", self.tftDevice, self.klipperDevice)
            self.close_ports()
            return
        self.stopEvent.clear()
        self.threads = [threading.Thread(target=f, name="tftbridge-" + f.__name__)
                        for f in (self.tft2klipper, self.klipper2tft, self.watcher)]
        for t in self.threads:
            t.daemon = True
            t.start()
        logging.info("tftbridge: bridging %s <-> %s", self.tftDevice, self.klipperDevice)

    def handle_disconnect(self):
        self.stopEvent.set()
        for t in self.threads:
            t.join(timeout=2.0)
        self.threads = []
        self.close_ports()
        logging.info("tftbridge: stopped")

    def close_ports(self):
        for s in (self.tftSerial, self.klipperSerial):
            try:
                if s is not None:
                    s.close()
            except Exception:
                pass
        self.tftSerial = self.klipperSerial = None

    # ---------- helpers ----------
    def position_line(self):
        """Marlin M114 format, which the TFT parses whether or not it asked for it."""
        p = self.gcodeMove._get_gcode_position()
        return "X:%.3f Y:%.3f Z:%.3f E:%.3f\n" % tuple(p)

    def to_tft(self, text):
        with self.writeLock:
            self.tftSerial.write(text.encode())

    def to_klipper(self, text):
        self.klipperSerial.write(text.encode())

    def send(self, text, pending):
        """Forward to Klipper and remember how to treat its replies (queued before writing)."""
        self.pending.append(pending)
        self.to_klipper(text)

    def status_reply(self, cmd, words=()):
        """Reply for status polls answered locally, or None to forward to Klipper."""
        now = self.reactor.monotonic()
        has_s = any(w[:1] in ("S", "s") for w in words[1:])
        if cmd == "M105" and self.heaters is not None:
            return "ok %s\n" % (self.heaters._get_temp(now),)
        if cmd == "M27" and self.sdcard is not None:
            f = self.sdcard.current_file
            if f is None:
                return "Not SD printing.\nok\n"
            return "SD printing byte %d/%d\nok\n" % (self.sdcard.file_position, self.sdcard.file_size)
        if cmd == "M114" and self.gcodeMove is not None:
            return self.position_line() + "ok\n"
        if cmd == "M20" and self.sdcard is not None:
            return self.file_list_reply()
        # Bare M220/M221 are queries on Marlin (the TFT's Speed menu sends them), but in Klipper
        # they default to S100 and RESET speed/flow. Answer them here; with S they go to Klipper.
        # Adapted from oterek/klipper-btt-tft-bridge (MIT, Copyright (c) 2026 Piter (garagescunny)),
        # including the "Feedrate: .. FR:" reply that both older and newer TFT firmware parse.
        if cmd == "M220" and not has_s and self.gcodeMove is not None:
            pct = self.gcodeMove.speed_factor * 60. * 100.
            return "echo:Feedrate: %d%% FR:%d%%\nok\n" % (round(pct), round(pct))
        if cmd == "M221" and not has_s and self.gcodeMove is not None:
            return "echo:Flow: %d%%\nok\n" % (round(self.gcodeMove.extrude_factor * 100.),)
        if cmd == "M115":
            version = self.printer.get_start_args().get('software_version', '?')
            return ("FIRMWARE_NAME:Klipper %s FIRMWARE_VERSION:%s MACHINE_TYPE:Ender-3 Pro\n"
                    # Cap:BABYSTEPPING:1 as in oterek/klipper-btt-tft-bridge (MIT), see NOTICE
                    "Cap:SDCARD:1\nCap:AUTOREPORT_SD_STATUS:0\nCap:AUTOREPORT_TEMP:0\nCap:BABYSTEPPING:1\n"
                    "Cap:EEPROM:0\nCap:PROMPT_SUPPORT:0\nCap:AUTOLEVEL:%d\nCap:Z_PROBE:%d\nok\n"
                    % (version, version, self.bedMesh is not None, self.probe is not None))
        if cmd == "M503":
            return self.settings_dump() + "ok\n"
        if cmd == "M92" and len(words) == 1:
            return self.steps_line() + "ok\n"
        if cmd == "M851" and self.probe is not None:
            # query, or the Z the Babystep menu sends on exit: Klipper's z_offset lives in printer.cfg
            return self.probe_line() + "ok\n"
        if cmd == "M206" and len(words) == 1:
            return "M206 X0.00 Y0.00 Z0.00\nok\n"
        if cmd == "M420" and any(w.upper() == "V1" for w in words[1:]):
            return self.mesh_report() + "ok\n"
        if cmd == "M401" and any(w.upper() == "H" for w in words[1:]):
            return "ok\n"                  # no HS mode query; the TFT then stows the pin with M280 S90
        if cmd in READ_ONLY and not (cmd == "M92"):
            return "echo:%s is read only with Klipper: change it in printer.cfg\nok\n" % (cmd,)
        if cmd in NOT_SUPPORTED:
            return "echo:%s is not supported with Klipper\nok\n" % (NOT_SUPPORTED[cmd],)
        return None

    # ---------- Marlin-style reports built from Klipper's state ----------
    def cfg(self, section, option, default=None):
        try:
            return self.configfile.get_status(None)['settings'][section].get(option, default)
        except Exception:
            return default

    def axis_steps(self):
        steps = {}
        if self.toolhead is not None:
            for s in self.toolhead.get_kinematics().get_steppers():
                name = s.get_name()
                if name in ("stepper_x", "stepper_y", "stepper_z"):
                    steps[name[-1].upper()] = 1. / s.get_step_dist()
        if self.extruder is not None and self.extruder.extruder_stepper is not None:
            steps["E"] = 1. / self.extruder.extruder_stepper.stepper.get_step_dist()
        return steps

    def steps_line(self):
        st = self.axis_steps()
        return "M92 " + " ".join("%s%.2f" % (a, st[a]) for a in "XYZE" if a in st) + "\n"

    def probe_line(self):
        x, y, z = self.probe.get_offsets()
        return "M851 X%.2f Y%.2f Z%.2f\n" % (x, y, -z)     # Marlin: nozzle below the trigger point is negative

    def settings_dump(self):
        """M503 replay lines the TFT's Parameter settings menu parses (values only, read only)."""
        out = [self.steps_line()]
        if self.toolhead is not None:
            th = self.toolhead.get_status(self.reactor.monotonic())
            v, a = th['max_velocity'], th['max_accel']
            zv = float(self.cfg('printer', 'max_z_velocity', v))
            za = float(self.cfg('printer', 'max_z_accel', a))
            ev = getattr(self.extruder, 'max_e_velocity', 0.) if self.extruder else 0.
            ea = getattr(self.extruder, 'max_e_accel', 0.) if self.extruder else 0.
            out.append("M203 X%.2f Y%.2f Z%.2f E%.2f\n" % (v, v, zv, ev))
            out.append("M201 X%d Y%d Z%d E%d\n" % (a, a, za, ea))
            out.append("M204 P%.2f R%.2f T%.2f\n" % (a, ea or a, a))
        for cmd, sec in (("M301", "extruder"), ("M304", "heater_bed")):
            kp, ki, kd = (self.cfg(sec, k) for k in ("pid_kp", "pid_ki", "pid_kd"))
            if kp is not None:
                out.append("%s P%.2f I%.2f D%.2f\n" % (cmd, float(kp), float(ki), float(kd)))
        if self.probe is not None:
            out.append(self.probe_line())
        if self.extruder is not None:
            pa = self.extruder.get_status(self.reactor.monotonic()).get('pressure_advance')
            if pa is not None:
                out.append("M900 K%.4f\n" % (pa,))
        cur = {}
        for name in ("stepper_x", "stepper_y", "stepper_z", "stepper_z1", "extruder"):
            c = self.cfg("tmc2209 " + name, "run_current")
            if c is not None:
                cur[name] = int(round(float(c) * 1000))
        if cur:
            xyz = " ".join("%s%d" % (n[-1].upper(), cur[n]) for n in ("stepper_x", "stepper_y", "stepper_z") if n in cur)
            out.append("M906 %s\n" % (xyz,))
            if "stepper_z1" in cur:
                out.append("M906 I1 Z%d\n" % (cur["stepper_z1"],))
            if "extruder" in cur:
                out.append("M906 T0 E%d\n" % (cur["extruder"],))
        return "".join(out)

    def mesh_report(self):
        """Klipper's active mesh as Marlin's "Bilinear Leveling Grid" (the Mesh editor parses it)."""
        if self.bedMesh is None:
            return "Invalid mesh\n"
        m = self.bedMesh.get_status(self.reactor.monotonic()).get('probed_matrix') or [[]]
        if not m or not m[0]:
            return "Invalid mesh\n"
        cols = len(m[0])
        lines = ["Bilinear Leveling Grid:\n", "      " + "".join("%7d" % i for i in range(cols)) + "\n"]
        for j, row in enumerate(m):
            lines.append("%2d " % j + " ".join("%+.3f" % z for z in row) + "\n")
        return "".join(lines)

    def file_list_reply(self):
        """Newest gcode files first, as many as fit the TFT's buffer."""
        top = self.sdcard.sdcard_dirname
        found = []
        for root, dirs, names in os.walk(top, followlinks=True):
            dirs[:] = [d for d in dirs if not d.startswith('.')]
            for n in names:
                if n.startswith('.') or n.rsplit('.', 1)[-1].lower() not in GCODE_EXTS:
                    continue
                full = os.path.join(root, n)
                try:
                    st = os.stat(full)
                except OSError:
                    continue
                found.append((st.st_mtime, full[len(top) + 1:], st.st_size))
        found.sort(reverse=True)
        head, tail = "Begin file list\n", "End file list\nok\n"
        total = len(head) + len(tail)
        lines, names = [], {}
        for mtime, rel, size in found[:self.maxFiles]:
            shown = rel.replace(' ', '_')
            line = "%s %d\n" % (shown, size)
            if total + len(line) > M20_MAX_BYTES:
                break
            names[shown.lower()] = rel
            lines.append(line)
            total += len(line)
        self.nameMap = names
        return head + "".join(lines) + tail

    def translated(self, cmd, line):
        """Returns the text for Klipper, or (text, Pending) when replies must be rewritten, or None."""
        words = line.strip().split()
        args = {}
        for w in words[1:]:
            if w[:1].isalpha():
                args[w[0].upper()] = w[1:]
        if cmd == "M303":           # PID autotune from Tuning -> PID
            heater = "heater_bed" if args.get("E", "0").startswith("-") else "extruder"
            target = args.get("S", "200")
            done = lambda err: ["PID Autotune %s\n" % ("failed" if err else "finished")]
            return ("PID_CALIBRATE HEATER=%s TARGET=%s\n" % (heater, target), Pending(done=done))
        if cmd == "M92":
            # Tune extruder (and the Parameter settings editor) set E steps/mm: apply it live as a
            # rotation_distance; X/Y/Z are refused
            if "E" in args and self.extruder is not None and not any(a in args for a in "XYZ"):
                try:
                    new_steps = float(args["E"])
                    rd, spr = self.extruder.extruder_stepper.stepper.get_rotation_distance()
                    new_rd = spr / new_steps
                except (ValueError, ZeroDivisionError):
                    return None
                note = lambda err: [] if err else [
                    "echo:rotation_distance %.4f applied (live). Put it in the [extruder] config to keep it\n" % (new_rd,)]
                return ("SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=extruder DISTANCE=%.4f\n" % (new_rd,),
                        Pending(done=note))
            self.to_tft("echo:M92 X/Y/Z is read only with Klipper: change it in printer.cfg\n")
            return None
        if cmd == "G29":            # ABL start (G28 comes first from the TFT)
            return "BED_MESH_CALIBRATE\n"
        if cmd == "M118" and "ABL Completed" in line:
            # queued behind G29 in Klipper, so the TFT hears it when the mesh is really done
            filt = lambda l: ["ABL Completed\n"] if "ABL Completed" in l else [l]
            return ("M118 ABL Completed\n", Pending(filt=filt))
        if cmd == "G30":            # Level corner / probe offset: probe at a bed position
            return self.probe_at(args)
        if cmd == "M48":
            return ("PROBE_ACCURACY\n", Pending(filt=self.accuracy_filter))
        if cmd == "PROBE_ACCURACY":
            return (line, Pending(filt=self.accuracy_filter))
        if cmd == "M280":
            try:
                c = BLTOUCH.get(int(float(args.get("S", "-1"))))
            except ValueError:
                c = None
            return "BLTOUCH_DEBUG COMMAND=%s\n" % (c,) if c else None
        if cmd == "G34":            # Z align
            return "Z_TILT_ADJUST\n"
        if cmd == "M150":
            r, g, b = (float(args.get(k, "0") or 0) for k in ("R", "U", "B"))
            p = float(args.get("P", "255") or 255) / 255.
            return "_TFT_LED_COLOR R=%.3f G=%.3f B=%.3f\n" % (r / 255. * p, g / 255. * p, b / 255. * p)
        if cmd == "SCREWS_TILT_CALCULATE":
            return (line, self.screws_pending())
        if cmd == "M23":
            shown = line.strip().split(None, 1)[1].strip() if len(line.split()) > 1 else ""
            real = self.nameMap.get(shown.lstrip('/').lower())
            if real is not None:
                return "M23 %s\n" % (real,)
            return line
        if cmd == "M25":
            return "PAUSE\n"
        if cmd == "M24":
            if self.pauseResume is not None and self.pauseResume.is_paused:
                return "RESUME\n"
            return line                 # start a file selected on the TFT (M23 + M24)
        if cmd == "M524":
            return "CANCEL_PRINT\n"
        if cmd == "M290":
            # Babystep from the TFT: live Z adjust of the gcode offset
            for w in line.strip().split()[1:]:
                if w[:1] in ("Z", "z"):
                    try:
                        return "SET_GCODE_OFFSET Z_ADJUST=%.3f MOVE=1\n" % (float(w[1:]),)
                    except ValueError:
                        break
            return None
        return line

    def probe_at(self, args):
        """Marlin "G30 X Y" probes with the probe over X,Y and reports "Bed X: Y: Z:"."""
        if self.probe is None:
            return None
        try:
            x, y = float(args["X"]), float(args["Y"])
        except (KeyError, ValueError):
            return None
        xo, yo, zo = self.probe.get_offsets()
        state = {}
        def filt(l):
            m = PROBE_AT.search(l)
            if m:
                state['z'] = float(m.group(3))
            return [l]
        def done(err):
            if err or 'z' not in state:
                return ["Z Probe Past Bed\n"]        # ends the TFT's wait for this point
            return ["Bed X: %.3f Y: %.3f Z: %.3f\n" % (x, y, state['z'])]
        script = "G90\nG1 Z10 F600\nG1 X%.3f Y%.3f F6000\nPROBE\n" % (x - xo, y - yo)
        return (script, Pending(oks=4, filt=filt, done=done))

    def accuracy_filter(self, l):
        m = ACCURACY.search(l)
        if m:
            return [l, "Mean: %s\n" % (m.group(1),), "Standard Deviation: %s\n" % (m.group(2),)]
        return [l]

    def screws_pending(self):
        found = []
        def filt(l):
            m = SCREW.match(l.strip())
            if m:
                name = "".join(w[0].upper() for w in m.group(1).split()[:2])   # "Rear left screw" -> RL
                if m.group(2) is None:
                    found.append("%s base" % (name,))
                else:
                    found.append("%s %s %d:%02d" % (name, m.group(2), int(m.group(3)), int(m.group(4))))
            return [l]
        def done(err):
            if err or not found:
                return []
            return ["//action:notification Screws: %s\n" % (", ".join(found),)]
        return Pending(filt=filt, done=done)

    # ---------- threads ----------
    def tft2klipper(self):
        while not self.stopEvent.is_set():
            try:
                raw = self.tftSerial.readline()
            except Exception:
                logging.exception("tftbridge: TFT read failed")
                time.sleep(1.0)
                continue
            if not raw:
                continue
            line = raw.decode('ascii', 'replace')
            words = line.strip().split()
            # skip a Marlin-style line number ("N123 M105*45"); keep checksummed lines untouched
            if words and words[0][:1] in ("N", "n") and words[0][1:].isdigit():
                words = words[1:]
            cmd = words[0].upper() if words else ""
            try:
                if "*" in line:
                    self.send(line, Pending())
                    continue
                if self.answerLocal:
                    reply = self.status_reply(cmd, words)
                    if reply is not None:
                        self.to_tft(reply)
                        continue
                if cmd in SWALLOW:
                    if cmd not in self.seenUnknown:
                        self.seenUnknown.add(cmd)
                        logging.info("tftbridge: acknowledging unsupported TFT command %s", cmd)
                    self.to_tft("ok\n")
                    continue
                out = self.translated(cmd, line) if self.translate else line
                if out is None:
                    self.to_tft("ok\n")     # nothing for Klipper to do (e.g. M290 without Z)
                elif isinstance(out, tuple):
                    self.send(*out)
                else:
                    self.send(out, Pending())
            except Exception:
                logging.exception("tftbridge: forwarding %r failed", line)

    def klipper2tft(self):
        while not self.stopEvent.is_set():
            try:
                raw = self.klipperSerial.readline()
                if raw:
                    self.from_klipper(raw.decode('ascii', 'replace'))
            except Exception:
                logging.exception("tftbridge: Klipper->TFT forward failed")
                time.sleep(1.0)

    def from_klipper(self, line):
        """Pass one Klipper line to the TFT, rewriting replies of translated commands."""
        p = self.pending[0] if self.pending else None
        if p is None:
            self.to_tft(line)
            return
        text = line.strip()
        if text == "ok" or text.startswith("ok "):
            p.oks -= 1
            if p.oks > 0:
                return                          # inner line of a multi-line translation
            self.pending.popleft()
            out = []
            if p.done is not None:
                try:
                    out = p.done(p.error)
                except Exception:
                    logging.exception("tftbridge: reply rewrite failed")
            self.to_tft("".join(out) + line)
            return
        if text.startswith("!!"):
            p.error = True
        out = [line]
        if p.filt is not None:
            try:
                out = p.filt(line)
            except Exception:
                logging.exception("tftbridge: reply rewrite failed")
        if out:
            self.to_tft("".join(out))

    def fan_report(self):
        """Part fan speed on the slicer scale (0-255), as the TFT's F0 shows it. The M106 macro maps
        slicer 100 % to part_fan_max real power, so report real / part_fan_max (capped at 255)."""
        real = self.partFan.fan.last_req_value
        if real <= 0:
            return 0
        top = 0.45
        if self.settings is not None:
            top = float(self.settings.variables.get('part_fan_max', top)) or top
        return min(255, int(round(real / top * 255)))

    def watcher(self):
        """Push host actions for state changes the TFT can't see by itself."""
        lastState = None
        lastMsg = None
        lastFan = None
        posTick = 0
        while not self.stopEvent.wait(WATCH_INTERVAL):
            if not self.notify:
                continue
            try:
                state = self.printStats.state if self.printStats is not None else None
                if state != lastState and lastState is not None:
                    if state == "paused":
                        self.to_tft("//action:paused\n")
                    elif state == "printing" and lastState == "paused":
                        # The TFT stops its print timer on "paused" and only restarts it on "resumed"
                        self.to_tft("//action:resumed\n")
                    elif state in ("cancelled", "error"):
                        self.to_tft("//action:cancel\n")
                lastState = state
                # The print screen shows Z from position reports; push one every POS_INTERVAL while a
                # print is running instead of relying on the TFT's own M114 polling
                # Temperatures too, at all times: the TFT stops polling M105 while it waits for an "ok"
                # (e.g. behind a long START_PRINT), which froze its readings during heat-up and soak
                posTick += 1
                if posTick >= POS_EVERY:
                    posTick = 0
                    if self.heaters is not None:
                        self.to_tft("%s\n" % (self.heaters._get_temp(self.reactor.monotonic()),))
                    if state in ("printing", "paused") and self.gcodeMove is not None:
                        self.to_tft(self.position_line())
                msg = self.displayStatus.message if self.displayStatus is not None else None
                if msg != lastMsg:
                    if msg:
                        self.to_tft("//action:notification %s\n" % (msg,))
                    lastMsg = msg
                if self.partFan is not None:
                    fan = self.fan_report()
                    if fan != lastFan:
                        # The TFT updates its F0 display from "M106 P0 S<n>" lines (parseACK.c)
                        self.to_tft("M106 P0 S%d\n" % (fan,))
                        lastFan = fan
            except Exception:
                logging.exception("tftbridge: watcher failed")

def load_config(config):
    return TftBridge(config)
