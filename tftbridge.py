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
#   - M20 (file list) is answered here with the newest files only: the TFT's receive buffer is
#     5000 bytes (V27 CMD_MAX_REV) and Klipper's full list (~1000 files) overflowed both it and the
#     Klipper pty (lost "End file list" -> TFT stuck loading). The TFT also cuts names at the first
#     space, so spaces are shown as "_" and M23 maps the shown name back to the real file.
#
import logging, os, threading, time
import serial

M20_MAX_BYTES = 4200    # stay well under the TFT's 5000-byte buffer
GCODE_EXTS = ("gcode", "g", "gco")

READ_TIMEOUT = 0.2      # seconds; lets the threads notice a stop request
WATCH_INTERVAL = 1.0    # seconds between print-state checks for host actions
# Marlin commands the TFT sends that Klipper doesn't implement: acknowledge, do nothing
SWALLOW = {"M92", "M503", "M150", "M108", "M876", "M155", "M211", "M420"}

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
    def to_tft(self, text):
        with self.writeLock:
            self.tftSerial.write(text.encode())

    def to_klipper(self, text):
        self.klipperSerial.write(text.encode())

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
            p = self.gcodeMove._get_gcode_position()
            return "X:%.3f Y:%.3f Z:%.3f E:%.3f\nok\n" % tuple(p)
        if cmd == "M20" and self.sdcard is not None:
            return self.file_list_reply()
        # Bare M220/M221 are queries on Marlin (the TFT's Speed menu sends them), but in Klipper
        # they default to S100 and RESET speed/flow. Answer them here; with S they go to Klipper.
        if cmd == "M220" and not has_s and self.gcodeMove is not None:
            pct = self.gcodeMove.speed_factor * 60. * 100.
            return "echo:Feedrate: %d%% FR:%d%%\nok\n" % (round(pct), round(pct))
        if cmd == "M221" and not has_s and self.gcodeMove is not None:
            return "echo:Flow: %d%%\nok\n" % (round(self.gcodeMove.extrude_factor * 100.),)
        if cmd == "M115":
            version = self.printer.get_start_args().get('software_version', '?')
            return ("FIRMWARE_NAME:Klipper %s FIRMWARE_VERSION:%s MACHINE_TYPE:Ender-3 Pro\n"
                    "Cap:SDCARD:1\nCap:AUTOREPORT_SD_STATUS:0\nCap:AUTOREPORT_TEMP:0\nCap:BABYSTEPPING:1\n"
                    "Cap:EEPROM:0\nCap:PROMPT_SUPPORT:0\nok\n" % (version, version))
        return None

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
                    self.to_klipper(line)
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
                else:
                    self.to_klipper(out)
            except Exception:
                logging.exception("tftbridge: forwarding %r failed", line)

    def klipper2tft(self):
        while not self.stopEvent.is_set():
            try:
                raw = self.klipperSerial.readline()
                if raw:
                    with self.writeLock:
                        self.tftSerial.write(raw)
            except Exception:
                logging.exception("tftbridge: Klipper->TFT forward failed")
                time.sleep(1.0)

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
        while not self.stopEvent.wait(WATCH_INTERVAL):
            if not self.notify:
                continue
            try:
                state = self.printStats.state if self.printStats is not None else None
                if state != lastState and lastState is not None:
                    if state == "paused":
                        self.to_tft("//action:paused\n")
                    elif state in ("cancelled", "error"):
                        self.to_tft("//action:cancel\n")
                lastState = state
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
