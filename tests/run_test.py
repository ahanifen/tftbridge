#!/usr/bin/env python3
"""Local test harness for tftbridge.py: fake Klipper objects, real ptys for the TFT and Klipper ends.

Usage: python3 tests/run_test.py      (needs pyserial; runs entirely on the workstation)
"""
import os, pty, select, sys, threading, time, tty, types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import tftbridge as tb

# ---------- fakes ----------
class Reactor:
    def monotonic(self):
        return time.monotonic()

class Stepper:
    def __init__(self, name, step_dist, rd=40., spr=3200):
        self.name, self.sd, self.rd, self.spr = name, step_dist, rd, spr
    def get_name(self):
        return self.name
    def get_step_dist(self):
        return self.sd
    def get_rotation_distance(self):
        return self.rd, self.spr

class Kin:
    def get_steppers(self):
        return [Stepper("stepper_x", 1 / 80.), Stepper("stepper_y", 1 / 80.), Stepper("stepper_z", 1 / 400.),
                Stepper("stepper_z1", 1 / 400.)]

class Toolhead:
    def get_kinematics(self):
        return Kin()
    def get_status(self, t):
        return {'max_velocity': 300., 'max_accel': 7500.}

class Extruder:
    max_e_velocity, max_e_accel = 120., 5000.
    def __init__(self):
        self.extruder_stepper = types.SimpleNamespace(stepper=Stepper("extruder", 7.71 / 3200, 7.71, 3200))
    def get_status(self, t):
        return {'pressure_advance': 0.04}

class Probe:
    def get_offsets(self):
        return (-45., -7., 3.165)

class BedMesh:
    def get_status(self, t):
        return {'probed_matrix': [[0.01, -0.02, 0.03], [0.0, 0.05, -0.01]]}

class ConfigFile:
    def get_status(self, t):
        return {'settings': {
            'printer': {'max_z_velocity': 15., 'max_z_accel': 300.},
            'extruder': {'pid_kp': 21.5, 'pid_ki': 1.1, 'pid_kd': 99.2},
            'heater_bed': {'pid_kp': 70.1, 'pid_ki': 1.5, 'pid_kd': 800.0},
            'tmc2209 stepper_x': {'run_current': 0.8}, 'tmc2209 stepper_y': {'run_current': 0.8},
            'tmc2209 stepper_z': {'run_current': 0.6}, 'tmc2209 stepper_z1': {'run_current': 0.6},
            'tmc2209 extruder': {'run_current': 0.6}}}

class Heaters:
    def _get_temp(self, t):
        return "B:24.0 /0.0 T0:25.0 /0.0"   # Klipper heaters._get_temp order (sorted ids)

class Printer:
    def __init__(self, objs):
        self.objs = objs
    def get_reactor(self):
        return Reactor()
    def lookup_object(self, name, default=None):
        return self.objs.get(name, default)
    def register_event_handler(self, ev, cb):
        pass
    def get_start_args(self):
        return {'software_version': 'v0.13.0-test'}

class Config:
    def __init__(self, printer, values):
        self.p, self.v = printer, values
    def get_printer(self):
        return self.p
    def get(self, k, default=None):
        return self.v.get(k, default)
    def getint(self, k, default=None, minval=None):
        return int(self.v.get(k, default))
    def getboolean(self, k, default=None):
        return self.v.get(k, default)

# ---------- harness ----------
def make_pty():
    m, s = pty.openpty()
    tty.setraw(m)
    tty.setraw(s)
    return m, os.ttyname(s)

class End:
    """Our side of a pty: write lines, read lines with a timeout."""
    def __init__(self, fd):
        self.fd, self.buf = fd, b""
    def write(self, text):
        os.write(self.fd, text.encode())
    def lines(self, timeout=1.0, until=None):
        out, end = [], time.time() + timeout
        while time.time() < end:
            while b"\n" in self.buf:
                l, self.buf = self.buf.split(b"\n", 1)
                out.append(l.decode())
                if until is not None and until(out[-1]):
                    return out
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if r:
                self.buf += os.read(self.fd, 4096)
        return out

FAILS = []
def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + ("" if cond else "  -> " + str(detail)))
    if not cond:
        FAILS.append(name)

def main():
    tft_m, tft_s = make_pty()
    kl_m, kl_s = make_pty()
    objs = {'heaters': Heaters(), 'toolhead': Toolhead(), 'extruder': Extruder(), 'probe': Probe(),
            'bed_mesh': BedMesh(), 'configfile': ConfigFile(),
            'gcode_move': types.SimpleNamespace(speed_factor=1 / 60., extrude_factor=1.0,
                                                _get_gcode_position=lambda: [1, 2, 3, 4]),
            'pause_resume': types.SimpleNamespace(is_paused=False)}
    printer = Printer(objs)
    b = tb.TftBridge(Config(printer, {'tft_device': tft_s, 'tft_baud': 250000, 'klipper_device': kl_s,
                                      'klipper_baud': 250000, 'host_actions': False}))
    b.handle_ready()
    tft, kl = End(tft_m), End(kl_m)
    ok = lambda l: l == "ok" or l.startswith("ok ")

    def tft_cmd(line, klipper_reply=None, expect_lines=1, timeout=1.5):
        tft.write(line + "\n")
        sent = kl.lines(timeout=0.6, until=lambda l: False) if klipper_reply is not None or expect_lines else []
        if klipper_reply is not None:
            kl.write(klipper_reply)
        got = tft.lines(timeout=timeout, until=ok)
        return sent, got

    # --- local answers ---
    s, g = tft_cmd("M115", expect_lines=0)
    check("M115 has AUTOLEVEL and Z_PROBE", "Cap:AUTOLEVEL:1" in g and "Cap:Z_PROBE:1" in g and "Cap:EEPROM:0" in g, g)
    check("M115 not forwarded", s == [], s)
    s, g = tft_cmd("M503 S0", expect_lines=0)
    check("M503 dump steps", "M92 X80.00 Y80.00 Z400.00 E415.05" in g, g)
    check("M503 dump feed/accel", "M203 X300.00 Y300.00 Z15.00 E120.00" in g and "M201 X7500 Y7500 Z300 E5000" in g, g)
    check("M503 dump PID", "M301 P21.50 I1.10 D99.20" in g and "M304 P70.10 I1.50 D800.00" in g, g)
    check("M503 dump probe/PA", "M851 X-45.00 Y-7.00 Z-3.17" in g and "M900 K0.0400" in g, g)
    check("M503 dump currents", "M906 X800 Y800 Z600" in g and "M906 I1 Z600" in g and "M906 T0 E600" in g, g)
    s, g = tft_cmd("M92", expect_lines=0)
    check("M92 query", g[:1] == ["M92 X80.00 Y80.00 Z400.00 E415.05"] and ok(g[-1]), g)
    s, g = tft_cmd("M851 Z-3.05", expect_lines=0)
    check("M851 set answered locally", s == [] and g[0].startswith("M851 X-45.00"), (s, g))
    s, g = tft_cmd("M206", expect_lines=0)
    check("M206 query", g[0] == "M206 X0.00 Y0.00 Z0.00", g)
    s, g = tft_cmd("M206 Z1", expect_lines=0)
    check("M206 set refused", s == [] and "read only" in g[0], (s, g))
    s, g = tft_cmd("M301 P1 I2 D3", expect_lines=0)
    check("M301 set refused", s == [] and "read only" in g[0], (s, g))
    s, g = tft_cmd("M92 X100", expect_lines=0)
    check("M92 X refused", s == [] and "read only" in g[0] and ok(g[-1]), (s, g))
    s, g = tft_cmd("G26 H200 B60 R99", expect_lines=0)
    check("G26 not supported", s == [] and "not supported" in g[0], (s, g))
    s, g = tft_cmd("M81", expect_lines=0)
    check("M81 not supported", s == [] and "Mainsail" in g[0], (s, g))
    s, g = tft_cmd("M420 V1 T1", expect_lines=0)
    check("Mesh report", g[0] == "Bilinear Leveling Grid:" and g[2].startswith(" 0 +0.010 -0.020 +0.030") and g[3].startswith(" 1 +0.000"), g)
    s, g = tft_cmd("M401 H", expect_lines=0)
    check("M401 H acked", s == [] and g == ["ok"], (s, g))
    s, g = tft_cmd("M105", expect_lines=0)
    check("M105 local", g == ["ok B:24.0 /0.0 T0:25.0 /0.0"], g)

    # --- translations with rewritten replies ---
    s, g = tft_cmd("M303 E0 C8 U1 S220", "// PID parameters: pid_Kp=22.0\nok\n")
    check("PID forwarded", s == ["PID_CALIBRATE HEATER=extruder TARGET=220"], s)
    check("PID finished", "PID Autotune finished" in g and ok(g[-1]), g)
    s, g = tft_cmd("M303 E-1 C8 U1 S60", "!! Heater heater_bed not heating at expected rate\nok\n")
    check("bed PID failed", s == ["PID_CALIBRATE HEATER=heater_bed TARGET=60"] and "PID Autotune failed" in g, (s, g))
    s, g = tft_cmd("M92 T0 E400", "ok\n")
    check("E steps -> rotation distance", s == ["SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=extruder DISTANCE=8.0000"], s)
    check("E steps note", any("rotation_distance 8.0000" in l for l in g), g)
    s, g = tft_cmd("G29", "ok\n")
    check("G29 -> BED_MESH_CALIBRATE", s == ["BED_MESH_CALIBRATE"], s)
    s, g = tft_cmd("M118 P0 ABL Completed", "echo: ABL Completed\nok\n")
    check("ABL Completed rewritten", s == ["M118 ABL Completed"] and g[0] == "ABL Completed", (s, g))
    tft.write("G30 E1 X30 Y30\n")
    s = kl.lines(timeout=0.6)
    check("G30 script", s == ["G90", "G1 Z10 F600", "G1 X75.000 Y37.000 F6000", "PROBE"], s)
    kl.write("ok\nok\nok\n// probe at 75.000,37.000 is z=3.152500\nok\n")
    g = tft.lines(timeout=1.0)
    check("G30 one ok + Bed line", g.count("ok") == 1 and "Bed X: 30.000 Y: 30.000 Z: 3.152" in g, g)
    tft.write("G30 E1 X300 Y30\n")
    kl.lines(timeout=0.6)
    kl.write("ok\nok\n!! Move out of range: 345.000 23.000 10.000 [0.000]\nok\n!! Probe error\nok\n")
    g = tft.lines(timeout=1.0)
    check("G30 error -> Z Probe Past Bed", "Z Probe Past Bed" in g and g.count("ok") == 1, g)
    s, g = tft_cmd("M48", "// probe accuracy results: maximum 3.170000, minimum 3.160000, range 0.010000, average 3.165000, median 3.165000, standard deviation 0.003000\nok\n")
    check("M48 -> PROBE_ACCURACY", s == ["PROBE_ACCURACY"], s)
    check("Mean/StdDev lines", "Mean: 3.165000" in g and "Standard Deviation: 0.003000" in g, g)
    s, g = tft_cmd("M280 P0 S120", "ok\n")
    check("BLTouch self test", s == ["BLTOUCH_DEBUG COMMAND=self_test"], s)
    s, g = tft_cmd("G34", "ok\n")
    check("G34 -> Z_TILT_ADJUST", s == ["Z_TILT_ADJUST"], s)
    s, g = tft_cmd("M150 R255 U128 B0 W255 P255 I0", "ok\n")
    check("M150 -> _TFT_LED_COLOR", s == ["_TFT_LED_COLOR R=1.000 G=0.502 B=0.000"], s)
    s, g = tft_cmd("SCREWS_TILT_CALCULATE", "// front left screw (base) : x=74.0, y=37.0, z=3.15000\n"
                   "// rear left screw : x=74.0, y=207.0, z=3.10000 : adjust CW 00:15\n"
                   "// front right screw : x=244.0, y=37.0, z=3.20000 : adjust CCW 01:05\nok\n")
    check("screws summary", any(l.startswith("//action:notification Screws: FL base, RL CW 0:15, FR CCW 1:05") for l in g), g)
    s, g = tft_cmd("G28", "ok\n")
    check("plain forward", s == ["G28"] and g == ["ok"], (s, g))
    # async line from another client while nothing is pending
    kl.write("// hello from Mainsail\n")
    g = tft.lines(timeout=0.5)
    check("async passthrough", g == ["// hello from Mainsail"], g)

    # --- existing behaviour ---
    s, g = tft_cmd("M25", "ok\n")
    check("M25 -> PAUSE", s == ["PAUSE"], s)
    s, g = tft_cmd("M524", "ok\n")
    check("M524 -> CANCEL_PRINT", s == ["CANCEL_PRINT"], s)
    s, g = tft_cmd("M290 Z0.05", "ok\n")
    check("M290 babystep", s == ["SET_GCODE_OFFSET Z_ADJUST=0.050 MOVE=1"], s)
    s, g = tft_cmd("M220", expect_lines=0)
    check("bare M220 local", s == [] and "FR:100%" in g[0], (s, g))
    s, g = tft_cmd("M211 S0", expect_lines=0)
    check("M211 swallowed", s == [] and g == ["ok"], (s, g))

    # --- watcher: resume push and position reports while printing ---
    ps = types.SimpleNamespace(state="printing")
    b.printStats, b.displayStatus, b.partFan = ps, None, None
    b.notify = True
    g = tft.lines(timeout=2.6)
    check("position pushed while printing", "X:1.000 Y:2.000 Z:3.000 E:4.000" in g, g)
    ps.state = "paused"
    g = tft.lines(timeout=1.5)
    check("paused pushed", "//action:paused" in g, g)
    ps.state = "printing"
    g = tft.lines(timeout=1.5)
    check("resumed pushed", "//action:resumed" in g, g)
    ps.state = "standby"
    tft.lines(timeout=1.2)
    g = tft.lines(timeout=2.5)
    check("no position reports when idle", not any(l.startswith("X:") for l in g), g)
    check("temperatures pushed when idle", any("T0:" in l and "B:" in l and not l.startswith("ok") for l in g), g)
    b.notify = False

    b.handle_disconnect()
    print("\n%d failed" % len(FAILS) if FAILS else "\nall passed")
    sys.exit(1 if FAILS else 0)

if __name__ == "__main__":
    main()
