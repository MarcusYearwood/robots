"""SO-101 follower daemon. Sole owner of the serial port; applies safety.py to every command.
Clients (armctl.py) talk JSON-lines over a unix socket. Run:  python armd.py   (foreground; logs to stdout)
"""
import collections, json, os, signal, socket, sys, threading, time, traceback
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
import safety as S

class Arm:
    def __init__(self):
        cal = {k: MotorCalibration(**v) for k, v in S.load_cal().items()}
        motors = {n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.RANGE_M100_100)
                  for i, n in enumerate(S.JOINTS)}
        self.bus = FeetechMotorsBus(port=S.PORT, motors=motors, calibration=cal)
        self.lock = threading.Lock()
        self.setpoint = {}      # what we are commanding right now
        self.target = {}        # where we are heading
        self.speeds = {j: S.DEFAULT_SPEED for j in S.JOINTS}   # per-joint units/s
        self.trace = collections.deque(maxlen=6000)   # (t, present, setpoint, load|None)
        self.telemetry = False
        self.trim = {j: 0.0 for j in S.JOINTS}   # integral sag compensation (gravity joints only), added to goal
        self.tick_ms = 0.0
        self.torque = False
        self.present = {}
        self.health = {}
        self.fault = None
        self.parking = False
        self.last_cmd = time.time()
        self.stall_since = {}
        self.stop = False

    # ---- lifecycle
    def start(self):
        self.bus.connect()
        self.present = self.bus.sync_read("Present_Position")
        self.setpoint = dict(self.present); self.target = dict(self.present)
        self._read_health(); self._check_health()
        print("connected; torque OFF; present:", {k: round(v, 1) for k, v in self.present.items()}, flush=True)

    def _enable(self):
        """Torque on without a jump: goal := present first."""
        self.present = self.bus.sync_read("Present_Position")
        self.setpoint = dict(self.present); self.target = dict(self.present)
        self.bus.sync_write("Goal_Position", self.present)
        for m in S.JOINTS:
            self.bus.write("Acceleration", m, S.SERVO_ACCEL)
            self.bus.write("Goal_Velocity", m, S.SERVO_GOAL_VELOCITY)
            if m in S.P_GAIN: self.bus.write("P_Coefficient", m, S.P_GAIN[m])
            self.bus.write("Torque_Limit", m, S.GRIPPER_TORQUE_LIMIT if m == "gripper" else S.SERVO_TORQUE_LIMIT)
        self.bus.enable_torque()
        self.torque = True
        print("torque ON", flush=True)

    def _disable(self):
        self.bus.disable_torque(); self.torque = False
        print("torque OFF", flush=True)

    # ---- health
    def _read_health(self):
        r = lambda reg: self.bus.sync_read(reg, normalize=False)
        self.health = {"temp": r("Present_Temperature"), "volt": {k: v / 10 for k, v in r("Present_Voltage").items()},
                       "load": r("Present_Load")}

    def _check_health(self):
        h = self.health
        hot = [k for k, v in h["temp"].items() if v >= S.TEMP_STOP_C]
        bad_v = [k for k, v in h["volt"].items() if not S.VOLT_MIN <= v <= S.VOLT_MAX]
        if hot: self._fault(f"overtemp {hot}")
        if bad_v: self._fault(f"bad voltage {bad_v} {h['volt']}")

    def _fault(self, msg):
        if self.fault is None:
            print("FAULT:", msg, flush=True)
        self.fault = msg
        self.target = dict(self.setpoint)       # freeze, then park
        if self.torque and not self.parking:
            self.park()

    # ---- commands (called under lock)
    def goto(self, targets, speed=None, duration=None):
        if self.fault: raise RuntimeError(f"fault latched: {self.fault} (send 'clear' after inspecting)")
        if S.ESTOP_FILE.exists(): raise RuntimeError("ESTOP file present")
        t = S.validate_targets(targets)
        if not self.torque: self._enable()
        self.parking = False
        self.target.update(t)
        if duration:   # synchronized arrival: every moving joint finishes in `duration` s (still capped at MAX_SPEED)
            for j in t:
                self.speeds[j] = min(abs(self.target[j] - self.setpoint[j]) / max(float(duration), 0.05), S.MAX_SPEED)
        else:
            sp = min(float(speed) if speed else S.DEFAULT_SPEED, S.MAX_SPEED)
            for j in t: self.speeds[j] = sp
        for j in t: self.trim[j] = 0.0
        self.last_cmd = time.time()
        return {j: self.target[j] for j in t}

    def park(self):
        self.parking = True
        self.target = dict(S.REST)
        self.speeds = {j: 25.0 for j in S.JOINTS}
        print("parking", flush=True)

    def hold(self):
        self.target = dict(self.setpoint); self.parking = False

    def at_target(self, tol=1.5):
        return all(abs(self.present[j] - self.target[j]) < tol for j in S.JOINTS)

    # ---- control tick
    def tick(self, dt, n):
        t0 = time.perf_counter()
        self._tick(dt, n)
        self.tick_ms = 0.9 * self.tick_ms + 0.1 * (time.perf_counter() - t0) * 1e3

    def _tick(self, dt, n):
        self.present = self.bus.sync_read("Present_Position")
        if n % S.TICK_HZ == 0:
            self._read_health(); self._check_health()
        if S.ESTOP_FILE.exists() and self.torque and not self.parking:
            self._fault("ESTOP file")
        if self.torque and not self.parking and time.time() - self.last_cmd > S.IDLE_PARK_S:
            print("idle timeout", flush=True); self.park()
        if not self.torque: return
        # stall detection: setpoint running ahead of reality
        for j in S.JOINTS:
            if abs(self.setpoint[j] - self.present[j]) > S.STALL_ERR + 0.25 * self.speeds[j]:   # lag ~0.15s*speed is normal
                self.stall_since.setdefault(j, time.time())
                if time.time() - self.stall_since[j] > S.STALL_SECONDS:
                    self.stall_since.clear()
                    self.setpoint = dict(self.present); self.target = dict(self.present)
                    self.fault = f"stall on {j} (blocked?)"; print("FAULT:", self.fault, flush=True)
                    break
            else: self.stall_since.pop(j, None)
        new = {j: S.step_toward(self.setpoint[j], self.target[j], self.speeds[j], dt) for j in S.JOINTS}
        if not self.parking:
            new = {j: S.clamp_target(j, v) for j, v in new.items()}
        self.setpoint = new
        load = self.bus.sync_read("Present_Load", normalize=False) if self.telemetry else None
        self.trace.append((time.time(), dict(self.present), dict(new), load))
        goal = dict(new)
        for j in S.TRIM_JOINTS:   # once the ramp has finished, creep goal to cancel steady-state error
            e = self.target[j] - self.present[j]
            if abs(new[j] - self.target[j]) < 0.01 and abs(e) > S.TRIM_DEADBAND and not self.parking:
                self.trim[j] = min(max(self.trim[j] + S.TRIM_GAIN * e, -S.TRIM_MAX), S.TRIM_MAX)
            goal[j] = S.clamp_target(j, new[j] + self.trim[j])
        self.bus.sync_write("Goal_Position", goal)
        if self.parking and self.at_target(2.0) and all(abs(self.setpoint[j] - S.REST[j]) < 1e-6 for j in S.JOINTS):
            time.sleep(0.5); self._disable(); self.parking = False

    def state(self):
        return {"present": {k: round(v, 2) for k, v in self.present.items()}, "target": {k: round(v, 2) for k, v in self.target.items()}, "setpoint": {k: round(v, 2) for k, v in self.setpoint.items()},
                "torque": self.torque, "parking": self.parking, "fault": self.fault, "health": self.health,
                "tick_ms": round(self.tick_ms, 1), "at_target": self.at_target() if self.torque else None, "idle_s": round(time.time() - self.last_cmd)}

    def handle(self, req):
        op = req.get("op")
        with self.lock:
            if op == "state": return self.state()
            if op == "goto": return {"target": self.goto(req["joints"], req.get("speed"), req.get("duration"))}
            if op == "gain":   # {"joint":..,"p":..} servo P coeff (EEPROM, persists; default/lerobot value 16). Bounded 8..40.
                p = int(req["p"]); assert 8 <= p <= 40, "p out of 8..40"
                self.bus.write("P_Coefficient", req["joint"], p); return {"p": p}
            if op == "telemetry": self.telemetry = bool(req["on"]); return {}
            if op == "trace":
                since = req.get("since", 0); out = [r for r in self.trace if r[0] > since]
                if req.get("clear"): self.trace.clear()
                return out
            if op == "raw":   # read-only register read, e.g. Present_Current
                return self.bus.sync_read(req["reg"], normalize=False)
            if op == "park": self.last_cmd = time.time(); self.park() if self.torque else None; return {"parking": self.torque}
            if op == "hold": self.hold(); return {}
            if op == "clear": self.fault = None; return {}
            if op == "relax":   # drop torque NOW (arm will fall under gravity unless already folded)
                self._disable(); return {}
        raise ValueError(f"unknown op {op!r}")

def serve(arm):
    if os.path.exists(S.SOCK): os.unlink(S.SOCK)
    srv = socket.socket(socket.AF_UNIX); srv.bind(S.SOCK); os.chmod(S.SOCK, 0o600); srv.listen(8)
    def client(c):
        f = c.makefile("rw")
        for line in f:
            try: resp = {"ok": True, "result": arm.handle(json.loads(line))}
            except Exception as e: resp = {"ok": False, "error": str(e)}
            f.write(json.dumps(resp) + "\n"); f.flush()
    def accept():
        while True:
            c, _ = srv.accept(); threading.Thread(target=client, args=(c,), daemon=True).start()
    threading.Thread(target=accept, daemon=True).start()

def main():
    arm = Arm(); arm.start(); serve(arm)
    signal.signal(signal.SIGTERM, lambda *_: setattr(arm, "stop", True))
    signal.signal(signal.SIGINT, lambda *_: setattr(arm, "stop", True))
    dt = 1 / S.TICK_HZ; n = 0; nxt = time.time()
    try:
        while not arm.stop:
            with arm.lock:
                try: arm.tick(dt, n)
                except Exception: traceback.print_exc(); arm.hold()
            n += 1; nxt += dt; time.sleep(max(0, nxt - time.time()))
    finally:
        print("shutting down: parking", flush=True)
        if arm.torque:
            with arm.lock:
                arm.fault = None; arm.park()
            t0 = time.time()
            while arm.torque and time.time() - t0 < 30:
                with arm.lock: arm.tick(dt, 1)
                time.sleep(dt)
        arm.bus.disconnect(disable_torque=False)
        if os.path.exists(S.SOCK): os.unlink(S.SOCK)

if __name__ == "__main__":
    main()
