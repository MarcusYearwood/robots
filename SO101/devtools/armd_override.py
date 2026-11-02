"""armd with a leader-arm override. Subclasses EXP001's Arm; EXP001 files are not modified.

Same unix socket and ops as armd (so armctl.py and the agent keep working), plus:
    {"op": "override", "on": true|false}   take over with the leader / hand control back
While override is ON, the leader arm sets the follower's target every tick and agent `goto` is refused.
Override still goes through safety.py: soft limits, speed cap, stall/temp/volt faults, ESTOP, parking.
Turning override OFF holds the follower where it is; the agent can then `goto` again.

Run INSTEAD of armd.py (both own the follower port). Stop the old daemon first (SIGTERM parks it and cuts torque):
    cd SO101/devtools && python armd_override.py
"""
import signal, sys, time, traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "EXP001" / "scripts"))
import armd as A          # noqa: E402  (EXP001/scripts/armd.py)
import safety as S        # noqa: E402

from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig  # noqa: E402

LEADER_PORT = "/dev/tty.usbmodem5C821102991"
LEADER_ID = "my_awesome_leader_arm"
RAMP_SPEED = 30.0       # units/s while easing the follower onto the leader's pose
RAMP_TOL = 3.0          # units; all joints within this => ramp done, track at full speed
OVERRIDE_SPEED = S.MAX_SPEED


class OverrideArm(A.Arm):
    def __init__(self):
        super().__init__()
        # We depend on EXP001's per-joint `speeds` dict. If armd.py's internals change again, fail at startup, not mid-move.
        assert isinstance(getattr(self, "speeds", None), dict), "EXP001 armd.Arm API changed (no `speeds` dict); update armd_override.py"
        self.leader = SO101Leader(SO101LeaderConfig(port=LEADER_PORT, id=LEADER_ID))
        self.override = False
        self.ramping = False
        self.leader_pose = {}

    def start(self):
        super().start()
        self.leader.connect(calibrate=False)   # leader torque stays off; we only read it
        print("leader connected", flush=True)

    # ---- override control (called under lock)
    def set_override(self, on):
        if on:
            if self.fault: raise RuntimeError(f"fault latched: {self.fault}")
            if S.ESTOP_FILE.exists(): raise RuntimeError("ESTOP file present")
            if not self.torque: self._enable()
            self.parking = False
            self.hold()
            self.override, self.ramping = True, True
            self.last_cmd = time.time()
            print("override ON", flush=True)
        else:
            if self.override: print("override OFF (holding)", flush=True)
            self.override = self.ramping = False
            self.hold()

    def goto(self, targets, *args, **kwargs):     # pass-through so EXP001 can add goto params (e.g. duration)
        if self.override: raise RuntimeError("override active: leader arm has control (turn override off first)")
        return super().goto(targets, *args, **kwargs)

    def handle(self, req):
        if req.get("op") == "override":
            with self.lock:
                self.set_override(bool(req.get("on")))
                return {"override": self.override}
        return super().handle(req)

    def state(self):
        st = super().state()
        st["override"] = self.override
        st["ramping"] = self.ramping
        st["leader"] = {k: round(v, 2) for k, v in self.leader_pose.items()}
        return st

    # ---- tick
    def tick(self, dt, n):
        if self.override:
            if self.fault or self.parking or not self.torque or S.ESTOP_FILE.exists():
                self.override = self.ramping = False       # a fault/park/estop always takes control away
                print("override OFF (fault/park/estop)", flush=True)
            else:
                try:
                    act = self.leader.get_action()
                    self.leader_pose = {k[:-4]: v for k, v in act.items()}
                    self.target.update(S.validate_targets(self.leader_pose))
                    self.last_cmd = time.time()
                    err = max(abs(self.target[j] - self.present[j]) for j in S.JOINTS)
                    if self.ramping and err < RAMP_TOL: self.ramping = False
                    sp = RAMP_SPEED if self.ramping else OVERRIDE_SPEED
                    self.speeds = {j: sp for j in S.JOINTS}
                except Exception:
                    traceback.print_exc()
                    self.override = self.ramping = False
                    self.hold()
                    print("override OFF (leader read failed)", flush=True)
        super().tick(dt, n)


def main():
    arm = OverrideArm(); arm.start(); A.serve(arm)
    signal.signal(signal.SIGTERM, lambda *_: setattr(arm, "stop", True))
    signal.signal(signal.SIGINT, lambda *_: setattr(arm, "stop", True))
    dt = 1 / S.TICK_HZ; n = 0; nxt = time.time()
    try:
        while not arm.stop:
            with arm.lock:
                try: arm.tick(dt, n)
                except Exception: traceback.print_exc(); arm.override = False; arm.hold()
            n += 1; nxt += dt; time.sleep(max(0, nxt - time.time()))
    finally:
        print("shutting down: parking", flush=True)
        if arm.torque:
            with arm.lock:
                arm.override = False; arm.fault = None; arm.park()
            t0 = time.time()
            while arm.torque and time.time() - t0 < 30:
                with arm.lock: arm.tick(dt, 1)
                time.sleep(dt)
        arm.leader.disconnect()
        arm.bus.disconnect(disable_torque=False)
        import os
        if os.path.exists(S.SOCK): os.unlink(S.SOCK)


if __name__ == "__main__":
    main()
