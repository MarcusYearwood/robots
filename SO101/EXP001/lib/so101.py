"""Client library for armd (+ camera). Import in experiments:  import sys; sys.path.insert(0, '<EXP001>/lib'); sys.path.insert(0, '<EXP001>/scripts')
Everything goes through the daemon, so safety.py limits always apply."""
import json, socket, threading, time
import safety as S

JOINTS = S.JOINTS

class Arm:
    def __init__(self):
        self.s = socket.socket(socket.AF_UNIX); self.s.connect(S.SOCK); self.f = self.s.makefile("rw")
    def call(self, op, **kw):
        self.f.write(json.dumps({"op": op, **kw}) + "\n"); self.f.flush()
        r = json.loads(self.f.readline())
        if not r["ok"]: raise RuntimeError(r["error"])
        return r["result"]
    def state(self): return self.call("state")
    def pos(self): return self.state()["present"]
    def goto(self, joints=None, speed=None, duration=None, wait=True, timeout=30, tol=1.5, **kw):
        """goto({'shoulder_pan': 10}, duration=2) or goto(shoulder_pan=10, speed=20). Blocks until arrival if wait."""
        joints = {**(joints or {}), **kw}
        self.call("goto", joints=joints, speed=speed, duration=duration)
        if wait: self.wait(timeout, tol)
    def wait(self, timeout=30, tol=1.5, settle_tol=8.0):
        """Done when setpoint has reached target and the arm has stopped moving. Returns state with 'ss_err' (max |present-target|).
        Raises if still off by > settle_tol (blocked / sagging badly). Gravity sag of a few units on lift/elbow is normal."""
        t0 = time.time(); hist = []
        while time.time() - t0 < timeout:
            st = self.state()
            if st["fault"]: raise RuntimeError("FAULT: " + st["fault"])
            if not st["torque"]: return st
            err = max(abs(st["present"][j] - st["target"][j]) for j in JOINTS)
            sp_done = all(abs(st["setpoint"][j] - st["target"][j]) < 0.5 for j in JOINTS)
            hist.append((time.time(), dict(st["present"])))
            hist = [h for h in hist if h[0] > time.time() - 0.4]
            still = len(hist) > 5 and all(max(abs(hist[0][1][j] - h[1][j]) for h in hist) < 0.3 for j in JOINTS)
            if err < tol or (sp_done and still):
                st["ss_err"] = err
                if err > settle_tol: raise RuntimeError(f"stopped {err:.1f} short of target (blocked?)")
                return st
            time.sleep(0.03)
        raise TimeoutError("move did not finish")
    def park(self, wait=True):
        self.call("park")
        while wait and self.state()["torque"]: time.sleep(0.2)
    def telemetry(self, on): self.call("telemetry", on=on)
    def trace(self, since=0, clear=False): return self.call("trace", since=since, clear=clear)

class Camera:
    """Background-thread capture so .frame() is always the freshest image (no stale buffer). rotate: image rotated so 'up' is up at wrist_roll=0."""
    def __init__(self, index=0, rotate=True):
        import cv2; self.cv2 = cv2
        self.cap = cv2.VideoCapture(index); self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.rotate = rotate; self._f = None; self._t = 0; self._run = True
        threading.Thread(target=self._loop, daemon=True).start()
        while self._f is None: time.sleep(0.05)
        time.sleep(0.8)   # auto-exposure
    def _loop(self):
        while self._run:
            ok, f = self.cap.read()
            if ok: self._f, self._t = f, time.time()
    def frame(self):
        f = self._f.copy()
        return self.cv2.rotate(f, self.cv2.ROTATE_90_CLOCKWISE) if self.rotate else f
    def save(self, path): self.cv2.imwrite(str(path), self.frame())
    def close(self): self._run = False; time.sleep(0.1); self.cap.release()
