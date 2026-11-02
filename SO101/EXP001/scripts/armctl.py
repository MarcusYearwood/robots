"""CLI client for armd. Examples:
  armctl.py state
  armctl.py goto shoulder_pan=10 wrist_roll=-20 --speed 25 --wait
  armctl.py park | hold | clear | relax
"""
import argparse, json, socket, sys, time
import safety as S

def call(op, **kw):
    s = socket.socket(socket.AF_UNIX); s.connect(S.SOCK)
    f = s.makefile("rw"); f.write(json.dumps({"op": op, **kw}) + "\n"); f.flush()
    r = json.loads(f.readline()); s.close()
    if not r["ok"]: sys.exit("ERROR: " + r["error"])
    return r["result"]

def main():
    p = argparse.ArgumentParser(); p.add_argument("op"); p.add_argument("joints", nargs="*")
    p.add_argument("--speed", type=float); p.add_argument("--wait", action="store_true"); p.add_argument("--timeout", type=float, default=30)
    a = p.parse_args()
    if a.op == "goto":
        j = {k: float(v) for k, v in (x.split("=") for x in a.joints)}
        print(call("goto", joints=j, speed=a.speed))
        if a.wait:
            t0 = time.time()
            while time.time() - t0 < a.timeout:
                st = call("state")
                if st["fault"]: sys.exit("FAULT: " + st["fault"])
                if st["at_target"]: break
                time.sleep(0.1)
        print(json.dumps(call("state")["present"]))
    else:
        print(json.dumps(call(a.op), indent=1))
main()
