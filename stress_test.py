"""Stress and regression tests for failure modes found in review.

Each test reproduces a specific bug: it fails on the code as it was and passes
once fixed. Hardware-free - the camera is a mock that can be made to hang the
way a Kinect or Femto does when it drops off USB.

    .venv/bin/python stress_test.py
"""
import os, sys, tempfile, threading, time, tkinter as tk
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import kinect_app as ka

failures = 0
def check(name, cond, detail=""):
    global failures
    print("  %s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (%s)" % detail) if detail else ""))
    failures += 0 if cond else 1


class HangingCamera:
    """Delivers frames, then on demand blocks inside frame() - exactly what
    libfreenect2's waitForNewFrame does on a dead device. close() records
    whether it was called while a frame() call was still running: in the C
    backends that means freeing the listener the other thread is blocked on."""
    kind = "mock"
    def __init__(self, *a, **k):
        self.w, self.h, self.serial = 64, 48, "MOCK"
        self.hang = threading.Event()        # set -> frame() blocks
        self.release = threading.Event()     # set -> a blocked frame() returns
        self.in_frame = 0
        self.closed_mid_frame = False
        self.closed = False
    def frame(self, near, far, want_colour=True):
        self.in_frame += 1
        try:
            if self.hang.is_set():
                self.release.wait(8.0)       # like the 5 s USB timeout, longer
                return None, None
            time.sleep(1 / 30.0)
            return bytes(self.w * self.h), bytes(self.w * self.h * 3)
        finally:
            self.in_frame -= 1
    def close(self):
        self.closed_mid_frame = self.closed_mid_frame or self.in_frame > 0
        self.closed = True
    def cloud_frame(self): return None
    def enable_cloud(self, on=True): pass
    def set_filters(self, *a): pass
    def intrinsics(self): return (50.0, 50.0, 32.0, 24.0)
    def raw_frame(self): return None, None


def make_app(cam):
    ka.open_camera = lambda *a, **k: cam
    root = tk.Tk()
    app = ka.App(root, camera_kind="mock")
    return root, app

def closes_itself(root, s=12.0):
    """Pump until the app's own quit() destroys the window. No manual destroy:
    that races the app's teardown and isn't what happens in real use."""
    end = time.time() + s
    while time.time() < end:
        try:
            root.update()
            if not root.winfo_exists():
                return True
        except tk.TclError:
            return True
        time.sleep(0.01)
    return False

def pump(root, s, until=None):
    end = time.time() + s
    while time.time() < end:
        root.update(); time.sleep(0.01)
        if until and until():
            return True
    return False


print("[1] stall recovery must not close the camera under a blocked frame()")
cam = HangingCamera()
root, app = make_app(cam)
pump(root, 1.0)
frames_before = app.total
cam.hang.set()                                 # the sensor "drops off USB"
pump(root, 9.0, until=lambda: cam.closed)       # stall detected after 5 s
check("frames were flowing first", frames_before > 10, "%d frames" % frames_before)
check("stall was detected and the camera closed", cam.closed)
check("close() never ran while frame() was in flight (no use-after-free)",
      not cam.closed_mid_frame)
cam.release.set(); app.quit()
check("quit completes on its own", closes_itself(root))

print("\n[2] quit must not close the camera under a blocked frame()")
cam = HangingCamera()
root, app = make_app(cam)
pump(root, 0.8)
cam.hang.set(); pump(root, 0.3)                # worker now blocked in frame()
app.quit(); pump(root, 1.5)
check("quit waits while frame() is still blocked", not cam.closed,
      "closed early" if cam.closed else "still waiting, as it should")
cam.release.set()                              # the hung frame() finally returns
try:
    pump(root, 3.0, until=lambda: cam.closed)
except tk.TclError:
    pass                                       # quit destroyed the window: expected
check("...then the camera IS closed, by the worker", cam.closed)
check("and never while frame() was in flight", not cam.closed_mid_frame)
check("quit completes on its own", closes_itself(root))

print("\n[3] one exception in tick() must not freeze the UI")
cam = HangingCamera()
root, app = make_app(cam)
pump(root, 0.8)
ticks = {"n": 0}
real_draw = app.draw_overlay
def counting_draw():
    ticks["n"] += 1
    if ticks["n"] == 5:
        raise RuntimeError("simulated failure inside tick")
    return real_draw()
app.draw_overlay = counting_draw
pump(root, 1.5)
check("tick keeps running after an exception", ticks["n"] > 10,
      "%d ticks in 1.5 s (~45 expected)" % ticks["n"])
app.quit()
check("quit completes on its own", closes_itself(root))

print("\n[4] OSC sender survives an out-of-range port")
import tracker as T
s = T.OscSender(port=9000)
s.retarget("127.0.0.1", 70000)
try:
    s.send([("/x", 1.0)]); raised = None
except Exception as e:
    raised = type(e).__name__
check("send() to port 70000 doesn't raise", raised is None, "raised %s" % raised)

check("port 70000 is refused, previous port kept", s.addr[1] == 9000, "addr=%s" % (s.addr,))
check("junk ports refused", T.valid_port("abc", 1) == 1 and T.valid_port(-1, 1) == 1
      and T.valid_port(0, 1) == 1 and T.valid_port("9001", 1) == 9001)

print("\n[5] tracker errors clear once things recover")
tr = T.Tracker.__new__(T.Tracker)             # no models needed for this
tr.error = None
def boom(*a): raise ValueError("one bad frame")
tr._process = boom; tr._step(None, 0, 0.0)
set_after_failure = tr.error
tr._process = lambda *a: None; tr._step(None, 0, 0.0)
check("a failing frame reports its error", bool(set_after_failure), "%s" % set_after_failure)
check("the next good frame clears it", tr.error is None, "still: %s" % tr.error)

print("\n[5b] a MediaPipe graph that breaks is rebuilt, not dead forever")
class BrokenGraph:
    """Behaves like the failed hand graph: once broken, every call raises."""
    def __init__(self): self.broken, self.closed = False, False
    def recognize_for_video(self, img, ts):
        if self.broken:
            raise RuntimeError("Graph has errors: Packet isn't the sole owner of the holder.")
        return "ok"
    def close(self): self.closed = True
tr = T.Tracker.__new__(T.Tracker)
tr.rebuilds, tr.rebuild_reason, tr.last_rebuild = 0, None, {}
first = BrokenGraph(); tr.gesture = first
made = []
tr._make = lambda name: made.append(name) or BrokenGraph()
check("healthy graph answers", tr._infer("gesture", None, 1) == "ok")
first.broken = True
check("broken graph's frame is skipped, not raised", tr._infer("gesture", None, 2) is None)
check("...and the model is rebuilt", made == ["gesture"] and first.closed and tr.rebuilds == 1,
      "made=%s closed=%s rebuilds=%d" % (made, first.closed, tr.rebuilds))
check("the rebuilt model works", tr._infer("gesture", None, 3) == "ok")
tr.gesture.broken = True
tr._infer("gesture", None, 4); tr._infer("gesture", None, 5); tr._infer("gesture", None, 6)
check("backoff: no rebuild storm on repeated failure", tr.rebuilds == 1, "rebuilds=%d" % tr.rebuilds)

print("\n[5c] a model whose close() never returns can't freeze the tracker")
class HangsOnClose(BrokenGraph):
    def close(self):
        threading.Event().wait()          # forever - like joining a stuck dispatcher
tr = T.Tracker.__new__(T.Tracker)
tr.rebuilds, tr.rebuild_reason, tr.last_rebuild = 0, None, {}
bad = HangsOnClose(); bad.broken = True; tr.gesture = bad
tr._make = lambda name: BrokenGraph()
t0 = time.monotonic(); tr._infer("gesture", None, 1); took = time.monotonic() - t0
check("rebuild returns promptly despite a hanging close()", took < 1.0, "%.3f s" % took)
check("...and the replacement model answers", tr._infer("gesture", None, 2) == "ok")

print("\n[5d] a stuck tracker is detected, and its fps stops lying")
tr = T.Tracker.__new__(T.Tracker)
now = time.monotonic()
tr.fps, tr.ready_at, tr.last_submit, tr.last_done = 30.0, now - 20, now - 0.1, now - 10
check("frames in, none out for 10 s -> hung()", tr.hung())
check("stale fps reads 0, not the last value (30)", tr.current_fps() == 0.0)
tr.last_done = now - 0.2
check("healthy tracker is not hung", not tr.hung())
check("...and reports its real fps", tr.current_fps() == 30.0)
tr.last_submit, tr.last_done = now - 30, now - 30
check("nothing submitted (tracking off) is not 'hung'", not tr.hung())
tr.ready_at, tr.last_submit, tr.last_done = now - 2, now - 0.1, 0.0
check("still loading models is not 'hung'", not tr.hung())
tr.error, tr.ready_at, tr.created_at = None, 0.0, now - 5
check("a fresh tracker loading models is not 'hung'", not tr.hung())
tr.created_at = now - 45
check("models that never finish loading ARE 'hung'", tr.hung())

print("\n[6] logs written as root must not follow symlinks")
d = tempfile.mkdtemp()
target = os.path.join(d, "victim.txt")
open(target, "w").write("original\n")
os.symlink(target, os.path.join(d, "crash.log"))  # attacker-planted
ka.LOGDIR = d
try:
    log = ka.HealthLog()
    log.crash.write("INJECTED\n"); log.crash.flush()
    followed = "INJECTED" in open(target).read()
except OSError as e:
    followed = False
check("crash.log symlink is not followed", not followed,
      "wrote into the symlink's target" if followed else "refused")

print("\n[6b] a symlinked logs DIRECTORY must not redirect root's writes")
protected = tempfile.mkdtemp()                    # stands in for /etc
link = os.path.join(tempfile.mkdtemp(), "logs")
os.symlink(protected, link)                       # attacker swaps logs/ for a link
ka.LOGDIR = link
log = ka.HealthLog()
log.write("TEST", "hello")
check("nothing written into the linked-to directory", os.listdir(protected) == [],
      "found %s" % os.listdir(protected))
check("logs went to a private fallback instead",
      not os.path.realpath(log.path).startswith(os.path.realpath(protected)), log.path)

print("\n[6c] pre-planted health log name is refused, not followed")
d = tempfile.mkdtemp(); ka.LOGDIR = d
victim = os.path.join(d, "victim2.txt"); open(victim, "w").write("x\n")
stamp = time.strftime("%Y%m%d-%H%M%S")
os.symlink(victim, os.path.join(d, "health-%s-%d.log" % (stamp, os.getpid())))
try:
    log = ka.HealthLog(); log.write("START", "x")
    planted_followed = "START" in open(victim).read()
    raised = None
except OSError as e:
    planted_followed, raised = False, type(e).__name__
check("pre-existing name is not followed", not planted_followed)
check("...and the app still starts (no denial of service)", raised is None,
      "raised %s" % raised if raised else "logging to %s" % os.path.basename(log.path))

print("\n%s" % ("ALL PASSED" if failures == 0 else "%d FAILED" % failures))
sys.exit(1 if failures else 0)
