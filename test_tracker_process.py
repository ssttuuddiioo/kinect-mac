"""Fault-injection tests for TrackerProcess - MediaPipe in a child process.

MediaPipe 1.0.1's macOS GPU path fails after ~2-3 minutes of use
(kCVReturnAllocationFailed, then abort) and cannot be recreated in the same
process. These inject the same outcomes - child killed, child frozen - and
check tracking comes back on its own, without ever blocking the camera thread.

    .venv/bin/python test_tracker_process.py     # needs the GPU: normal terminal
"""
import os, signal, socket, subprocess, sys, threading, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np, mediapipe as mp
from tracker import TrackerProcess

def main():
    PORT = 9141
    fails = [0]
    def check(name, ok, detail=""):
        print("  %s  %s%s" % ("PASS" if ok else "FAIL", name, ("  (%s)" % detail) if detail else ""))
        fails[0] += 0 if ok else 1

    colour = np.ascontiguousarray(np.asarray(mp.Image.create_from_file(
        os.path.join(HERE, "models/test/pose_640x576.jpg")).numpy_view())[..., :3])
    h, w = colour.shape[:2]
    depth = np.full((h, w), 1500.0, np.float32)
    K = (w * 0.7, w * 0.7, w / 2.0, h / 2.0)

    osc = {"t": []}
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); rx.bind(("127.0.0.1", PORT)); rx.settimeout(0.2)
    stop = threading.Event()
    def listen():
        while not stop.is_set():
            try: rx.recvfrom(65535); osc["t"].append(time.monotonic())
            except socket.timeout: pass
    threading.Thread(target=listen, daemon=True).start()

    tp = TrackerProcess(port=PORT)
    submit_ms = []
    def feed():                                  # the camera thread, at 30 fps
        nxt = time.monotonic()
        while not stop.is_set():
            nxt += 1 / 30.0
            t = time.perf_counter(); tp.submit(colour, depth, K, 500, 2500)
            submit_ms.append(1000 * (time.perf_counter() - t))
            time.sleep(max(0.0, nxt - time.monotonic()))
    threading.Thread(target=feed, daemon=True).start()

    def osc_rate(since):
        return sum(1 for t in osc["t"] if t >= since)

    def wait_flowing(timeout, since):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if osc_rate(since) >= 30: return True
            time.sleep(0.1)
        return False

    print("[A] tracking comes up in a child process")
    t0 = time.monotonic()
    up = wait_flowing(40, t0); took_up = time.monotonic() - t0
    check("OSC flowing from the child", up, "in %.1f s" % took_up)
    time.sleep(1.0)
    check("overlay reported back to the parent", tp.overlay.get("pose") is not None)
    check("fps reported", tp.current_fps() > 20, "%.1f fps" % tp.current_fps())
    child_pid = tp._child["proc"].pid
    check("MediaPipe is not in this process", child_pid != os.getpid(), "child pid %d" % child_pid)

    print("\n[B] child killed outright (what MediaPipe's abort does)")
    os.kill(child_pid, signal.SIGKILL); t_kill = time.monotonic()
    time.sleep(0.5)
    back = wait_flowing(45, t_kill + 0.5); gap = time.monotonic() - t_kill
    check("tracking recovers by itself", back, "gap %.1f s" % gap)
    check("restart recorded with the reason", tp.restarts == 1 and "died" in (tp.restart_reason or ""),
          "restarts=%d reason=%s" % (tp.restarts, tp.restart_reason))
    check("a new child, not the old one", tp._child["proc"].pid != child_pid)

    print("\n[C] child frozen solid (wedged inside MediaPipe)")
    frozen_pid = tp._child["proc"].pid
    n_before = len(submit_ms)
    os.kill(frozen_pid, signal.SIGSTOP); t_stop = time.monotonic()
    time.sleep(0.5)
    back = wait_flowing(60, t_stop + 0.5); gap = time.monotonic() - t_stop
    during = submit_ms[n_before:]
    check("frozen child detected and replaced", back, "gap %.1f s" % gap)
    check("reason names the silence/stall", tp.restarts == 2,
          "restarts=%d reason=%s" % (tp.restarts, tp.restart_reason))
    check("camera thread never blocked while the child was frozen",
          max(during) < 20, "max submit %.2f ms over %d frames" % (max(during), len(during)))
    try: os.kill(frozen_pid, 0); lingering = True
    except ProcessLookupError: lingering = False
    check("frozen child was killed, not left behind", not lingering)

    print("\n[D] parent killed -9: the child must not run on as an orphan")
    script = """
    import sys, time, numpy as np
    sys.path.insert(0, %r)
    from tracker import TrackerProcess
    tp = TrackerProcess(port=9142)
    c = np.zeros((48, 64, 3), np.uint8); d = np.full((48, 64), 1500.0, np.float32)
    for _ in range(400):
        tp.submit(c, d, (50.0, 50.0, 32.0, 24.0), 500, 2500); time.sleep(0.05)
        if tp._child is not None:
            print(tp._child["proc"].pid, flush=True); break
    time.sleep(60)
    """ % HERE
    import textwrap
    parent = subprocess.Popen([sys.executable, "-c", textwrap.dedent(script)],
                              stdout=subprocess.PIPE, text=True)
    orphan_pid = int(parent.stdout.readline())
    time.sleep(3.0)
    parent.kill(); parent.wait()
    gone_in = None
    for i in range(40):
        time.sleep(0.1)
        try: os.kill(orphan_pid, 0)
        except ProcessLookupError: gone_in = (i + 1) * 0.1; break
    check("orphaned child exits on its own", gone_in is not None,
          "gone %.1f s after parent killed" % gone_in if gone_in else "STILL RUNNING")
    if gone_in is None:
        os.kill(orphan_pid, signal.SIGKILL)

    print("\n[E] close() leaves nothing running")
    last_pid = tp._child["proc"].pid
    stop.set(); tp.close(); time.sleep(0.5)
    try: os.kill(last_pid, 0); still = True
    except ProcessLookupError: still = False
    check("child gone after close()", not still)

    print("\n%s" % ("ALL PASSED" if not fails[0] else "%d FAILED" % fails[0]))
    sys.stdout.flush()
    os._exit(1 if fails[0] else 0)


if __name__ == "__main__":
    # Required: multiprocessing's spawn re-imports this script in the child.
    # Unguarded, every child re-ran the whole test and died binding the port.
    main()
