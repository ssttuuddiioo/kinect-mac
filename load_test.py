"""Sustained load + chaos against the real app. Hardware-free: the camera
replays a reference photo, so tracking has a real body to find.

Watches for what only shows up over time - memory or thread growth, frame
rate collapse, silent failures - while deliberately abusing every control.

    .venv/bin/python load_test.py [seconds]      # needs the GPU: normal terminal
"""
import os, random, socket, subprocess, sys, threading, time, tkinter as tk
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kinect_app as ka

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
STEADY = "--steady" in sys.argv          # normal use: no chaos, tracking always on
OSC_PORT = 9121
rng = random.Random(42)

def rss_mb():                                  # CURRENT resident memory, not peak
    out = subprocess.run(["ps", "-o", "rss=", "-p", str(os.getpid())],
                         capture_output=True, text=True).stdout.strip()
    return int(out) / 1024.0 if out else 0.0

# --- OSC listener -----------------------------------------------------------
osc = {"n": 0}
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.bind(("127.0.0.1", OSC_PORT)); rx.settimeout(0.2)
stop = threading.Event()
def listen():
    while not stop.is_set():
        try: rx.recvfrom(65535); osc["n"] += 1
        except socket.timeout: pass
threading.Thread(target=listen, daemon=True).start()

# --- MJPEG clients: 30 readers + 5 that connect and never read ---------------
mjpeg = {"bytes": 0, "errors": 0}
def reader(path):
    try:
        s = socket.create_connection(("127.0.0.1", 8010), timeout=5)
        s.sendall(b"GET %s HTTP/1.0\r\n\r\n" % path.encode())
        while not stop.is_set():
            d = s.recv(65536)
            if not d: break
            mjpeg["bytes"] += len(d)
        s.close()
    except OSError:
        mjpeg["errors"] += 1
slow = []
def start_clients():
    for i in range(30):
        threading.Thread(target=reader, args=(["/depth.mjpg", "/colour.mjpg"][i % 2],),
                         daemon=True).start()
    for _ in range(5):                         # slow-loris: connect, request, never read
        try:
            s = socket.create_connection(("127.0.0.1", 8010), timeout=5)
            s.sendall(b"GET /depth.mjpg HTTP/1.0\r\n\r\n"); slow.append(s)
        except OSError:
            mjpeg["errors"] += 1

# --- the app -------------------------------------------------------------------
root = tk.Tk()
# 640x576 = the Femto Mega's depth resolution, with a real body in frame
app = ka.App(root, camera_kind="synthetic",
             synthetic_image=os.path.join(HERE, "models/test/pose_640x576.jpg"))
def pump(s):
    end = time.time() + s
    while time.time() < end:
        root.update(); time.sleep(0.005)

pump(2.0)
app.osc_port.set(str(OSC_PORT)); app.track_pose.set(True); app.track_hands.set(True)
app.mjpeg_on.set(True); app.toggle_mjpeg()
pump(3.0); start_clients(); pump(5.0)          # warm-up: models loaded, clients attached

base_threads = threading.active_count()
samples, chaos_log = [], []
t0 = time.time(); next_chaos = t0; next_sample = t0
bad_ports = ["abc", "70000", "0", "-5", "", "9121x"]
while time.time() - t0 < DURATION:
    now = time.time()
    if now >= next_chaos and not STEADY:
        act = rng.choice(["filters", "slab", "track", "port", "mjpeg", "mode"])
        if act == "filters":
            for k, hi in (("temporal", 90), ("erode", 5), ("median", 3)):
                app.vars[k].set(rng.choice([0, hi, rng.randint(0, hi)]))
        elif act == "slab":                        # includes near >= far on purpose
            app.vars["near"].set(rng.randint(300, 4500)); app.vars["far"].set(rng.randint(300, 4500))
        elif act == "track":
            app.track_pose.set(rng.random() < 0.7); app.track_hands.set(rng.random() < 0.7)
        elif act == "port":
            app.osc_port.set(rng.choice(bad_ports + [str(OSC_PORT)] * 3))
        elif act == "mjpeg":
            app.mjpeg_on.set(not app.mjpeg_on.get()); app.toggle_mjpeg()
        elif act == "mode":
            app.mode.set(rng.choice(["depth", "colour"]))
        chaos_log.append(act); next_chaos = now + rng.uniform(0.3, 2.0)
    if now >= next_sample:
        samples.append((now - t0, app.fps, app.tracker.fps if app.tracker else 0,
                        rss_mb(), threading.active_count(),
                        len(app.canvas.find_all()), osc["n"]))
        next_sample = now + 5.0
    pump(0.05)

# restore realistic settings and confirm recovery from all the abuse
app.osc_port.set(str(OSC_PORT)); app.track_pose.set(True); app.track_hands.set(True)
app.vars["near"].set(500); app.vars["far"].set(2500)
app.vars["temporal"].set(0); app.vars["median"].set(1); app.vars["erode"].set(1)
app.mjpeg_on.set(True); app.toggle_mjpeg()
pump(3.0)                                          # settle
n_before = osc["n"]; real_fps = []
for _ in range(10):
    pump(1.0); real_fps.append(app.fps)
osc_after_restore = osc["n"] - n_before
tracker_err = app.tracker.error if app.tracker else "no tracker"

# --- report ------------------------------------------------------------------
print("%6s %7s %7s %8s %8s %7s %8s" % ("t(s)", "fps", "track", "RSS MB", "threads", "canvas", "osc"))
for t, f, tf, r, th, c, o in samples:
    print("%6.0f %7.1f %7.1f %8.1f %8d %7d %8d" % (t, f, tf, r, th, c, o))
warm = [s for s in samples if s[0] > 20] or samples
fps = [s[1] for s in warm]; rss = [s[3] for s in warm]
half = len(warm) // 2
early, late = (sum(r for r in rss[:half]) / max(half, 1)), (sum(r for r in rss[half:]) / max(len(rss) - half, 1))
print("\nchaos actions: %d  (%s)" % (len(chaos_log), ", ".join("%s=%d" % (k, chaos_log.count(k)) for k in sorted(set(chaos_log)))))
print("MJPEG: %.1f MB streamed, %d client errors; 5 clients never read" % (mjpeg["bytes"] / 1e6, mjpeg["errors"]))

fails = []
def check(name, ok, detail):
    print("  %s  %s  (%s)" % ("PASS" if ok else "FAIL", name, detail))
    if not ok: fails.append(name)
print()
print("  info  chaos-phase fps min %.1f / mean %.1f - dips are maxed filters (7x7 = 39 ms here)"
      % (min(fps), sum(fps) / len(fps)))
check("30 fps at realistic settings, tracking + MJPEG + 30 clients on",
      min(real_fps) >= 27, "min %.1f, mean %.1f fps" % (min(real_fps), sum(real_fps) / len(real_fps)))
check("no memory growth (current RSS, first vs second half)", late - early < 40,
      "%.0f -> %.0f MB" % (early, late))
check("threads bounded (slow clients didn't pile up)", max(s[4] for s in warm) - base_threads < 20,
      "baseline %d, peak %d" % (base_threads, max(s[4] for s in warm)))
check("canvas items bounded (overlay not leaking)", max(s[5] for s in warm) < 400,
      "peak %d items" % max(s[5] for s in warm))
check("OSC recovered after junk ports", osc_after_restore > 100, "%d datagrams in 10 s after restore" % osc_after_restore)
check("tracker error cleared after recovery", tracker_err is None, "error=%s" % tracker_err)
log = open(app.log.path).read()
for bad in ("WORKER_DIED", "TICK_ERROR"):
    check("no %s in the health log" % bad, bad not in log, "%d occurrences" % log.count(bad))
rb = [l.split("TRACKER_REBUILD", 1)[1].strip()[:110] for l in log.splitlines() if "TRACKER_REBUILD" in l]
print("  info  MediaPipe graph rebuilds: %d%s" % (len(rb), "".join("\n          " + r for r in rb)))

stop.set()
for s in slow: s.close()
app.quit()
end = time.time() + 12
done = False
while time.time() < end:
    try:
        root.update()
        if not root.winfo_exists(): done = True; break
    except tk.TclError:
        done = True; break
    time.sleep(0.01)
check("quit completes under load", done, "window gone" if done else "hung")
print("\n%s" % ("ALL PASSED" if not fails else "%d FAILED: %s" % (len(fails), ", ".join(fails))))
# Leave the way the real app does (see kinect_app.py __main__). This harness
# runs the app in-process, so a MediaPipe dispatcher thread stuck in a broken
# graph would otherwise hold the interpreter open - as the previous run did.
sys.stdout.flush()
os._exit(1 if fails else 0)
