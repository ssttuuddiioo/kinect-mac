#!/usr/bin/env python3
"""Kinect v2 - one app, all outputs.

Holds the sensor once and drives everything from the same frames:

  * preview window with live depth / colour
  * Syphon servers  "Kinect Depth", "Kinect Colour", "Kinect Cloud"
  * optional MJPEG server for OBS  (http://127.0.0.1:8010/...)
  * depth gate and noise filters, adjustable live

Only one process can hold a Kinect, which is why this is a single app rather
than several - the outputs are not alternatives, they run at once.

Ordering below is load-bearing: Tk must create NSApp before libfreenect2's
GLFW-based pipeline does, and both Syphon and libfreenect2 build hidden
NSWindows so both must be constructed on the main thread.

    /opt/homebrew/bin/python3.14 kinect_app.py
"""

import faulthandler
import os
import resource
import sys
import threading
import time
import tkinter as tk
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from camera import detect, open_camera
from stream import Jpeg, BOUNDARY
from syphon_out import Syphon, DEPTH, COLOUR, CLOUD

LOGDIR = os.path.join(HERE, "logs")
HEARTBEAT_S = 30.0

BG, FG, DIM = "#16181d", "#e8eaed", "#8b919b"
GREEN, RED, AMBER = "#3fbf6f", "#e2564d", "#e0a33e"
MJPEG_PORT = 8010


def _safe_log(path, exclusive=False):
    """Open a log for appending without following symlinks.

    The Femto Mega runs this app as root. A plain open() follows a symlink, so
    anything running as the user could swap logs/crash.log for a link to a
    system file and have root append to it. O_NOFOLLOW refuses a symlink as the
    final component; O_EXCL refuses any pre-existing name, link or not.
    """
    flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW
    if exclusive:
        flags |= os.O_EXCL
    return os.fdopen(os.open(path, flags, 0o644), "a", buffering=1)


def _safe_logdir(path):
    """Refuse a symlinked log directory: O_NOFOLLOW only guards the last path
    component, and root creating files inside a linked-to directory is the
    same attack one level up. Falls back to a private temp directory."""
    import stat
    import tempfile
    try:
        st = os.lstat(path)
        if stat.S_ISLNK(st.st_mode) or not stat.S_ISDIR(st.st_mode):
            return tempfile.mkdtemp(prefix="kinect-logs-")
    except FileNotFoundError:
        os.makedirs(path, exist_ok=True)
    return path


class HealthLog:
    """Overnight diagnostics. Line-buffered and flushed, so a hard crash still
    leaves everything up to the last event on disk."""

    def __init__(self):
        logdir = _safe_logdir(LOGDIR)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.path = os.path.join(logdir, "health-%s-%d.log" % (stamp, os.getpid()))
        try:
            self.file = _safe_log(self.path, exclusive=True)
        except FileExistsError:
            # Something already holds this name - possibly planted. Refusing is
            # right, but refusing to start would turn a write attack into a
            # denial of service, so take an unguessable name instead.
            import secrets
            self.path = os.path.join(logdir, "health-%s-%d-%s.log"
                                     % (stamp, os.getpid(), secrets.token_hex(4)))
            self.file = _safe_log(self.path, exclusive=True)
        self.started = time.monotonic()

        # A segfault in libfreenect2, Syphon or the shim kills Python without a
        # traceback. faulthandler catches SIGSEGV/SIGABRT/SIGBUS and dumps every
        # thread's stack, which is the only way to see where a C crash happened.
        # A Femto Mega session runs as root, leaving crash.log root-owned; a
        # later normal launch would then die here. Fall back to a per-user file.
        try:
            self.crash = _safe_log(os.path.join(logdir, "crash.log"))
        except OSError:
            # root-owned from an earlier Femto session, or a planted symlink
            self.crash = _safe_log(os.path.join(logdir, "crash-%d.log" % os.getuid()))
        faulthandler.enable(file=self.crash, all_threads=True)

    def uptime(self):
        s = int(time.monotonic() - self.started)
        return "%02d:%02d:%02d" % (s // 3600, (s % 3600) // 60, s % 60)

    def write(self, kind, message=""):
        self.file.write("%s  up %s  %-12s %s\n"
                        % (time.strftime("%Y-%m-%d %H:%M:%S"),
                           self.uptime(), kind, message))
        self.file.flush()

    def dump_threads(self, why):
        """Where is it actually stuck? Called on a stall."""
        self.crash.write("\n=== %s  %s  (not a crash - stall snapshot) ===\n"
                         % (time.strftime("%Y-%m-%d %H:%M:%S"), why))
        faulthandler.dump_traceback(file=self.crash, all_threads=True)
        self.crash.flush()


class App:
    def __init__(self, root, camera_kind="auto", synthetic_size=(640, 576),
                 synthetic_image=None):
        self.root = root
        self.camera_kind = camera_kind
        self.synthetic_size = synthetic_size
        self.synthetic_image = synthetic_image
        self.cw, self.ch = 512, 424          # replaced once a camera opens
        self.latest_dims = (512, 424)
        self.go = True
        self.latest = {"depth": None, "colour": None}      # raw, for preview
        self.jpeg_frames = {"depth": None, "colour": None}  # encoded, for MJPEG
        self.image = None
        self.fps = 0.0
        self.total = 0
        self.last_frame = 0.0
        self.t0 = time.monotonic()
        self.server = None
        self.jpeg = None
        self.cond = threading.Condition()
        self.seq = 0
        # Defaults must exist before the worker starts: it reads these, and
        # tick() only refreshes them once the UI loop is running.
        self.f_near, self.f_far = 500, 2000
        self.f_temporal, self.f_erode, self.f_median = 0, 0, 1
        self.log = HealthLog()
        self.last_beat = time.monotonic()
        self.stall_logged = False
        self.log.write("START", "kinect_app")

        # --- 1. Tk first, so it owns NSApp
        root.title("Kinect")
        root.configure(bg=BG)
        root.geometry("+60+60")
        root.resizable(False, False)

        self.canvas = tk.Canvas(root, width=512, height=424, bg="black",
                                highlightthickness=0, bd=0)
        self.canvas.pack()
        self.item = self.canvas.create_image(0, 0, anchor="nw")

        panel = tk.Frame(root, bg=BG, padx=16, pady=10)
        panel.pack(fill="x")

        self.vars = {}
        for label, key, lo, hi, default in (
                ("Near (mm)", "near", 300, 4500, 500),
                ("Far (mm)", "far", 300, 4500, 2000),
                ("Smooth (%)", "temporal", 0, 90, 0),
                ("Erode (px)", "erode", 0, 5, 0),
                ("Despeckle", "median", 0, 3, 1)):
            line = tk.Frame(panel, bg=BG)
            line.pack(fill="x", pady=1)
            tk.Label(line, text=label, bg=BG, fg=DIM, width=11, anchor="w",
                     font=("Helvetica Neue", 12)).pack(side="left")
            var = tk.IntVar(value=default)
            tk.Scale(line, from_=lo, to=hi, variable=var, orient="horizontal",
                     length=330, bg=BG, fg=FG, troughcolor="#23262d",
                     highlightthickness=0, font=("Helvetica Neue", 10)).pack(side="left")
            self.vars[key] = var

        row = tk.Frame(panel, bg=BG)
        row.pack(fill="x", pady=(6, 0))
        tk.Label(row, text="Preview", bg=BG, fg=DIM, width=11, anchor="w",
                 font=("Helvetica Neue", 12)).pack(side="left")
        self.mode = tk.StringVar(value="depth")
        for value, text in (("depth", "Depth"), ("colour", "Colour")):
            tk.Radiobutton(row, text=text, variable=self.mode, value=value,
                           bg=BG, fg=DIM, selectcolor=BG, activebackground=BG,
                           activeforeground=FG, highlightthickness=0,
                           font=("Helvetica Neue", 12)).pack(side="left", padx=(0, 8))

        self.mjpeg_on = tk.BooleanVar(value=False)
        tk.Checkbutton(row, text="MJPEG for OBS (:%d)" % MJPEG_PORT,
                       variable=self.mjpeg_on, command=self.toggle_mjpeg,
                       bg=BG, fg=DIM, selectcolor=BG, activebackground=BG,
                       activeforeground=FG, highlightthickness=0,
                       font=("Helvetica Neue", 12)).pack(side="left", padx=(10, 0))

        trow = tk.Frame(panel, bg=BG)
        trow.pack(fill="x", pady=(6, 0))
        tk.Label(trow, text="Tracking", bg=BG, fg=DIM, width=11, anchor="w",
                 font=("Helvetica Neue", 12)).pack(side="left")
        self.track_pose = tk.BooleanVar(value=False)
        self.track_hands = tk.BooleanVar(value=False)
        self.track_gate = tk.BooleanVar(value=True)
        for text, var in (("Body", self.track_pose), ("Hands", self.track_hands),
                          ("Gate to slab", self.track_gate)):
            tk.Checkbutton(trow, text=text, variable=var, bg=BG, fg=DIM,
                           selectcolor=BG, activebackground=BG, activeforeground=FG,
                           highlightthickness=0,
                           font=("Helvetica Neue", 12)).pack(side="left", padx=(0, 8))
        tk.Label(trow, text="OSC :", bg=BG, fg=DIM,
                 font=("Helvetica Neue", 12)).pack(side="left", padx=(6, 0))
        self.osc_port = tk.StringVar(value="9000")
        tk.Entry(trow, textvariable=self.osc_port, width=6,
                 font=("Helvetica Neue", 12)).pack(side="left")

        bar = tk.Frame(panel, bg=BG)
        bar.pack(fill="x", pady=(8, 0))
        self.retry = tk.Button(bar, text="Retry", command=self.open_sensor,
                               highlightbackground=BG, font=("Helvetica Neue", 12))
        self.status = tk.Label(bar, text="opening sensor...", bg=BG, fg=DIM,
                               anchor="w", font=("Helvetica Neue", 11))
        self.status.pack(side="left")
        root.update()

        # --- 2. Syphon, then sensor, both on this thread
        self.syphon = Syphon()
        self.syphon.start("Kinect Depth", "Kinect Colour", "Kinect Cloud")

        self.sensor = None
        self.serial = ""
        self.intrinsics = None

        # --- 3. sensor, with a Retry path. Missing hardware is the normal case
        # here, not an error worth a traceback - the sensor drops off USB
        # regularly and the user just replugs it.
        self.tracker = None
        self.t_pose = self.t_hands = False
        self.t_gate = True
        self.chooser = None
        if self.camera_kind == "ask":
            self.show_chooser()
        else:
            self.open_sensor()

        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.bind("<Escape>", lambda _e: self.quit())
        import signal
        signal.signal(signal.SIGTERM, lambda _s, _f: self.quit())
        signal.signal(signal.SIGINT, lambda _s, _f: self.quit())
        self.tick()

    # --- camera chooser -----------------------------------------------------

    CAMERAS = (("femto", "Orbbec Femto Mega"),
               ("kinect", "Kinect v2"),
               ("synthetic", "Test pattern  (no camera)"))

    def show_chooser(self, message=""):
        """Ask which camera to run, showing what is actually plugged in."""
        if self.chooser is not None:
            self.chooser.destroy()
        found = detect()
        root_user = os.geteuid() == 0

        box = tk.Frame(self.root, bg=BG, padx=26, pady=22,
                       highlightthickness=1, highlightbackground="#2c3038")
        box.place(in_=self.canvas, relx=0.5, rely=0.5, anchor="center")
        self.chooser = box

        tk.Label(box, text="Choose a camera", bg=BG, fg=FG, anchor="w",
                 font=("Helvetica Neue", 19, "bold")).pack(fill="x")
        tk.Label(box, text=message or "Detected devices are marked.", bg=BG,
                 fg=RED if message else DIM, anchor="w", justify="left",
                 wraplength=360, font=("Helvetica Neue", 11)).pack(fill="x", pady=(2, 14))

        for kind, label in self.CAMERAS:
            row = tk.Frame(box, bg=BG)
            row.pack(fill="x", pady=3)
            tk.Button(row, text=label, width=24, anchor="w",
                      command=lambda k=kind: self.choose(k),
                      highlightbackground=BG,
                      font=("Helvetica Neue", 13)).pack(side="left")
            if kind == "synthetic":
                note, colour = "no hardware", DIM
            elif found.get(kind):
                note, colour = "plugged in", GREEN
                if kind == "femto" and not root_user:
                    note = "plugged in - needs root"
                    colour = AMBER
            else:
                note, colour = "not detected", DIM
            tk.Label(row, text=note, bg=BG, fg=colour,
                     font=("Helvetica Neue", 11)).pack(side="left", padx=(10, 0))

        if found.get("femto") and not root_user:
            tk.Label(box, text="Choosing the Femto Mega reopens this app in Terminal "
                               "with sudo - macOS won't let it open the camera otherwise.",
                     bg=BG, fg=DIM, anchor="w", justify="left", wraplength=360,
                     font=("Helvetica Neue", 10)).pack(fill="x", pady=(10, 0))

        tk.Button(box, text="Rescan", command=self.show_chooser,
                  highlightbackground=BG,
                  font=("Helvetica Neue", 11)).pack(anchor="e", pady=(12, 0))
        self.status.configure(text="waiting for a camera choice", fg=DIM)
        self.log.write("CHOOSER", "detected %s" % found)

    def choose(self, kind):
        if kind == "femto" and os.geteuid() != 0:
            # Can't take the camera without root. Hand over to the sudo
            # launcher in Terminal, which prompts for the password, then quit
            # - nothing has been opened yet, so there is nothing to release.
            import subprocess
            launcher = os.path.join(HERE, "Run Femto Mega.command")
            self.log.write("CHOOSER", "femto chosen without root - relaunching via Terminal")
            subprocess.Popen(["open", launcher])
            self.quit()
            return
        if self.chooser is not None:
            self.chooser.destroy()
            self.chooser = None
        self.camera_kind = kind
        self.asked = True
        self.log.write("CHOOSER", "chose %s" % kind)
        self.open_sensor()

    def adopt_resolution(self, sensor):
        """Size everything to this camera. The Femto Mega's depth is 640x576
        (or 1024x1024 wide-FOV), not the Kinect's 512x424."""
        self.cw, self.ch = sensor.w, sensor.h
        # Preview only - outputs always go out at full resolution.
        self.preview_step = 2 if max(self.cw, self.ch) > 720 else 1
        self.canvas.configure(width=self.cw // self.preview_step,
                              height=self.ch // self.preview_step)
        self.root.title("Kinect - %s %dx%d" % (getattr(sensor, "kind", "camera"),
                                               self.cw, self.ch))
        self.log.write("CAMERA", "%s %s %dx%d" % (getattr(sensor, "kind", "?"),
                                                  sensor.serial, self.cw, self.ch))

    def open_sensor(self):
        """Try to acquire the Kinect. Shows Retry instead of dying if absent."""
        if self.sensor is not None:
            return
        old = getattr(self, "worker_thread", None)
        if old is not None and old.is_alive():
            # The previous worker still holds the device until its blocked
            # frame() returns. Opening now would find the camera busy.
            self.status.configure(text="waiting for the camera to be released...", fg=DIM)
            self.root.after(250, self.open_sensor)
            return
        self.retry.pack_forget()
        self.status.configure(text="opening sensor...", fg=DIM)
        self.root.update_idletasks()
        try:
            sensor = open_camera(self.camera_kind, colour=True,
                                 synthetic_size=self.synthetic_size,
                                 synthetic_image=self.synthetic_image)
            sensor.enable_cloud(True)
        except Exception as exc:
            if self.camera_kind == "auto":
                # detect() already produced a specific reason - show it as-is
                msg = str(exc)
            elif self.camera_kind == "femto":
                msg = "Femto Mega: run with sudo, and quit TD's Orbbec TOP"
            else:
                msg = "plug in the Kinect and its 12V power"
            self.status.configure(text="%s - then Retry" % msg[:88], fg=RED)
            if getattr(self, "asked", False):
                self.show_chooser("Couldn't open that camera: %s" % str(exc)[:160])
                self.log.write("SENSOR_FAIL", str(exc))
                return
            self.retry.pack(side="left", padx=(0, 10))
            self.status.pack_forget()
            self.status.pack(side="left")
            print("sensor open failed: %s" % exc, flush=True)
            self.log.write("SENSOR_FAIL", str(exc))
            return
        self.sensor = sensor
        self.serial = sensor.serial
        self.adopt_resolution(sensor)
        self.log.write("SENSOR_OPEN", "serial %s" % sensor.serial)
        self.stall_logged = False
        self.intrinsics = sensor.intrinsics()
        if self.intrinsics:
            print("%s intrinsics  fx=%.3f fy=%.3f cx=%.3f cy=%.3f  (%dx%d)"
                  % ((getattr(sensor, "kind", "?"),) + tuple(self.intrinsics)
                     + (sensor.w, sensor.h)), flush=True)
        self.t0 = time.monotonic()
        self.total = 0
        self.last_frame = 0.0
        self.worker_thread = threading.Thread(target=self.worker, daemon=True)
        self.worker_thread.start()

    # --- outputs ----------------------------------------------------------

    def toggle_mjpeg(self):
        if self.mjpeg_on.get() and self.server is None:
            if self.jpeg is None:
                self.jpeg = Jpeg()
            app = self

            class Handler(BaseHTTPRequestHandler):
                protocol_version = "HTTP/1.0"

                def log_message(self, *a):
                    pass

                def do_GET(self):
                    name = {"/depth.mjpg": "depth",
                            "/colour.mjpg": "colour"}.get(self.path.split("?")[0])
                    if not name:
                        self.send_error(404)
                        return
                    self.send_response(200)
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Content-Type",
                                     "multipart/x-mixed-replace; boundary=%s" % BOUNDARY)
                    self.end_headers()
                    last = 0
                    try:
                        # Stop as soon as MJPEG is switched off (app.server is
                        # replaced/cleared), not up to 10 s later on a timeout.
                        while app.go and app.server is self.server:
                            with app.cond:
                                while app.go and (app.seq == last
                                                  or app.jpeg_frames[name] is None):
                                    if app.server is not self.server:
                                        return          # switched off while waiting
                                    if not app.cond.wait(timeout=10.0):
                                        return
                                if app.server is not self.server or not app.go:
                                    return
                                last = app.seq
                                frame = app.jpeg_frames[name]
                            self.wfile.write(b"--%s\r\n" % BOUNDARY.encode())
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(frame))
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        pass

            try:
                self.server = ThreadingHTTPServer(("127.0.0.1", MJPEG_PORT), Handler)
            except OSError as exc:
                # Port taken (another instance, or stream.py). Say so and
                # untick, rather than leaving the box on with nothing serving.
                self.mjpeg_on.set(False)
                self.status.configure(text="MJPEG: port %d unavailable (%s)"
                                           % (MJPEG_PORT, exc.strerror), fg=RED)
                self.log.write("MJPEG", "bind failed: %s" % exc)
                return
            threading.Thread(target=self.server.serve_forever, daemon=True).start()
            print("MJPEG on http://127.0.0.1:%d/depth.mjpg" % MJPEG_PORT, flush=True)
            self.log.write("MJPEG", "started on :%d" % MJPEG_PORT)
        elif not self.mjpeg_on.get() and self.server is not None:
            server, self.server = self.server, None
            server.shutdown()        # stops serve_forever...
            server.server_close()    # ...but only this releases the port. Without
                                     # it, turning MJPEG back on failed: in use.
            with self.cond:          # wake handlers so they notice and leave
                self.cond.notify_all()
            print("MJPEG stopped", flush=True)
            self.log.write("MJPEG", "stopped")

    def worker(self):
        frames, since = 0, time.monotonic()
        # Hold our own reference: a stall or quit clears self.sensor between
        # statements, which used to blow up mid-iteration.
        sensor = self.sensor
        try:
            while self.go and self.sensor is sensor:
                sensor.set_filters(self.f_temporal, self.f_median, self.f_erode)
                grey, rgb = sensor.frame(self.f_near, self.f_far, True)
                if grey is None:
                    continue
                cloud = sensor.cloud_frame()
                w, h = sensor.w, sensor.h

                self.syphon.publish(DEPTH, grey, w, h, 1)
                if rgb is not None:
                    self.syphon.publish(COLOUR, rgb, w, h, 3)
                if cloud is not None:
                    self.syphon.publish(CLOUD, cloud, w, h, 3)

                self.latest["depth"] = grey
                self.latest["colour"] = rgb
                self.latest_dims = (w, h)

                tr = self.tracker
                if tr is not None and (self.t_pose or self.t_hands) and self.intrinsics:
                    # Unmasked: MediaPipe was trained on ordinary photos, and our
                    # background-removed cutout is out of distribution for it.
                    # Depth is used to validate and lift, not to pre-mask.
                    raw_rgb, raw_depth = sensor.raw_frame()
                    if raw_rgb is not None:
                        tr.submit(raw_rgb, raw_depth, self.intrinsics,
                                  self.f_near, self.f_far)

                if self.server is not None and self.jpeg is not None:
                    enc = {"depth": self.jpeg.encode(grey, w, h, True, 80)}
                    if rgb is not None:
                        enc["colour"] = self.jpeg.encode(rgb, w, h, False, 80)
                    with self.cond:
                        self.jpeg_frames.update(enc)
                        self.seq += 1
                        self.cond.notify_all()

                frames += 1
                self.total += 1
                now = time.monotonic()
                self.last_frame = now
                if now - since >= 1.0:
                    self.fps = frames / (now - since)
                    frames, since = 0, now
        except Exception as exc:
            # A worker that dies silently looks exactly like a stalled sensor.
            self.log.write("WORKER_DIED", "%s: %s" % (type(exc).__name__, exc))
            raise
        finally:
            # The worker closes its own camera, and only here - after its last
            # frame() has returned. Closing from any other thread would free
            # the device while this thread is blocked inside it (a hung USB
            # device holds frame() for seconds): a use-after-free.
            try:
                sensor.close()
            except Exception as exc:
                self.log.write("CLOSE_ERROR", str(exc)[:200])
            self.log.write("WORKER_END", "after %d frames" % self.total)

    # --- ui ---------------------------------------------------------------

    def tick(self):
        """Runs the UI loop. Reschedules in `finally`, so no single exception
        can freeze it - which would also stop the Syphon pump, and with it
        TouchDesigner's view of the servers."""
        if not self.go:
            return
        delay = 33
        try:
            delay = self._tick() or 33
        except Exception as exc:
            self._tick_failed(exc)
        finally:
            if self.go:
                self._tick_id = self.root.after(delay, self.tick)

    def _tick_failed(self, exc):
        """Log each distinct failure once - a fault that repeats every frame
        would otherwise write 30 lines a second."""
        key = "%s: %s" % (type(exc).__name__, exc)
        seen = getattr(self, "_tick_errors", set())
        if key not in seen:
            seen.add(key)
            self._tick_errors = seen
            self.log.write("TICK_ERROR", key[:200])
            import traceback
            traceback.print_exc()

    def _tick(self):
        """One UI frame. Returns the delay before the next, in ms."""
        # Mirror Tk vars into plain attributes; the worker must never touch Tk.
        self.f_near = self.vars["near"].get()
        self.f_far = max(self.vars["far"].get(), self.f_near + 50)
        self.f_temporal = self.vars["temporal"].get()
        self.f_erode = self.vars["erode"].get()
        self.f_median = self.vars["median"].get()

        self.sync_tracker()
        self.draw_overlay()
        self.syphon.lib.syphon_pump()
        clients = (self.syphon.clients(DEPTH), self.syphon.clients(COLOUR),
                   self.syphon.clients(CLOUD))

        mode = self.mode.get()
        data = self.latest.get(mode)
        if data:
            w, h = self.latest_dims
            head = (b"P6\n%d %d\n255\n" if mode == "colour"
                    else b"P5\n%d %d\n255\n") % (w, h)
            img = tk.PhotoImage(data=head + data)
            step = getattr(self, "preview_step", 1)
            self.image = img.subsample(step) if step > 1 else img
            self.canvas.itemconfigure(self.item, image=self.image)

        if self.sensor is None:          # waiting on Retry; message already set
            return 200

        stalled = self.last_frame > 0 and time.monotonic() - self.last_frame > 5.0
        never = self.total == 0 and time.monotonic() - self.t0 > 15.0
        if self.tracker is not None and self.tracker.error and (self.t_pose or self.t_hands):
            self.status.configure(text=self.tracker.error[:90], fg=RED)
        elif never:
            self.status.configure(text="NO FRAMES - sensor never started. Replug it.",
                                  fg=RED)
        elif stalled:
            self.status.configure(
                text="STALLED - no frame for %ds. Replug, then Retry."
                     % int(time.monotonic() - self.last_frame), fg=RED)
            if not self.stall_logged:
                self.stall_logged = True
                self.log.write("STALL", "no frame for %ds after %d frames"
                               % (int(time.monotonic() - self.last_frame), self.total))
                self.log.dump_threads("stalled")
            if not self.retry.winfo_ismapped():
                # Only signal. The worker closes the camera itself once its
                # current frame() returns; closing it here was a use-after-free.
                self.sensor = None
                self.retry.pack(side="left", padx=(0, 10))
                self.status.pack_forget()
                self.status.pack(side="left")
        else:
            self.status.configure(
                text="%s   %.1f fps   Syphon: depth=%d colour=%d cloud=%d%s%s"
                % (self.serial, self.fps, clients[0], clients[1], clients[2],
                   "   MJPEG :%d" % MJPEG_PORT if self.server else "",
                   ("   track %.0f fps %.0fms" % (self.tracker.current_fps(), self.tracker.ms)
                    if self.tracker and (self.t_pose or self.t_hands) else "")),
                fg=GREEN if any(clients) else DIM)
        now = time.monotonic()
        if now - self.last_beat >= HEARTBEAT_S:
            self.last_beat = now
            # Peak RSS is monotonic, so steady growth across a night means a
            # leak - the per-frame PhotoImage is the obvious suspect.
            rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / (1024.0 * 1024.0)
            self.log.write("OK", "%.1f fps  frames=%d  clients=%s  peakRSS=%.0fMB  threads=%d"
                           % (self.fps, self.total, clients, rss_mb,
                              threading.active_count()))
        return 33

    # --- tracking -------------------------------------------------------------

    def sync_tracker(self):
        """Mirror the Tracking row into plain attributes and the tracker.
        Main thread only - the worker never touches Tk."""
        self.t_pose, self.t_hands = self.track_pose.get(), self.track_hands.get()
        self.t_gate = self.track_gate.get()
        if self.tracker is not None and self.tracker.hung():
            # Frames going in, none coming out. Abandon it and start fresh;
            # its thread is a daemon, so it can't hold anything up.
            self.log.write("TRACKER_HUNG", "no frame completed in 6 s - replacing it")
            # Keep sending where the old one was. Re-reading the entry box gave
            # the default 9000 when it held half-typed text, silently moving
            # all tracking off the port TouchDesigner listens on.
            self._tracker_port = self.tracker.osc.addr[1]
            self.tracker.go = False
            self.tracker = None
        if (self.t_pose or self.t_hands) and self.tracker is None:
            from tracker import TrackerProcess, valid_port
            port = valid_port(self.osc_port.get(), getattr(self, "_tracker_port", 9000))
            # A child process: MediaPipe wedges itself after a couple of minutes
            # of normal use and can't be recreated in-process. See tracker.py.
            self.tracker = TrackerProcess(port=port)
            self.log.write("TRACKING", "started, OSC -> 127.0.0.1:%d" % port)
            print("tracking: OSC -> 127.0.0.1:%d" % port, flush=True)
        tr = self.tracker
        if tr is None:
            return
        if getattr(tr, "restarts", 0) != getattr(self, "_restarts_logged", 0):
            self._restarts_logged = tr.restarts
            self.log.write("TRACKER_RESTART", "#%d %s" % (tr.restarts, tr.restart_reason))
        if tr.rebuilds != getattr(self, "_rebuilds_logged", 0):
            self._rebuilds_logged = tr.rebuilds
            self.log.write("TRACKER_REBUILD", "#%d %s" % (tr.rebuilds, tr.rebuild_reason))
        tr.want_pose, tr.want_hands, tr.gate = self.t_pose, self.t_hands, self.t_gate
        from tracker import valid_port
        port = valid_port(self.osc_port.get(), None)   # None: ignore half-typed input
        if port is not None and port != tr.osc.addr[1]:
            tr.osc.retarget("127.0.0.1", port)
            self.log.write("TRACKING", "OSC -> 127.0.0.1:%d" % port)

    def draw_overlay(self):
        """Tracked landmarks over the preview, so you can see what TD gets."""
        self.canvas.delete("overlay")
        tr = self.tracker
        if tr is None or not (self.t_pose or self.t_hands):
            return
        step = getattr(self, "preview_step", 1)
        cw, ch = self.cw / step, self.ch / step
        ov = tr.overlay
        if self.t_pose and ov.get("pose") is not None:
            from tracker import BONES
            xs, ys = ov["pose"]
            for a, b in BONES:
                self.canvas.create_line(xs[a] * cw, ys[a] * ch, xs[b] * cw, ys[b] * ch,
                                        fill=GREEN, width=2, tags="overlay")
            for x, y in zip(xs, ys):
                self.canvas.create_oval(x * cw - 3, y * ch - 3, x * cw + 3, y * ch + 3,
                                        fill=GREEN, outline="", tags="overlay")
        if self.t_hands:
            for side, xs, ys, is_open in ov.get("hands", []):
                col = AMBER if side == "left" else "#4aa8ff"
                r = 2 if len(xs) > 1 else 8           # a single palm point gets a big dot
                for x, y in zip(xs, ys):
                    # filled = open (sending 1), ring = closed (sending 0)
                    self.canvas.create_oval(x * cw - r, y * ch - r, x * cw + r, y * ch + r,
                                            fill=col if is_open else "", outline=col,
                                            width=3, tags="overlay")

    def quit(self):
        self.log.write("EXIT", "clean, %d frames" % self.total)
        if self.tracker is not None:
            self.tracker.close()
        self.go = False
        with self.cond:
            self.cond.notify_all()
        if self.server:
            server, self.server = self.server, None
            server.shutdown()
            server.server_close()
        # Signal the worker; it closes the camera itself once frame() returns.
        self.sensor = None
        self._finish_quit(time.monotonic())

    def _finish_quit(self, started):
        """Tear down only once the worker has let go of the camera. Bounded:
        a device that never returns gets abandoned after 10 s rather than
        hanging the app on exit."""
        w = getattr(self, "worker_thread", None)
        if w is not None and w.is_alive() and time.monotonic() - started < 10.0:
            self.root.after(100, lambda: self._finish_quit(started))
            return
        if w is not None and w.is_alive():
            self.log.write("QUIT", "camera did not release within 10 s")
        # A tick already queued would otherwise fire into a destroyed window
        # ("invalid command name ...tick").
        if getattr(self, "_tick_id", None):
            try:
                self.root.after_cancel(self._tick_id)
            except tk.TclError:
                pass
        try:
            self.syphon.stop()
        finally:
            self.root.destroy()


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Depth camera -> Syphon / MJPEG")
    ap.add_argument("--camera", default="ask",
                    choices=("ask", "auto", "kinect", "femto", "synthetic"),
                    help="ask shows a chooser (default); auto picks by what's "
                         "plugged in, for unattended runs")
    ap.add_argument("--size", default="640x576",
                    help="synthetic camera resolution, WxH")
    ap.add_argument("--image", default=None,
                    help="with --camera synthetic: replay this photo as the colour "
                         "stream, e.g. to build a TD tracking patch without a camera")
    ap.add_argument("--track", default="none", choices=("none", "pose", "hands", "both"),
                    help="start with tracking on (unattended runs)")
    ap.add_argument("--osc-port", type=int, default=9000)
    ap.add_argument("--mjpeg", action="store_true",
                    help="start the MJPEG server at launch (headless use)")
    args = ap.parse_args()
    size = tuple(int(v) for v in args.size.lower().split("x"))

    root = tk.Tk()
    app = App(root, camera_kind=args.camera, synthetic_size=size,
              synthetic_image=args.image)
    if args.mjpeg:
        app.mjpeg_on.set(True)
        app.toggle_mjpeg()
    app.osc_port.set(str(args.osc_port))
    app.track_pose.set(args.track in ("pose", "both"))
    app.track_hands.set(args.track in ("hands", "both"))
    root.mainloop()
    # Everything has been shut down in order by now: camera released by its
    # worker, Syphon stopped, logs flushed line by line. Leave explicitly.
    # MediaPipe runs non-daemon dispatcher threads, and one stuck in a broken
    # graph kept the process alive after the window closed - holding the
    # launcher's PID file, so the next launch said the camera was in use.
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(0)
