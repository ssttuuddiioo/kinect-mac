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

from depth_view import Sensor
from stream import Jpeg, BOUNDARY
from syphon_out import Syphon, DEPTH, COLOUR, CLOUD

LOGDIR = os.path.join(HERE, "logs")
HEARTBEAT_S = 30.0

BG, FG, DIM = "#16181d", "#e8eaed", "#8b919b"
GREEN, RED, AMBER = "#3fbf6f", "#e2564d", "#e0a33e"
MJPEG_PORT = 8010


class HealthLog:
    """Overnight diagnostics. Line-buffered and flushed, so a hard crash still
    leaves everything up to the last event on disk."""

    def __init__(self):
        os.makedirs(LOGDIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.path = os.path.join(LOGDIR, "health-%s.log" % stamp)
        self.file = open(self.path, "a", buffering=1)
        self.started = time.monotonic()

        # A segfault in libfreenect2, Syphon or the shim kills Python without a
        # traceback. faulthandler catches SIGSEGV/SIGABRT/SIGBUS and dumps every
        # thread's stack, which is the only way to see where a C crash happened.
        self.crash = open(os.path.join(LOGDIR, "crash.log"), "a", buffering=1)
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
    def __init__(self, root):
        self.root = root
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

        self.header = b"P5\n512 424\n255\n"
        self.rgb_header = b"P6\n512 424\n255\n"
        self.sensor = None
        self.serial = ""
        self.intrinsics = None

        # --- 3. sensor, with a Retry path. Missing hardware is the normal case
        # here, not an error worth a traceback - the sensor drops off USB
        # regularly and the user just replugs it.
        self.open_sensor()

        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.bind("<Escape>", lambda _e: self.quit())
        import signal
        signal.signal(signal.SIGTERM, lambda _s, _f: self.quit())
        signal.signal(signal.SIGINT, lambda _s, _f: self.quit())
        self.tick()

    def open_sensor(self):
        """Try to acquire the Kinect. Shows Retry instead of dying if absent."""
        if self.sensor is not None:
            return
        self.retry.pack_forget()
        self.status.configure(text="opening sensor...", fg=DIM)
        self.root.update_idletasks()
        try:
            sensor = Sensor(colour=True)
            sensor.enable_cloud(True)
        except Exception as exc:
            self.status.configure(
                text="No Kinect found - plug in USB and 12V power, then Retry",
                fg=RED)
            self.retry.pack(side="left", padx=(0, 10))
            self.status.pack_forget()
            self.status.pack(side="left")
            print("sensor open failed: %s" % exc, flush=True)
            self.log.write("SENSOR_FAIL", str(exc))
            return
        self.sensor = sensor
        self.serial = sensor.serial
        self.log.write("SENSOR_OPEN", "serial %s" % sensor.serial)
        self.stall_logged = False
        self.intrinsics = sensor.intrinsics()
        if self.intrinsics:
            print("IR intrinsics  fx=%.3f fy=%.3f cx=%.3f cy=%.3f"
                  % self.intrinsics, flush=True)
        self.t0 = time.monotonic()
        self.total = 0
        self.last_frame = 0.0
        threading.Thread(target=self.worker, daemon=True).start()

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
                        while app.go:
                            with app.cond:
                                while app.go and (app.seq == last
                                                  or app.jpeg_frames[name] is None):
                                    if not app.cond.wait(timeout=10.0):
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

            self.server = ThreadingHTTPServer(("127.0.0.1", MJPEG_PORT), Handler)
            threading.Thread(target=self.server.serve_forever, daemon=True).start()
            print("MJPEG on http://127.0.0.1:%d/depth.mjpg" % MJPEG_PORT, flush=True)
            self.log.write("MJPEG", "started on :%d" % MJPEG_PORT)
        elif not self.mjpeg_on.get() and self.server is not None:
            self.server.shutdown()
            self.server = None
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

                self.syphon.publish(DEPTH, grey, 512, 424, 1)
                if rgb is not None:
                    self.syphon.publish(COLOUR, rgb, 512, 424, 3)
                if cloud is not None:
                    self.syphon.publish(CLOUD, cloud, 512, 424, 3)

                self.latest["depth"] = grey
                self.latest["colour"] = rgb

                if self.server is not None and self.jpeg is not None:
                    enc = {"depth": self.jpeg.encode(grey, 512, 424, True, 80)}
                    if rgb is not None:
                        enc["colour"] = self.jpeg.encode(rgb, 512, 424, False, 80)
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
            self.log.write("WORKER_END", "after %d frames" % self.total)

    # --- ui ---------------------------------------------------------------

    def tick(self):
        if not self.go:
            return
        # Mirror Tk vars into plain attributes; the worker must never touch Tk.
        self.f_near = self.vars["near"].get()
        self.f_far = max(self.vars["far"].get(), self.f_near + 50)
        self.f_temporal = self.vars["temporal"].get()
        self.f_erode = self.vars["erode"].get()
        self.f_median = self.vars["median"].get()

        self.syphon.lib.syphon_pump()
        clients = (self.syphon.clients(DEPTH), self.syphon.clients(COLOUR),
                   self.syphon.clients(CLOUD))

        mode = self.mode.get()
        data = self.latest.get(mode)
        if data:
            head = self.rgb_header if mode == "colour" else self.header
            self.image = tk.PhotoImage(data=head + data)
            self.canvas.itemconfigure(self.item, image=self.image)

        if self.sensor is None:          # waiting on Retry; message already set
            self.root.after(200, self.tick)
            return

        stalled = self.last_frame > 0 and time.monotonic() - self.last_frame > 5.0
        never = self.total == 0 and time.monotonic() - self.t0 > 15.0
        if never:
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
                self.sensor.close()
                self.sensor = None
                self.retry.pack(side="left", padx=(0, 10))
                self.status.pack_forget()
                self.status.pack(side="left")
        else:
            self.status.configure(
                text="%s   %.1f fps   Syphon: depth=%d colour=%d cloud=%d%s"
                % (self.serial, self.fps, clients[0], clients[1], clients[2],
                   "   MJPEG :%d" % MJPEG_PORT if self.server else ""),
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
        self.root.after(33, self.tick)

    def quit(self):
        self.log.write("EXIT", "clean, %d frames" % self.total)
        self.go = False
        with self.cond:
            self.cond.notify_all()
        if self.server:
            self.server.shutdown()
        if self.sensor:
            sensor, self.sensor = self.sensor, None
            self.root.after(300, sensor.close)
        self.root.after(500, lambda: (self.syphon.stop(), self.root.destroy()))


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
