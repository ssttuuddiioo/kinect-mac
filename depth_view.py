#!/usr/bin/env python3
"""Live Kinect v2 depth view with distance-gated background subtraction.

Drag the two sliders to set a near and far cutoff in millimetres. Anything
outside that slab is blacked out - that is the background subtraction, done
on raw depth, so it does not care about colour, lighting or a learned plate.
Surviving pixels are shaded near-bright to far-dim.

Requires the arm64 Python with Tk 9 and the shim built alongside it:

    /opt/homebrew/bin/python3.14 depth_view.py

Depth only - the RGB stream is left off to save USB bandwidth.
"""

import ctypes
import os
import threading
import time
import tkinter as tk

LOG_DIR = None  # set at startup

HERE = os.path.dirname(os.path.abspath(__file__))
SHIM = os.path.join(HERE, "libk2shim.dylib")

NEAR_DEFAULT, FAR_DEFAULT = 500, 2000
LIMIT_MIN, LIMIT_MAX = 300, 4500

BG = "#16181d"
FG = "#e8eaed"
DIM = "#8b919b"
RED = "#e2564d"


class EventLog:
    """Append-only session log. Line-buffered and flushed, so a hard crash
    still leaves everything up to the last event on disk."""

    def __init__(self, directory):
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(
            directory, "session-%s.log" % time.strftime("%Y%m%d-%H%M%S"))
        self.file = open(self.path, "a", buffering=1)
        self.started = time.monotonic()
        self.dropouts = 0
        self.misses = 0

    def uptime(self):
        s = int(time.monotonic() - self.started)
        return "%02d:%02d:%02d" % (s // 3600, (s % 3600) // 60, s % 60)

    def write(self, kind, message=""):
        self.file.write("%s  up %s  %-13s %s\n"
                        % (time.strftime("%Y-%m-%d %H:%M:%S"),
                           self.uptime(), kind, message))
        self.file.flush()


class Sensor:
    def __init__(self, colour=True):
        self.lib = ctypes.CDLL(SHIM)
        self.lib.k2_open.argtypes = [ctypes.c_int]
        self.lib.k2_open.restype = ctypes.c_void_p
        self.lib.k2_has_colour.argtypes = [ctypes.c_void_p]
        self.lib.k2_has_colour.restype = ctypes.c_int
        self.lib.k2_close.argtypes = [ctypes.c_void_p]
        self.lib.k2_serial.argtypes = [ctypes.c_void_p]
        self.lib.k2_serial.restype = ctypes.c_char_p
        self.lib.k2_frame.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int, ctypes.c_int
        ]
        self.lib.k2_frame.restype = ctypes.c_int
        self.lib.k2_set_filters.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        self.lib.k2_set_filters.restype = None
        self.lib.k2_set_cloud.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.lib.k2_set_cloud.restype = None
        self.lib.k2_get_cloud.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte)]
        self.lib.k2_get_cloud.restype = ctypes.c_int
        self.lib.k2_intrinsics.argtypes = [ctypes.c_void_p] + [ctypes.POINTER(ctypes.c_float)] * 4
        self.lib.k2_intrinsics.restype = ctypes.c_int
        self.lib.k2_get_raw.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
        self.lib.k2_get_raw.restype = ctypes.c_int
        self.lib.k2_width.restype = ctypes.c_int
        self.lib.k2_height.restype = ctypes.c_int

        self.w = self.lib.k2_width()
        self.h = self.lib.k2_height()
        self.colour = colour
        self.handle = self.lib.k2_open(1 if colour else 0)
        if not self.handle:
            raise RuntimeError("no Kinect v2 found (is it plugged into USB 3?)")
        self.serial = self.lib.k2_serial(self.handle).decode(errors="replace")
        self.grey = (ctypes.c_ubyte * (self.w * self.h))()
        self.rgb = (ctypes.c_ubyte * (self.w * self.h * 3))() if colour else None
        self.cloud = None

    def frame(self, near, far, want_colour=True):
        """Returns (grey_bytes, rgb_bytes_or_None), or (None, None) on timeout."""
        rgb_ptr = self.rgb if (self.rgb is not None and want_colour) else None
        rc = self.lib.k2_frame(self.handle, self.grey, rgb_ptr, near, far)
        if not rc:
            return None, None
        return bytes(self.grey), (bytes(self.rgb) if rc & 2 else None)

    def enable_cloud(self, on=True):
        """Also produce packed 16-bit depth for point-cloud use."""
        self.lib.k2_set_cloud(self.handle, 1 if on else 0)
        if on and self.cloud is None:
            self.cloud = (ctypes.c_ubyte * (self.w * self.h * 3))()

    def cloud_frame(self):
        """Packed depth from the last frame: R=high byte, G=low byte, B=valid."""
        if self.cloud is None:
            return None
        return bytes(self.cloud) if self.lib.k2_get_cloud(self.handle, self.cloud) else None

    def raw_frame(self):
        """Unmasked colour (h,w,3 RGB) and depth (h,w mm) from the last frame,
        mirrored like the outputs. For tracking."""
        import numpy as np
        depth = np.zeros((self.h, self.w), np.float32)
        rgb = np.zeros((self.h, self.w, 3), np.uint8) if self.colour else None
        rc = self.lib.k2_get_raw(self.handle, depth.ctypes.data,
                                 None if rgb is None else rgb.ctypes.data)
        if not rc:
            return None, None
        return (rgb if rc & 2 else None), depth

    def intrinsics(self):
        """IR camera fx, fy, cx, cy - needed to unproject depth to XYZ."""
        fx, fy = ctypes.c_float(), ctypes.c_float()
        cx, cy = ctypes.c_float(), ctypes.c_float()
        if not self.lib.k2_intrinsics(self.handle, ctypes.byref(fx), ctypes.byref(fy),
                                      ctypes.byref(cx), ctypes.byref(cy)):
            return None
        return fx.value, fy.value, cx.value, cy.value

    def set_filters(self, temporal, median, erode):
        """temporal 0-90 (% of previous frame), median 0/1, erode 0-5 px."""
        self.lib.k2_set_filters(self.handle, int(temporal), int(median), int(erode))

    def reopen(self):
        """Re-acquire after a USB dropout. The device re-enumerates on its own."""
        self.close()
        self.handle = self.lib.k2_open()
        if not self.handle:
            raise RuntimeError("device not back on the bus yet")
        self.serial = self.lib.k2_serial(self.handle).decode(errors="replace")

    def close(self):
        if self.handle:
            self.lib.k2_close(self.handle)
            self.handle = None


class Viewer:
    def __init__(self, root):
        self.root = root
        self.latest = None        # depth greyscale
        self.latest_rgb = None    # colour, background removed
        self.image = None
        self.running = True
        self.dropped = False
        self.note = ""
        self.log = EventLog(LOG_DIR)
        # Plain mirrors of the Tk variables. Tk objects are not thread-safe, so
        # the grab thread must never touch them - draw() refreshes these instead.
        self.near_v = NEAR_DEFAULT
        self.far_v = FAR_DEFAULT
        self.mode_v = "depth"
        self.total = 0
        self.beat = time.monotonic()
        self.frames = 0
        self.fps = 0.0
        self.since = time.monotonic()

        root.title("Kinect Depth")
        root.configure(bg=BG)
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.quit)

        try:
            # Colour costs about half the depth frame rate, even unread,
            # because the stream still eats USB bandwidth. Opt out for 30 fps.
            self.sensor = Sensor(colour=os.environ.get("KINECT_NO_COLOUR") != "1")
        except Exception as exc:
            self.sensor = None
            tk.Label(root, text=str(exc), bg=BG, fg=RED, padx=30, pady=40,
                     font=("Helvetica Neue", 14)).pack()
            return

        self.header = b"P5\n%d %d\n255\n" % (self.sensor.w, self.sensor.h)
        self.rgb_header = b"P6\n%d %d\n255\n" % (self.sensor.w, self.sensor.h)
        self.mode = tk.StringVar(value="depth")

        self.canvas = tk.Canvas(
            root, width=self.sensor.w, height=self.sensor.h,
            bg="black", highlightthickness=0, bd=0,
        )
        self.canvas.pack()
        self.item = self.canvas.create_image(0, 0, anchor="nw")

        panel = tk.Frame(root, bg=BG, padx=16, pady=10)
        panel.pack(fill="x")

        self.near = tk.IntVar(value=NEAR_DEFAULT)
        self.far = tk.IntVar(value=FAR_DEFAULT)
        for label, var in (("Near (mm)", self.near), ("Far (mm)", self.far)):
            row = tk.Frame(panel, bg=BG)
            row.pack(fill="x")
            tk.Label(row, text=label, bg=BG, fg=DIM, width=9, anchor="w",
                     font=("Helvetica Neue", 12)).pack(side="left")
            tk.Scale(row, from_=LIMIT_MIN, to=LIMIT_MAX, variable=var,
                     orient="horizontal", length=380, bg=BG, fg=FG,
                     troughcolor="#23262d", highlightthickness=0,
                     font=("Helvetica Neue", 10)).pack(side="left")

        modes = tk.Frame(panel, bg=BG)
        modes.pack(fill="x", pady=(6, 0))
        tk.Label(modes, text="Feed", bg=BG, fg=DIM, width=9, anchor="w",
                 font=("Helvetica Neue", 12)).pack(side="left")
        for value, text in (("depth", "Depth"), ("colour", "Colour, bg removed")):
            tk.Radiobutton(modes, text=text, variable=self.mode, value=value,
                           bg=BG, fg=DIM, selectcolor=BG, activebackground=BG,
                           activeforeground=FG, highlightthickness=0,
                           font=("Helvetica Neue", 12)).pack(side="left", padx=(0, 10))

        bar = tk.Frame(panel, bg=BG)
        bar.pack(fill="x", pady=(6, 0))
        self.button = tk.Button(bar, text="Reconnect", command=self.reconnect,
                                highlightbackground=BG, font=("Helvetica Neue", 12))
        self.button.pack(side="left")
        self.status = tk.Label(bar, text="", bg=BG, fg=DIM, anchor="w",
                               font=("Helvetica Neue", 11))
        self.status.pack(side="left", padx=(12, 0))

        root.bind("<Escape>", lambda _e: self.quit())

        self.log.write("START", "serial %s" % self.sensor.serial)
        threading.Thread(target=self.grab, daemon=True).start()
        self.draw()

    def grab(self):
        """Pull frames off the sensor as fast as it delivers them.

        Two consecutive misses (10s) means the sensor fell off the bus - very
        common on a shared USB hub. Bail out and let the user reconnect rather
        than hammering a dead handle."""
        misses = 0
        while self.running and not self.dropped:
            near, far = self.near_v, self.far_v
            if far <= near:
                far = near + 50
            data, colour = self.sensor.frame(near, far, self.mode_v == "colour")
            if colour is not None:
                self.latest_rgb = colour
            if data:
                misses = 0
                self.latest = data
                self.frames += 1
                self.total += 1
            else:
                misses += 1
                self.log.misses += 1
                self.log.write("MISS", "no frame for 5s (miss %d)" % self.log.misses)
                if misses >= 2:
                    self.dropped = True
                    self.log.dropouts += 1
                    self.note = "sensor dropped off USB - press Reconnect"
                    self.log.write("DROPOUT",
                                   "dropout %d after %d frames"
                                   % (self.log.dropouts, self.total))

    def draw(self):
        if not self.running:
            return
        self.near_v = self.near.get()      # main thread: refresh the mirrors
        self.far_v = self.far.get()
        self.mode_v = self.mode.get()
        if self.mode_v == "colour" and self.latest_rgb:
            payload = self.rgb_header + self.latest_rgb
        elif self.latest:
            payload = self.header + self.latest
        else:
            payload = None
        if payload:
            # Keep a reference; Tk drops the pixels if the object is collected.
            self.image = tk.PhotoImage(data=payload)
            self.canvas.itemconfigure(self.item, image=self.image)

        now = time.monotonic()
        if now - self.since >= 1.0:
            self.fps = self.frames / (now - self.since)
            self.frames, self.since = 0, now
        now2 = time.monotonic()
        if now2 - self.beat >= 60.0:          # heartbeat, so a silent hang is visible
            self.beat = now2
            self.log.write("OK", "%.1f fps, %d frames, %d misses, %d dropouts"
                           % (self.fps, self.total, self.log.misses, self.log.dropouts))
        if self.note:
            self.status.configure(text=self.note, fg=RED)
        else:
            self.status.configure(
                text="up %s   %.1f fps   %d frames   %d drops   slab %d-%d mm"
                % (self.log.uptime(), self.fps, self.total, self.log.dropouts,
                   self.near_v, self.far_v),
                fg=DIM,
            )
        self.root.after(33, self.draw)

    def reconnect(self):
        """Reopen the device and restart the grab loop."""
        self.button.configure(state="disabled", text="Connecting...")
        self.note = "reopening..."

        def work():
            try:
                self.sensor.reopen()
            except Exception as exc:
                self.note = "%s" % exc
                self.log.write("RECONNECT-FAIL", str(exc))
            else:
                self.dropped = False
                self.note = ""
                self.log.write("RECONNECT", "ok, serial %s" % self.sensor.serial)
                threading.Thread(target=self.grab, daemon=True).start()
            self.root.after(0, lambda: self.button.configure(
                state="normal", text="Reconnect"))

        threading.Thread(target=work, daemon=True).start()

    def quit(self):
        self.running = False
        if getattr(self, "log", None):
            self.log.write("EXIT", "clean, %d frames, %d misses, %d dropouts"
                           % (self.total, self.log.misses, self.log.dropouts))
        if getattr(self, "sensor", None):
            self.root.after(200, self._teardown)
        else:
            self.root.destroy()

    def _teardown(self):
        self.sensor.close()
        self.root.destroy()


if __name__ == "__main__":
    import signal

    LOG_DIR = os.environ.get("KINECT_LOG_DIR", os.path.join(HERE, "logs"))
    root = tk.Tk()
    root.geometry("+60+60")
    app = Viewer(root)

    # Without this a plain `kill` tears the process down without closing the
    # device, and the next program to open the Kinect hangs in k2_open.
    def _bye(_sig, _frame):
        app.quit()

    signal.signal(signal.SIGTERM, _bye)
    signal.signal(signal.SIGINT, _bye)
    root.mainloop()
