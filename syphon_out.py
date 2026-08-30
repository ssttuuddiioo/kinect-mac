#!/usr/bin/env python3
"""Publish the Kinect feeds as Syphon servers - real-time, GPU-side.

Two servers appear to any Syphon-aware app:

    Kinect Depth    gated depth, near-bright / far-dim
    Kinect Colour   colour with the background removed

In TouchDesigner: add a Syphon Spout In TOP and pick the server from the menu.
Unlike the MJPEG route this has no encode/decode step and no buffering, so it
runs at sensor latency.

    /opt/homebrew/bin/python3.14 syphon_out.py [--near 500] [--far 2000] [--no-colour]
    /opt/homebrew/bin/python3.14 syphon_out.py --list    # show visible servers
"""

import argparse
import ctypes
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from depth_view import Sensor

SYPHON_LIB = os.path.join(HERE, "libk2syphon.dylib")
DEPTH, COLOUR, CLOUD = 0, 1, 2


class Syphon:
    def __init__(self, lib_path=SYPHON_LIB):
        self.lib = ctypes.CDLL(lib_path)
        self.lib.syphon_init.argtypes = [ctypes.c_char_p] * 3
        self.lib.syphon_init.restype = ctypes.c_int
        self.lib.syphon_publish.argtypes = [
            ctypes.c_int, ctypes.POINTER(ctypes.c_ubyte),
            ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ]
        self.lib.syphon_publish.restype = ctypes.c_int
        self.lib.syphon_has_clients.argtypes = [ctypes.c_int]
        self.lib.syphon_has_clients.restype = ctypes.c_int
        self.lib.syphon_servers.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self.lib.syphon_servers.restype = ctypes.c_int
        self.lib.syphon_pump.argtypes = []
        self.lib.syphon_shutdown.argtypes = []
        self.started = False

    def start(self, depth_name, colour_name, cloud_name):
        if not self.lib.syphon_init(depth_name.encode(), colour_name.encode(),
                                    cloud_name.encode()):
            raise RuntimeError("syphon_init failed (no GL context?)")
        self.started = True

    def publish(self, index, data, width, height, channels):
        # cast avoids copying the frame on every publish
        ptr = ctypes.cast(ctypes.c_char_p(data), ctypes.POINTER(ctypes.c_ubyte))
        return self.lib.syphon_publish(index, ptr, width, height, channels)

    def clients(self, index):
        return self.lib.syphon_has_clients(index)

    def servers(self):
        buf = ctypes.create_string_buffer(8192)
        n = self.lib.syphon_servers(buf, len(buf))
        rows = [r for r in buf.value.decode(errors="replace").split("\n") if r]
        return n, rows

    def stop(self):
        if self.started:
            self.lib.syphon_shutdown()
            self.started = False


def list_servers():
    n, rows = Syphon().servers()
    print("Syphon servers visible: %d" % n)
    for r in rows:
        app, name = (r.split("|", 1) + ["?"])[:2]
        print("   %-28s (app: %s)" % (name, app))
    return 0


BG, FG, DIM, GREEN, AMBER = "#16181d", "#e8eaed", "#8b919b", "#3fbf6f", "#e0a33e"


def gui_main(args):
    """Publisher plus a control panel.

    Ordering here is load-bearing. Tk must create NSApp first: libfreenect2's
    OpenGL depth pipeline uses GLFW, which installs its own NSApplication
    subclass, and whichever initialises NSApp first wins - the other then dies
    on an unrecognised selector. Syphon and libfreenect2 both build hidden
    NSWindows, so both must be constructed on the main thread. Only the frame
    loop is backgrounded; waitForNewFrame is thread-agnostic.
    """
    import threading
    import tkinter as tk

    state = {"near": args.near, "far": args.far, "temporal": 0, "median": 0,
             "erode": 0, "fps": 0.0, "clients": (0, 0, 0), "serial": "",
             "error": None, "go": True, "last_frame": 0.0, "frames": 0}

    # 1. Tk first, so it owns NSApp.
    root = tk.Tk()
    root.title("Kinect Filters")
    root.configure(bg=BG)
    root.geometry("+60+60")
    root.resizable(False, False)

    wrap = tk.Frame(root, bg=BG, padx=18, pady=14)
    wrap.pack()

    rows = [
        ("Near (mm)", "near", 300, 4500),
        ("Far (mm)", "far", 300, 4500),
        ("Smooth (%)", "temporal", 0, 90),
        ("Erode (px)", "erode", 0, 5),
        ("Despeckle", "median", 0, 3),
    ]
    variables = {}
    for label, key, lo, hi in rows:
        line = tk.Frame(wrap, bg=BG)
        line.pack(fill="x", pady=2)
        tk.Label(line, text=label, bg=BG, fg=DIM, width=11, anchor="w",
                 font=("Helvetica Neue", 12)).pack(side="left")
        var = tk.IntVar(value=state[key])
        tk.Scale(line, from_=lo, to=hi, variable=var, orient="horizontal",
                 length=300, bg=BG, fg=FG, troughcolor="#23262d",
                 highlightthickness=0, font=("Helvetica Neue", 10)).pack(side="left")
        variables[key] = var

    kernel = tk.Label(wrap, text="", bg=BG, fg=DIM, anchor="w",
                      font=("Helvetica Neue", 11))
    kernel.pack(fill="x", pady=(2, 0))

    status = tk.Label(wrap, text="opening sensor...", bg=BG, fg=DIM, anchor="w",
                      font=("Helvetica Neue", 11))
    status.pack(fill="x", pady=(10, 0))
    root.update()

    # 2. Syphon and the sensor, both on this thread.
    syphon = Syphon()
    syphon.start(args.depth_name, args.colour_name, args.cloud_name)
    sensor = Sensor(colour=not args.no_colour)
    sensor.enable_cloud(True)
    state["serial"] = sensor.serial
    k = sensor.intrinsics()
    if k:
        print("IR intrinsics  fx=%.3f fy=%.3f cx=%.3f cy=%.3f" % k, flush=True)
        print("Paste those into the GLSL TOP - see POINTCLOUD.md", flush=True)

    # 3. Frame loop in the background.
    def worker():
        frames, since = 0, time.monotonic()
        try:
            while state["go"]:
                sensor.set_filters(state["temporal"], state["median"], state["erode"])
                grey, rgb = sensor.frame(state["near"], state["far"],
                                         not args.no_colour)
                if grey is None:
                    continue
                syphon.publish(DEPTH, grey, sensor.w, sensor.h, 1)
                if rgb is not None:
                    syphon.publish(COLOUR, rgb, sensor.w, sensor.h, 3)
                cloud = sensor.cloud_frame()
                if cloud is not None:
                    syphon.publish(CLOUD, cloud, sensor.w, sensor.h, 3)
                frames += 1
                state["frames"] += 1
                now = time.monotonic()
                state["last_frame"] = now
                if now - since >= 1.0:
                    state["fps"] = frames / (now - since)
                    frames, since = 0, now
        finally:
            sensor.close()

    state["t0"] = time.monotonic()
    state["logged"] = 0.0
    threading.Thread(target=worker, daemon=True).start()

    def tick():
        for key, var in variables.items():
            state[key] = var.get()
        kernel.configure(text="despeckle kernel: %s"
                         % {0: "off", 1: "3x3", 2: "5x5", 3: "7x7"}[state["median"]])
        syphon.lib.syphon_pump()          # discovery, on the owning thread
        state["clients"] = (syphon.clients(DEPTH), syphon.clients(COLOUR),
                            syphon.clients(CLOUD))
        # A stalled sensor still advertises its Syphon servers, so silence is
        # indistinguishable from "idle" unless we say so explicitly.
        stalled = (state["last_frame"] > 0
                   and time.monotonic() - state["last_frame"] > 5.0)
        never = state["frames"] == 0 and time.monotonic() - state["t0"] > 15.0
        if state["error"]:
            status.configure(text=state["error"][:60], fg="#e2564d")
        elif never:
            status.configure(text="NO FRAMES - sensor never started. Restart me.",
                             fg="#e2564d")
        elif stalled:
            status.configure(text="STALLED - no frame for %ds"
                             % int(time.monotonic() - state["last_frame"]),
                             fg="#e2564d")
        else:
            d, c, p = state["clients"]
            status.configure(
                text="%s   %.1f fps   TD: depth=%d colour=%d cloud=%d"
                % (state["serial"] or "opening", state["fps"], d, c, p),
                fg=GREEN if (d or c or p) else DIM)

        # Also to stdout, so the log is diagnosable without seeing the window.
        if time.monotonic() - state["logged"] >= 10.0:
            state["logged"] = time.monotonic()
            print("%.1f fps  frames=%d  clients=%s%s"
                  % (state["fps"], state["frames"], state["clients"],
                     "  <-- NO FRAMES" if (never or stalled) else ""), flush=True)
        root.after(200, tick)

    def quit_now():
        state["go"] = False
        root.after(400, lambda: (syphon.stop(), root.destroy()))

    root.protocol("WM_DELETE_WINDOW", quit_now)
    root.bind("<Escape>", lambda _e: quit_now())

    # A plain `kill` must still release the Kinect, or the next program to
    # open it hangs. The 200ms tick keeps Python bytecode running so the
    # handler actually fires while Tk owns the loop.
    import signal
    signal.signal(signal.SIGTERM, lambda _s, _f: quit_now())
    signal.signal(signal.SIGINT, lambda _s, _f: quit_now())

    tick()
    root.mainloop()
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--near", type=int, default=500)
    p.add_argument("--far", type=int, default=2000)
    p.add_argument("--no-colour", action="store_true",
                   help="depth only, roughly doubles the frame rate")
    p.add_argument("--depth-name", default="Kinect Depth")
    p.add_argument("--colour-name", default="Kinect Colour")
    p.add_argument("--cloud-name", default="Kinect Cloud")
    p.add_argument("--list", action="store_true", help="list servers and exit")
    p.add_argument("--gui", action="store_true",
                   help="show a control panel with live filter sliders")
    args = p.parse_args()

    if args.list:
        return list_servers()
    if args.gui:
        return gui_main(args)

    syphon = Syphon()
    syphon.start(args.depth_name, args.colour_name, args.cloud_name)
    sensor = Sensor(colour=not args.no_colour)
    print("Kinect %s publishing to Syphon" % sensor.serial)
    print("   %s" % args.depth_name)
    if not args.no_colour:
        print("   %s" % args.colour_name)
    print("In TouchDesigner: Syphon Spout In TOP -> pick the server.")
    print("Ctrl-C to stop.")

    running = {"go": True}

    def stop(_sig, _frame):
        running["go"] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    frames, since = 0, time.monotonic()
    try:
        while running["go"]:
            grey, rgb = sensor.frame(args.near, args.far, not args.no_colour)
            if grey is None:
                continue
            syphon.publish(DEPTH, grey, sensor.w, sensor.h, 1)
            if rgb is not None:
                syphon.publish(COLOUR, rgb, sensor.w, sensor.h, 3)
            syphon.lib.syphon_pump()   # service Syphon discovery
            frames += 1
            now = time.monotonic()
            if now - since >= 3.0:
                print("%.1f fps   clients: depth=%d colour=%d"
                      % (frames / (now - since),
                         syphon.clients(DEPTH), syphon.clients(COLOUR)), flush=True)
                frames, since = 0, now
    finally:
        print("stopping...")
        syphon.stop()
        sensor.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
