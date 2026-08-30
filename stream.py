#!/usr/bin/env python3
"""Serve the Kinect feeds as MJPEG over HTTP, so other apps can consume them.

Two streams from one sensor session:

    http://127.0.0.1:8010/depth.mjpg    gated depth, near-bright / far-dim
    http://127.0.0.1:8010/colour.mjpg   colour with the background removed
    http://127.0.0.1:8010/              index page with both

Consumers:
  TouchDesigner  Video Stream In TOP, paste the URL
  OBS            Media Source, uncheck "Local File", paste the URL,
                 then Start Virtual Camera to reach Zoom / Meet / browsers
  Browser        just open the URL

Frames are encoded once per feed and shared by every client, so extra
viewers cost bandwidth but not CPU.

    /opt/homebrew/bin/python3.14 stream.py [--port 8010] [--quality 80]
                                           [--near 500] [--far 2000] [--no-colour]
"""

import argparse
import ctypes
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, "/Users/pablognecco/kinect-check")
from depth_view import Sensor

TURBO = "/opt/homebrew/lib/libturbojpeg.dylib"
TJPF_RGB, TJPF_GRAY = 0, 6
TJSAMP_420, TJSAMP_GRAY = 2, 3
TJFLAG_FASTDCT = 2048
BOUNDARY = "kinectframe"


class Jpeg:
    """Minimal libjpeg-turbo wrapper. One instance per encoding thread."""

    def __init__(self):
        self.lib = ctypes.CDLL(TURBO)
        self.lib.tjInitCompress.restype = ctypes.c_void_p
        self.lib.tjCompress2.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int,
            ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)),
            ctypes.POINTER(ctypes.c_ulong), ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ]
        self.lib.tjCompress2.restype = ctypes.c_int
        self.lib.tjFree.argtypes = [ctypes.POINTER(ctypes.c_ubyte)]
        self.lib.tjGetErrorStr.restype = ctypes.c_char_p
        self.handle = self.lib.tjInitCompress()
        if not self.handle:
            raise RuntimeError("tjInitCompress failed")

    def encode(self, raw, width, height, grey, quality):
        pixfmt = TJPF_GRAY if grey else TJPF_RGB
        pitch = width if grey else width * 3
        subsamp = TJSAMP_GRAY if grey else TJSAMP_420
        src = (ctypes.c_ubyte * len(raw)).from_buffer_copy(raw)
        out = ctypes.POINTER(ctypes.c_ubyte)()
        size = ctypes.c_ulong(0)
        rc = self.lib.tjCompress2(
            self.handle, src, width, pitch, height, pixfmt,
            ctypes.byref(out), ctypes.byref(size), subsamp, quality, TJFLAG_FASTDCT,
        )
        if rc != 0:
            raise RuntimeError(self.lib.tjGetErrorStr().decode(errors="replace"))
        try:
            return ctypes.string_at(out, size.value)
        finally:
            self.lib.tjFree(out)


class Feeds:
    """Owns the sensor. Encodes each frame once; clients share the result."""

    def __init__(self, args):
        self.args = args
        self.sensor = Sensor(colour=not args.no_colour)
        self.jpeg = Jpeg()
        self.lock = threading.Condition()
        self.frames = {"depth": None, "colour": None}
        self.seq = 0
        self.count = 0
        self.fps = 0.0
        self.started = time.monotonic()
        self.running = True
        threading.Thread(target=self.pump, daemon=True).start()

    def pump(self):
        window, since = 0, time.monotonic()
        while self.running:
            grey, rgb = self.sensor.frame(
                self.args.near, self.args.far, not self.args.no_colour)
            if grey is None:
                continue
            encoded = {"depth": self.jpeg.encode(
                grey, self.sensor.w, self.sensor.h, True, self.args.quality)}
            if rgb is not None:
                encoded["colour"] = self.jpeg.encode(
                    rgb, self.sensor.w, self.sensor.h, False, self.args.quality)
            with self.lock:
                self.frames.update(encoded)
                self.seq += 1
                self.count += 1
                self.lock.notify_all()
            window += 1
            now = time.monotonic()
            if now - since >= 2.0:
                self.fps = window / (now - since)
                window, since = 0, now

    def wait(self, name, last):
        """Block until a frame newer than `last` exists. Returns (seq, jpeg)."""
        with self.lock:
            while self.running and (self.seq == last or self.frames.get(name) is None):
                if not self.lock.wait(timeout=10.0):
                    return last, None
            return self.seq, self.frames.get(name)

    def close(self):
        self.running = False
        with self.lock:
            self.lock.notify_all()
        self.sensor.close()


FEEDS = None

INDEX = """<!doctype html><meta charset=utf-8><title>Kinect feeds</title>
<style>body{font:15px -apple-system,sans-serif;background:#16181d;color:#e8eaed;
margin:0;padding:28px}h1{font-size:19px;margin:0 0 4px}p{color:#8b919b}
a{color:#3fbf6f}img{background:#000;border-radius:6px;margin:8px 16px 0 0;width:384px}
code{background:#23262d;padding:2px 6px;border-radius:4px}</style>
<h1>Kinect feeds</h1><p>%s &middot; %.1f fps</p>
<img src="/depth.mjpg"><img src="/colour.mjpg">
<p>TouchDesigner: Video Stream In TOP &rarr;
<code>http://127.0.0.1:%d/depth.mjpg</code></p>
<p>OBS: Media Source, uncheck Local File, same URL, then Start Virtual Camera.</p>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, *a):
        pass  # keep the console readable

    def do_GET(self):
        route = self.path.split("?")[0]
        if route == "/":
            body = (INDEX % (FEEDS.sensor.serial, FEEDS.fps, FEEDS.args.port)).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        name = {"/depth.mjpg": "depth", "/colour.mjpg": "colour"}.get(route)
        if not name:
            self.send_error(404)
            return
        if name == "colour" and FEEDS.args.no_colour:
            self.send_error(503, "colour stream disabled")
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=%s" % BOUNDARY)
        self.end_headers()

        last = 0
        try:
            while True:
                last, jpeg = FEEDS.wait(name, last)
                if jpeg is None:
                    break
                self.wfile.write(b"--%s\r\n" % BOUNDARY.encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass  # viewer closed the tab; normal


def main():
    global FEEDS
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8010)
    p.add_argument("--quality", type=int, default=80)
    p.add_argument("--near", type=int, default=500)
    p.add_argument("--far", type=int, default=2000)
    p.add_argument("--no-colour", action="store_true",
                   help="depth only, roughly doubles the frame rate")
    args = p.parse_args()

    FEEDS = Feeds(args)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print("Kinect %s streaming on http://127.0.0.1:%d" % (FEEDS.sensor.serial, args.port))
    print("  depth : http://127.0.0.1:%d/depth.mjpg" % args.port)
    if not args.no_colour:
        print("  colour: http://127.0.0.1:%d/colour.mjpg" % args.port)
    print("Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        FEEDS.close()
        server.shutdown()


if __name__ == "__main__":
    main()
