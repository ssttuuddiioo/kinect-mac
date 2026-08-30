#!/usr/bin/env python3
"""Kinect Check - status window for Kinect v1 and v2 sensors.

Kinect v1 (Xbox 360) is driven by libfreenect, so this can open its streams and
confirm depth and colour really deliver a frame.

Kinect v2 (Xbox One) speaks a different USB protocol and needs libfreenect2,
which Homebrew does not package. So v2 is detected over USB - model, port,
link speed - but its streams cannot be opened here. See V2_NOTE below.

Must run under an arm64 Python. On this Mac:
    /opt/homebrew/bin/python3.14 kinect_check.py          # window
    /opt/homebrew/bin/python3.14 kinect_check.py --cli    # one-shot text probe

/usr/local/bin/python3 is an x86_64 leftover and cannot load the arm64 dylibs.
/usr/bin/python3 works but ships Apple's Tk 8.5, which draws Label text
invisibly on macOS 15 - hence the Canvas-based UI below.
"""

import ctypes
import queue
import sys
import threading
import time
import tkinter as tk

LIB_DIR = "/opt/homebrew/lib"
FREENECT = LIB_DIR + "/libfreenect.dylib"
FREENECT_SYNC = LIB_DIR + "/libfreenect_sync.dylib"
LIBUSB = LIB_DIR + "/libusb-1.0.dylib"

FREENECT_LOG_FATAL = 0
FREENECT_DEPTH_11BIT = 0
FREENECT_VIDEO_RGB = 0

MS_VID = 0x045E

# Product ids from the libfreenect and libfreenect2 sources.
V1_PIDS = {
    0x02AE: "Kinect v1 camera",
    0x02B0: "Kinect v1 motor",
    0x02AD: "Kinect v1 audio",
    0x02BF: "Kinect for Windows v1 camera",
    0x02BE: "Kinect for Windows v1 audio",
    0x02C2: "Kinect for Windows v1 camera",
}
V2_PIDS = {
    0x02C4: "Kinect v2 sensor",
    0x02D8: "Kinect v2 sensor",
    0x02D9: "Kinect v2 sensor",
}

SPEEDS = {
    0: "unknown",
    1: "1.5 Mbps (low)",
    2: "12 Mbps (full)",
    3: "480 Mbps (USB 2)",
    4: "5 Gbps (USB 3)",
    5: "10 Gbps (USB 3.1)",
}
SPEED_SUPER = 4  # v2 needs at least this

V2_NOTE = "v2 detected - streams need libfreenect2"

PROBE_TIMEOUT = 15.0
POLL_MS = 2000

BG = "#16181d"
FG = "#e8eaed"
DIM = "#8b919b"
GREEN = "#3fbf6f"
RED = "#e2564d"
AMBER = "#e0a33e"


# --- libfreenect (Kinect v1) ------------------------------------------------

class DeviceAttributes(ctypes.Structure):
    pass


DeviceAttributes._fields_ = [
    ("next", ctypes.POINTER(DeviceAttributes)),
    ("camera_serial", ctypes.c_char_p),
]


class Freenect:
    """Thin ctypes wrapper over the bits of libfreenect we need."""

    def __init__(self):
        self.lib = ctypes.CDLL(FREENECT)
        self.sync = ctypes.CDLL(FREENECT_SYNC)

        self.lib.freenect_init.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_void_p]
        self.lib.freenect_init.restype = ctypes.c_int
        self.lib.freenect_shutdown.argtypes = [ctypes.c_void_p]
        self.lib.freenect_shutdown.restype = ctypes.c_int
        self.lib.freenect_set_log_level.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.lib.freenect_set_log_level.restype = None
        self.lib.freenect_num_devices.argtypes = [ctypes.c_void_p]
        self.lib.freenect_num_devices.restype = ctypes.c_int
        self.lib.freenect_list_device_attributes.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(DeviceAttributes))
        ]
        self.lib.freenect_list_device_attributes.restype = ctypes.c_int
        self.lib.freenect_free_device_attributes.argtypes = [ctypes.POINTER(DeviceAttributes)]
        self.lib.freenect_free_device_attributes.restype = None

        for name in ("freenect_sync_get_depth", "freenect_sync_get_video"):
            fn = getattr(self.sync, name)
            fn.argtypes = [
                ctypes.POINTER(ctypes.c_void_p),
                ctypes.POINTER(ctypes.c_uint32),
                ctypes.c_int,
                ctypes.c_int,
            ]
            fn.restype = ctypes.c_int
        self.sync.freenect_sync_stop.argtypes = []
        self.sync.freenect_sync_stop.restype = None

    def scan(self):
        """How many v1 devices, and their serials. Opens no stream."""
        ctx = ctypes.c_void_p()
        if self.lib.freenect_init(ctypes.byref(ctx), None) < 0:
            raise RuntimeError("freenect_init failed")
        try:
            self.lib.freenect_set_log_level(ctx, FREENECT_LOG_FATAL)
            attrs = ctypes.POINTER(DeviceAttributes)()
            count = self.lib.freenect_list_device_attributes(ctx, ctypes.byref(attrs))
            serials = []
            if count > 0 and attrs:
                node = attrs
                while node:
                    raw = node.contents.camera_serial
                    serials.append(raw.decode(errors="replace") if raw else "unknown")
                    node = node.contents.next
                self.lib.freenect_free_device_attributes(attrs)
            if count < 0:
                count = max(self.lib.freenect_num_devices(ctx), 0)
            return count, serials
        finally:
            self.lib.freenect_shutdown(ctx)

    def grab(self, stream, index=0):
        """Try to pull one frame. True if the stream really delivered."""
        buf = ctypes.c_void_p()
        stamp = ctypes.c_uint32()
        fn = self.sync.freenect_sync_get_depth if stream == "depth" else self.sync.freenect_sync_get_video
        fmt = FREENECT_DEPTH_11BIT if stream == "depth" else FREENECT_VIDEO_RGB
        rc = fn(ctypes.byref(buf), ctypes.byref(stamp), index, fmt)
        return rc == 0 and bool(buf)

    def release(self):
        """Hand the device back so glview and friends can open it."""
        try:
            self.sync.freenect_sync_stop()
        except Exception:
            pass


# --- libusb (hardware detection for both generations) -----------------------

class UsbDescriptor(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8),
        ("bDescriptorType", ctypes.c_uint8),
        ("bcdUSB", ctypes.c_uint16),
        ("bDeviceClass", ctypes.c_uint8),
        ("bDeviceSubClass", ctypes.c_uint8),
        ("bDeviceProtocol", ctypes.c_uint8),
        ("bMaxPacketSize0", ctypes.c_uint8),
        ("idVendor", ctypes.c_uint16),
        ("idProduct", ctypes.c_uint16),
        ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8),
        ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8),
        ("bNumConfigurations", ctypes.c_uint8),
    ]


class Usb:
    """Enumerate Microsoft sensors over USB, whichever generation."""

    def __init__(self):
        self.lib = ctypes.CDLL(LIBUSB)
        self.lib.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.libusb_init.restype = ctypes.c_int
        self.lib.libusb_exit.argtypes = [ctypes.c_void_p]
        self.lib.libusb_exit.restype = None
        self.lib.libusb_get_device_list.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))
        ]
        self.lib.libusb_get_device_list.restype = ctypes.c_ssize_t
        self.lib.libusb_free_device_list.argtypes = [
            ctypes.POINTER(ctypes.c_void_p), ctypes.c_int
        ]
        self.lib.libusb_free_device_list.restype = None
        self.lib.libusb_get_device_descriptor.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(UsbDescriptor)
        ]
        self.lib.libusb_get_device_descriptor.restype = ctypes.c_int
        for name in ("libusb_get_device_speed", "libusb_get_bus_number",
                     "libusb_get_device_address"):
            fn = getattr(self.lib, name)
            fn.argtypes = [ctypes.c_void_p]
            fn.restype = ctypes.c_int

    def scan(self):
        """Every Microsoft VID device on the bus, tagged by generation."""
        ctx = ctypes.c_void_p()
        if self.lib.libusb_init(ctypes.byref(ctx)) < 0:
            raise RuntimeError("libusb_init failed")
        found = []
        try:
            devices = ctypes.POINTER(ctypes.c_void_p)()
            n = self.lib.libusb_get_device_list(ctx, ctypes.byref(devices))
            if n < 0:
                raise RuntimeError("libusb_get_device_list failed")
            try:
                for i in range(n):
                    dev = devices[i]
                    desc = UsbDescriptor()
                    if self.lib.libusb_get_device_descriptor(dev, ctypes.byref(desc)) < 0:
                        continue
                    if desc.idVendor != MS_VID:
                        continue
                    pid = desc.idProduct
                    if pid in V2_PIDS:
                        gen, name = 2, V2_PIDS[pid]
                    elif pid in V1_PIDS:
                        gen, name = 1, V1_PIDS[pid]
                    else:
                        gen, name = 0, "unrecognised Microsoft device"
                    found.append({
                        "gen": gen,
                        "name": name,
                        "pid": pid,
                        "speed": self.lib.libusb_get_device_speed(dev),
                        "bus": self.lib.libusb_get_bus_number(dev),
                        "address": self.lib.libusb_get_device_address(dev),
                    })
            finally:
                self.lib.libusb_free_device_list(devices, 1)
        finally:
            self.lib.libusb_exit(ctx)
        return found


# --- probing ----------------------------------------------------------------

def full_probe(freenect, usb):
    """Detect both generations; open streams on v1, which is all we can drive."""
    result = {
        "v1_count": 0, "v1_serials": [], "v2": [], "unknown": [],
        "depth": None, "video": None, "speed": None, "error": None, "usb_error": None,
    }

    try:
        devices = usb.scan()
        result["v2"] = [d for d in devices if d["gen"] == 2]
        result["unknown"] = [d for d in devices if d["gen"] == 0]
        v1_usb = [d for d in devices if d["gen"] == 1]
        for group in (result["v2"], v1_usb):
            if group:
                result["speed"] = group[0]["speed"]
                break
    except Exception as exc:
        result["usb_error"] = str(exc)

    try:
        result["v1_count"], result["v1_serials"] = freenect.scan()
        if result["v1_count"] > 0:
            result["depth"] = freenect.grab("depth")
            result["video"] = freenect.grab("video")
            freenect.release()
    except Exception as exc:
        result["error"] = str(exc)

    return result


def quick_count(freenect, usb):
    """Cheap 'has anything changed' number for the auto-detect poll."""
    total = 0
    try:
        total += freenect.scan()[0]
    except Exception:
        pass
    try:
        total += sum(1 for d in usb.scan() if d["gen"] in (0, 2))
    except Exception:
        pass
    return total


# --- window -----------------------------------------------------------------

class App:
    """Text is drawn on a Canvas rather than with Labels - Apple's bundled
    Tk 8.5 renders Label text invisibly on recent macOS."""

    W, H = 460, 290

    def __init__(self, root):
        self.root = root
        self.results = queue.Queue()
        self.seq = 0
        self.pending = None
        self.last_count = None
        self.freenect = None
        self.usb = None
        self.error = None

        try:
            self.freenect = Freenect()
            self.usb = Usb()
        except Exception as exc:
            self.error = "Cannot load libraries: %s" % exc

        root.title("Kinect Check")
        root.configure(bg=BG)
        root.resizable(False, False)

        self.canvas = tk.Canvas(
            root, width=self.W, height=self.H, bg=BG, highlightthickness=0, bd=0,
        )
        self.canvas.pack()

        def text(x, y, size, colour, weight="normal", content=""):
            return self.canvas.create_text(
                x, y, text=content, fill=colour, anchor="w",
                font=("Helvetica Neue", size, weight),
            )

        self.headline = text(24, 38, 30, DIM, "bold", "CHECKING...")
        self.subline = text(24, 70, 12, DIM)

        self.rows = {}
        y = 116
        for key, label in (("device", "Device"), ("link", "Link"),
                           ("depth", "Depth"), ("video", "RGB video")):
            text(24, y, 13, DIM, "normal", label)
            self.rows[key] = text(150, y, 15, DIM, "bold", "-")
            y += 30

        self.status = text(24, 258, 11, DIM)

        controls = tk.Frame(root, bg=BG)
        controls.pack(fill="x", padx=24, pady=(0, 18))

        self.button = tk.Button(
            controls, text="Refresh", command=self.refresh,
            highlightbackground=BG, font=("Helvetica Neue", 13),
        )
        self.button.pack(side="left")

        self.auto = tk.BooleanVar(value=True)
        tk.Checkbutton(
            controls, text="auto-detect", variable=self.auto, bg=BG, fg=DIM,
            selectcolor=BG, activebackground=BG, activeforeground=FG,
            highlightthickness=0, font=("Helvetica Neue", 12),
        ).pack(side="left", padx=(12, 0))

        root.bind("<r>", lambda _e: self.refresh())
        root.bind("<Return>", lambda _e: self.refresh())
        root.bind("<Escape>", lambda _e: root.destroy())

        if self.error:
            self.show_error(self.error)
        else:
            self.refresh()
            self.root.after(POLL_MS, self.tick)
        self.root.after(120, self.drain)

    def set(self, item, content, colour=None):
        self.canvas.itemconfigure(item, text=content)
        if colour:
            self.canvas.itemconfigure(item, fill=colour)

    def get(self, item):
        return self.canvas.itemcget(item, "text")

    # --- work dispatch -----------------------------------------------------

    def refresh(self):
        if self.error or self.pending:
            return
        self.seq += 1
        seq = self.seq
        self.pending = (seq, time.monotonic())
        self.button.configure(state="disabled", text="Checking...")
        self.set(self.status, "Probing - this takes a few seconds.", DIM)

        def work():
            self.results.put((seq, full_probe(self.freenect, self.usb)))

        threading.Thread(target=work, daemon=True).start()

    def tick(self):
        self.root.after(POLL_MS, self.tick)
        if self.error or self.pending or not self.auto.get():
            return
        count = quick_count(self.freenect, self.usb)
        if count != self.last_count:
            self.refresh()

    def drain(self):
        self.root.after(120, self.drain)
        try:
            seq, result = self.results.get_nowait()
        except queue.Empty:
            if self.pending and time.monotonic() - self.pending[1] > PROBE_TIMEOUT:
                self.pending = None
                self.button.configure(state="normal", text="Refresh")
                self.set(self.headline, "NOT RESPONDING", AMBER)
                self.set(self.subline, "Device is there but never sent a frame.", DIM)
                for item in self.rows.values():
                    self.set(item, "timed out", AMBER)
                self.set(self.status, "Try a powered USB port or reseat the adapter.", AMBER)
            return
        if not self.pending or seq != self.pending[0]:
            return
        self.pending = None
        self.button.configure(state="normal", text="Refresh")
        self.render(result)

    # --- rendering ---------------------------------------------------------

    def render(self, result):
        if result["error"] and not result["v2"]:
            self.show_error(result["error"])
            return

        v1 = result["v1_count"]
        v2 = result["v2"]
        self.last_count = v1 + len(v2) + len(result["unknown"])

        speed = result["speed"]
        if speed is None:
            self.set(self.rows["link"], "-", DIM)
        else:
            fast_enough = speed >= SPEED_SUPER or not v2
            self.set(self.rows["link"], SPEEDS.get(speed, "unknown"),
                     GREEN if fast_enough else RED)

        if not v1 and not v2:
            self.set(self.headline, "NO CAMERA", RED)
            if result["unknown"]:
                self.set(self.subline, "Microsoft device seen, but not a Kinect.", AMBER)
                self.set(self.rows["device"],
                         "unknown 045e:%04x" % result["unknown"][0]["pid"], AMBER)
            else:
                self.set(self.subline, "Plug in a Kinect, then hit Refresh.", DIM)
                self.set(self.rows["device"], "not connected", RED)
            self.set(self.rows["depth"], "-", DIM)
            self.set(self.rows["video"], "-", DIM)
            self.set(self.status, "Kinect v1 also needs its 12V power brick.", DIM)
            return

        # --- v2 present, with or without a v1 alongside
        if v2 and not v1:
            self.set(self.headline, "KINECT V2", AMBER)
            self.set(self.subline, "Detected over USB. Streams need libfreenect2.", DIM)
            self.set(self.rows["device"],
                     "%s (bus %d.%d)" % (v2[0]["name"], v2[0]["bus"], v2[0]["address"]),
                     GREEN)
            for key in ("depth", "video"):
                self.set(self.rows[key], "not checkable", AMBER)
            if speed is not None and speed < SPEED_SUPER:
                self.set(self.status, "On a USB 2 port - v2 needs USB 3 to stream.", RED)
            else:
                self.set(self.status, "Hardware looks right. libfreenect2 not installed.", AMBER)
            return

        # --- v1 present (the case we can actually verify)
        serial = result["v1_serials"][0] if result["v1_serials"] else "unknown"
        extra = " (+%d more)" % (v1 - 1) if v1 > 1 else ""
        self.set(self.rows["device"], "v1 %s%s" % (serial, extra), GREEN)

        for key in ("depth", "video"):
            ok = result[key]
            self.set(self.rows[key], "yes" if ok else "no", GREEN if ok else RED)

        also_v2 = "  (v2 also attached)" if v2 else ""
        if result["depth"] and result["video"]:
            self.set(self.headline, "CAMERA OK", GREEN)
            self.set(self.subline, "Depth and colour are both streaming.", DIM)
            self.set(self.status, "Ready. freenect-glview should work." + also_v2, DIM)
        elif result["depth"] or result["video"]:
            missing = "colour" if result["depth"] else "depth"
            self.set(self.headline, "PARTIAL", AMBER)
            self.set(self.subline, "Connected, but no %s stream." % missing, DIM)
            self.set(self.status, "Reseat the cable and refresh." + also_v2, AMBER)
        else:
            self.set(self.headline, "NO STREAMS", AMBER)
            self.set(self.subline, "Device enumerates but sends nothing.", DIM)
            self.set(self.status, "Usually means the 12V brick is unplugged." + also_v2, AMBER)

    def show_error(self, message):
        self.set(self.headline, "ERROR", RED)
        self.set(self.subline, message[:64], DIM)
        for item in self.rows.values():
            self.set(item, "-", DIM)
        self.button.configure(state="normal", text="Refresh")
        self.set(self.status, "Check that libfreenect is installed.", RED)


# --- terminal ---------------------------------------------------------------

def cli():
    try:
        freenect, usb = Freenect(), Usb()
    except Exception as exc:
        print("cannot load libraries:", exc)
        return 2

    r = full_probe(freenect, usb)
    if r["usb_error"]:
        print("usb scan failed:", r["usb_error"])
    if r["error"] and not r["v2"]:
        print("error:", r["error"])
        return 2

    v1, v2 = r["v1_count"], r["v2"]
    if not v1 and not v2:
        for d in r["unknown"]:
            print("unknown:   Microsoft 045e:%04x (not a recognised Kinect)" % d["pid"])
        print("camera:    no")
        print("depth:     -")
        print("rgb video: -")
        return 1

    if r["speed"] is not None:
        print("link:      %s" % SPEEDS.get(r["speed"], "unknown"))

    if v2:
        for d in v2:
            print("camera:    yes - %s (045e:%04x, bus %d.%d)"
                  % (d["name"], d["pid"], d["bus"], d["address"]))
        if r["speed"] is not None and r["speed"] < SPEED_SUPER:
            print("           WARNING: USB 2 link, v2 needs USB 3")
        if not v1:
            print("depth:     %s" % V2_NOTE)
            print("rgb video: %s" % V2_NOTE)
            return 1

    if v1:
        print("camera:    yes - Kinect v1 (%d device%s, serial %s)"
              % (v1, "" if v1 == 1 else "s",
                 r["v1_serials"][0] if r["v1_serials"] else "unknown"))
        print("depth:     %s" % ("yes" if r["depth"] else "no"))
        print("rgb video: %s" % ("yes" if r["video"] else "no"))
        return 0 if (r["depth"] and r["video"]) else 1
    return 1


def main():
    if "--cli" in sys.argv:
        return cli()
    root = tk.Tk()
    root.geometry("+60+60")
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
