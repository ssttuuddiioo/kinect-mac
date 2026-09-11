"""Camera backends behind one interface.

Every backend exposes the same surface, which is what the app talks to:

    cam.kind, cam.w, cam.h, cam.serial
    cam.frame(near_mm, far_mm, want_colour) -> (grey_bytes, rgb_bytes | None)
                                               or (None, None) on timeout
    cam.cloud_frame()            -> packed-depth bytes from the last frame
    cam.enable_cloud(on)
    cam.set_filters(temporal, median, erode)
    cam.intrinsics()             -> (fx, fy, cx, cy) for the OUTPUT grid
    cam.close()

Backends:
    kinect     Kinect v2 via libfreenect2 (the k2shim)
    femto      Orbbec Femto Mega via pyorbbecsdk2, processed through kproc
    synthetic  moving test pattern, no hardware - for testing the app

All three produce a horizontally mirrored image, exactly once, so the point
cloud shader in POINTCLOUD.md is correct for every camera.
"""

import ctypes
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KPROC = os.path.join(HERE, "libkproc.dylib")


class _KProc:
    """ctypes wrapper for the shared, camera-agnostic depth pipeline."""

    def __init__(self, w, h):
        lib = ctypes.CDLL(KPROC)
        lib.kproc_create.argtypes = [ctypes.c_int, ctypes.c_int]
        lib.kproc_create.restype = ctypes.c_void_p
        lib.kproc_set_filters.argtypes = [ctypes.c_void_p] + [ctypes.c_int] * 3
        lib.kproc_run.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
                                  ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                  ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
        lib.kproc_run.restype = ctypes.c_int
        lib.kproc_destroy.argtypes = [ctypes.c_void_p]
        self.lib = lib
        self.w, self.h = w, h
        self.handle = lib.kproc_create(w, h)
        if not self.handle:
            raise RuntimeError("kproc_create failed for %dx%d" % (w, h))
        self.grey = np.zeros((h, w), np.uint8)
        self.rgb = np.zeros((h, w, 3), np.uint8)
        self.cloud = np.zeros((h, w, 3), np.uint8)

    def set_filters(self, temporal, median, erode):
        self.lib.kproc_set_filters(self.handle, int(temporal), int(median), int(erode))

    def run(self, depth_mm, colour_rgb, near, far, mirror, want_cloud):
        depth = np.ascontiguousarray(depth_mm, dtype=np.float32)
        colour = None if colour_rgb is None else np.ascontiguousarray(colour_rgb, np.uint8)
        self.lib.kproc_run(
            self.handle, depth.ctypes.data,
            None if colour is None else colour.ctypes.data,
            int(near), int(far), 1 if mirror else 0,
            self.grey.ctypes.data,
            None if colour is None else self.rgb.ctypes.data,
            self.cloud.ctypes.data if want_cloud else None)
        return self.grey.tobytes(), (None if colour is None else self.rgb.tobytes())

    def close(self):
        if self.handle:
            self.lib.kproc_destroy(self.handle)
            self.handle = None


def _unmirrored_raw(raw, device_mirrored=False):
    """The camera's true, unmirrored view of the last frame, for tracking.

    Normally the frame as captured, with no copy at all. Only a device that
    mirrors in hardware needs flipping back first.
    """
    colour, depth = raw
    if depth is None:
        return None, None
    if device_mirrored:
        depth = depth[:, ::-1]
        colour = None if colour is None else colour[:, ::-1]
    return (None if colour is None else np.ascontiguousarray(colour),
            np.ascontiguousarray(depth, dtype=np.float32))


# --- Orbbec Femto Mega -------------------------------------------------------

class FemtoCamera:
    """Orbbec Femto Mega over USB.

    On macOS this MUST run as root: the SDK's only Mac USB backend is libuvc,
    which has to take the device away from the system camera driver. Without
    root, opening fails with `uvc_open failed ... Return Code: -3`.

    Colour is aligned onto the depth grid (AlignFilter to DEPTH_STREAM), so
    every output is at depth resolution and the depth intrinsics apply.
    """

    kind = "femto"

    def __init__(self, colour=True):
        import pyorbbecsdk as ob
        self.ob = ob
        ctx = ob.Context()
        # The SDK writes a Log/ directory into the cwd by default; keep it quiet.
        try:
            ctx.set_logger_to_console(ob.OBLogLevel.ERROR)
            ctx.set_logger_level(ob.OBLogLevel.ERROR)
        except Exception:
            pass
        if ctx.query_devices().get_count() == 0:
            raise RuntimeError("no Orbbec device found")

        self.pipeline = ob.Pipeline()
        config = ob.Config()
        dprof = (self.pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
                 .get_default_video_stream_profile())
        config.enable_stream(dprof)

        self.colour = False
        if colour:
            try:
                cprof = (self.pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
                         .get_video_stream_profile(0, 0, ob.OBFormat.RGB, 0))
                config.enable_stream(cprof)
                config.set_frame_aggregate_output_mode(
                    ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
                self.colour = True
            except Exception as exc:
                print("femto: no RGB colour profile, depth only (%s)" % exc, flush=True)

        try:
            self.pipeline.start(config)
        except Exception as exc:
            raise RuntimeError(
                "could not start the Femto Mega (%s). On macOS it needs root - run "
                "with sudo - and nothing else may hold it (quit TouchDesigner's "
                "Orbbec TOP first)." % exc)

        self.align = (ob.AlignFilter(align_to_stream=ob.OBStreamType.DEPTH_STREAM)
                      if self.colour else None)
        self.w, self.h = dprof.get_width(), dprof.get_height()

        info = self.pipeline.get_device().get_device_info()
        self.serial = info.get_serial_number()
        self.name = info.get_name()

        # Mirror exactly once. If the device already mirrors, don't do it again.
        self.mirror = True
        try:
            self.mirror = not bool(self.pipeline.get_camera_param().is_mirrored)
        except Exception:
            pass

        self.proc = _KProc(self.w, self.h)
        self.want_cloud = False
        self.cloud = None
        print("femto: %s %s  depth %dx%d  colour=%s  mirror=%s"
              % (self.name, self.serial, self.w, self.h, self.colour, self.mirror),
              flush=True)

    def _frameset(self, frames):
        if frames is None:
            return None
        if self.align is not None:
            frames = self.align.process(frames)
            if frames is None:
                return None
            if not hasattr(frames, "get_depth_frame") and hasattr(frames, "as_frame_set"):
                frames = frames.as_frame_set()
        return frames

    def frame(self, near, far, want_colour=True):
        frames = self._frameset(self.pipeline.wait_for_frames(1000))
        if frames is None:
            return None, None
        d = frames.get_depth_frame()
        if d is None:
            return None, None
        dw, dh = d.get_width(), d.get_height()
        if (dw, dh) != (self.w, self.h):          # profile changed underneath us
            self.proc.close()
            self.w, self.h = dw, dh
            self.proc = _KProc(dw, dh)
        depth = (np.frombuffer(d.get_data(), dtype=np.uint16).reshape(dh, dw)
                 .astype(np.float32) * d.get_depth_scale())

        colour = None
        if want_colour and self.colour:
            c = frames.get_color_frame()
            if (c is not None and c.get_format() == self.ob.OBFormat.RGB
                    and (c.get_width(), c.get_height()) == (dw, dh)):
                colour = np.frombuffer(c.get_data(), dtype=np.uint8).reshape(dh, dw, 3)

        grey, rgb = self.proc.run(depth, colour, near, far, self.mirror, self.want_cloud)
        if self.want_cloud:
            self.cloud = self.proc.cloud.tobytes()
        self._raw = (colour, depth)
        return grey, rgb

    def raw_frame(self):
        return _unmirrored_raw(getattr(self, "_raw", (None, None)),
                               device_mirrored=not self.mirror)

    def cloud_frame(self):
        return self.cloud

    def enable_cloud(self, on=True):
        self.want_cloud = bool(on)

    def set_filters(self, temporal, median, erode):
        self.proc.set_filters(temporal, median, erode)

    def intrinsics(self):
        try:
            k = self.pipeline.get_camera_param().depth_intrinsic
            return (k.fx, k.fy, k.cx, k.cy)
        except Exception:
            return None

    def close(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        self.proc.close()


# --- Synthetic ---------------------------------------------------------------

class SyntheticCamera:
    """Moving test pattern at any resolution. No hardware.

    A sphere drifting in front of a far wall, plus a colour gradient - enough
    to exercise gating, filters, cloud packing and every output end to end.
    Defaults to the Femto Mega's 640x576 so resolution handling is tested at
    the size that actually matters.
    """

    kind = "synthetic"

    def __init__(self, colour=True, w=640, h=576, fps=30.0, image=None):
        # image: replay a still photo as the colour stream, at a flat 1.5 m.
        # Lets tracking and TD patches be developed with no camera attached.
        self.image = None
        if image:
            import mediapipe as mp
            self.image = np.ascontiguousarray(
                np.asarray(mp.Image.create_from_file(image).numpy_view())[..., :3])
            h, w = self.image.shape[:2]
        self.w, self.h = w, h
        self.serial = "SYNTHETIC-%dx%d" % (w, h)
        self.colour = colour
        self.period = 1.0 / fps
        self.t0 = time.monotonic()
        self.next = self.t0
        self.proc = _KProc(w, h)
        self.want_cloud = False
        self.cloud = None
        yy, xx = np.mgrid[0:h, 0:w]
        self.xx, self.yy = xx.astype(np.float32), yy.astype(np.float32)
        self.gradient = np.stack([(xx * 255 // max(w - 1, 1)),
                                  (yy * 255 // max(h - 1, 1)),
                                  np.full_like(xx, 128)], axis=-1).astype(np.uint8)

    def frame(self, near, far, want_colour=True):
        now = time.monotonic()
        if now < self.next:
            time.sleep(self.next - now)
        self.next += self.period
        if self.image is not None:
            depth = np.full((self.h, self.w), 1500.0, np.float32)
            colour = self.image if (want_colour and self.colour) else None
            grey, rgb = self.proc.run(depth, colour, near, far, True, self.want_cloud)
            if self.want_cloud:
                self.cloud = self.proc.cloud.tobytes()
            self._raw = (self.image, depth)
            return grey, rgb
        t = time.monotonic() - self.t0
        cx = self.w * (0.5 + 0.3 * np.sin(t * 0.8))
        cy = self.h * (0.5 + 0.2 * np.cos(t * 0.6))
        r = min(self.w, self.h) * 0.18
        d2 = (self.xx - cx) ** 2 + (self.yy - cy) ** 2
        depth = np.full((self.h, self.w), 2800.0, np.float32)        # far wall
        inside = d2 < r * r
        depth[inside] = 900.0 + 400.0 * np.sqrt(d2[inside]) / r      # sphere
        colour = self.gradient if (want_colour and self.colour) else None
        grey, rgb = self.proc.run(depth, colour, near, far, True, self.want_cloud)
        if self.want_cloud:
            self.cloud = self.proc.cloud.tobytes()
        self._raw = (self.gradient if self.colour else None, depth)
        return grey, rgb

    def raw_frame(self):
        return _unmirrored_raw(getattr(self, "_raw", (None, None)))

    def cloud_frame(self):
        return self.cloud

    def enable_cloud(self, on=True):
        self.want_cloud = bool(on)

    def set_filters(self, temporal, median, erode):
        self.proc.set_filters(temporal, median, erode)

    def intrinsics(self):
        f = self.w * 0.7                                  # plausible, not real
        return (f, f, self.w / 2.0, self.h / 2.0)

    def close(self):
        self.proc.close()


# --- what's physically plugged in ---------------------------------------------

ORBBEC_VID = 0x2BC5
KINECT_V2 = {(0x045E, 0x02C4), (0x045E, 0x02D8), (0x045E, 0x02D9)}


def usb_devices():
    """(vid, pid) of everything on the USB bus, from the IOKit registry.

    Uses ioreg rather than libusb. libusb's enumeration drops a device that
    another process holds exclusively - with TouchDesigner's Orbbec TOP open,
    it reported an empty bus while the Femto Mega was plugged in. ioreg reads
    the registry macOS itself uses, needs no privileges, and lists a device
    regardless of who has it open.
    """
    import re
    import subprocess
    try:
        out = subprocess.run(["ioreg", "-p", "IOUSB", "-l", "-w0"],
                             capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return set()
    found = set()
    for block in out.split("+-o "):
        vid = re.search(r'"idVendor" = (\d+)', block)
        pid = re.search(r'"idProduct" = (\d+)', block)
        if vid and pid:
            found.add((int(vid.group(1)), int(pid.group(1))))
    return found


def detect():
    """Which supported cameras are plugged in, by USB id. Needs no root."""
    present = usb_devices()
    return {"femto": any(v == ORBBEC_VID for v, _ in present),
            "kinect": bool(present & KINECT_V2)}


# --- factory -----------------------------------------------------------------

def open_camera(kind="auto", colour=True, synthetic_size=(640, 576),
                synthetic_image=None):
    """kind: kinect | femto | synthetic | auto (Femto if present, else Kinect)."""
    if kind == "synthetic":
        return SyntheticCamera(colour, *synthetic_size, image=synthetic_image)
    if kind == "femto":
        return FemtoCamera(colour)
    if kind == "kinect":
        from depth_view import Sensor
        cam = Sensor(colour=colour)
        cam.kind = "kinect"
        return cam
    if kind == "auto":
        found = detect()
        root = os.geteuid() == 0
        if found["femto"] and root:
            return open_camera("femto", colour)
        if found["kinect"]:
            if found["femto"]:
                print("auto: Femto Mega also plugged in, but it needs root - "
                      "using the Kinect", flush=True)
            return open_camera("kinect", colour)
        if found["femto"]:
            raise RuntimeError("Femto Mega is plugged in, but on macOS it needs "
                               "root - use 'Run Femto Mega.command'")
        raise RuntimeError("no camera plugged in (looked for a Femto Mega and "
                           "a Kinect v2)")
    raise ValueError("unknown camera kind %r" % kind)
