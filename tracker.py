"""Body and hand tracking, lifted into 3D with our depth, sent to OSC.

MediaPipe finds landmarks in the colour image; this looks each one up in the
depth map and unprojects it, so joints arrive in TouchDesigner as real
positions in metres - in the same coordinate space as the point-cloud shader,
so the skeleton lands exactly on the cloud.

Runs on its own thread and always takes the newest frame, dropping any it
couldn't get to, so tracking can never stall the depth pipeline or Syphon.

Findings this code depends on, all measured on this Mac:

  * MediaPipe 1.0.1's CPU delegate crashes on macOS ("Service is
    unavailable") - its CPU graph still demands the GPU service. GPU only.
  * The GPU path goes through CVPixelBuffer and rejects 3-channel images
    ("unsupported ImageFrame format: 1"). Input must be RGBA.
  * GPU inference: pose ~3.6 ms, hands+gestures ~4.8 ms at 640x576.
  * Pose labels left/right from image position, not anatomy: fed a mirrored
    image it calls your left shoulder your right. So Pose gets the unmirrored
    view. Hands is documented to assume a mirrored view, so it gets that.
  * macOS caps UDP datagrams at 9216 bytes and fails larger sends outright.
    A full frame is ~17 KB, so it is split across bundles.

OSC layout (one float per address, so TD's OSC In CHOP gets clean channels):

  /pose/present                      1 while a body is tracked
  /pose/<joint>/x /y                 0-1 across the image (mirrored, like the feeds)
  /pose/<joint>/tx /ty /tz           metres, TD space (Y up, -Z forward)
  /pose/<joint>/v                    visibility 0-1
  /hand/<left|right>/present
  /hand/<side>/x /y                  palm, 0-1 across the image (mirrored)
  /hand/<side>/tx /ty /tz            palm, metres, TD space
  (palm = wrist + four knuckles averaged, steadier than any fingertip.
   Set Tracker.hand_detail = True for all 21 joints and gesture flags.)
"""

import math
import os
import socket
import struct
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, "models")

POSE_JOINTS = (
    "nose", "left_eye_inner", "left_eye", "left_eye_outer", "right_eye_inner",
    "right_eye", "right_eye_outer", "left_ear", "right_ear", "mouth_left",
    "mouth_right", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_pinky", "right_pinky", "left_index",
    "right_index", "left_thumb", "right_thumb", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle", "left_heel",
    "right_heel", "left_foot_index", "right_foot_index")
HAND_JOINTS = (
    "wrist", "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
    "index_mcp", "index_pip", "index_dip", "index_tip",
    "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
    "ring_mcp", "ring_pip", "ring_dip", "ring_tip",
    "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip")
GESTURES = ("None", "Closed_Fist", "Open_Palm", "Pointing_Up",
            "Thumb_Down", "Thumb_Up", "Victory", "ILoveYou")
TORSO = (11, 12, 23, 24)            # shoulders and hips: the body's depth
PALM = (0, 5, 9, 13, 17)            # wrist + knuckles: steadier than a fingertip
BONES = ((11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24),
         (23, 24), (23, 25), (25, 27), (24, 26), (26, 28))


# --- OSC ----------------------------------------------------------------------

UDP_LIMIT = 8192          # macOS maxdgram is 9216; stay clear of it


def _osc_str(s):
    b = s.encode()
    return b + b"\0" * (4 - len(b) % 4)          # null-terminate, pad to 4


class OscSender:
    """Minimal OSC-over-UDP, float messages only, bundled and size-split.

    Addresses never change, so each one is encoded once and cached; per frame
    it only packs floats.
    """

    _HEAD = b"#bundle\0" + struct.pack(">Q", 1)   # timetag 1 = "immediately"
    _TAG = _osc_str(",f")

    def __init__(self, host="127.0.0.1", port=9000):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.prefix = {}
        self.sent = 0
        self.errors = 0

    def retarget(self, host, port):
        self.addr = (host, int(port))

    def send(self, items):
        """items: iterable of (address, float). Split across bundles by size."""
        parts, size = [], len(self._HEAD)
        for addr, value in items:
            p = self.prefix.get(addr)
            if p is None:
                p = self.prefix[addr] = _osc_str(addr) + self._TAG
            msg = p + struct.pack(">f", float(value))
            if size + 4 + len(msg) > UDP_LIMIT and parts:
                self._flush(parts)
                parts, size = [], len(self._HEAD)
            parts.append(struct.pack(">i", len(msg)) + msg)
            size += 4 + len(msg)
        if parts:
            self._flush(parts)

    def _flush(self, parts):
        try:
            self.sock.sendto(self._HEAD + b"".join(parts), self.addr)
            self.sent += 1
        except OSError:
            self.errors += 1

    def close(self):
        self.sock.close()


# --- smoothing ----------------------------------------------------------------

class OneEuro:
    """One Euro filter over a whole array at once. Smooths hard when still,
    gets out of the way when moving fast - the usual choice for pointers."""

    def __init__(self, min_cutoff=1.2, beta=0.02, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def reset(self):
        self.x = self.dx = self.t = None

    @staticmethod
    def _alpha(dt, cutoff):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        x = np.asarray(x, np.float64)
        if self.x is None or self.x.shape != x.shape:
            self.x, self.dx, self.t = x.copy(), np.zeros_like(x), t
            return x
        dt = max(t - self.t, 1e-3)
        dx = (x - self.x) / dt
        ad = self._alpha(dt, self.d_cutoff)
        self.dx = ad * dx + (1 - ad) * self.dx
        cutoff = self.min_cutoff + self.beta * np.abs(self.dx)
        a = 1.0 / (1.0 + (1.0 / (2.0 * math.pi * cutoff)) / dt)
        self.x = a * x + (1 - a) * self.x
        self.t = t
        return self.x


# --- geometry -----------------------------------------------------------------

def sample_depth(depth, xs, ys, radius=2):
    """Median of valid depth in a small window at each normalised point.

    A single pixel is often a hole or an edge; the window rides over both.
    Returns mm, NaN where nothing valid is nearby.
    """
    h, w = depth.shape
    px = np.clip(np.rint(np.asarray(xs) * w).astype(int), 0, w - 1)
    py = np.clip(np.rint(np.asarray(ys) * h).astype(int), 0, h - 1)
    out = np.full(len(px), np.nan)
    for i, (x, y) in enumerate(zip(px, py)):
        win = depth[max(y - radius, 0):y + radius + 1, max(x - radius, 0):x + radius + 1]
        v = win[win > 0]
        if v.size:
            out[i] = float(np.median(v))
    return out


def unproject(xs, ys, z_mm, w, h, K):
    """Normalised UNMIRRORED image coords + depth -> TD-space metres.

    Matches kinect_xyz.glsl exactly: the shader un-mirrors each texel to get
    the camera column, and here the coords are already unmirrored, so the
    column is the coordinate itself. Pixel-centre convention (x*w - 0.5) to
    match texelFetch. (x, -y, -z) for TD's Y-up, -Z-forward. Sharing the
    shader's maths is what makes the skeleton sit on the cloud.
    """
    fx, fy, cx, cy = K
    u = np.asarray(xs) * w - 0.5
    v = np.asarray(ys) * h - 0.5
    z = np.asarray(z_mm) / 1000.0
    return (u - cx) * z / fx, -((v - cy) * z / fy), -z


# --- tracker ------------------------------------------------------------------

class Tracker:
    def __init__(self, host="127.0.0.1", port=9000, pose=True, hands=True):
        self.osc = OscSender(host, port)
        self.want_pose, self.want_hands = pose, hands
        self.gate = True           # drop bodies/hands outside the depth slab
        # Hands send presence + palm position only - "where is the hand".
        # True restores all 21 joints and the gesture flags (+113 channels/hand).
        self.hand_detail = False
        self.hold_s = 0.3          # ride over brief dropouts before "gone"
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.job = None
        self.go = True
        self.error = None
        self.fps = 0.0
        self.ms = 0.0
        self.overlay = {"pose": None, "hands": []}
        self.state = {}            # per group: filters, last values, last seen
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run, name="tracker", daemon=True)
        self.thread.start()

    # called from the camera thread - never blocks.
    # colour and depth are the camera's natural, UNMIRRORED view: Pose needs
    # exactly that, and it saves mirroring the image only to un-mirror it.
    def submit(self, colour, depth, intrinsics, near, far):
        with self.lock:
            self.job = (colour, depth, intrinsics, near, far)
        self.wake.set()

    def _build(self):
        from mediapipe.tasks.python import BaseOptions, vision
        gpu = BaseOptions.Delegate.GPU
        self.pose = vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
            base_options=BaseOptions(
                model_asset_path=os.path.join(MODELS, "pose_landmarker_full.task"),
                delegate=gpu),
            running_mode=vision.RunningMode.VIDEO, num_poses=1))
        self.gesture = vision.GestureRecognizer.create_from_options(
            vision.GestureRecognizerOptions(
                base_options=BaseOptions(
                    model_asset_path=os.path.join(MODELS, "gesture_recognizer.task"),
                    delegate=gpu),
                running_mode=vision.RunningMode.VIDEO, num_hands=2))

    def _run(self):
        try:
            import mediapipe as mp
            self.mp = mp
            self._build()      # GPU context belongs to this thread
        except Exception as exc:
            self.error = "tracking unavailable: %s" % exc
            self.ready.set()
            return
        self.ready.set()
        frames, since, last_ts = 0, time.monotonic(), -1
        while self.go:
            self.wake.wait(0.5)
            self.wake.clear()
            with self.lock:
                job, self.job = self.job, None
            if job is None:
                continue
            t0 = time.monotonic()
            ts = int(t0 * 1000)
            if ts <= last_ts:          # VIDEO mode needs strictly rising timestamps
                ts = last_ts + 1
            last_ts = ts
            try:
                self._process(job, ts, t0)
            except Exception as exc:
                self.error = "%s: %s" % (type(exc).__name__, exc)
            self.ms = 1000 * (time.monotonic() - t0)
            frames += 1
            if t0 - since >= 1.0:
                self.fps, frames, since = frames / (t0 - since), 0, t0

    def _group(self, key):
        g = self.state.get(key)
        if g is None:
            g = self.state[key] = {"f2": OneEuro(), "f3": OneEuro(min_cutoff=0.8),
                                   "seen": 0.0, "last": None}
        return g

    def _buffers(self, h, w):
        """RGBA frames for the GPU, allocated once. Alpha is set here and never
        touched again; per frame only the colour channels are written."""
        if getattr(self, "_shape", None) != (h, w):
            self._rgba_u = np.full((h, w, 4), 255, np.uint8)
            self._rgba_m = np.full((h, w, 4), 255, np.uint8)
            self._shape = (h, w)
        return self._rgba_u, self._rgba_m

    def _process(self, job, ts, now):
        colour, depth, K, near, far = job
        h, w = depth.shape
        rgba_u, rgba_m = self._buffers(h, w)
        out = []

        # --- pose: the unmirrored view, so left/right labels are anatomical.
        # Coordinates stay unmirrored for depth lookup and unprojection, and
        # are flipped to mirrored space (1-x) only for the published x.
        body = None
        if self.want_pose:
            rgba_u[..., :3] = colour
            r = self.pose.detect_for_video(
                self.mp.Image(image_format=self.mp.ImageFormat.SRGBA, data=rgba_u), ts)
            if r.pose_landmarks:
                L = r.pose_landmarks[0]
                xs = np.array([p.x for p in L])
                ys = np.array([p.y for p in L])
                vis = np.array([p.visibility for p in L])
                z = sample_depth(depth, xs, ys)
                torso = np.nanmedian(z[list(TORSO)]) if np.any(~np.isnan(z[list(TORSO)])) else np.nan
                inside = not np.isnan(torso) and near <= torso <= far
                if inside or not self.gate:
                    z = np.where(np.isnan(z), torso if not np.isnan(torso) else near, z)
                    body = (xs, ys, z, vis)
        out += self._emit_pose(body, w, h, K, now)

        # --- hands: the mirrored view, which is what Hands assumes for
        # handedness. Its coords come back mirrored; unmirror for depth.
        seen = {}
        if self.want_hands:
            rgba_m[..., :3] = colour[:, ::-1]
            r = self.gesture.recognize_for_video(
                self.mp.Image(image_format=self.mp.ImageFormat.SRGBA, data=rgba_m), ts)
            for i, lms in enumerate(r.hand_landmarks):
                side = r.handedness[i][0].category_name.lower()     # left / right
                if side in seen:
                    continue
                xs = 1.0 - np.array([p.x for p in lms])             # -> unmirrored
                ys = np.array([p.y for p in lms])
                z = sample_depth(depth, xs, ys)
                palm = np.nanmedian(z[list(PALM)]) if np.any(~np.isnan(z[list(PALM)])) else np.nan
                if (np.isnan(palm) or not (near <= palm <= far)) and self.gate:
                    continue
                z = np.where(np.isnan(z), palm if not np.isnan(palm) else near, z)
                g = r.gestures[i][0].category_name if r.gestures and r.gestures[i] else "None"
                seen[side] = (xs, ys, z, g)
        hands_overlay = []
        for side in ("left", "right"):
            out += self._emit_hand(side, seen.get(side), w, h, K, now)
            if seen.get(side):
                xs, ys = seen[side][0], seen[side][1]
                if not self.hand_detail:            # just the palm
                    xs, ys = np.array([xs[list(PALM)].mean()]), np.array([ys[list(PALM)].mean()])
                hands_overlay.append((side, 1.0 - xs, ys))

        # the preview is mirrored like every other output
        self.overlay = {"pose": (1.0 - body[0], body[1]) if body else None,
                        "hands": hands_overlay}
        self.osc.send(out)

    def _present(self, g, found, now):
        """Hold the last values briefly so a one-frame dropout isn't 'gone'."""
        if found:
            g["seen"] = now
            return True
        if g["last"] is not None and now - g["seen"] < self.hold_s:
            return True
        g["f2"].reset(); g["f3"].reset(); g["last"] = None
        return False

    def _emit_pose(self, body, w, h, K, now):
        g = self._group("pose")
        if body:
            xs, ys, z, vis = body
            tx, ty, tz = unproject(xs, ys, z, w, h, K)
            xy = g["f2"](np.concatenate([1.0 - xs, ys]), now)      # mirrored, like the feeds
            t3 = g["f3"](np.concatenate([tx, ty, tz]), now)
            g["last"] = (xy, t3, vis)
        present = self._present(g, body is not None, now)
        items = [("/pose/present", 1.0 if present else 0.0)]
        if present and g["last"]:
            xy, t3, vis = g["last"]
            n = len(POSE_JOINTS)
            for i, name in enumerate(POSE_JOINTS):
                b = "/pose/" + name + "/"
                items += [(b + "x", xy[i]), (b + "y", xy[n + i]),
                          (b + "tx", t3[i]), (b + "ty", t3[n + i]), (b + "tz", t3[2 * n + i]),
                          (b + "v", vis[i])]
        return items

    def _emit_hand(self, side, hand, w, h, K, now):
        if not self.hand_detail:
            return self._emit_palm(side, hand, w, h, K, now)
        g = self._group("hand_" + side)
        if hand:
            xs, ys, z, gesture = hand
            pxs = np.append(xs, xs[list(PALM)].mean())
            pys = np.append(ys, ys[list(PALM)].mean())
            pz = np.append(z, z[list(PALM)].mean())
            tx, ty, tz = unproject(pxs, pys, pz, w, h, K)
            xy = g["f2"](np.concatenate([1.0 - pxs, pys]), now)    # mirrored, like the feeds
            t3 = g["f3"](np.concatenate([tx, ty, tz]), now)
            g["last"] = (xy, t3, gesture)
        present = self._present(g, hand is not None, now)
        base = "/hand/" + side + "/"
        items = [(base + "present", 1.0 if present else 0.0)]
        if present and g["last"]:
            xy, t3, gesture = g["last"]
            n = len(HAND_JOINTS) + 1
            for i, name in enumerate(HAND_JOINTS + ("palm",)):
                b = base + name + "/"
                items += [(b + "x", xy[i]), (b + "y", xy[n + i]),
                          (b + "tx", t3[i]), (b + "ty", t3[n + i]), (b + "tz", t3[2 * n + i])]
            items += [(base + "gesture/" + gname, 1.0 if gname == gesture else 0.0)
                      for gname in GESTURES]
        return items

    def _emit_palm(self, side, hand, w, h, K, now):
        """Presence and palm position only. The palm - wrist plus the four
        knuckles averaged - is far steadier than any fingertip, which moves
        every time a finger bends."""
        g = self._group("palm_" + side)
        if hand:
            xs, ys, z, _gesture = hand
            px, py, pz = xs[list(PALM)].mean(), ys[list(PALM)].mean(), z[list(PALM)].mean()
            tx, ty, tz = unproject([px], [py], [pz], w, h, K)
            xy = g["f2"](np.array([1.0 - px, py]), now)     # mirrored, like the feeds
            t3 = g["f3"](np.array([tx[0], ty[0], tz[0]]), now)
            g["last"] = (xy, t3)
        present = self._present(g, hand is not None, now)
        b = "/hand/" + side + "/"
        items = [(b + "present", 1.0 if present else 0.0)]
        if present and g["last"]:
            xy, t3 = g["last"]
            items += [(b + "x", xy[0]), (b + "y", xy[1]),
                      (b + "tx", t3[0]), (b + "ty", t3[1]), (b + "tz", t3[2])]
        return items

    def close(self):
        self.go = False
        self.wake.set()
        self.osc.close()
