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
  /hand/<side>/open                  1 while the hand is open, 0 when closed
  /hand/<side>/openness              0 = fist .. 1 = fully open, continuous
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
TIPS = (8, 12, 16, 20)              # fingertips; the thumb is left out - it moves
                                    # independently and a thumbs-up is still a fist

# Hand openness = mean fingertip-to-palm distance / (wrist -> middle knuckle),
# measured in MediaPipe's metric hand-space coords so it survives the hand
# turning sideways. Calibrated on reference photos:
#   fist 0.30   thumbs-up 0.28   pointing 0.48   victory 0.77   open 0.85-0.93
OPEN_RATIO_CLOSED = 0.30            # -> openness 0
OPEN_RATIO_OPEN = 0.95              # -> openness 1
# Hysteresis: switch on above ON, only back off below OFF. A single threshold
# would chatter on a hand held half-open; this holds its state until you
# clearly open or close. Victory (0.72) sits in the gap and keeps its state.
OPEN_ON = 0.75
OPEN_OFF = 0.45
PALM = (0, 5, 9, 13, 17)            # wrist + knuckles: steadier than a fingertip
BONES = ((11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24),
         (23, 24), (23, 25), (25, 27), (24, 26), (26, 28))


# --- OSC ----------------------------------------------------------------------

UDP_LIMIT = 8192          # macOS maxdgram is 9216; stay clear of it


def valid_port(port, fallback):
    """A usable UDP port, or `fallback`. Typing 70000 or -1 into the app used
    to raise OverflowError on every frame."""
    try:
        p = int(port)
    except (TypeError, ValueError):
        return fallback
    return p if 1 <= p <= 65535 else fallback


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
        self.addr = (host, valid_port(port, 9000))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.prefix = {}
        self.sent = 0
        self.errors = 0

    def retarget(self, host, port):
        self.addr = (host, valid_port(port, self.addr[1]))

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
        except (OSError, OverflowError):
            # OverflowError is what sendto raises for a port outside 0-65535;
            # it isn't an OSError, and escaping here used to kill the frame.
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


def hand_openness(world):
    """0 = fist, 1 = fully open, from MediaPipe's metric hand landmarks."""
    p = np.array([[q.x, q.y, q.z] for q in world])
    scale = np.linalg.norm(p[9] - p[0])
    if scale <= 1e-6:
        return 0.0
    centre = p[list(PALM)].mean(0)
    ratio = np.mean([np.linalg.norm(p[t] - centre) for t in TIPS]) / scale
    return float(np.clip((ratio - OPEN_RATIO_CLOSED)
                         / (OPEN_RATIO_OPEN - OPEN_RATIO_CLOSED), 0.0, 1.0))


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
        self.rebuilds = 0
        self.rebuild_reason = None
        self.last_rebuild = {}
        self.last_submit = 0.0
        self.last_done = 0.0
        self.ready_at = 0.0
        self.created_at = time.monotonic()
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
        self.last_submit = time.monotonic()
        self.wake.set()

    def _make(self, name):
        from mediapipe.tasks.python import BaseOptions, vision
        gpu = BaseOptions.Delegate.GPU
        if name == "pose":
            return vision.PoseLandmarker.create_from_options(vision.PoseLandmarkerOptions(
                base_options=BaseOptions(
                    model_asset_path=os.path.join(MODELS, "pose_landmarker_full.task"),
                    delegate=gpu),
                running_mode=vision.RunningMode.VIDEO, num_poses=1))
        return vision.GestureRecognizer.create_from_options(
            vision.GestureRecognizerOptions(
                base_options=BaseOptions(
                    model_asset_path=os.path.join(MODELS, "gesture_recognizer.task"),
                    delegate=gpu),
                running_mode=vision.RunningMode.VIDEO, num_hands=2))

    @staticmethod
    def _abandon(model):
        """Close a MediaPipe model without risking the calling thread.

        close() shuts down the model's dispatcher and joins its thread. If that
        thread is stuck inside a broken graph, close() never returns - in the
        load test it froze tracking mid-rebuild, and the stuck non-daemon
        thread then kept the process from exiting. A throwaway daemon thread
        takes that risk instead.
        """
        def _close():
            try:
                model.close()
            except Exception:
                pass
        threading.Thread(target=_close, name="mediapipe-close", daemon=True).start()

    def current_fps(self):
        """fps only updates when a frame completes, so a stalled tracker would
        keep showing its last value. Report 0 once nothing has finished."""
        return self.fps if time.monotonic() - self.last_done < 1.5 else 0.0

    def hung(self, after=6.0, load_timeout=30.0):
        """Frames are going in but none are coming out: the thread is stuck.

        Also true if the models never finish loading. After MediaPipe's graph
        broke, a replacement tracker sat forever in model construction - and a
        check that only looked at completed frames never noticed it.
        """
        now = time.monotonic()
        if self.ready_at == 0:
            return self.error is None and now - self.created_at > load_timeout
        started = max(self.last_done, self.ready_at)
        return now - self.last_submit < 1.0 and now - started > after

    def _build(self):
        self.pose = self._make("pose")
        self.gesture = self._make("gesture")

    REBUILD_BACKOFF_S = 1.0

    def _infer(self, name, image, ts):
        """Run one model, rebuilding it if its graph has failed.

        Under a sustained load test the hand model failed internally ("Packet
        isn't the sole owner of the holder") and from then on every call
        raised "Graph has errors": a MediaPipe graph that errors stays broken,
        and tracking was dead for the rest of the session. Rebuilding recovers
        it whatever the cause. Returns None for a frame that failed.
        """
        model = getattr(self, name)
        try:
            if name == "pose":
                return model.detect_for_video(image, ts)
            return model.recognize_for_video(image, ts)
        except Exception as exc:
            now = time.monotonic()
            last = self.last_rebuild.get(name, 0.0)
            if now - last < self.REBUILD_BACKOFF_S:
                return None                    # don't rebuild in a tight loop
            self.last_rebuild[name] = now
            self._abandon(model)
            setattr(self, name, self._make(name))
            self.rebuilds += 1
            self.rebuild_reason = "%s: %s" % (name, str(exc).split("\n")[0][:160])
            return None

    def _run(self):
        try:
            import mediapipe as mp
            self.mp = mp
            self._build()      # GPU context belongs to this thread
        except Exception as exc:
            self.error = "tracking unavailable: %s" % exc
            self.ready.set()
            return
        self.ready_at = time.monotonic()
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
            self._step(job, ts, t0)
            self.last_done = time.monotonic()
            self.ms = 1000 * (time.monotonic() - t0)
            frames += 1
            if t0 - since >= 1.0:
                self.fps, frames, since = frames / (t0 - since), 0, t0
        # loop ended: release the models (without blocking on a stuck one)
        for name in ("pose", "gesture"):
            model = getattr(self, name, None)
            if model is not None:
                self._abandon(model)

    def _step(self, job, ts, now):
        """Process one frame. A success clears any earlier error - previously
        one bad frame left an error in the status line for the whole session."""
        try:
            self._process(job, ts, now)
            self.error = None
        except Exception as exc:
            self.error = "%s: %s" % (type(exc).__name__, exc)

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
            r = self._infer(
                "pose", self.mp.Image(image_format=self.mp.ImageFormat.SRGBA, data=rgba_u), ts)
            if r is not None and r.pose_landmarks:
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
            r = self._infer(
                "gesture", self.mp.Image(image_format=self.mp.ImageFormat.SRGBA, data=rgba_m), ts)
            for i, lms in enumerate(r.hand_landmarks if r is not None else []):
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
                opn = hand_openness(r.hand_world_landmarks[i]) if r.hand_world_landmarks else 0.0
                seen[side] = (xs, ys, z, g, opn)
        hands_overlay = []
        for side in ("left", "right"):
            out += self._emit_hand(side, seen.get(side), w, h, K, now)
            if seen.get(side):
                xs, ys = seen[side][0], seen[side][1]
                if not self.hand_detail:            # just the palm
                    xs, ys = np.array([xs[list(PALM)].mean()]), np.array([ys[list(PALM)].mean()])
                state = self.state.get("palm_" + side, {}).get("open", False)
                hands_overlay.append((side, 1.0 - xs, ys, state))

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
            xs, ys, z, gesture, _opn = hand
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
            xs, ys, z, _gesture, opn = hand
            px, py, pz = xs[list(PALM)].mean(), ys[list(PALM)].mean(), z[list(PALM)].mean()
            tx, ty, tz = unproject([px], [py], [pz], w, h, K)
            xy = g["f2"](np.array([1.0 - px, py]), now)     # mirrored, like the feeds
            t3 = g["f3"](np.array([tx[0], ty[0], tz[0]]), now)
            opn = float(g.setdefault("fo", OneEuro(min_cutoff=2.0))(np.array([opn]), now)[0])
            g["open"] = self._open_state(g.get("open", False), opn)
            g["last"] = (xy, t3, opn)
        present = self._present(g, hand is not None, now)
        if not present:
            g["open"] = False
            if "fo" in g:
                g["fo"].reset()
        b = "/hand/" + side + "/"
        items = [(b + "present", 1.0 if present else 0.0),
                 (b + "open", 1.0 if (present and g.get("open")) else 0.0)]
        if present and g["last"]:
            xy, t3, opn = g["last"]
            items += [(b + "x", xy[0]), (b + "y", xy[1]),
                      (b + "tx", t3[0]), (b + "ty", t3[1]), (b + "tz", t3[2]),
                      (b + "openness", opn)]
        return items

    @staticmethod
    def _open_state(was_open, openness):
        """Hysteresis: on above OPEN_ON, off below OPEN_OFF, else unchanged."""
        if openness > OPEN_ON:
            return True
        if openness < OPEN_OFF:
            return False
        return was_open

    def close(self):
        self.go = False
        self.wake.set()
        self.osc.close()


# --- process isolation ----------------------------------------------------------
#
# MediaPipe runs in a child process, not a thread. Measured on this Mac, in
# ordinary use - tracking left on, no stress: after ~107 s the hand graph fails
# internally ("Packet isn't the sole owner of the holder"), a thread wedges
# inside MediaPipe, and from then on MediaPipe cannot be recreated in that
# process - every replacement tracker hung while loading its models. Tracking
# was dead until the app was restarted.
#
# A process can be killed and started fresh, which a thread cannot. This also
# keeps MediaPipe's non-daemon threads out of the app (they once held the
# process open after quit) and its GPU work apart from Syphon's OpenGL.
#
# Frames travel through shared memory under a seqlock - no cross-process locks
# at all. A POSIX semaphore is not released when the process holding it dies,
# and an earlier version shared a multiprocessing Event: a child killed while
# holding its internal lock left it held forever, and the parent deadlocked the
# next time it signalled it. Surviving a child that dies abruptly is the entire
# point of this, so nothing shared may be lockable. The parent marks the
# sequence odd while writing and even when done; the child re-reads it after
# copying and discards a frame that changed underneath it. At worst a frame is
# dropped. The camera thread never waits on anything.

_HDR = struct.Struct("<QiiddddBBBxi")          # seq near far fx fy cx cy pose hands gate port
_HDR_SIZE = 64


def _layout(h, w):
    colour = h * w * 3
    depth_at = _HDR_SIZE + (colour + 7) // 8 * 8
    return _HDR_SIZE, colour, depth_at, depth_at + h * w * 4


_STOP_SEQ = 2 ** 64 - 1


def _send(fd, msg):
    """Length-prefixed pickle over a pipe fd."""
    import pickle
    data = pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL)
    os.write(fd, struct.pack("<I", len(data)) + data)


def _child_main(shm_name, h, w, port, out_fd):
    """Child entry point (tracker.py --child): attach to the frame buffer by
    name, run a Tracker, report back over out_fd. Runs as a plain subprocess,
    not via multiprocessing - that re-imports the parent's main script in the
    child, which crashes the child whenever that script isn't import-safe."""
    import _posixshmem
    import mmap
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)     # the parent owns Ctrl-C
    col_at, col_n, dep_at, total = _layout(h, w)
    fd = _posixshmem.shm_open("/" + shm_name, os.O_RDWR, mode=0o600)
    buf = mmap.mmap(fd, total)
    os.close(fd)
    parent = os.getppid()
    tr = Tracker(port=port)
    tr.ready.wait(90)
    try:
        _send(out_fd, {"ready": tr.error is None, "error": tr.error})
    except OSError:
        return
    last_seq, last_report = 0, 0.0
    while True:
        time.sleep(0.004)                              # poll: nothing shared to wait on
        if os.getppid() != parent:
            break              # parent gone (even kill -9): don't run on as an orphan
        seq = _HDR.unpack_from(buf, 0)[0]
        if seq == _STOP_SEQ:
            break
        if seq != last_seq and seq % 2 == 0 and seq > 0:
            _, near, far, fx, fy, cx, cy, wp, wh, gate, p = _HDR.unpack_from(buf, 0)
            colour = np.frombuffer(buf, np.uint8, col_n, col_at).reshape(h, w, 3).copy()
            depth = np.frombuffer(buf, np.float32, h * w, dep_at).reshape(h, w).copy()
            if _HDR.unpack_from(buf, 0)[0] != seq:
                continue                               # rewritten mid-copy: drop it
            last_seq = seq
            tr.want_pose, tr.want_hands, tr.gate = bool(wp), bool(wh), bool(gate)
            if p != tr.osc.addr[1]:
                tr.osc.retarget("127.0.0.1", p)
            tr.submit(colour, depth, (fx, fy, cx, cy), near, far)
        now = time.monotonic()
        if now - last_report >= 0.05:
            last_report = now
            done_age = (now - tr.last_done) if tr.last_done else None
            try:
                _send(out_fd, {"overlay": tr.overlay, "fps": tr.current_fps(), "ms": tr.ms,
                               "error": tr.error, "rebuilds": tr.rebuilds,
                               "rebuild_reason": tr.rebuild_reason, "done_age": done_age})
            except OSError:
                break                                  # parent closed the pipe
    tr.close()
    buf.close()
    os._exit(0)        # don't wait on MediaPipe's non-daemon threads on the way out


class TrackerProcess:
    """Tracker with MediaPipe in a child process that is replaced when it
    fails. Same surface as Tracker, so the app can use either."""

    HUNG_AFTER = 6.0          # frames going in, none coming out
    LOAD_TIMEOUT = 45.0       # models never finished loading

    def __init__(self, host="127.0.0.1", port=9000, pose=True, hands=True):
        self.want_pose, self.want_hands, self.gate = pose, hands, True
        self._port = valid_port(port, 9000)
        self.osc = self                       # app calls tracker.osc.addr / .retarget
        self.overlay = {"pose": None, "hands": []}
        self.fps = self.ms = 0.0
        self.error = None
        self.rebuilds, self.rebuild_reason = 0, None
        self.restarts, self.restart_reason = 0, None
        self.last_submit = 0.0
        self._done_age = None
        self._last_report = 0.0
        self._ready = False
        self._child = None
        self._shape = None
        self._seq = 0
        self._fail_streak = 0       # children that died before becoming ready
        self._respawn_at = 0.0      # backoff: no respawn before this
        self._pending = None        # restart reason waiting out its backoff
        self._life = threading.Lock()         # guards child lifecycle
        self._alive = True
        threading.Thread(target=self._watch, name="tracker-watch", daemon=True).start()

    # -- surface the app uses --------------------------------------------------

    @property
    def addr(self):
        return ("127.0.0.1", self._port)

    def retarget(self, host, port):
        self._port = valid_port(port, self._port)

    @property
    def go(self):
        return self._alive

    @go.setter
    def go(self, value):
        if not value:
            self.close()

    def current_fps(self):
        age = self._done_age
        return self.fps if (age is not None and age < 1.5) else 0.0

    def hung(self, *a, **k):
        return False           # replaces its own child; nothing for the app to do

    def submit(self, colour, depth, intrinsics, near, far):
        if not self._alive:
            return
        h, w = depth.shape
        # Never wait: the watchdog holds _life while it retires a dying child,
        # which can take seconds. This runs on the camera thread.
        if not self._life.acquire(blocking=False):
            return
        try:
            if self._child is None or self._child["shape"] != (h, w):
                if time.monotonic() < self._respawn_at:
                    return                  # backing off after failed starts
                reason = self._pending or ("resolution now %dx%d" % (w, h) if self._child else None)
                self._pending = None
                self._spawn((h, w), reason)
            c = self._child
            buf = c["shm"].buf
            fx, fy, cx, cy = intrinsics
            flags = (int(near), int(far), fx, fy, cx, cy,
                     int(self.want_pose), int(self.want_hands), int(self.gate), self._port)
            self._seq += 1                   # odd: writing
            _HDR.pack_into(buf, 0, self._seq, *flags)
            c["colour"][:] = colour
            c["depth"][:] = depth
            self._seq += 1                   # even: complete
            _HDR.pack_into(buf, 0, self._seq, *flags)
        finally:
            self._life.release()
        self.last_submit = time.monotonic()

    def close(self):
        self._alive = False
        with self._life:
            self._stop_child()

    # -- child lifecycle ------------------------------------------------------------

    def _spawn(self, shape, reason):
        """Start a fresh child. Caller holds self._life."""
        import subprocess
        import sys
        from multiprocessing import shared_memory
        self._stop_child()
        h, w = shape
        _, col_n, dep_at, total = _layout(h, w)
        shm = shared_memory.SharedMemory(create=True, size=total)
        _HDR.pack_into(shm.buf, 0, 0, 0, 0, 1.0, 1.0, 0.0, 0.0, 0, 0, 0, self._port)
        self._seq = 0
        rfd, wfd = os.pipe()
        proc = subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), "--child",
             shm.name, str(h), str(w), str(self._port), str(wfd)],
            pass_fds=(wfd,), close_fds=True)
        os.close(wfd)                        # the child holds the sending end now
        self._child = {
            "proc": proc, "shm": shm, "rx": rfd, "shape": shape,
            "colour": np.ndarray((h, w, 3), np.uint8, shm.buf, _HDR_SIZE),
            "depth": np.ndarray((h, w), np.float32, shm.buf, dep_at),
            "born": time.monotonic(),
        }
        self._ready, self._done_age, self._last_report = False, None, time.monotonic()
        if reason:
            self.restarts += 1
            self.restart_reason = reason
        threading.Thread(target=self._read, args=(self._child,), name="tracker-read",
                         daemon=True).start()

    def _stop_child(self):
        c, self._child = self._child, None
        if c is None:
            return
        try:
            _HDR.pack_into(c["shm"].buf, 0, _STOP_SEQ, 0, 0, 1.0, 1.0, 0.0, 0.0, 0, 0, 0, 0)
        except Exception:
            pass
        p = c["proc"]
        for step in (None, p.terminate, p.kill):  # a wedged MediaPipe ignores SIGTERM
            if step:
                try:
                    step()
                except ProcessLookupError:
                    break
            try:
                p.wait(1.0)
                break
            except Exception:
                continue
        try:
            os.close(c["rx"])
        except OSError:
            pass
        # arrays view the segment; drop them before closing it
        c["colour"] = c["depth"] = None
        try:
            c["shm"].close()
            c["shm"].unlink()
        except Exception:
            pass

    @staticmethod
    def _recv(fd):
        import pickle
        def exact(n):
            out = b""
            while len(out) < n:
                chunk = os.read(fd, n - len(out))
                if not chunk:
                    raise EOFError
                out += chunk
            return out
        return pickle.loads(exact(struct.unpack("<I", exact(4))[0]))

    def _read(self, c):
        """Mirror the child's reports into attributes the app reads."""
        rx = c["rx"]
        while True:
            try:
                msg = self._recv(rx)
            except (EOFError, OSError, ValueError):
                return
            if self._child is not c:
                return
            if "ready" in msg:
                self._ready = msg["ready"]
                self.error = msg.get("error")
                continue
            self._last_report = time.monotonic()
            self.overlay = msg["overlay"]
            self.fps, self.ms = msg["fps"], msg["ms"]
            self.error = msg["error"]
            self.rebuilds, self.rebuild_reason = msg["rebuilds"], msg["rebuild_reason"]
            self._done_age = msg["done_age"]

    def _watch(self):
        """Replace the child if it dies, wedges, or never finishes loading.

        Nothing may kill this loop: an uncaught exception in a daemon thread
        ends it silently, and then no child would ever be replaced again.
        """
        while self._alive:
            time.sleep(0.5)
            try:
                self._watch_once()
            except Exception as exc:
                import traceback
                self.error = "tracker watchdog: %s: %s" % (type(exc).__name__, exc)
                traceback.print_exc()

    def _watch_once(self):
        if True:
            with self._life:
                c = self._child
                if c is None or not self._alive:
                    return
                now = time.monotonic()
                reason = None
                if c["proc"].poll() is not None:
                    reason = "tracker process died (exit %s)" % c["proc"].returncode
                elif not self._ready and now - c["born"] > self.LOAD_TIMEOUT:
                    reason = "models did not load in %.0f s" % self.LOAD_TIMEOUT
                elif (self._ready and now - self.last_submit < 1.0
                      and now - c["born"] > self.HUNG_AFTER + 5
                      and (self._done_age is None or self._done_age > self.HUNG_AFTER)):
                    reason = "no frame completed in %.0f s" % self.HUNG_AFTER
                elif self._ready and now - self._last_report > self.HUNG_AFTER:
                    # a child frozen outright stops reporting, so its last
                    # done_age would look healthy forever
                    reason = "tracker process silent for %.0f s" % self.HUNG_AFTER
                if reason:
                    self._failed(c, reason, now)

    def _failed(self, c, reason, now):
        """Retire a failed child. One that ran fine and then died (MediaPipe's
        allocator giving out after a few minutes) is replaced immediately. One
        that never became ready is failing to start - a missing model, say -
        and respawning it every half second was an endless crash loop, so
        those back off: 2 s, 4 s, 8 s... up to 30 s, and say why."""
        shape = c["shape"]
        self._fail_streak = self._fail_streak + 1 if not self._ready else 0
        self._stop_child()
        delay = 0.0 if self._fail_streak <= 1 else min(30.0, 2.0 ** (self._fail_streak - 1))
        if self._fail_streak >= 3:
            self.error = "tracker keeps failing to start (%d in a row): %s" % (
                self._fail_streak, reason)
        if delay == 0.0:
            self._spawn(shape, reason)
        else:
            self._pending, self._respawn_at = reason, now + delay


if __name__ == "__main__" and len(__import__("sys").argv) > 1 and __import__("sys").argv[1] == "--child":
    import sys
    _child_main(sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6]))
