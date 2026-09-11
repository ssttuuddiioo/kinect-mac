"""End-to-end test of tracker.py: photos in, OSC out, decoded by python-osc.

Needs the GPU (MediaPipe 1.0.1's CPU path crashes on macOS), so run it from a
normal terminal:   .venv/bin/python test_tracker.py
Test images come from MediaPipe's own asset bucket; build.sh fetches them.
"""
import os, socket, sys, time
import numpy as np, mediapipe as mp
from pythonosc.osc_bundle import OscBundle
import tracker as T

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, "models", "test")
failures = 0
def check(name, cond, detail=""):
    global failures
    print("  %s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (%s)" % detail) if detail else ""))
    failures += 0 if cond else 1

def load_mirrored(path):
    """The camera's natural (unmirrored) view - what cameras hand the tracker.
    Published x is still mirrored, like every feed."""
    return np.ascontiguousarray(np.asarray(mp.Image.create_from_file(path).numpy_view())[..., :3])

def run(colour, depth_mm, near=500, far=2500, port=9101, frames=8):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", port)); rx.settimeout(0.3)
    tr = T.Tracker(port=port)
    tr.ready.wait(30)
    assert tr.error is None, tr.error
    h, w = depth_mm.shape
    K = (w * 0.7, w * 0.7, w / 2.0, h / 2.0)
    got, sizes = {}, []
    for _ in range(frames):                       # VIDEO mode wants a few frames
        tr.submit(colour, depth_mm, K, near, far)
        time.sleep(0.12)
    end = time.time() + 1.0
    while time.time() < end:
        try:
            d, _ = rx.recvfrom(65535)
        except socket.timeout:
            continue
        sizes.append(len(d))
        for m in OscBundle(d):                    # python-osc parses our encoding
            got[m.address] = m.params[0]
    tr.close(); rx.close()
    return got, sizes, K, (w, h)

print("[pose - yoga photo, mirrored, subject at 1.5 m]")
col = load_mirrored(os.path.join(IMG, "pose.jpg"))
h, w = col.shape[:2]
got, sizes, K, _ = run(col, np.full((h, w), 1500.0, np.float32))
check("bundles decode with python-osc", len(got) > 0, "%d addresses" % len(got))
check("every datagram under macOS 9216-byte cap", sizes and max(sizes) <= 9216,
      "largest %d B across %d datagrams" % (max(sizes) if sizes else 0, len(sizes)))
check("/pose/present = 1", got.get("/pose/present") == 1.0)
check("all 33 joints x6 channels", sum(k.startswith("/pose/") for k in got) == 33 * 6 + 1)
lx = got.get("/pose/left_wrist/x", -1); rx_ = got.get("/pose/right_wrist/x", -1)
check("left wrist still labelled LEFT, at mirrored x~0.31", abs(lx - 0.31) < 0.05, "x=%.3f" % lx)
check("right wrist at mirrored x~0.70", abs(rx_ - 0.70) < 0.05, "x=%.3f" % rx_)
check("wrists level with shoulders",
      abs(got["/pose/left_wrist/y"] - got["/pose/left_shoulder/y"]) < 0.05)
tz = got.get("/pose/nose/tz", 0)
check("depth lifted: tz = -1.5 m", abs(tz + 1.5) < 0.01, "tz=%.3f" % tz)
# unprojection matches the shader: left wrist is camera-right of centre -> +tx
u = (1 - lx) * w - 0.5; exp = (u - K[2]) * 1.5 / K[0]   # unmirror, pixel centre
check("tx matches the shader's unprojection", abs(got["/pose/left_wrist/tx"] - exp) < 0.02,
      "%.3f vs %.3f expected" % (got["/pose/left_wrist/tx"], exp))
check("subject's left is +tx in TD (camera right)", got["/pose/left_wrist/tx"] > 0)

print("\n[depth gate - same photo, subject at 3.5 m, slab ends at 2.5 m]")
got, _, _, _ = run(col, np.full((h, w), 3500.0, np.float32), port=9102)
check("body outside the slab is dropped", got.get("/pose/present") == 0.0,
      "present=%s" % got.get("/pose/present"))

print("\n[hands - two hands, mirrored, at 0.8 m]")
col = load_mirrored(os.path.join(IMG, "woman_hands.jpg"))
h, w = col.shape[:2]
got, sizes, _, _ = run(col, np.full((h, w), 800.0, np.float32), port=9103)
both = got.get("/hand/left/present") == 1.0 and got.get("/hand/right/present") == 1.0
check("both hands present", both, "L=%s R=%s" % (got.get("/hand/left/present"), got.get("/hand/right/present")))
left = sorted(k for k in got if k.startswith("/hand/left/"))
check("palm: present open openness x y tx ty tz", left == ["/hand/left/%s" % c for c in
      ("open", "openness", "present", "tx", "ty", "tz", "x", "y")],
      "%s" % [k.split("/")[-1] for k in left])
check("open hands read OPEN (on)", got.get("/hand/left/open") == 1.0 and got.get("/hand/right/open") == 1.0,
      "L=%s R=%s  openness %.2f / %.2f" % (got.get("/hand/left/open"), got.get("/hand/right/open"),
                                          got.get("/hand/left/openness", -1), got.get("/hand/right/openness", -1)))
check("no finger joints or gestures sent", not any("tip" in k or "gesture" in k for k in got))
check("palm depth-lifted to 0.8 m", abs(got.get("/hand/left/tz", 0) + 0.8) < 0.01,
      "tz=%.3f" % got.get("/hand/left/tz", 0))
check("every datagram under the cap", max(sizes) <= 9216, "largest %d B" % max(sizes))

print("\n[fist - closed hand at 0.8 m]")
col = load_mirrored(os.path.join(IMG, "fist.jpg"))
h, w = col.shape[:2]
got, _, _, _ = run(col, np.full((h, w), 800.0, np.float32), port=9104)
sides = [s for s in ("left", "right") if got.get("/hand/%s/present" % s) == 1.0]
check("fist detected", len(sides) == 1, "%s" % sides)
if sides:
    s = sides[0]
    check("fist reads CLOSED (off)", got.get("/hand/%s/open" % s) == 0.0,
          "open=%s openness=%.2f" % (got.get("/hand/%s/open" % s), got.get("/hand/%s/openness" % s, -1)))

print("\n[hysteresis - no chatter in the gap]")
st = T.Tracker._open_state
check("closed -> 0.80 turns ON", st(False, 0.80) is True)
check("open -> 0.40 turns OFF", st(True, 0.40) is False)
check("open stays open at 0.60 (in the gap)", st(True, 0.60) is True)
check("closed stays closed at 0.60 (in the gap)", st(False, 0.60) is False)
seq, state, flips = [0.1, 0.7, 0.5, 0.7, 0.5, 0.7, 0.9, 0.6, 0.5, 0.6, 0.3], False, 0
for v in seq:
    new = st(state, v); flips += new != state; state = new
check("a hand wobbling around one level flips only twice", flips == 2,
      "%d flips over %s" % (flips, seq))

print("\n%s" % ("ALL PASSED" if failures == 0 else "%d FAILED" % failures))
sys.exit(1 if failures else 0)
