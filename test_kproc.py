"""Unit tests for kproc - the camera-agnostic depth pipeline.

Runs on synthetic depth, so it needs no camera. Exercises the resolutions the
Femto Mega uses, not just the Kinect's 512x424.

    .venv/bin/python test_kproc.py
"""
import ctypes, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
L = ctypes.CDLL(os.path.join(HERE, "libkproc.dylib"))
L.kproc_create.argtypes = [ctypes.c_int, ctypes.c_int]
L.kproc_create.restype = ctypes.c_void_p
L.kproc_set_filters.argtypes = [ctypes.c_void_p] + [ctypes.c_int] * 3
L.kproc_run.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
                        ctypes.c_int, ctypes.c_int, ctypes.c_int,
                        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
L.kproc_run.restype = ctypes.c_int
L.kproc_destroy.argtypes = [ctypes.c_void_p]

failures = 0
def check(name, cond, detail=""):
    global failures
    print("  %s  %s%s" % ("PASS" if cond else "FAIL", name, ("  (" + detail + ")") if detail else ""))
    if not cond: failures += 1

def run(w, h, depth, colour=None, near=500, far=2000, mirror=0, filt=(0, 0, 0)):
    p = L.kproc_create(w, h)
    L.kproc_set_filters(p, *filt)
    d = np.ascontiguousarray(depth, dtype=np.float32)
    grey = np.zeros((h, w), np.uint8)
    rgb = np.zeros((h, w, 3), np.uint8)
    cloud = np.zeros((h, w, 3), np.uint8)
    c = None if colour is None else np.ascontiguousarray(colour, dtype=np.uint8)
    rc = L.kproc_run(p, d.ctypes.data, None if c is None else c.ctypes.data,
                     near, far, mirror, grey.ctypes.data, rgb.ctypes.data, cloud.ctypes.data)
    L.kproc_destroy(p)
    return rc, grey, rgb, cloud

for (w, h) in ((640, 576), (1024, 1024), (512, 424)):
    print("\n[%dx%d]" % (w, h))
    depth = np.zeros((h, w), np.float32)
    depth[:, : w // 4] = 3000                    # beyond far -> background
    depth[:, w // 4 : w // 2] = 800              # near-ish
    depth[:, w // 2 : 3 * w // 4] = 1800         # far-ish
    colour = np.zeros((h, w, 3), np.uint8); colour[..., 0] = 200; colour[..., 2] = 50

    rc, g, rgb, cl = run(w, h, depth, colour)
    check("returns grey|rgb|cloud", rc == 7, "rc=%d" % rc)
    check("background beyond far is removed", g[h // 2, w // 8] == 0)
    check("no-depth pixels removed", g[h // 2, 7 * w // 8] == 0)
    near_v, far_v = int(g[h // 2, 3 * w // 8]), int(g[h // 2, 5 * w // 8])
    check("near reads brighter than far", near_v > far_v > 0, "%d vs %d" % (near_v, far_v))
    check("colour kept inside slab", tuple(rgb[h // 2, 3 * w // 8]) == (200, 0, 50))
    check("colour removed outside slab", tuple(rgb[h // 2, w // 8]) == (0, 0, 0))

    mm = int(cl[h // 2, 3 * w // 8, 0]) * 256 + int(cl[h // 2, 3 * w // 8, 1])
    check("cloud decodes to true depth", mm == 800, "decoded %d mm" % mm)
    check("cloud validity byte set", cl[h // 2, 3 * w // 8, 2] == 255)
    check("cloud invalid outside slab", cl[h // 2, w // 8, 2] == 0)

    _, gm, _, _ = run(w, h, depth, mirror=1)
    check("mirror flips horizontally", gm[h // 2, w - 1 - 3 * w // 8] == near_v)

# filters, at Femto resolution
w, h = 640, 576
print("\n[filters @ %dx%d]" % (w, h))
rng = np.random.default_rng(7)
base = np.zeros((h, w), np.float32); base[150:450, 200:450] = 1000     # a solid body
speck = base.copy()
ys, xs = rng.integers(10, h - 10, 400), rng.integers(10, w - 10, 400)
speck[ys, xs] = 1200                                                    # isolated noise
def isolated(g):
    m = g > 0
    nb = sum(np.roll(np.roll(m, dy, 0), dx, 1) for dy in (-1, 0, 1) for dx in (-1, 0, 1))
    return int((m & (nb < 4)).sum())
_, raw, _, _ = run(w, h, speck, filt=(0, 0, 0))
_, med, _, _ = run(w, h, speck, filt=(0, 1, 0))
_, ero, _, _ = run(w, h, speck, filt=(0, 0, 2))
check("raw has speckle", isolated(raw) > 300, "%d isolated" % isolated(raw))
check("despeckle removes isolated noise", isolated(med) < isolated(raw) // 10,
      "%d -> %d" % (isolated(raw), isolated(med)))
check("erode removes isolated noise", isolated(ero) == 0, "%d left" % isolated(ero))
body_raw, body_ero = int((raw[200:400, 250:400] > 0).sum()), int((ero[200:400, 250:400] > 0).sum())
check("erode keeps the body intact", body_ero == body_raw, "%d vs %d px" % (body_ero, body_raw))

print("\n%s" % ("ALL PASSED" if failures == 0 else "%d FAILED" % failures))
sys.exit(1 if failures else 0)
