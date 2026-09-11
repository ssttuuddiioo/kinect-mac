# kinect-mac

Use a **Kinect v2** or an **Orbbec Femto Mega** on **Apple Silicon macOS**, and
get depth, colour and a point cloud into TouchDesigner, OBS, or anything that
speaks Syphon or MJPEG.

| Camera | How | Notes |
|---|---|---|
| Kinect v2 | libfreenect2 | `open ~/Applications/Kinect.app` |
| Orbbec Femto Mega | Orbbec SDK v2 | needs root on macOS — see [FEMTO.md](FEMTO.md) |
| none | synthetic test pattern | `--camera synthetic`, for testing without hardware |

Both cameras run through the same filters and publish the same outputs.

**Body and hand tracking** go out over OSC with real 3D positions in metres —
33 body joints, 21 per hand, and gestures — lined up with the point cloud. See
[TRACKING.md](TRACKING.md).

TouchDesigner's built-in Kinect operators are Windows-only, and Microsoft's
Kinect SDK was never ported to the Mac. This drives the sensor directly through
libfreenect2 and publishes the results itself.

---

## What you need

**Hardware**

- Kinect v2 (model 1520 — the Xbox One sensor, not the Xbox 360 one)
- The official Microsoft Kinect Adapter, with its 12 V power brick. The sensor
  cannot run on USB bus power; there is no way around this.
- A **USB 3** port. USB 2 will enumerate the device but never stream.
  Plug it **directly into the Mac** if you can — a Kinect v2 sharing a hub with
  other devices drops off the bus, sometimes hourly.

**Software**

- Apple Silicon Mac, macOS 12 or later
- [Homebrew](https://brew.sh) at `/opt/homebrew`
- Xcode Command Line Tools: `xcode-select --install`
- TouchDesigner — only needed at build time, for the arm64 Syphon.framework it
  bundles. The public Syphon SDK is x86_64-only and won't link.

## Install

```bash
git clone https://github.com/ssttuuddiioo/kinect-mac.git
cd kinect-mac
./build.sh
```

That installs the Homebrew dependencies, clones and builds libfreenect2,
compiles the two shims, and creates `~/Applications/Kinect.app`. Takes a few
minutes, mostly compiling libfreenect2.

## Run

```bash
open ~/Applications/Kinect.app
```

or `/opt/homebrew/bin/python3.14 kinect_app.py`.

You get one window: a live preview, five sliders, and a status line. It holds
the sensor and serves everything at once:

| Output | What it is |
|---|---|
| Syphon **Kinect Depth** | depth, gated to your near/far slab, near-bright |
| Syphon **Kinect Colour** | colour with the background removed |
| Syphon **Kinect Cloud** | 16-bit depth packed for point-cloud use |
| MJPEG (checkbox) | `http://127.0.0.1:8010/depth.mjpg` and `/colour.mjpg` |

**Only one program can hold a Kinect at a time.** That's why this is one app
rather than several — the outputs aren't alternatives, they run simultaneously.

### The sliders

- **Near / Far (mm)** — the depth slab. Anything outside it is removed. This is
  the background subtraction: it works on distance, so lighting and clothing
  colour are irrelevant.
- **Despeckle** — median filter, 0=off / 1=3×3 / 2=5×5 / 3=7×7. Removes
  isolated noise pixels. **3×3 is the sweet spot**; larger kernels cost frame
  rate without cleaning up more.
- **Erode (px)** — morphological opening. Removes the noisy fringe that haloes
  every depth edge. 1–2 is usually enough.
- **Smooth (%)** — temporal blending. Modest effect; the dominant noise is
  pixels flickering in and out of validity, which this deliberately skips.
  Reach for Despeckle and Erode first.

### Status line

Shows serial, fps, and how many clients are attached to each Syphon feed. If it
turns red saying `STALLED` or `NO FRAMES`, the sensor fell off USB — replug it
and press **Retry**.

## In TouchDesigner

Add a **Syphon Spout In TOP** and pick the server. That's it for depth and
colour.

For the point cloud, see **[POINTCLOUD.md](POINTCLOUD.md)** — it has the
unprojection shader, the network layout, and the troubleshooting list.
`td_setup.py` can build the whole network for you: paste it into TD's Textport.

**The camera intrinsics are per-sensor.** The app prints yours on startup
(`IR intrinsics fx=… fy=… cx=… cy=…`); the ones in POINTCLOUD.md are from one
particular unit. Use your own.

## In OBS (for Zoom, Meet, a browser)

Tick **MJPEG for OBS** in the app. In OBS add a **Media Source**, uncheck
*Local File*, and enter `http://127.0.0.1:8010/colour.mjpg`. Then **Start
Virtual Camera** and every app that takes a webcam will see it.

## What this cannot do

**No depth-based skeleton tracking, body/player index or face tracking of the
Microsoft kind.** Those came from Microsoft's SDK, which inferred bodies from
depth with a trained classifier; it exists only on Windows (and, for the Femto
Mega's Azure Kinect body tracking, Linux).

What this does instead is run MediaPipe on the colour stream and lift its
landmarks into 3D using our depth — see [TRACKING.md](TRACKING.md). It gets you
33 body joints, 21 per hand and gestures, in metres. It is weaker than
Microsoft's tracker at occlusion and unusual poses, because it reads the colour
image rather than depth, and it has no per-pixel player index.

Also worth knowing: [FreenectTD](https://github.com/stosumarte/FreenectTD) is a
native TouchDesigner plugin covering the Kinect v2's raw streams. If TD is your
only destination and you need neither the filtering nor tracking, it is less
machinery than this.

## Overnight testing

```bash
./soak.sh            # runs with crash capture, restarts on failure
./soak.sh --report   # summarise afterwards
```

Logs land in `logs/`: a 30-second heartbeat with fps, frames, memory and thread
count; `crash.log` with `faulthandler` armed to catch segfaults in the C
libraries; and thread dumps on stalls showing exactly where it blocked. A
22-hour run managed 1.5 million frames with flat memory.

## Gotchas

Recorded so nobody has to rediscover them.

**libfreenect2 won't configure without two flags.** It declares
`cmake_minimum_required(2.8.12.1)`, which CMake 4 refuses — pass
`-DCMAKE_POLICY_VERSION_MINIMUM=3.5`. And CMake may pick up a stale
`/usr/local/bin/pkg-config` from an old Intel Homebrew that can't see
`/opt/homebrew`'s libusb — pass
`-DPKG_CONFIG_EXECUTABLE=/opt/homebrew/bin/pkg-config`. `build.sh` does both.

**Syphon answers discovery on the runloop.** A tight publish loop never
services it, so the servers publish fine but stay invisible to every other app.
Hence `syphon_pump()` each frame.

**Tk and libfreenect2 fight over NSApp.** libfreenect2's OpenGL pipeline uses
GLFW, which installs its own `NSApplication` subclass; Tk installs
`TKApplication`. Whichever touches `NSApp` first wins and the loser dies on
`-[NSApplication macOSVersion]: unrecognized selector`. Create `tk.Tk()` first,
then Syphon, then the sensor. Both build hidden NSWindows, so both must be
constructed on the main thread — only the frame loop may be backgrounded.

**Never interpolate the cloud feed.** Depth is split across two bytes, so any
bilinear filtering averages a high byte with a low byte and produces nonsense
distances. The shader uses `texelFetch`; set the Syphon TOP's filter to
Nearest.

**A stalled sensor looks exactly like an idle one.** If USB transfers fail at
open, libfreenect2 keeps advertising its Syphon servers while never delivering
a frame. The tell is CPU: ~6% when dead versus ~33% when decoding.

**Stop it with a signal it handles, never `kill -9`.** A process killed without
closing the device leaves the Kinect half-configured, and the next program to
open it hangs in `k2_open`.

## Files

| File | |
|---|---|
| `kinect_app.py` | the app — preview, filters, all outputs, tracking |
| `camera.py` | camera backends: Kinect v2, Femto Mega, synthetic |
| `tracker.py` | MediaPipe body + hand tracking, depth-lifted, sent as OSC |
| `kproc.cpp`, `kfilters.h` | camera-agnostic depth pipeline and shared filters |
| `k2shim.cpp` | C shim over libfreenect2: gating, filtering, depth packing |
| `k2syphon.mm` | Syphon publisher (Objective-C++) |
| `kinect_check.py` | standalone sensor check, works for v1 and v2 |
| `depth_view.py` | the `Sensor` class, plus a standalone preview |
| `syphon_out.py` | standalone Syphon publisher |
| `stream.py` | standalone MJPEG server |
| `td_setup.py` | builds the TD network from the Textport |
| `kinect_xyz.glsl` | depth → XYZ unprojection shader |
| `soak.sh` | overnight soak test |
| `build.sh` | builds dependencies and shims |
| `make_apps.sh` | builds the .app bundle |

## Licence

MIT. libfreenect2 is Apache 2.0; Syphon is BSD.
