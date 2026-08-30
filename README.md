# kinect-mac

Kinect v2 (Xbox One) on Apple Silicon macOS, with output into TouchDesigner,
OBS, and anything that speaks Syphon or MJPEG.

TouchDesigner's own Kinect operators are Windows-only, and Microsoft's SDK was
never ported. This drives the sensor through libfreenect2 and publishes the
results itself.

## What it does

One app holds the sensor and drives every output at once:

| Output | Detail |
|---|---|
| Preview window | live depth or colour, with the filters applied |
| Syphon `Kinect Depth` | gated depth, near-bright / far-dim |
| Syphon `Kinect Colour` | colour with the background removed |
| Syphon `Kinect Cloud` | 16-bit depth packed for point-cloud use |
| MJPEG (optional) | `http://127.0.0.1:8010/…` for OBS |

Plus a depth gate and noise filtering — near/far slab, 3×3–7×7 median
despeckle, morphological erode, temporal smoothing — all applied in C before
the data leaves, so consumers get clean frames.

Only one process can hold a Kinect. That is why this is a single app rather
than several: the outputs are not alternatives, they run simultaneously.

## What it does not do

No skeleton tracking, body/player index, gesture or face tracking. Those came
from Microsoft's SDK, which computed them from depth using a trained
classifier, and none of it exists outside Windows. libfreenect2 provides raw
depth, IR and colour only.

A viable substitute is to run a pose model (MediaPipe) on the colour stream and
lift its 2D landmarks into metric 3D by sampling the depth map — feed it the
*unmasked* colour and use depth to validate detections, not to pre-mask the
image.

## Setup

    ./build.sh

Requires Homebrew at `/opt/homebrew`, working Command Line Tools, and
TouchDesigner installed (only for its bundled Syphon.framework).

Then `open ~/Applications/Kinect.app`.

## Point cloud in TouchDesigner

See [POINTCLOUD.md](POINTCLOUD.md) — the shader, the network, and the
intrinsics. `td_setup.py` can build the network for you from TD's Textport.

## Things that cost time here

Recorded so they don't have to be rediscovered.

**libfreenect2 will not configure without two flags.** It declares
`cmake_minimum_required(2.8.12.1)`, which CMake 4 refuses — pass
`-DCMAKE_POLICY_VERSION_MINIMUM=3.5`. And CMake may find a stale
`/usr/local/bin/pkg-config` from an old Intel Homebrew that cannot see
`/opt/homebrew`'s libusb — pass `-DPKG_CONFIG_EXECUTABLE=/opt/homebrew/bin/pkg-config`.

**The public Syphon SDK is x86_64 only** and cannot link into an arm64 build.
Building it for arm64 needs full Xcode. TouchDesigner ships a universal
Syphon.framework with headers; `build.sh` copies that.

**Syphon answers discovery on the runloop.** A tight publish loop never
services it and the servers stay invisible to every other app, despite
publishing fine. Hence `syphon_pump()` each frame.

**Tk and libfreenect2 fight over NSApp.** libfreenect2's OpenGL pipeline uses
GLFW, which installs its own `NSApplication` subclass; Tk installs
`TKApplication`. Whichever touches `NSApp` first wins and the other dies on
`-[NSApplication macOSVersion]: unrecognized selector`. Create `tk.Tk()` first,
then Syphon, then the sensor. Both Syphon and libfreenect2 build hidden
NSWindows, so both must be constructed on the main thread; only the frame loop
may be backgrounded.

**Never interpolate the cloud feed.** Depth is split across two bytes, so any
bilinear filtering averages a high byte with a low byte and yields nonsense
distances. The shader uses `texelFetch`; set the Syphon TOP to Nearest.

**A stalled sensor looks identical to an idle one.** If USB transfers fail at
open, libfreenect2 keeps advertising its Syphon servers while never delivering
a frame. The tell is CPU: ~6% when dead versus ~33% when decoding. The app now
detects this explicitly.

**Kill it with a signal it handles, not `-9`.** A process killed without
closing the device leaves the Kinect half-configured, and the next program to
open it hangs in `k2_open`.

## Files

| File | |
|---|---|
| `kinect_app.py` | the app — preview, filters, all outputs |
| `k2shim.cpp` | C shim over libfreenect2: gating, filtering, depth packing |
| `k2syphon.mm` | Syphon publisher (Objective-C++) |
| `kinect_check.py` | standalone sensor status check, v1 and v2 |
| `depth_view.py` | `Sensor` class plus a standalone preview |
| `syphon_out.py` | standalone Syphon publisher |
| `stream.py` | standalone MJPEG server |
| `soak.sh` | overnight soak test with crash capture |
| `build.sh` | builds dependencies and shims |
| `make_apps.sh` | builds the .app bundle |

## Licence

libfreenect2 is Apache 2.0, Syphon is BSD. This code is MIT.
