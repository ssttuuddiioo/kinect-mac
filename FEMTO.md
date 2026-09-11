# Orbbec Femto Mega

The app drives an Orbbec Femto Mega as well as the Kinect v2, and publishes the
same outputs — depth, background-removed colour, and the packed point-cloud
feed — through the same filters.

    ./build.sh                       # once, sets up the Orbbec SDK
    double-click  Run Femto Mega.command

## Root is required on macOS

The Orbbec SDK has exactly one USB backend on the Mac, **libuvc**, and it has to
take the camera away from macOS's own camera driver to open it. That needs
root. Without it you'll see:

    uvc_open failed: [Path: 1-2-1.2, Return Code: -3]

That's an access-denied, not a broken camera. It's the same reason
TouchDesigner's Orbbec TOP needs `sudo` on a Mac. `Run Femto Mega.command`
handles it: asks for your password, runs the app as root, and hands the log
files back to you afterwards so the next normal launch can still write them.

**Consequence worth knowing:** Syphon servers from a root process are visible
only to other root processes. With a `sudo`'d TouchDesigner that's fine. A
normally-launched OBS or Resolume won't see the feeds — use the MJPEG output
for those, which is plain HTTP and doesn't care who owns the process.

## Only one program can hold it

Same rule as the Kinect. If TouchDesigner's own Orbbec TOP is active, the app
can't open the camera, and vice versa. Quit one before starting the other.

You can use either:

- **TouchDesigner's native Orbbec TOP** when TD is the only consumer and you
  don't need our filtering. Less machinery.
- **This app** when you want the depth gate and noise filters applied before
  TD sees the data, the packed point-cloud feed, or MJPEG for OBS.

## What changes versus the Kinect

**Resolution.** The Femto Mega's depth is 640×576 in its default narrow-FOV
mode (1024×1024 wide-FOV), not the Kinect's 512×424. Every output follows the
camera. The point-cloud shader reads the size from its input, so it needs no
change. `td_setup.py` reads the resolution from the live Syphon feed.

**Intrinsics.** Printed at startup as `femto intrinsics fx=… fy=… cx=… cy=…`.
Put those into the GLSL TOP's `uIntrinsics` — the Kinect values in
POINTCLOUD.md are wrong for this camera, and the point cloud will be subtly
distorted with them.

**Alignment.** Colour is aligned onto the depth grid in software
(`AlignFilter` to the depth stream), so every output is at depth resolution
and the depth intrinsics apply to all of it.

**Mirroring.** Outputs are mirrored exactly once, like the Kinect's. If the
device reports it already mirrors (`is_mirrored`), the app doesn't flip again.
The startup line prints `mirror=True/False` so you can see which happened.

## Not available on the Mac

The Femto Mega is Azure Kinect–compatible and can run **Microsoft's Azure
Kinect Body Tracking** — 32-joint skeletons. That SDK is Windows and Linux
only, so body tracking isn't possible here. Depth quality is excellent; the
skeletons still need another machine or the MediaPipe route described in the
README.

## Status

Written against pyorbbecsdk2 2.1.2's real API and the official Orbbec examples,
and the full app is verified end to end at 640×576 using the synthetic camera.
**It has not yet run against the physical camera**, because testing needs root
and the camera free of TouchDesigner. First real run:

    sudo .venv/bin/python kinect_app.py --camera femto

Things most likely to need adjusting on first contact: whether the default
colour profile offers RGB (the app falls back to depth-only if not, and says
so), and the mirroring on the point cloud.
