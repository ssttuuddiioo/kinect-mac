# Body and hand tracking → TouchDesigner over OSC

Tick **Body** and/or **Hands** in the app's Tracking row. Landmarks are drawn
over the preview, and every frame goes out as OSC to `127.0.0.1:9000` (the port
is editable in the app).

MediaPipe finds the landmarks in the colour image. Each one is then looked up
in the depth map and unprojected — so joints arrive as **real positions in
metres**, not image coordinates with a guessed depth.

## Receiving it in TD

Add an **OSC In CHOP**, set its port to match the app (9000 by default). Each
address becomes one channel. Pull what you need with a **Select CHOP** —
patterns like `pose/*/tx` or `hand/right/palm/*`.

## What's sent

One float per address, so every value is its own clean CHOP channel.

| Address | Meaning |
|---|---|
| `/pose/present` | 1 while a body is tracked, else 0 |
| `/pose/<joint>/x`, `/y` | 0–1 across the image, mirrored like the video feeds |
| `/pose/<joint>/tx`, `/ty`, `/tz` | metres, TD space: Y up, −Z forward |
| `/pose/<joint>/v` | visibility, 0–1 |
| `/hand/left/present`, `/hand/right/present` | per hand |
| `/hand/<side>/<joint>/x /y /tx /ty /tz` | 21 joints per hand |
| `/hand/<side>/palm/…` | wrist + four knuckles averaged — steadier than a fingertip |
| `/hand/<side>/gesture/<name>` | 1 for the recognised gesture, 0 for the rest |

Pose joints (33): `nose`, `left_eye_inner`, `left_eye`, `left_eye_outer`,
`right_eye_inner`, `right_eye`, `right_eye_outer`, `left_ear`, `right_ear`,
`mouth_left`, `mouth_right`, `left_shoulder`, `right_shoulder`, `left_elbow`,
`right_elbow`, `left_wrist`, `right_wrist`, `left_pinky`, `right_pinky`,
`left_index`, `right_index`, `left_thumb`, `right_thumb`, `left_hip`,
`right_hip`, `left_knee`, `right_knee`, `left_ankle`, `right_ankle`,
`left_heel`, `right_heel`, `left_foot_index`, `right_foot_index`.

Hand joints (21): `wrist`, `thumb_cmc/mcp/ip/tip`, `index_mcp/pip/dip/tip`,
`middle_…`, `ring_…`, `pinky_…` — plus `palm`.

Gestures: `None`, `Closed_Fist`, `Open_Palm`, `Pointing_Up`, `Thumb_Down`,
`Thumb_Up`, `Victory`, `ILoveYou`.

Left and right always mean **the person's** left and right, never the image's.

## Recipes

**Palm as a pointer.** `hand/right/palm/x` and `hand/right/palm/y` are 0–1 and
already One Euro–smoothed — map them straight onto whatever you drive.
`present` holds for 300 ms after a hand is lost, so a one-frame dropout doesn't
snap the pointer away.

**Skeleton on the point cloud.** The `tx/ty/tz` values use *exactly* the
unprojection maths of the point-cloud shader in POINTCLOUD.md, so joints land
on the cloud. Select `pose/*/tx`, `pose/*/ty`, `pose/*/tz`, run each through a
Shuffle CHOP set to sequence all channels (33 channels of 1 sample → 1 channel
of 33 samples), merge them into `tx ty tz`, and instance small spheres from that
in the same Render TOP as the cloud.

**Push and pull.** `tz` is metric depth. A hand moving toward the camera is
`tz` rising toward zero — something 2D tracking can't give you.

## Gate to slab

On by default. A body whose torso — or a hand whose palm — falls outside the
app's Near/Far slab is dropped before it's sent. Someone walking past behind
you never reaches TD. Turn it off to track at any distance.

## Why it's built this way

Measured on this Mac while building it:

- **Tracking gets the unmasked colour.** MediaPipe was trained on ordinary
  photographs; the background-removed cutout is out of distribution. Depth is
  used to validate and to lift joints into 3D — not to pre-mask the image.
- **GPU only.** MediaPipe 1.0.1's CPU path crashes on macOS (`Service is
  unavailable`). The GPU path needs RGBA input. Pose takes ~3.6 ms and
  hands+gestures ~4.8 ms of inference at 640×576. The whole tracker, including
  depth lookup and OSC, ran at 25 ms median at 1000×667 — about a 39 fps
  ceiling.
- **Its own thread.** Tracking always takes the newest frame and drops any it
  couldn't reach, so it can never slow the depth outputs or Syphon.
- **Pose sees the unmirrored image; Hands sees it mirrored.** Pose labels
  left/right by image position, so on a mirrored feed it called the left
  shoulder the right. Hands is documented to assume a mirrored view.
- **Frames are split across several bundles.** macOS rejects any UDP datagram
  over 9,216 bytes, and a full frame with both hands is ~17 KB.

## Without a camera

    .venv/bin/python kinect_app.py --camera synthetic --image models/test/pose.jpg

Replays a still photo as the colour stream at a flat 1.5 m — enough to build
and test a TD patch with no hardware attached.

## Tests

    .venv/bin/python test_tracker.py

Feeds reference photos through the tracker and decodes the OSC with
python-osc's own parser: joint labels, mirroring, depth lifting, the
unprojection matching the shader, the depth gate, datagram sizes. Needs the GPU,
so run it from a normal terminal.

## Root and the Femto Mega

OSC is a plain network socket, so tracking works however the app is launched
— including as root for the Femto Mega, where Syphon does not reach a
normally-launched TouchDesigner.
