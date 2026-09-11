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
| `/hand/left/present`, `/hand/right/present` | 1 while that hand is tracked |
| `/hand/<side>/open` | **1 while the hand is open, 0 when closed** — the on/off switch |
| `/hand/<side>/openness` | 0 = fist … 1 = fully open, continuous |
| `/hand/<side>/x`, `/y` | palm, 0–1 across the image, mirrored like the feeds |
| `/hand/<side>/tx`, `/ty`, `/tz` | palm, metres, TD space |

Hands send **where the hand is and whether it's open**: eight channels each. The palm
point is the wrist and four knuckles averaged, which stays put while fingers
bend — far steadier than any fingertip.

Pose joints (33): `nose`, `left_eye_inner`, `left_eye`, `left_eye_outer`,
`right_eye_inner`, `right_eye`, `right_eye_outer`, `left_ear`, `right_ear`,
`mouth_left`, `mouth_right`, `left_shoulder`, `right_shoulder`, `left_elbow`,
`right_elbow`, `left_wrist`, `right_wrist`, `left_pinky`, `right_pinky`,
`left_index`, `right_index`, `left_thumb`, `right_thumb`, `left_hip`,
`right_hip`, `left_knee`, `right_knee`, `left_ankle`, `right_ankle`,
`left_heel`, `right_heel`, `left_foot_index`, `right_foot_index`.

All 21 finger joints and gesture flags (`Closed_Fist`, `Open_Palm`,
`Pointing_Up`, `Thumb_Up`, …) are still computed, just not sent. Set
`Tracker.hand_detail = True` in `tracker.py` to send them — 119 channels per
hand instead of 6, under `/hand/<side>/<joint>/…` and `/hand/<side>/gesture/…`.

Left and right always mean **the person's** left and right, never the image's.

## Recipes

**Hand as a pointer.** `hand/right/x` and `hand/right/y` are 0–1 and
already One Euro–smoothed — map them straight onto whatever you drive.
`present` holds for 300 ms after a hand is lost, so a one-frame dropout doesn't
snap the pointer away.

**Skeleton on the point cloud.** The `tx/ty/tz` values use *exactly* the
unprojection maths of the point-cloud shader in POINTCLOUD.md, so joints land
on the cloud. Select `pose/*/tx`, `pose/*/ty`, `pose/*/tz`, run each through a
Shuffle CHOP set to sequence all channels (33 channels of 1 sample → 1 channel
of 33 samples), merge them into `tx ty tz`, and instance small spheres from that
in the same Render TOP as the cloud.

**Open = on, closed = off.** `hand/right/open` is 1 while your right hand is
open and 0 while it's closed — wire it straight to whatever you're switching.
For a one-shot event at the moment you open (rather than a state), put a
**Trigger CHOP** or a **Logic CHOP** set to *off to on* after it.

It's worked out from how far your fingertips are from your palm, not from
MediaPipe's gesture labels: on clearly open hands the gesture model returned
`None`, which would have read as "not open". The measurement uses MediaPipe's
3D hand coordinates, so it still works with your hand turned sideways.

It has two thresholds, not one: it switches **on above 0.75** openness and only
back **off below 0.45**. A single threshold chatters on/off/on when a hand
hovers near it; with a gap between the two it holds its state until you clearly
open or close. Calibrated on reference photos:

| Hand | openness | |
|---|---|---|
| fist | 0.00 | off |
| thumbs up | ~0.00 | off — the thumb is ignored |
| pointing | ~0.28 | off |
| victory (two fingers) | ~0.72 | in the gap: keeps its state |
| open | 0.84–0.99 | on |

To move the switch points, change `OPEN_ON` and `OPEN_OFF` at the top of
`tracker.py`. `openness` itself is continuous, for fading things rather than
switching them.

In the app's preview, a **filled** dot means the hand is reading open (sending
1) and a **ring** means closed (sending 0).

**Push and pull.** `tz` is metric depth. A hand moving toward the camera is
`tz` rising toward zero — something 2D tracking can't give you.

## Gate to slab

On by default. A body whose torso — or a hand whose palm — falls outside the
app's Near/Far slab is dropped before it's sent. Someone walking past behind
you never reaches TD. Turn it off to track at any distance.

## Reliability: MediaPipe runs in its own process

MediaPipe 1.0.1's macOS GPU path fails after a few minutes of use — its pixel
buffer allocator gives out (`kCVReturnAllocationFailed`) and it aborts, or its
graph wedges. That's inside MediaPipe; it reproduces with nothing of ours in the
process. And once it happens, MediaPipe can't be recreated in that process.

So tracking runs in a **child process**, and a watchdog replaces the child when
it dies, stops completing frames, goes silent, or fails to load. Expect a brief
gap each time — about 2–3 s after a crash, about 8–9 s after a wedge — and then
tracking resumes on its own. Each restart is logged as `TRACKER_RESTART` in the
health log, with the reason. Frames reach the child through shared memory, and
the camera thread never waits on it.

See [REVIEW.md](REVIEW.md) for how this was found and tested.

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
