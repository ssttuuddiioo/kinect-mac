# Feature roadmap — what's portable from Kinect-Xbox-360-Remold

Assessment of [Spidoug/Kinect-Xbox-360-Remold](https://github.com/Spidoug/Kinect-Xbox-360-Remold)
against **Kinect v2 on Apple Silicon macOS**. That project targets Kinect v1
(Xbox 360) on Windows and Linux, so its hardware assumptions differ from ours
in ways that matter.

Verified against the libfreenect2 source in this tree, not assumed.

| Feature | Feasible here | Notes |
|---|---|---|
| 3D scanner — ICP, TSDF fusion, OBJ/STL/PLY | **Yes** | Best candidate. Open3D does ICP, TSDF and mesh export; we already have metric depth, registered colour and intrinsics. |
| Surveillance — motion detect, IR day/night, pre-roll | **Yes** | libfreenect2 exposes IR (`Frame::Ir`, 512×424 float). Motion detection is a frame difference on gated depth. |
| Body pose / hands / gestures | **Yes, differently** | Not their depth-classifier approach. MediaPipe on colour, landmarks lifted to metric 3D by sampling our depth. |
| Multi-Kinect | **Partly** | `openDevice(idx)` supports it, but a v2 saturates a USB 3 controller. Two sensors realistically need separate controllers. |
| 4-mic array, GCC-PHAT, beam steering | **No** | See below. |
| Tilt motor, LED | **N/A** | v2 has neither. v1-only hardware. |

## The audio blocker

Their acoustic scanner and microphone modules are not portable. The Kinect v2
does physically have a four-microphone array, but **libfreenect2 implements no
audio support whatsoever** — the only match for "audio" in its entire source is
an interface-ID constant in `protocol/usb_control.h`. Compare libfreenect (v1),
which ships a whole `libfreenect_audio.h`.

The v2 audio interface is undocumented and was never reverse-engineered. Adding
it is a USB protocol research project, not a feature.

If audio matters, the practical answer is a separate USB mic array.

## Python version constraint

We currently run Python 3.14, which is too new for the scientific ecosystem.
Open3D 0.19 ships wheels for **3.10 / 3.11 / 3.12 only** (`universal2`, so
arm64 is covered). Anything using it needs Python ≤ 3.12.

Options: move the whole project to 3.12 (`brew install python@3.12
python-tk@3.12`), or run the scanner as a separate process under its own
interpreter. Moving wholesale is probably right — 3.14 buys us nothing here and
keeps colliding with prebuilt wheels.

## Suggested order

1. **IR feed** — small, immediately useful, unlocks the day/night half of
   surveillance. `Frame::Ir` alongside the existing depth and colour.
2. **3D scanner** — the highest-value module and the most self-contained.
   Capture depth frames, ICP-align, TSDF-fuse, export a mesh.
3. **Motion detection / recording** — pre-roll buffer on gated depth.
4. **Pose and hands** — MediaPipe plus depth lifting.

Each is independent; none blocks the others.
