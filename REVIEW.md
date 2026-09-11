# Review and stress test — September 2026

A pass over the whole app for bugs and security problems, followed by sustained
load and fault injection. Every finding below was **reproduced first**: each
has a test that fails against the code as it was and passes now. Nothing is
listed on inspection alone.

Run everything: `.venv/bin/python test_kproc.py`, `stress_test.py`,
`test_tracker.py`, `test_tracker_process.py`, `test_exit.py`, and
`load_test.py 300 [--steady]`. The tracking tests need the GPU, so run them
from a normal terminal. 112 checks at the time of writing, all passing.

## Fixed

### Critical — hand and body tracking died permanently ~2 minutes in
In **ordinary use** — tracking simply left on — MediaPipe's hand graph failed
at 1 min 47 s and tracking never came back until the app was restarted.

Root cause is inside MediaPipe 1.0.1's macOS GPU path, not this code:
reproduced standalone (one thread, no Syphon, no app), it aborts after ~3
minutes with `Error creating pixel buffer: -6662` (`kCVReturnAllocationFailed`).
Inside the app it sometimes surfaced instead as `Packet isn't the sole owner of
the holder` and a wedged graph. Either way, MediaPipe could not be recreated in
the same process: every replacement hung while loading its models.

**Fix:** MediaPipe now runs in a child process (`TrackerProcess`), launched as a
plain subprocess. A watchdog kills and restarts it if it dies, stops completing
frames, goes silent, or never finishes loading. In the same 5-minute run,
tracking now survived both failure modes — an abort (exit −6) and a wedge — and
kept going. The app process also fell from ~1.4 GB to ~150 MB, since MediaPipe's
memory now lives in the child. Tests: `test_tracker_process.py`,
`load_test.py 300 --steady`.

**Still true:** each failure costs a visible gap — about 2–3 s after an abort,
about 8–9 s after a wedge — every few minutes. See *Not fixed*.

### High — use-after-free when the camera stalls, and on quit
Stall recovery closed the camera from the UI thread while the capture thread
was blocked inside `frame()` — in the C backends, freeing the device that thread
was waiting on. `quit()` did the same with a 300 ms delay against a 5 s wait.
**Fix:** the capture thread owns the camera and is the only thing that closes it,
after its last `frame()` returns. Quit waits for that, with a 10 s bound.
Tests: `stress_test.py` [1] [2].

### High — root log writes followed symlinks
The Femto Mega runs the app as root, and it opened `logs/crash.log` with a plain
`open()`. Anything running as the user could swap that file — or the whole
`logs/` directory — for a symlink, and root would write through it: an
arbitrary-file write primitive. **Fix:** `O_NOFOLLOW` on every log, `O_EXCL` on
per-run logs, and a symlinked log directory is refused in favour of a private
temp directory. Tests: `stress_test.py` [6] [6b] [6c].

### High — one exception froze the UI for good
`tick()` rescheduled itself only on its last line, so any single exception
stopped the preview, the status line and the **Syphon pump** — silently taking
TouchDesigner's view of the servers with it. **Fix:** it reschedules in
`finally`, and each distinct failure is logged once. Test: `stress_test.py` [3].

### Medium — MJPEG couldn't be switched back on
Turning MJPEG off called `shutdown()` but not `server_close()`, so the port was
never released; turning it back on failed with `Address already in use` while the
checkbox read *on*. Existing clients also kept receiving frames for up to 10 s.
**Fix:** the socket is closed, handlers leave as soon as their server is switched
off, and a failed bind unticks the box and says why. Found by `load_test.py`.

### Medium — the process could outlive its window
MediaPipe runs non-daemon dispatcher threads; one stuck in a broken graph held
the interpreter open after quit. `Kinect.app`'s launcher waits on that process,
so its PID file stayed and the next launch said the camera was in use. **Fix:**
MediaPipe is out of the app process, and the app leaves with `os._exit` after an
orderly shutdown. `test_exit.py` confirms the normal path exits in ~0.2 s; the
original hang needed a broken graph and couldn't be reproduced on demand.

### Medium — the first isolation design could deadlock
The first version of the fix shared a multiprocessing `Event` and `Lock` with the
child. A POSIX semaphore isn't released when the process holding it dies, so a
child killed at the wrong moment left the parent deadlocked the next time it
signalled — defeating the purpose. **Fix:** no cross-process locks at all; frames
use a seqlock. Test: `test_tracker_process.py` [B].

### Medium — the watchdog's replacement went to the wrong port
When the tracker was replaced, it re-read the OSC port box; if that held
half-typed text, it fell back to 9000 and quietly moved all tracking off the
port TouchDesigner listens on. **Fix:** a replacement keeps its predecessor's port.

### Medium — crash-looping child with no backoff
A child that couldn't start was respawned every half second, forever, with no
explanation. **Fix:** startup failures back off 2, 4, 8… up to 30 s, and after
three the reason appears in the status line.

### Medium — the child crashed when its launcher wasn't import-safe
`multiprocessing`'s spawn re-imports the parent's main script in the child.
Anything unguarded — here, the load test — re-ran in every child and killed it.
**Fix:** the child is a plain `python tracker.py --child` subprocess and imports
nothing of its caller's.

### Low — bad OSC ports, sticky errors, a stale frame rate
An out-of-range port raised `OverflowError` on every frame (it isn't an
`OSError`), and a tracker error, once set, was never cleared. The tracker's fps
also froze at its last value when it stalled, reading 30 while dead. **Fix:**
ports are validated, `OverflowError` is caught, a good frame clears the error,
and fps drops to 0 when nothing completes. Tests: `stress_test.py` [4] [5] [5d].

### Low — a log-name collision stopped the app starting
My own symlink fix initially refused to start at all if a log name was
pre-planted — trading a write attack for a denial of service. **Fix:** a
collision now gets an unguessable name. Test: `stress_test.py` [6c].

### Supply chain
`build.sh` installed whatever was newest and cloned libfreenect2's head. It now
pins mediapipe 1.0.1, pyorbbecsdk2 2.1.2, numpy 2.5.3, python-osc 1.10.2 and
libfreenect2 `fd64c5d`, and checks SHA-256s of every downloaded model and test
image.

## Checked and cleared

- **`chown -R` as root in the Femto launcher.** On macOS `-R` defaults to `-P`,
  which never follows symlinks, so a symlinked `logs/` can't redirect it. Now
  written explicitly as `-R -P`.
- **MJPEG server.** Binds `127.0.0.1` only; serves two fixed routes, so there's no
  path traversal.
- **OSC.** Send-only, always to `127.0.0.1`. Nothing is parsed from the network.
- **Pickle between the app and the tracker child.** Both run as the same user,
  so there's no trust boundary for it to cross.

## Not fixed — decisions for you

**The Femto launcher runs user-writable code as root.** `sudo python
kinect_app.py` executes `.py` files and a virtualenv that your user account can
modify, so any process running as you could plant code that runs as root at the
next Femto launch. The proper fix is privilege separation: a minimal root helper
that only opens the camera and hands frames to the normal, unprivileged app. It
would also fix Syphon, which from a root process is invisible to a normal
TouchDesigner. It's an architectural change, so it's your call.

**Tracking gaps.** Isolation turns MediaPipe's failures into gaps instead of
dead tracking, but the gaps remain: 2–3 s, or 8–9 s for a wedge, every few
minutes. They could be removed by rotating the child proactively — start the next
one before the current one fails and switch over when it's ready. Alternatively,
test another MediaPipe version: 1.0.1 is where both macOS bugs were found.

**Despeckle 7×7 at the Femto's resolution.** 39 ms per frame on its own, so it
can't hold 30 fps at 640×576. It cleans no better than 3×3; leave Despeckle at 1.

**MJPEG is unauthenticated on localhost.** Any local process can watch the feed.
Normal for a local video tool, but worth knowing.

**Kinect v2 path.** The capture-lifecycle fix and the new raw-frame export are
in the Kinect shim, but no Kinect was connected, so they haven't run on one.
