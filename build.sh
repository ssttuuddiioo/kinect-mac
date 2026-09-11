#!/bin/bash
# Build everything from a fresh checkout.
#
#   ./build.sh
#
# Needs: Homebrew (Apple Silicon, at /opt/homebrew), working Command Line
# Tools, and TouchDesigner installed (only for its bundled Syphon.framework).

set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
BREW=/opt/homebrew/bin/brew
PY="$HERE/.venv/bin/python"
TD_SYPHON="/Applications/TouchDesigner.app/Contents/Frameworks/Syphon.framework"

echo "==> dependencies"
$BREW install libusb jpeg-turbo glfw cmake pkg-config python@3.12 python-tk@3.12 || true

echo "==> python 3.12 venv"
# 3.12, not newer: pyorbbecsdk2 ships wheels up to 3.13 and Open3D only to
# 3.12. Both have native arm64 macOS builds.
if [ ! -x "$HERE/.venv/bin/python" ]; then
    /opt/homebrew/bin/python3.12 -m venv "$HERE/.venv"
fi
"$HERE/.venv/bin/pip" install -q --upgrade pip
# Pinned to the exact versions tested. MediaPipe especially: 1.0.1 has macOS
# bugs this code works around (CPU delegate crash, GPU allocator abort), and a
# different version could change that behaviour without warning.
"$HERE/.venv/bin/pip" install -q pyorbbecsdk2==2.1.2 numpy==2.5.3 \
    mediapipe==1.0.1 python-osc==1.10.2

echo "==> MediaPipe models"
# Google's official model bucket. Tracking is GPU-only on macOS (see TRACKING.md).
mkdir -p "$HERE/models/test"
MP=https://storage.googleapis.com/mediapipe-models
[ -f "$HERE/models/pose_landmarker_full.task" ] || curl -sSfL -o "$HERE/models/pose_landmarker_full.task" \
    "$MP/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"
[ -f "$HERE/models/gesture_recognizer.task" ] || curl -sSfL -o "$HERE/models/gesture_recognizer.task" \
    "$MP/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task"
for img in pose.jpg woman_hands.jpg fist.jpg; do
    [ -f "$HERE/models/test/$img" ] || curl -sSfL -o "$HERE/models/test/$img" \
        "https://storage.googleapis.com/mediapipe-assets/$img"
done

# Refuse anything that isn't byte-for-byte what was tested.
( cd "$HERE/models" && shasum -a 256 -c - <<'SUMS'
4eaa5eb7a98365221087693fcc286334cf0858e2eb6e15b506aa4a7ecdcec4ad  pose_landmarker_full.task
97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482  gesture_recognizer.task
c8a830ed683c0276d713dd5aeda28f415f10cd6291972084a40d0d8b934ed62b  test/pose.jpg
70cbeb38e198c9862202e0979c21a99b40ca980d3e7b250176c85b1636a40f12  test/woman_hands.jpg
43fa1cabf3f90d574accc9a56986e2ee48638ce59fc65af1846487f73bb2ef24  test/fist.jpg
SUMS
) || { echo "ERROR: a downloaded model or test image failed its checksum" >&2; exit 1; }

echo "==> libfreenect2"
LIBFREENECT2_COMMIT=fd64c5d9b214df6f6a55b4419357e51083f15d93   # the commit tested
if [ ! -d "$HERE/libfreenect2" ]; then
    git clone https://github.com/OpenKinect/libfreenect2.git "$HERE/libfreenect2"
    git -C "$HERE/libfreenect2" checkout -q "$LIBFREENECT2_COMMIT"
fi
if [ "$(git -C "$HERE/libfreenect2" rev-parse HEAD)" != "$LIBFREENECT2_COMMIT" ]; then
    echo "WARNING: libfreenect2 is not at the tested commit $LIBFREENECT2_COMMIT" >&2
fi
mkdir -p "$HERE/libfreenect2/build"
cd "$HERE/libfreenect2/build"
# Two flags this will not build without:
#  - CMAKE_POLICY_VERSION_MINIMUM: libfreenect2 declares cmake_minimum_required
#    2.8.12.1 and CMake 4 refuses anything below 3.5.
#  - PKG_CONFIG_EXECUTABLE: a stale /usr/local/bin/pkg-config (from an Intel
#    Homebrew) cannot see /opt/homebrew's libusb, and configure fails.
PKG_CONFIG_PATH=/opt/homebrew/lib/pkgconfig /opt/homebrew/bin/cmake .. \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DPKG_CONFIG_EXECUTABLE=/opt/homebrew/bin/pkg-config \
    -DCMAKE_INSTALL_PREFIX="$HERE/libfreenect2/install" \
    -DBUILD_OPENNI2_DRIVER=OFF -DENABLE_CUDA=OFF
make -j8
cd "$HERE"

echo "==> Syphon.framework"
# The public Syphon SDK 5 download is x86_64 only and cannot link into an
# arm64 build. TouchDesigner ships a universal one, with headers.
if [ ! -d "$HERE/vendor/Syphon.framework" ]; then
    if [ ! -d "$TD_SYPHON" ]; then
        echo "ERROR: $TD_SYPHON not found." >&2
        echo "Install TouchDesigner, or supply an arm64 Syphon.framework in vendor/." >&2
        exit 1
    fi
    mkdir -p "$HERE/vendor"
    cp -R "$TD_SYPHON" "$HERE/vendor/"
fi

echo "==> shims"
clang++ -std=c++11 -O2 -dynamiclib -o "$HERE/libk2shim.dylib" "$HERE/k2shim.cpp" \
    -I"$HERE/libfreenect2/include" -I"$HERE/libfreenect2/build" \
    -L"$HERE/libfreenect2/build/lib" -lfreenect2 \
    -Wl,-rpath,"$HERE/libfreenect2/build/lib"

clang++ -ObjC++ -std=c++11 -O2 -fobjc-arc -dynamiclib -o "$HERE/libk2syphon.dylib" \
    "$HERE/k2syphon.mm" -F"$HERE/vendor" -framework Syphon \
    -framework Foundation -framework OpenGL -framework CoreGraphics \
    -Wno-deprecated-declarations -Wl,-rpath,"$HERE/vendor"

clang++ -std=c++11 -O2 -dynamiclib -o "$HERE/libkproc.dylib" "$HERE/kproc.cpp"
clang -O2 -dynamiclib -o "$HERE/libcrashguard.dylib" "$HERE/crashguard.c"

cd "$HERE"
echo "==> Femto-sized test image"
# The Femto Mega's 640x576, cropped from the reference pose photo: the load and
# tracker-process tests run at the resolution that actually matters.
"$PY" - <<'CROP'
import sys, numpy as np, mediapipe as mp
sys.path.insert(0, ".")
from stream import Jpeg
a = np.asarray(mp.Image.create_from_file("models/test/pose.jpg").numpy_view())[..., :3]
h, w = a.shape[:2]; y, x = (h - 576) // 2, (w - 640) // 2
crop = np.ascontiguousarray(a[y:y + 576, x:x + 640])
open("models/test/pose_640x576.jpg", "wb").write(Jpeg().encode(crop.tobytes(), 640, 576, False, 95))
CROP

echo "==> tests"
"$PY" "$HERE/test_kproc.py"

echo "==> app bundle"
"$HERE/make_apps.sh"

echo
echo "Done."
echo "  Kinect v2 :  open ~/Applications/Kinect.app"
echo "  Femto Mega:  double-click 'Run Femto Mega.command'  (needs sudo on macOS)"
echo "  no camera :  $PY $HERE/kinect_app.py --camera synthetic"
