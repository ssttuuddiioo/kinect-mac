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
PY=/opt/homebrew/bin/python3.14
TD_SYPHON="/Applications/TouchDesigner.app/Contents/Frameworks/Syphon.framework"

echo "==> dependencies"
$BREW install libusb jpeg-turbo glfw cmake pkg-config python-tk || true

echo "==> libfreenect2"
if [ ! -d "$HERE/libfreenect2" ]; then
    git clone --depth 1 https://github.com/OpenKinect/libfreenect2.git "$HERE/libfreenect2"
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

echo "==> app bundle"
"$HERE/make_apps.sh"

echo
echo "Done.  Run:  $PY $HERE/kinect_app.py"
echo "       or:  open ~/Applications/Kinect.app"
