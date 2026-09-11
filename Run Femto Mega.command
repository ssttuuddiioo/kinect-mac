#!/bin/bash
# Double-click to run the app on an Orbbec Femto Mega.
#
# Root is unavoidable on macOS: the Orbbec SDK's only Mac USB backend is
# libuvc, which must take the camera away from the system camera driver.
# Syphon servers from a root process are visible only to other root processes
# - fine with a sudo'd TouchDesigner, invisible to a normally-launched OBS.

cd "$(dirname "$0")" || exit 1
echo "Orbbec Femto Mega needs root on macOS. You'll be asked for your password."
echo "Quit TouchDesigner's Orbbec TOP first - only one program can hold the camera."
echo
sudo "$PWD/.venv/bin/python" "$PWD/kinect_app.py" --camera femto "$@"
code=$?
# Files written as root would lock out the next normal launch.
sudo chown -R "$(id -un)" "$PWD/logs" 2>/dev/null
exit $code
