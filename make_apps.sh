#!/bin/bash
# Build double-clickable .app bundles for the Kinect tools.
#
# These are launcher bundles: they run the Python scripts in place rather than
# embedding an interpreter, so editing a .py takes effect on the next launch
# with no rebuild. They do depend on Homebrew's python3.14 staying installed.
#
# Re-run this after renaming or adding a tool.  ./make_apps.sh

set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
PY=/opt/homebrew/bin/python3.14
DEST="$HOME/Applications"
mkdir -p "$DEST"

make_app() {
    local name="$1" script="$2" args="${3:-}"
    local app="$DEST/$name.app"
    local bin="${name// /}"

    rm -rf "$app"
    mkdir -p "$app/Contents/MacOS"

    cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
 "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>$name</string>
  <key>CFBundleDisplayName</key><string>$name</string>
  <key>CFBundleIdentifier</key><string>nyc.studiostudio.kinect.$bin</string>
  <key>CFBundleExecutable</key><string>$bin</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleVersion</key><string>1.0</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>LSMinimumSystemVersion</key><string>11.0</string>
</dict>
</plist>
PLIST

    cat > "$app/Contents/MacOS/$bin" <<LAUNCH
#!/bin/bash
# Launcher for $name. A bundle launched from Finder has nowhere to print, so
# everything goes to logs/app.log and real failures raise a dialog.
DIR="$HERE"
LOG="\$DIR/logs/app.log"
PIDFILE="\$DIR/logs/kinect.pid"
mkdir -p "\$DIR/logs"

# Only one program can hold the Kinect; a second one hangs in k2_open with no
# explanation. Identify the holder by PID file, not by pgrep on the script
# name - that also matches shells that merely mention it, including the one
# that launched this.
if [ -f "\$PIDFILE" ]; then
    OLD=\$(cat "\$PIDFILE" 2>/dev/null || true)
    if [ -n "\$OLD" ] && kill -0 "\$OLD" 2>/dev/null &&
       ps -p "\$OLD" -o command= 2>/dev/null | grep -qE 'syphon_out|depth_view|stream'; then
        osascript -e 'display alert "Kinect already in use" message "Another Kinect app is running. Quit it first - only one program can hold the sensor at a time." as critical' >/dev/null 2>&1
        exit 1
    fi
    rm -f "\$PIDFILE"
fi

cd "\$DIR"
echo "=== \$(date '+%Y-%m-%d %H:%M:%S')  launching $script $args ===" >> "\$LOG"

$PY $script $args >> "\$LOG" 2>&1 &
CHILD=\$!
echo "\$CHILD" > "\$PIDFILE"
wait "\$CHILD"
CODE=\$?
rm -f "\$PIDFILE"

if [ "\$CODE" -ne 0 ]; then
    WHY=\$(grep -vE 'IMK|^\[Debug\]' "\$LOG" | tail -3 | tr '\n' ' ' | tr -d '"' | cut -c1-300)
    osascript -e "display alert \"$name could not start\" message \"\$WHY\" as critical" >/dev/null 2>&1
fi
exit "\$CODE"
LAUNCH

    chmod +x "$app/Contents/MacOS/$bin"
    # Ad-hoc sign so Gatekeeper treats it as a stable identity
    codesign --force --sign - "$app" 2>/dev/null || true
    echo "  built  $app"
}

echo "Building into $DEST"
# One app. All outputs run at once from a single sensor handle - splitting
# them into separate apps only created a lock they had to fight over.
rm -rf "$DEST/Kinect Syphon.app" "$DEST/Kinect Viewer.app" "$DEST/Kinect Stream.app"
make_app "Kinect" "kinect_app.py" ""
echo
echo "Done. Open them from Spotlight, Launchpad, or:"
echo "  open \"$DEST/Kinect.app\""
