#!/bin/sh
# Install (or remove with --uninstall) the face tracker autostart on the Pi.
# Run from the project directory on the Pi: sh autostart/install.sh
set -e
here=$(cd "$(dirname "$0")" && pwd)
unit_dir="$HOME/.config/systemd/user"
xdg_dir="$HOME/.config/autostart"

if [ "$1" = "--uninstall" ]; then
    systemctl --user stop vcceye.service 2>/dev/null || true
    rm -f "$unit_dir/vcceye.service" "$xdg_dir/vcceye-autostart.desktop"
    systemctl --user daemon-reload
    echo "Autostart removed."
    exit 0
fi

mkdir -p "$unit_dir" "$xdg_dir"
cp "$here/vcceye.service" "$unit_dir/"
cp "$here/vcceye-autostart.desktop" "$xdg_dir/"
systemctl --user daemon-reload
echo "Installed. The tracker will start when the desktop starts."
