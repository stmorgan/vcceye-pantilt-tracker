#!/bin/sh
# Install (or remove with --uninstall) the face tracker autostart on the Pi.
# Run from the project directory on the Pi: sh autostart/install.sh
set -e
here=$(cd "$(dirname "$0")" && pwd)
unit_dir="$HOME/.config/systemd/user"
xdg_dir="$HOME/.config/autostart"
labwc_rc="$HOME/.config/labwc/rc.xml"

reload_labwc() {
    # labwc re-reads its config on SIGHUP.
    pkill -HUP -x labwc 2>/dev/null || true
}

if [ "$1" = "--uninstall" ]; then
    systemctl --user stop vcceye.service 2>/dev/null || true
    rm -f "$unit_dir/vcceye.service" "$xdg_dir/vcceye-autostart.desktop"
    systemctl --user daemon-reload
    if cmp -s "$here/labwc-rc.xml" "$labwc_rc"; then
        rm -f "$labwc_rc"
        reload_labwc
    fi
    echo "Autostart removed."
    exit 0
fi

mkdir -p "$unit_dir" "$xdg_dir" "$(dirname "$labwc_rc")"
cp "$here/vcceye.service" "$unit_dir/"
cp "$here/vcceye-autostart.desktop" "$xdg_dir/"
systemctl --user daemon-reload

# Full-screen window rule for the desktop. Don't overwrite someone's own rc.xml.
if [ ! -e "$labwc_rc" ] || cmp -s "$here/labwc-rc.xml" "$labwc_rc"; then
    cp "$here/labwc-rc.xml" "$labwc_rc"
    reload_labwc
else
    echo "NOTE: $labwc_rc already exists and was not changed."
    echo "      To get the full-screen preview, add the rules from"
    echo "      $here/labwc-rc.xml to it, then run: pkill -HUP -x labwc"
fi
echo "Installed. The tracker will start when the desktop starts."
