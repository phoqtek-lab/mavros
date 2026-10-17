#!/usr/bin/env bash
set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }

here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
run_user="${GARUDANAV_RUN_USER:-garuda03}"
user_home="$(getent passwd "$run_user" | cut -d: -f6)"
[ -n "$user_home" ] || { echo "unknown user: $run_user" >&2; exit 1; }

params_file="${GARUDANAV_MAVROS_PARAMS:-$here/mavros-garuda03.yaml}"
[ -r "$params_file" ] || { echo "mavros params unreadable: $params_file" >&2; exit 1; }

install -d -o "$run_user" -g "$run_user" -m 755 "$user_home/.config/garudanav"
install -o "$run_user" -g "$run_user" -m 644 "$params_file" \
  "$user_home/.config/garudanav/$(basename "$params_file")"

install -d -o "$run_user" -g "$run_user" -m 755 "$user_home/garudanav_logs/logs"

install -m 644 "$here/99-pixracer-pro.rules" /etc/udev/rules.d/
install -m 644 "$here/99-pixracer-systemd.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

install -m 644 "$here/garudanav-mavros.service" /etc/systemd/system/
systemctl daemon-reload
systemd-analyze verify /etc/systemd/system/garudanav-mavros.service

systemctl enable garudanav-mavros.service

echo "garudanav-mavros installed and enabled; nothing was started"
echo "It starts when dev-pixracer.device appears, not at boot."
