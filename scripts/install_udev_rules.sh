#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RULES_FILE="${SCRIPT_DIR}/../config/99-omx-openrb.rules"

sudo cp "${RULES_FILE}" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Reconnect the OpenRB devices, then verify:"
echo "ls -l /dev/omx_leader /dev/omx_follower"