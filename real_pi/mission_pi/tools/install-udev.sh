#!/usr/bin/env bash
# real_pi: one-shot USB permissions for the Pixhawk + cameras.
# Run ON THE PI 5 as the user who flies:   bash tools/install-udev.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo cp "$HERE/pixhawk-udev.rules" /etc/udev/rules.d/99-drone.rules
sudo udevadm control --reload-rules
echo "installed /etc/udev/rules.d/99-drone.rules"
echo ""
echo "NEXT STEPS:"
echo "  1. unplug + replug the Pixhawk USB cable"
echo "  2. make sure you are in the dialout group (re-login if not):"
echo "       sudo usermod -aG dialout \$USER"
echo "  3. verify:"
echo "       dmesg | tail -5          (ttyACMx appeared?)"
echo "       python3 tools/list_cams.py"
echo "       python3 tools/fc_probe.py   (or: python3 main.py --check)"
