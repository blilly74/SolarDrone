#!/bin/bash
# MAVSDK_launch.sh

# 1. Navigate and activate environment
cd /home/blilly/Desktop/SolarDrone
source /home/blilly/venv/bin/activate

# 2. Set MAVLink dialect to 2.0 (standard for ArduCopter 4.6.x)
export MAVLINK20=1

# 3. Clean up any stale processes that might be holding the serial port
pkill -9 mavproxy.py || true
pkill -9 python3 || true

# 4. Run the new MAVSDK script
# Note: Ensure you renamed your script to match this or update the filename below
python3 AltitudeAndPanelDetection_MAVSDK.py