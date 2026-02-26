#!/bin/bash
# launch_test.sh

# Navigate to project folder
cd /home/blilly/Desktop/SolarDrone

# Activate virtual environment
source /home/blilly/venv/bin/activate

# Run the MAVSDK script
python3 AltitudeAndPanelDetection_MAVSDK.py >> /home/blilly/Desktop/SolarDrone/mission_debug.log 2>&1