import collections
import collections.abc
import sys

if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    collections.MutableMapping = collections.abc.MutableMapping

import time
from dronekit import connect

# Set the connection string
# This is the serial port for the Cube (usually /dev/ttyACM0)
# and the baud rate we set in ArduPilot (115200)
connection_string = 'tcp:127.0.0.1:14551'

print(f"Connecting to vehicle on: {connection_string}...")

# Connect to the Vehicle
try:
    vehicle = connect(connection_string, wait_ready=True)
except Exception as e:
    print(f"Failed to connect: {e}")
    print("Check: 1. Is the flight controller plugged in? 2. Is the port correct? 3. Do you have permissions?")
    exit()

print("Connection successful! Reading RC channels...")
print("Press Ctrl+C to exit.\n")

try:
    while True:
        # vehicle.channels is a dictionary-like object
        # It holds the PWM value for each channel.
        # Channels are 1-indexed (e.g., vehicle.channels[1], vehicle.channels[2])
        
        print("---")
        print(f" Ch1 (Roll):   {vehicle.channels[1]}")
        print(f" Ch2 (Pitch):  {vehicle.channels[2]}")
        print(f" Ch3 (Thr):    {vehicle.channels[3]}")
        print(f" Ch4 (Yaw):    {vehicle.channels[4]}")
        print(f" Ch5:          {vehicle.channels[5]}")
        print(f" Ch6:          {vehicle.channels[6]}")
        print(f" Ch7:          {vehicle.channels[7]}")
        print(f" Ch8:          {vehicle.channels[8]}")
        
        # Wait for 0.5 seconds before printing again
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nExiting script.")

finally:
    # Close the vehicle connection
    vehicle.close()
    print("Connection closed.")