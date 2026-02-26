import time
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from simple_pid import PID
from pymavlink import mavutil


# --- Camera Intrinsics ---
# NEED TO PERFORM CAMERA CALIBRATION TEST
focalX = 800.0
focalY = 800.0
centerX = 320.0
centerY = 240.0

class VisualServoController:
    """
    Manages the PID controllers and calculates velocity.
    """
    def __init__(self, kp=0.01, ki=0.001, kd=0.001, maxSpeed=0.5):
        print("Initializing PID controllers...")

        # Need 2 PID controllers: one for X (roll) and one for Y (Pitch)
        self.pidX = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=centerX, outputLimits=(-maxSpeed, maxSpeed))
        
        self.pidY = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=centerY, outputLimits=(-maxSpeed, maxSpeed))
    
    def update(self, originCoords):
        """
        Takes the current pixel location and returns velocity commands.
        """
        px, py = originCoords

        # Get the PID outputs
        # Note: The output is inverted by simple-pid so it must be flipped.
        # A positive error (px > 320) means drone needs to move right (positive vy).
        # A positive error (py >240) means the drone needs to move forward (positive vx).

        # simple-pid output = Kp * (setpoint - currentValue)
        # Want Kp * (currentValue - setpoint)
        # Therefore, use negative of the output.

        vy = -self.pidX(px) # Velocity RIGHT (Roll)
        vx = -self.pidY(py) # Velocity FORWARD (Pitch)

        return (vx, vy)
    
# --- MAVLink Velocity Function ---
def sendBodyVelocity(drone, vx, vy, vz):
    """
    Sends MAVLink velocity commands in the drone's body frame.
    vx > 0: Forward
    vy > 0: Right
    vz > 0: Down
    """

    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,                                      # time_boot_ms (not used)
        0, 0,                                   # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,     # Frame: relative to drone's body
        0b0000111111000111,                     # Bitmask: Use VELOCITY ONLY (ignore position, accel)
        0, 0, 0,                                # Position (not used)
        vx, vy, vz,                             # Velocity (m/s)
        0, 0, 0,                                # Acceleration (not used)
        0, 0,)                                  # Yaw, Yaw Rate (not used)
    drone.send_mavlink(msg)

def log_to_gcs(drone, message):
    """
    Prints a message to the local Pi console AND send it to the
    Mission Planner as a STATUSTEXT message.
    """
    # 1. Print to the local console
    print(message)

    # 2. Send to Mission Planner
    messageBytes = message.encode('utf-8')[:50]

    msg = drone.message_factory.statustext_encode(
        mavutil.mavlink.MAV_SEVERITY_INFO,
        messageBytes
    )
    drone.send_mavlink(msg)

def connectDrone(connectionString, baud=None, wait_ready=True):
    """Connects to the drone."""
    print(f"Connecting to the drone on: {connectionString}...")
    
    if baud:
        drone = connect(connectionString, baud=baud, wait_ready=wait_ready)
    else:
        drone = connect(connectionString, wait_ready=wait_ready)
    
    print("Vehicle connected.")
    return drone

def waitForMinThrottle(drone, throttleChannel, minPWM):
    """
    Waits for the throttle to be above a minimum value.
    This is a safety check to ensure the pilot's stick is not at zero.
    """
    print(f"Waiting for minimum throttle on RC channel {throttleChannel} (need > {minPWM})...")
    while True:
        # Read the RC channel value
        throttleValue = drone.channels.get(str(throttleChannel))
        
        if throttleValue is None:
            # This can happen briefly on connection, just wait
            print("Reading RC channels...")
            time.sleep(1)
            continue
            
        if throttleValue > minPWM:
            # Success
            print(f"Throttle OK ({throttleValue} > {minPWM}).")
            break
        
        # Print a warning but don't spam the log
        print(f"Throttle is too low: {throttleValue}. Please raise throttle stick.")
        time.sleep(1) # Wait 1 second before checking again

def waitForActivation(RCSwitchChannel, drone, activationPWM):
    """
    Waits for the activation signal from the RC channel.
    (This is the corrected, simplified version)
    """
    print(f"Waiting for activation signal on RC channel {RCSwitchChannel} (need > {activationPWM})...")
    while True:
        pwmValue = drone.channels.get(str(RCSwitchChannel))
        
        if pwmValue is not None and pwmValue >= activationPWM:
            print("Activation signal received.")
            break
        time.sleep(0.2) # Check 5 times a second

def armAndTakeoff(drone, targetAltitude):
    """Arms the drone and takes off to the target altitude."""
    print("Pre-arm checks...")
    while not drone.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors...")
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    while not drone.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {targetAltitude} meters...")
    drone.simple_takeoff(targetAltitude)

    # Wait until the vehicle reaches at least 95% of the target altitude
    while True:
        currentAltitude = drone.location.global_relative_frame.alt
        print(f" Current Altitude: {currentAltitude:.2f} m")
        
        if drone.mode.name != 'GUIDED':
            print("!!! PILOT OVERRIDE DETECTED (during takeoff) --- ABORTING SCRIPT !!!")
            return False
        
        if currentAltitude >= targetAltitude * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

    return True   

def change_altitude(drone, targetAltitude):
    """
    Commands the drone to change to a new altitude while in GUIDED mode.
    Maintains current Lat/Lon.
    Returns True on success, False on pilot override.
    """
    print(f"Changing altitude to {targetAltitude} meters...")
    
    # Get current location
    current_location = drone.location.global_relative_frame
    
    # Command the drone to go to its current lat/lon but new altitude
    target_location = LocationGlobalRelative(
        current_location.lat, 
        current_location.lon, 
        targetAltitude
    )
    drone.simple_goto(target_location)

    # Wait until the vehicle reaches at least 95% of the target altitude
    while True:
        currentAltitude = drone.location.global_relative_frame.alt
        print(f" Current Altitude: {currentAltitude:.2f} m")
        
        # Check for pilot override
        if drone.mode.name != 'GUIDED':
            print("!!! PILOT OVERRIDE DETECTED (during altitude change) --- ABORTING SCRIPT !!!")
            return False # Signal the main script that it failed
            
        if currentAltitude >= targetAltitude * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)
        
    return True # Signal that it was successful

def set_gimbal_tilt(drone,tilt_angle):
    """
    Sets the gimbal tilt. 0 is forward. -90 is straight down.
    """
    print(f"Setting gimbal tilt to {tilt_angle} degrees...")
    drone.gimbal.rotate(tilt_angle, 0, 0)
    time.sleep(2)

def get_downward_lidar_distance(drone):
    """
    Reads the downward-facing LiDAR.
    ArduPilot exposees this as vehicle.rangefinder.
    """
    return drone.rangefinder.distance

def get_forward_lidar_distance(drone):
    """
    Reads the second LiDAR (front-facing)
    If configured as Rangefinder 2, we read the raw MAVLink message.
    """
    msg = drone.recv_match(type='DISTANCE_SENSOR',blocking=True, timeout=1)
    if msg and msg.orientation == 0:
        return msg.current_distance / 100 # Convert cm to m
    return None

def set_gimbal_tilt(drone,tilt_angle):
    """
    Sets the gimbal tilt. 0 is forward. -90 is straight down.
    """
    print(f"Setting gimbal tilt to {tilt_angle} degrees...")
    drone.gimbal.rotate(tilt_angle, 0, 0)
    time.sleep(1.5)