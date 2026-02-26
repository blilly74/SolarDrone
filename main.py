import collections
import collections.abc
import sys
import time
import os
import cv2 as cv

if sys.version_info.major == 3 and sys.version_info.minor >= 10:
    collections.MutableMapping = collections.abc.MutableMapping

import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative

from src.captureRGB_Function import initializeRGBCamera, getRGBFrame, closeRGBCamera
from src.captureThermal_Function import initializeThermalCamera, getThermalFrame, closeThermalCamera
from src.processImage_Function import processImage
from src.findOriginFromContour_Function import findOriginFromContour
from src import navigationController_Function as nav
from src.navigationController_Function import VisualServoController, get_forward_lidar_distance, set_gimbal_tilt

# --- Constant and Configuration ---

connectionString = 'tcp:127.0.0.1:14551' # Or '/dev/ttyAMA0', 'tcp:127.0.0.1:5760'
startAltitude = 10      # Starting altitude in meters
altitudeIncrease = 5    # Altitude increase after each image capture
maxAttempts = 3         # Maximum number of attempts
qualityThreshold = 0.8  # Minimum rectangularity score
RCSwitchChannel = 8     # RC channel to listen for program activation (Need to verify this channel)
activationPWM = 1800    # PWM value to activate the program (Need to verify this value)
centeringTolerance = 10 # How many pixels from center counts as "centered"

# --- RGB Sensor Constants ---
sensor_width = 3.67 # mm
sensor_height = 2.69 # mm
image_width = 2592 # pixels
image_height = 1944 # pixels
focal_length = 3.60 # mm
overlap_percent = 0.80 # 80% overlap for stitching

# --- File Transfer & Save Configuration ---
savePath = "/home/blilly/Desktop/Mission_Data"
laptopIP = "192.168.1.101"
laptopUser = "Blake Lilly"

# --- Main Execution Logic ---

def main():
    initializeRGBCamera()
    initializeThermalCamera(cameraIndex=1)

    drone = None
    pidController = VisualServoController(kp=0.01, ki=0.001, kd=0.001)

    try:
        # --- 1. Connect and Wait ---
        drone = nav.connectDrone(connectionString, wait_ready=True)
        nav.waitForActivation(RCSwitchChannel,drone, activationPWM)

        # --- 2. Takeoff ---
        currentAltitude = startAltitude
        nav.armAndTakeoff(drone, currentAltitude)

        # -- 4. State Machine ---
        mapping_step = 0
        currentMode = "SEARCHING"
        searchStartTime = time.time()
        searchAttempt = 0

        while True:
            frame = getRGBFrame()
            if frame is None:
                print("Error: Failed to capture frame.")
                continue

            drawn_img, contour, score = processImage(frame)

            # --- SEARCHING STATE ---
            if currentMode == "SEARCHING":
                if score is not None and score > qualityThreshold:
                    # --- Target Locked ---
                    print(f"Target Locked. Score: {score:.4f}. Switching to Tracking.")
                    currentMode = "TRACKING"

                 #  Timeout Logic
                elif (time.time() - searchStartTime) > 5.0: # 5 second timeout
                        if searchAttempt < maxAttempts - 1:
                            currentAltitude += altitudeIncrease
                            print(f"Search timeout. Increasing altitude to {currentAltitude}m...")
                            nav.armAndTakeoff(drone, currentAltitude)
                            searchStartTime = time.time() # Reset timer
                            searchAttempt += 1 # Add one to the attempt counter
                        else:
                            print(f"Failed to find panel after {maxAttempts} attempts.")
                            currentMode = "LANDING"

            # --- TRACKING STATE ---
            elif currentMode == "TRACKING":
                # Check if the track was lost
                if score is None or score < qualityThreshold:
                    print("Target Lost. Reverting to SEARCHING.")
                    nav.sendBodyVelocity(drone, 0, 0, 0)
                    currentMode = "SEARCHING"
                    searchStartTime = time.time()
                    continue

                originCoords = findOriginFromContour(contour)
                if originCoords is None:
                    continue 

                # Check if image is centered
                errorX = abs(originCoords[0] - pidController.pidX.setpoint)
                errorY = abs(originCoords[1] - pidController.pidY.setpoint)

                if errorX < centeringTolerance and errorY < centeringTolerance:
                    # --- Target Centered: Transition to ALIGNING ---
                    print("Target is centered. Switching to ALIGNING for slope detection.")
                    nav.sendBodyVelocity(drone, 0, 0, 0)
                    currentMode = "ALIGNING"
                else:
                    # Still tracking: update PID
                    vx, vy = pidController.update(originCoords)
                    print(f"Tracking... Error: (x={errorX:.0f}, y={errorY:.0f}) -> Cmd:({vx:.2f}, vy={vy:.2f})")
                    nav.sendBodyVelocity(drone, vx, vy, 0)


            # --- ALIGNING STATE ---
            elif currentMode == "ALIGNING":
                print("--- Detecting roof slope ---")
                best_tilt = 0
                min_dist = float('inf')

                # Sweep gimbal to find the perpendicular angle
                for angle in range(0, -65, -5):
                      nav.set_gimbal_tilt(drone, angle)
                      dist = nav.get_forward_lidar_distance(drone)

                      if dist and dist < min_dist:
                            min_dist = dist
                            best_tilt = angle

                # Set gimbal to the optimal angle
                print(f"Detected optimal tilt at {best_tilt} degrees.")
                nav.set_gimbal_tilt(drone, best_tilt)

                rect = cv.minAreaRect(contour)
                (w_px, h_px) = rect[1]
                contour_w_pixels = max(w_px, h_px)
                contour_h_pixels = min(w_px, h_px)

                # 1. Calculate mission parameters from LiDAR
                h_dist = drone.rangefinder.distance 
                gsd = (h_dist * 1000 * sensor_width) / (focal_length * image_width)
                footprint_h = (image_height * gsd) / 1000 

                # 2. Determine actual panel height in meters
                panel_height_m = (contour_h_pixels * gsd) / 1000

                # 3. Calculate rows and steps autonomously
                effective_row_height = footprint_h * (1 - overlap_percent)
                rows_to_map = int(np.ceil(panel_height_m / effective_row_height))
                mapping_step = (image_width * gsd / 1000) * (1 - overlap_percent)

                print(f"Autonomous Planning: Panel is {panel_height_m:.2f}m tall.")
                print(f"Mapping {rows_to_map} rows with a step size of {mapping_step:.2f}m.")
                currentMode = "INSPECTING"

            # --- INSPECTING STATE ---
            elif currentMode == "INSPECTING":
                import csv
                metadata_file = os.path.join(savePath, "mission_metadata.csv")
                print(f"--- Starting Systematic Sweep. Calculated Step: {mapping_step:.2f}m ---")

                move_fwd = True
                os.makedirs(savePath, exist_ok=True)
                
                # Initialize CSV
                with open(metadata_file, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["Timestamp", "Filename", "Lat", "Lon", "Alt_LiDAR_m", "Alt_FC_m"])

                for row in range(rows_to_map):
                    print(f"Mapping row {row+1} of {rows_to_map}...")

                    while True:
                        # 1. Capture frames and generate timestamp
                        rgb_frame = getRGBFrame()
                        thermal_frame = getThermalFrame()
                        timestamp = time.strftime("%H%M%S")

                        fname = f"Thermal_{row}_{timestamp}.jpg"
                        if rgb_frame is not None:
                            cv.imwrite(f"{savePath}/RGB_{row}_{timestamp}.jpg", rgb_frame)
                        if thermal_frame is not None:
                            cv.imwrite(f"{savePath}/{fname}", thermal_frame)

                        # 2. Log metadata
                        curr_lat = drone.location.global_relative_frame.lat
                        curr_lon = drone.location.global_relative_frame.lon
                        curr_alt_fc = drone.location.global_relative_frame.alt
                        curr_alt_lidar = drone.rangefinder.distance

                        with open(metadata_file, mode='a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow([timestamp, fname, curr_lat, curr_lon, curr_alt_lidar, curr_alt_fc])
                        
                        # 3. Check for edge with forward LiDAR
                        fwd_dist = nav.get_forward_lidar_distance(drone)
                        if fwd_dist and fwd_dist > 10.0:
                            print("Edge detected. Terminating row.")
                            break
                        
                        # 4. Step forward/backward at 0.5m/s
                        vx = 0.5 if move_fwd else -0.5
                        nav.sendBodyVelocity(drone, vx, 0, 0)
                        time.sleep(mapping_step / 0.5) 
                        
                        # Stop for a steady image capture
                        nav.sendBodyVelocity(drone, 0, 0, 0) 
                        time.sleep(1.0) 

                    # 5. Move sideways to start the next pass
                    print("Shifting to next row...")
                    nav.sendBodyVelocity(drone, 0, 0.5, 0)
                    time.sleep(mapping_step / 0.5)
                    nav.sendBodyVelocity(drone, 0, 0, 0)

                    move_fwd = not move_fwd # Reverse direction for snake pattern

                print("Sweep complete. Returning Home.")
                currentMode = "LANDING"

            # --- LANDING STATE ---
            elif currentMode == "LANDING":
                print("Initiating RTL.")
                drone.mode = VehicleMode("RTL")
                break # Exit the while loop

            # --- Display the visual feed ---
            cv.imshow("Drone Feed", drawn_img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"An error occurred: {e}")
        if drone:
            drone.mode = VehicleMode("RTL")

    finally:
        if drone:
            nav.sendBodyVelocity(drone, 0, 0, 0)
            drone.close()
        closeRGBCamera()
        closeThermalCamera()
        cv.destroyAllWindows()
        
        # --- Data Transfer ---
        print(f"Mission finished. Syncing {savePath} to laptop...")
        # Note: 'rsync' requires an SSH server running on Windows laptop (e.g., OpenSSH)
        remote_dest = f"{laptopUser}@{laptopIP}:'C:/Users/lilly/Desktop/Thesis/MissionData'"
        os.system(f"rsync -avz {savePath}/ {remote_dest}")

        print("Resources released and data sync initiated.")

if __name__ == "__main__":
        main()
