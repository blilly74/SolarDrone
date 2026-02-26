import time
import cv2 as cv
import numpy as np
from dronekit import VehicleMode
from src import navigationController_Function as nav
from src.navigationController_Function import VisualServoController, log_to_gcs
from src.captureRGB_Function import initializeRGBCamera, getRGBFrame, closeRGBCamera
from src.processImage_Function import processImage
from src.findOriginFromContour_Function import findOriginFromContour

# --- Configuration (Aligned with main.py) ---
connectionString = 'udp:127.0.0.1:14551'
startAltitude = 10
qualityThreshold = 0.8
centeringTolerance = 10
RCSwitchChannel = 8
activationPWM = 1800

def main():
    initializeRGBCamera()
    drone = None
    # Initialize PID with gains from your main.py
    pidController = VisualServoController(kp=0.01, ki=0.001, kd=0.001)

    try:
        drone = nav.connectDrone(connectionString, wait_ready=True)
        log_to_gcs(drone, "Waiting for activation switch...")
        nav.waitForActivation(RCSwitchChannel, drone, activationPWM)

        # 1. Takeoff to starting altitude
        nav.armAndTakeoff(drone, startAltitude)
        currentMode = "SEARCHING"
        searchStartTime = time.time()

        while True:
            frame = getRGBFrame()
            if frame is None: continue

            drawn_img, contour, score = processImage(frame)

            if currentMode == "SEARCHING":
                if score is not None and score > qualityThreshold:
                    log_to_gcs(drone, f"Target Found (Score: {score:.2f}). Switching to TRACKING.")
                    currentMode = "TRACKING"
                elif (time.time() - searchStartTime) > 30.0:
                    log_to_gcs(drone, "Search timeout. Returning to Launch.")
                    currentMode = "LANDING"

            elif currentMode == "TRACKING":
                if score is None or score < qualityThreshold:
                    log_to_gcs(drone, "Target Lost. Reverting to SEARCHING.")
                    nav.sendBodyVelocity(drone, 0, 0, 0)
                    currentMode = "SEARCHING"
                    searchStartTime = time.time()
                    continue

                originCoords = findOriginFromContour(contour)
                if originCoords is None: continue 

                # Check if image is centered based on your tolerance
                errorX = abs(originCoords[0] - pidController.pidX.setpoint)
                errorY = abs(originCoords[1] - pidController.pidY.setpoint)

                if errorX < centeringTolerance and errorY < centeringTolerance:
                    log_to_gcs(drone, ">>> SUCCESS: ORIGIN CENTERED <<<")
                    nav.sendBodyVelocity(drone, 0, 0, 0)
                    time.sleep(2) 
                    currentMode = "LANDING"
                else:
                    # Update velocities using the servo controller
                    vx, vy = pidController.update(originCoords)
                    nav.sendBodyVelocity(drone, vx, vy, 0)

            elif currentMode == "LANDING":
                drone.mode = VehicleMode("RTL")
                break

            cv.imshow("Origin Alignment Test", drawn_img)
            if cv.waitKey(1) & 0xFF == ord('q'): break

    except Exception as e:
        print(f"Error: {e}")
        if drone: drone.mode = VehicleMode("RTL")
    finally:
        if drone: drone.close()
        closeRGBCamera()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()