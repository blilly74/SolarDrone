import asyncio
import time
import os
import csv
import cv2 as cv
from mavsdk import System
from mavsdk.telemetry import FlightMode

# Keeping your existing custom logic imports
from src.captureRGB_Function import initializeRGBCamera, getRGBFrame, closeRGBCamera
from src.captureThermal_Function import initializeThermalCamera, getThermalFrame, closeThermalCamera
from src.processImage_Function import processImage
from src.findOriginFromContour_Function import findOriginFromContour

# --- Configuration ---
# MAVSDK uses a URL-style connection string
# UPDATED: Use ttyAMA0 for GPIO/Telem2 or ttyACM0 for USB
connectionString = "serial:///dev/ttyACM0:115200" 
savePath = "/home/blilly/Desktop/Mission_Data/AltitudeAndPanelDetection_Test"

startAltitude = 2
altitudeIncrease = 1
maxAttempts = 3
qualityThreshold = 0.85
attemptWindowSec = 10.0
ROTATE_90_CW = True

def rotate(img):
    if img is None:
        return None
    return cv.rotate(img, cv.ROTATE_90_CLOCKWISE) if ROTATE_90_CW else img

async def main():
    os.makedirs(savePath, exist_ok=True)

    # --- Hardware Initialization with Stabilization Delays ---
    print("Initializing RGB Camera...")
    initializeRGBCamera()
    await asyncio.sleep(2)  # Give the Pi 2 seconds to stabilize the USB bus

    print("Initializing Thermal Camera...")
    # UPDATED: Changed to video-index1 based on your v4l-id logs
    initializeThermalCamera(1)
    await asyncio.sleep(2)

    # Initialize MAVSDK System
    drone = System()
    mission_log = []

    try:
        print(f"Connecting to CubeOrangePlus at {connectionString}...")
        await drone.connect(system_address=connectionString)

        # Wait for drone to be ready
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Connected to Flight Controller!")
                break

        # Wait for Global Position Estimate (Required for Altitude/RTL)
        print("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Global position estimate OK")
                break

        print("--- Starting Detection Test ---")
        
        # Arming and Takeoff
        print("Arming...")
        await drone.action.arm()
        
        print(f"Taking off to {startAltitude}m...")
        await drone.action.set_takeoff_altitude(startAltitude)
        await drone.action.takeoff()
        
        # Wait to reach takeoff altitude
        await asyncio.sleep(5) 

        currentAlt = startAltitude

        for attempt in range(1, maxAttempts + 1):
            print(f"Attempt {attempt}/{maxAttempts} at altitude {currentAlt}m")
            attemptStart = time.time()

            bestScore = None
            bestImg = None

            while (time.time() - attemptStart) < attemptWindowSec:
                frame = getRGBFrame()
                if frame is None:
                    await asyncio.sleep(0.02)
                    continue

                try:
                    drawn_img, contour, score = processImage(frame)
                except Exception as e:
                    print(f"WARN: processImage error: {e}")
                    drawn_img, contour, score = None, None, None

                base = drawn_img if drawn_img is not None else frame
                final_img = rotate(base)

                if score is not None and (bestScore is None or score > bestScore):
                    bestScore = float(score)
                    bestImg = final_img.copy() if final_img is not None else None

                # Success logic
                if score is not None and score >= qualityThreshold:
                    print(f"SUCCESS: Panel found! Score: {score:.4f}")
                    cv.imwrite(os.path.join(savePath, f"RGB_success_{attempt}.jpg"), final_img)
                    
                    # Log and break out to RTL
                    mission_log.append({
                        "timestamp": time.strftime("%H:%M:%S"), 
                        "altitude": currentAlt,
                        "result": "SUCCESS", 
                        "score": score
                    })
                    return # Exits main() to finally block

                await asyncio.sleep(0.05) # Yield to event loop

            # Handle failed attempt
            if bestImg is not None:
                cv.imwrite(os.path.join(savePath, f"RGB_best_fail_{attempt}.jpg"), bestImg)

            if attempt < maxAttempts:
                currentAlt += altitudeIncrease
                print(f"Increasing altitude to {currentAlt}m")
                # Using simple takeoff logic for altitude steps
                await drone.action.goto_location(0, 0, currentAlt, 0) 

    except Exception as e:
        print(f"Error during test execution: {e}")

    finally:
        # RTL and Cleanup
        print("Initiating RTL...")
        try:
            await drone.action.return_to_launch()
        except Exception as e:
            print(f"RTL failed: {e}")

        # Save CSV Data
        if mission_log:
            csv_file = os.path.join(savePath, "mission_log.csv")
            with open(csv_file, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=mission_log[0].keys())
                writer.writeheader()
                writer.writerows(mission_log)

        closeRGBCamera()
        closeThermalCamera()
        print(f"Mission data saved to {savePath}.")

if __name__ == "__main__":
    asyncio.run(main())