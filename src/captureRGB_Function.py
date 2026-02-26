import cv2 as cv
import numpy as np
import subprocess
import time

# Create a placeholder for consistency, but do NOT call Picamera2()
cap = None 

def initializeRGBCamera():
    """Verifies the Arducam is ready using the libcamera-apps stack."""
    print("Initializing Arducam OV5647 via libcamera-apps...")
    try:
        # Check if rpicam-still is available as per Arducam manual
        subprocess.run(["rpicam-still", "--version"], capture_output=True, check=True)
        print("Arducam system ready.")
        return True
    except Exception as e:
        print(f"Error: Arducam command not found. Ensure libcamera-apps is installed: {e}")
        return False

def getRGBFrame():
    """Captures a frame to memory via stdout. No file is saved to SD."""
    # Settings optimized for OV5647 5MP sensor
    command = [
        "rpicam-still",
        "--nopreview",
        "--immediate",      # Captures immediately
        "--width", "640",   # Match processing resolution
        "--height", "480",
        "-t", "1",          # Minimal delay
        "-e", "jpg",        # JPG encoding
        "-o", "-"           # Output to memory (stdout)
    ]
    try:
        result = subprocess.run(command, capture_output=True, check=True)
        # Decode the byte stream directly into an OpenCV array
        nparr = np.frombuffer(result.stdout, np.uint8)
        frame = cv.imdecode(nparr, cv.IMREAD_COLOR)
        return frame
    except Exception as e:
        print(f"Arducam capture failed: {e}")
        return None

def closeRGBCamera():
    print("RGB Camera resources released.")