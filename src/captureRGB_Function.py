import cv2 as cv
import numpy as np
import subprocess
import time

# Global variable for consistency with your other modules
cap = None 

def initializeRGBCamera():
    """
    Verifies Arducam OV5647 is ready via the libcamera stack.
    """
    print("Initializing Arducam OV5647 via libcamera-apps...")
    try:
        # Check if rpicam-still (or libcamera-still) is available
        result = subprocess.run(["rpicam-still", "--version"], capture_output=True, text=True)
        if result.returncode == 0:
            print("Arducam system ready.")
            return True
        return False
    except Exception as e:
        print(f"Error: rpicam-still not found. Ensure libcamera-apps is installed: {e}")
        return False

def getRGBFrame():
    """
    Captures a frame using rpicam-still and streams it directly to stdout.
    This bypasses the SD card and provides a BGR array for OpenCV.
    """
    # Parameters based on OV5647 Common Specs [cite: 13]
    command = [
        "rpicam-still",
        "--nopreview",
        "--immediate",      # Minimal delay [cite: 59]
        "--width", "640",   # Match your processImage resolution [cite: 13]
        "--height", "480",
        "-t", "1",          # 1ms timeout
        "-e", "jpg",        # Encode as JPG bytes
        "-o", "-"           # Output to stdout (memory)
    ]
    
    try:
        # Run capture and grab the byte stream
        result = subprocess.run(command, capture_output=True, check=True)
        image_bytes = result.stdout
        
        # Decode the byte stream into an OpenCV image
        nparr = np.frombuffer(image_bytes, np.uint8)
        frame = cv.imdecode(nparr, cv.IMREAD_COLOR)
        
        return frame
    except Exception as e:
        print(f"Arducam capture failed: {e}")
        return None

def closeRGBCamera():
    """Consistency cleanup."""
    print("RGB Camera resources released.")