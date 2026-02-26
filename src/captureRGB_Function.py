import cv2 as cv
import numpy as np
import subprocess
import time

# --- Global placeholder for mission script compatibility ---
cap = None 

def initializeRGBCamera():
    """
    Verifies Arducam OV5647 is accessible via libcamera.
    """
    print("Initializing Arducam OV5647 (stdout mode)...")
    try:
        # Per manual, verify rpicam-still is available
        subprocess.run(["rpicam-still", "--version"], capture_output=True, check=True)
        print("Arducam system ready.")
        return True
    except Exception as e:
        print(f"Error: Arducam command failed. Check connection or libcamera-apps: {e}")
        return False

def getRGBFrame():
    """
    Captures a frame using rpicam-still and streams it to memory via stdout.
    This provides the image array for the processImage function.
    """
    command = [
        "rpicam-still",
        "--nopreview",
        "--immediate",      # Captures frame without delay
        "--width", "640",   # Match your 640x480 processing
        "--height", "480",
        "-t", "1",          # Minimal 1ms delay
        "-e", "jpg",        # Encode as JPG bytes
        "-o", "-"           # "-" redirects output to stdout
    ]
    
    try:
        # Execute capture and retrieve bytes from stdout
        result = subprocess.run(command, capture_output=True, check=True)
        
        # Convert raw bytes into an OpenCV BGR array
        nparr = np.frombuffer(result.stdout, np.uint8)
        frame = cv.imdecode(nparr, cv.IMREAD_COLOR)
        
        return frame
    except Exception as e:
        print(f"Arducam frame capture failed: {e}")
        return None

def closeRGBCamera():
    """Consistency cleanup for the mission script."""
    print("RGB Camera resources released.")