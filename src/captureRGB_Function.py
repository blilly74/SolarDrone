import cv2 as cv
import numpy as np
import subprocess
import time

# --- Global object for consistency ---
cap = None 

def initializeRGBCamera():
    """
    Arducam initialization. Since we use rpicam-still via subprocess,
    we just verify the command is available.
    """
    print("Initializing Arducam via rpicam-still stdout...")
    try:
        # Quick check if rpicam-still exists
        subprocess.run(["rpicam-still", "--version"], capture_output=True, check=True)
        print("Arducam system ready.")
        return True
    except Exception as e:
        print(f"Error: rpicam-still not found or Arducam disconnected: {e}")
        return False

def getRGBFrame():
    """
    Captures a frame using rpicam-still and streams it directly into a 
    NumPy array via stdout. No file is saved to the SD card.
    """
    command = [
        "rpicam-still",
        "--nopreview",
        "--immediate",      # Capture as fast as possible
        "--width", "640",   # Match your processImage resolution
        "--height", "480",
        "-t", "1",          # Minimal timeout
        "-e", "jpg",        # Output as encoded jpg bytes
        "-o", "-"           # "-" tells rpicam to output to stdout
    ]
    
    try:
        # Run command and capture the byte stream
        result = subprocess.run(command, capture_output=True, check=True)
        image_bytes = result.stdout
        
        # Convert bytes to a NumPy array
        nparr = np.frombuffer(image_bytes, np.uint8)
        
        # Decode the array into an OpenCV BGR image
        frame = cv.imdecode(nparr, cv.IMREAD_COLOR)
        
        return frame
    except Exception as e:
        print(f"Capture error: {e}")
        return None

def closeRGBCamera():
    """Cleanup for code structure consistency."""
    print("RGB Camera resources released.")