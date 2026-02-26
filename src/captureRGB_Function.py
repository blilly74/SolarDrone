import cv2 as cv
import time
from picamera2 import Picamera2
import os

# --- Create a global camera object ---
picam2 = Picamera2()

def initializeRGBCamera():
    try:
        print("Initializing Pi Camera...")
        config = picam2.create_still_configuration()
        picam2.configure(config)
        # Start without the preview window to avoid headless hangs
        picam2.start(preview=None) 
        time.sleep(2.0)
        print("Camera initialized successfully.")
    except Exception as e:
        print(f"Error initializing camera: {e}")

def getRGBFrame():
    """
    Captures a single frame directly to memory as a NumPy array.
    
    Returns:
        numpy.ndarray: The captured image in BGR format (for OpenCV).
                       Returns None if capture fails.
    """
    try:
        # capture_array() gets the image as a NumPy array (in RGB format)
        frameRGB = picam2.capture_array()
        
        # OpenCV uses BGR format, so we must convert it
        frameBGR = cv.cvtColor(frameRGB, cv.COLOR_RGB2BGR)
        
        return frameBGR
        
    except Exception as e:
        print(f"An error occurred while capturing the frame: {e}")
        return None

def closeRGBCamera():
    """
    Stops and closes the camera.
    Call this at the very end of main.py (in the 'finally' block).
    """
    try:
        picam2.stop()
        print("Camera stopped.")
    except Exception as e:
        print(f"Error stopping camera: {e}")

# --- Main execution block (for testing) ---
if __name__ == "__main__":
    # This part only runs if you execute this file directly
    print("--- Running camera test ---")
    
    initializeRGBCamera()
    
    frame = getRGBFrame()
    
    if frame is not None:
        print(f"Frame captured successfully. Shape: {frame.shape}")
        
        # Save the image just to prove it worked
        home_dir = os.path.expanduser('~')
        save_path = os.path.join(home_dir, 'Desktop', 'test_RGB_capture.jpg')
        
        cv.imwrite(save_path, frame)
        print(f"Test image saved as 'test_RGB_capture.jpg' on your Desktop.")
    else:
        print("Failed to capture test frame.")
        
    closeRGBCamera()