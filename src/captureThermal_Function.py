import cv2 as cv
import os
import time

# --- Create a global camera object ---
cap = None

def initializeThermalCamera(device=0): # Changed default to 1 for your setup
    global cap
    try:
        # If an integer is passed, OpenCV uses it as the index
        print(f"Initializing Thermal Camera at: {device} ...")

        # Explicitly use the index or string with the V4L2 backend
        cap = cv.VideoCapture(device, cv.CAP_V4L2)

        if not cap.isOpened():
            print(f"Error: Could not open thermal device: {device}")
            cap = None
            return False

        # Essential for PureThermal 3 UYVY format
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('U','Y','V','Y'))
        time.sleep(1)
        print("Thermal camera initialized successfully.")
        return True
    except Exception as e:
        print(f"Error initializing thermal camera: {e}")
        if cap:
            cap.release()
        cap = None
        return False

def getThermalFrame():
    """
    Captures a single frame directly to memory as a NumPy array.
    
    Returns:
        numpy.ndarray: The captured image (frame).
                       Returns None if capture fails.
    """
    global cap
    
    if cap is None:
        print("Error: Thermal camera is not initialized. Call initializeThermalCamera() first.")
        return None
        
    try:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture thermal frame.")
            return None
            
        # Return the frame directly
        return frame
        
    except Exception as e:
        print(f"An error occurred while capturing the thermal frame: {e}")
        return None

def closeThermalCamera():
    """
    Stops and closes the thermal camera.
    """
    global cap
    if cap is not None:
        cap.release()
        cap = None
        print("Thermal camera resource released.")

# --- Main execution block (for testing) ---
if __name__ == "__main__":
    # This part only runs if this file is directly run
    print("--- Running thermal camera test ---")
    
    if initializeThermalCamera(): 
        
        frame = getThermalFrame()
        
        if frame is not None:
            print(f"Thermal frame captured successfully. Shape: {frame.shape}")
            
            
            # Save the image just to prove it worked
            home_dir = os.path.expanduser('~')
            save_path = os.path.join(home_dir, 'Desktop', 'test_thermal_capture.jpg')

            cv.imwrite(save_path, frame)
            print("Test thermal image saved as 'test_thermal_capture.jpg'")
        else:
            print("Failed to capture test thermal frame.")
            
        closeThermalCamera()
    else:
        print("Failed to initialize thermal camera.")