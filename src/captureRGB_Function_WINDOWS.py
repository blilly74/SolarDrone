import cv2 as cv
import numpy as np
import os

# --- A dummy camera object ---
testImagePath = "TestImages/TestImg1.jpg"
frame = None

def initializeCamera():
    """
    WINDOWS STUB: Loads a single test image from disk.
    """
    global frame
    
    baseDir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    fullPath = os.path.join(baseDir, testImagePath)

    print(f"--- WINDOWS STUB ---")
    print(f"Initializing dummy camera. Loading {fullPath}...")
    
    frame = cv.imread(fullPath)
    
    if frame is None:
        print(f"Error: Could not load test image from {fullPath}")
        print("Please check the path in captureRGBFunction_WINDOWS.py")
    else:
        print("Dummy camera 'initialized'.")

def getRGBFrame():
    """
    WINDOWS STUB: Returns the pre-loaded test image.
    """
    global frame
    
    if frame is None:
        print("Error: Dummy camera not initialized or test image failed to load.")
        # Return a blank black image to avoid crashing main.py
        return np.zeros((480, 640, 3), dtype=np.uint8) 
        
    print("--- WINDOWS STUB: Returning test image ---")
    return frame.copy() # Return a copy so it can be re-used

def closeCamera():
    """
    WINDOWS STUB: Does nothing.
    """
    print("--- WINDOWS STUB: 'Closing' dummy camera. ---")
    pass # Nothing to do

# --- Main execution block (for testing) ---
if __name__ == "__main__":
    # This part only runs if you execute this file directly
    import cv2 as cv
    
    print("--- Running camera test ---")
    
    initializeCamera()
    
    frame = getRGBFrame()
    
    if frame is not None:
        print(f"Frame captured successfully. Shape: {frame.shape}")
        print("Displaying test image. Press 'q' to close.")
        cv.imshow("Test Image (from dummy camera)", frame)
        cv.waitKey(0)
        cv.destroyAllWindows()
        # Save the image just to prove it worked
    else:
        print("Failed to capture test frame.")
        
    closeCamera()