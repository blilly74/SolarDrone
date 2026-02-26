import cv2
import numpy as np
import os

def captureThermal(filename="thermal_image.jpg", save_dir=None, camera_index=0):
    """
    Captures a single frame from a USB camera (like a thermal camera) using OpenCV.

    Args:
        filename (str): The desired name for the output image file.
                        Defaults to "thermal_image.jpg".
        save_dir (str, optional): The directory to save the image.
                                  Defaults to the user's Desktop if None.
        camera_index (int): The index of the camera (e.g., 0, 1, 2). 
                            Defaults to 0.

    Returns:
        bool: True if image capture was successful, False otherwise.
    """
    
    # --- 1. Set up save directory ---
    if save_dir is None:
        try:
            save_dir = os.path.join(os.path.expanduser('~'), 'Desktop')
        except Exception as e:
            print(f"Error finding home directory: {e}")
            return False

    # Ensure the save directory exists
    try:
        os.makedirs(save_dir, exist_ok=True)
    except OSError as e:
        print(f"Error creating directory {save_dir}: {e}")
        return False

    full_path = os.path.join(save_dir, filename)
    
    # --- 2. Initialize Camera ---
    print(f"Initializing camera at index {camera_index}...")
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"Error: Could not open video device at index {camera_index}.")
        return False
        
    try:
        # --- 3. Set Camera Properties ---
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('U', 'Y', 'V', 'Y'))

        # --- 4. Capture Frame ---
        print("Attempting to capture a frame...")
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame from camera.")
            return False
            
        print("Frame Captured Successfully.")

        # --- 5. Save Image ---
        cv2.imwrite(full_path, frame)
        print(f"Image saved successfully to {full_path}")
        return True
        
    except Exception as e:
        print(f"An unexpected error occurred during capture: {e}")
        return False
        
    finally:
        # --- 6. Release Camera ---
        # Ensures the camera is released even if an error occurs
        cap.release()
        print("Camera resource released.")

# --- Main execution block ---
if __name__ == "__main__":
    # This part only runs when you execute the script directly
    
    print("--- Running thermal capture function ---")
    
    # Call the function with default settings
    success = captureThermal()
    
    if success:
        print("Capture complete.")
    else:
        print("Capture failed.")