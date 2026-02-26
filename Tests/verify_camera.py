import subprocess
import os

def test_capture():
    # Define the save path
    save_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'verify_arducam.jpg')
    
    # The command verified by your manual
    command = [
        "rpicam-still",
        "--nopreview",
        "-t", "2000", # 2 second delay to let auto-focus/exposure settle
        "-o", save_path
    ]
    
    print(f"Attempting to capture image to {save_path}...")
    
    try:
        # Run the command
        subprocess.run(command, check=True, capture_output=True, text=True)
        print("SUCCESS: Image saved to your Desktop as 'verify_arducam.jpg'.")
    except subprocess.CalledProcessError as e:
        print(f"FAILED: Camera error.\nError Output: {e.stderr}")
    except Exception as e:
        print(f"ERROR: {e}")

if __name__ == "__main__":
    test_capture()