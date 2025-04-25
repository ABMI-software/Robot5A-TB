import cv2
import os
import numpy as np

# Site to do the Charuco https://calib.io/pages/camera-calibration-pattern-generator

# Paths for saving images
camera_1_path = "/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/extrinsic_images_charuco"


# Create directories if they do not exist
os.makedirs(camera_1_path, exist_ok=True)


def get_next_index(folder_path, prefix):
    """
    Get the next available index for images in the specified folder.
    Args:
        folder_path (str): The folder to check for existing images.
        prefix (str): The prefix of the image files to search for.
    Returns:
        int: The next available index for image files.
    """
    files = [f for f in os.listdir(folder_path) if f.startswith(prefix) and f.endswith(".jpg")]
    if not files:
        return 0
    indices = [int(f.split('_')[-1].split('.')[0]) for f in files]
    return max(indices) + 1

def main():
    # Detect next available indices
    camera_1_count = get_next_index(camera_1_path, "camera_1")


    print(f"Starting from index {camera_1_count} for Camera 1.")

    calibration_file="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml"
    mtx, dist = load_camera_parameters(calibration_file)

    dist = np.array(dist, dtype=np.float64)

    # Ensure it's a single-row or single-column vector
    if dist.shape == (1, 5):  # Convert to column vector
        dist = dist.T
    elif dist.shape == (5,):  # Reshape to (5,1)
        dist = dist.reshape(-1, 1)

    # Open cameras
    cap_1 = cv2.VideoCapture(4)  # First camera


    if not cap_1.isOpened():
        print("Error: Camera 1 not found.")
        return
    
    # Set type
    cap_1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))

    # Set resolution
    cap_1.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_1.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Set camera properties
    cap_1.set(cv2.CAP_PROP_BRIGHTNESS, 0.5) # Reduce brightness to avoid overexposure
    cap_1.set(cv2.CAP_PROP_CONTRAST, 0.7) # Higher contrast for clear edges
    cap_1.set(cv2.CAP_PROP_SATURATION, 0.6)
    cap_1.set(cv2.CAP_PROP_SHARPNESS, 4) # Increase sharpness
    cap_1.set(cv2.CAP_PROP_GAMMA, 0.5) # Adjust gamma to enhance details
    cap_1.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 5000) # Set a neutral white balance (~5000-6000K)
    cap_1.set(cv2.CAP_PROP_AUTO_WB, 0)  # Lock white balance

    cap_1.set(cv2.CAP_PROP_EXPOSURE, -5)  # Lower exposure for better contrast

    # Set frame rate
    cap_1.set(cv2.CAP_PROP_FPS, 30) # Higher FPS can cause motion blur, affecting accuracy

    # Check if the settings were applied
    width = cap_1.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap_1.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap_1.get(cv2.CAP_PROP_FPS)

    print(f"Resolution: {width}x{height}")
    print(f"FPS: {fps}")

    print("Press 'a' to save a picture from Camera 1.")

    print("Press 'q' to quit.")

    while True:
        # Capture frames from both cameras
        ret_1, frame_1 = cap_1.read()
        height, width = frame_1.shape[:2]  # Extract height and width
        camMatrixNew,roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width,height), 1, (width,height))
        imgUndist = cv2.undistort(frame_1, mtx, dist, None, camMatrixNew)



        if ret_1:
            cv2.imshow("Camera 1", imgUndist)
        else:
            print("Warning: Empty frame from Camera 1.")



        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('a'):  # Save image from Camera 1
            filename = os.path.join(camera_1_path, f"camera_1_{camera_1_count:03d}.jpg")
            cv2.imwrite(filename, imgUndist)
            print(f"Saved: {filename}")
            camera_1_count += 1



        elif key == ord('q'):  # Quit
            print("Quitting...")
            break

    # Release cameras and close windows
    cap_1.release()

    cv2.destroyAllWindows()


def load_camera_parameters(filename):
    """Load previously saved camera matrix and distortion coefficients."""
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)

    if not fs.isOpened():
        print(f"Error: Unable to open {filename}")
        return None, None

    mtx = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("distortion_coefficients").mat()  # Check YAML key name

    fs.release()

    # Debugging output
    print(f"Loaded Camera Matrix: {mtx}")
    print(f"Loaded Distortion Coefficients: {dist}")

    if mtx is None or dist is None:
        print("Error: Camera parameters not loaded correctly.")
        return None, None

    return mtx, dist

if __name__ == "__main__":
    main()
