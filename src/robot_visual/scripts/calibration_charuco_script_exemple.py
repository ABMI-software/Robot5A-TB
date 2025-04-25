import numpy as np
import glob
import cv2
import os


# Site to do the Charuco https://calib.io/pages/camera-calibration-pattern-generator

# ------------------------------
# ENTER YOUR REQUIREMENTS HERE:
ARUCO_DICT = cv2.aruco.DICT_6X6_250
SQUARES_VERTICALLY = 6
SQUARES_HORIZONTALLY = 8
SQUARE_LENGTH = 0.04
MARKER_LENGTH = 0.03
OUTPUT_FILE="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml",
PATH_TO_YOUR_IMAGES = '/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_images_charuco'
# ------------------------------

def calibrate_and_save_parameters():
    # Define the aruco dictionary and charuco board
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
    params = cv2.aruco.DetectorParameters()

    # Load images
    image_files = [os.path.join(PATH_TO_YOUR_IMAGES, f) for f in os.listdir(PATH_TO_YOUR_IMAGES) if f.endswith(".jpg")]
    image_files.sort()

    if not image_files:
        print("Error: No images found in the specified path.")
        return

    # Get image size from the first image
    first_image = cv2.imread(image_files[0])
    if first_image is None:
        print("Error: Failed to load the first image.")
        return
    image_size = first_image.shape[:2]

    all_charuco_corners = []
    all_charuco_ids = []

    for image_file in image_files:
        image = cv2.imread(image_file)
        if image is None:
            print(f"Warning: Failed to load image {image_file}")
            continue
        
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)
        
        if marker_ids is not None and len(marker_ids) > 0:
            charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, board)
            if charuco_retval:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)

    if not all_charuco_corners:
        print("Error: No valid Charuco corners detected for calibration.")
        return

    # Calibrate camera
    retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, image_size, None, None
    )

    # Save calibration parameters
    fs = cv2.FileStorage(OUTPUT_FILE, cv2.FileStorage_WRITE)
    fs.write("camera_matrix", camera_matrix)
    fs.write("distortion_coefficients", dist_coeffs)
    fs.release()

    print(f"Calibration complete. Parameters saved to {OUTPUT_FILE}")



calibrate_and_save_parameters()