import numpy as np
import cv2 as cv
import glob
import os
import time

# Prerequisite: Make sure that you have all required libraries and modules available.
# Ensure OpenCV is correctly installed and that you have the images for calibration.

# Calibration function that uses Charuco boards
def calibrate_camera_charuco(images_pattern, squares_x, squares_y, square_size, marker_size, output_file):
    # Define Charuco board parameters
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    charuco_board = cv.aruco.CharucoBoard_create(squares_x , squares_y , square_size, marker_size, aruco_dict)
    
    all_charuco_corners = []
    all_charuco_ids = []
    
    # Load calibration images
    images = glob.glob(images_pattern)

    # Capture valid frames
    valid_frames = []  # This will store valid frames for reprojection error calculation

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict)

        if ids is not None and len(ids) > 0:
            # Refine marker corners and detect Charuco corners
            valid, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board)

            if valid:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                
                # Store the valid frames for reprojection error calculation
                valid_frames.append({'image': img, 'charuco_corners': charuco_corners, 'charuco_ids': charuco_ids})

                # Draw detected markers and Charuco corners
                cv.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)
                cv.imshow("Charuco Detection", img)
                cv.waitKey(500)
            else:
                print(f"No valid Charuco corners detected in {fname}.")
        else:
            print(f"No Aruco markers detected in {fname}.")

    cv.destroyAllWindows()

    # Perform camera calibration
    ret, mtx, dist, rvecs, tvecs = cv.aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, charuco_board, gray.shape[::-1], None, None
    )

    # Calculate the reprojection error
    reprojection_error = calculate_reprojection_error_from_frames(valid_frames, mtx, dist, rvecs, tvecs, charuco_board, square_size)

    # Save calibration parameters
    fs = cv.FileStorage(output_file, cv.FileStorage_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.write("reprojection_error", reprojection_error)
    fs.release()

    print(f"Calibration complete. Parameters saved to {output_file}")
    print(f"Reprojection error in Pixels: {reprojection_error}")

def calculate_reprojection_error_from_frames(valid_frames, mtx, dist, rvecs, tvecs, charuco_board, square_size):
    """
    Calculate the reprojection error based on valid captured frames.
    Parameters:
    - valid_frames: List of valid frames with detected Charuco corners.
    - mtx: Camera matrix
    - dist: Distortion coefficients
    - rvecs: Rotation vectors
    - tvecs: Translation vectors
    - charuco_board: Charuco board object
    - square_size: Size of the squares on the board
    Returns:
    - Mean reprojection error
    """
    total_error = 0
    total_points = 0

    for frame in valid_frames:
        # Extract corners and ids for the frame
        image = frame['image']  # The original captured frame
        charuco_corners = frame['charuco_corners']
        charuco_ids = frame['charuco_ids']

        # Generate the 3D object points for the Charuco corners
        obj_points = []
        for corner_id in charuco_ids:
            # Calculate 3D coordinates based on corner ID and square size
            row = corner_id // charuco_board.getChessboardSize()[0]
            col = corner_id % charuco_board.getChessboardSize()[0]
            obj_points.append([col * square_size, row * square_size, 0.0])
        
        obj_points = np.float32(obj_points)

        # Reproject the 3D object points to 2D image points
        for rvec, tvec in zip(rvecs, tvecs):
            img_points, _ = cv.projectPoints(
                obj_points, rvec, tvec, mtx, dist
            )

            # Calculate the Euclidean distance between the detected and reprojected points
            error = cv.norm(charuco_corners, img_points, cv.NORM_L2) / len(img_points)
            total_error += error
            total_points += len(img_points)

    # Return the average error across all frames
    mean_error = total_error / total_points
    return mean_error


# Configuration for camera 1
calibrate_camera_charuco(
    images_pattern="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_images_charuco/*.jpg",
    squares_x=8,   # Number of squares along X-axis
    squares_y=6,   # Number of squares along Y-axis
    square_size=0.04,  # Size of squares in meters
    marker_size=0.03,  # Size of Aruco markers in meters
    output_file="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml",
)
