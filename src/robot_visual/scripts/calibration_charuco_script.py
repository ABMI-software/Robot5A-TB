import numpy as np
import cv2 as cv
import glob
import cv2


# Site to do the Charuco https://calib.io/pages/camera-calibration-pattern-generator

def calibrate_camera_charuco(images_pattern, squares_x, squares_y, square_size, marker_size, output_file):
    # Define Charuco board parameters
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    charuco_board = cv.aruco.CharucoBoard_create(squares_x , squares_y , square_size, marker_size, aruco_dict)

    
    all_charuco_corners = []
    all_charuco_ids = []

    # Load calibration images
    images = glob.glob(images_pattern)

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict)

        if ids is not None and len(ids) > 0:
            # Refine marker corners and detect chessboard corners
            valid, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board)

            # Ensure charuco_corners is a valid result
            if valid:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)

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
    ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, charuco_board, gray.shape[::-1], None, None)

    # Save calibration parameters
    fs = cv.FileStorage(output_file, cv.FileStorage_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.release()

    print(f"Calibration complete. Parameters saved to {output_file}")

def calculate_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
        total_error += error
    return total_error / len(objpoints)

# Configuration for camera 1
calibrate_camera_charuco(
    images_pattern="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_images_charuco/*.jpg",
    squares_x=8,   # Number of squares along X-axis
    squares_y=6,   # Number of squares along Y-axis
    square_size=0.04,  # Size of squares in meters
    marker_size=0.03,  # Size of Aruco markers in meters
    output_file="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml",
)