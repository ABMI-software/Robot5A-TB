import numpy as np
import cv2 as cv
import glob
import cv2


# Site to do the Charuco https://calib.io/pages/camera-calibration-pattern-generator

def calibrate_camera_charuco(images_pattern, squares_x, squares_y, square_size, marker_size, output_file):
    # Define Charuco board parameters
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    charuco_board = cv.aruco.CharucoBoard_create(squares_x, squares_y, square_size, marker_size, aruco_dict)
    
    # Storage for object points and image points
    objpoints = []  # 3D points in world coordinates
    imgpoints = []  # 2D points in image coordinates

    # Load calibration images
    images = glob.glob(images_pattern)

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict)

        if ids is not None and len(ids) > 0:
            # Refine marker corners and detect chessboard corners
            charuco_corners, charuco_ids, valid = cv.aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board)

            # Ensure charuco_corners is a valid result
            if charuco_corners is not None and isinstance(charuco_corners, np.ndarray) and len(charuco_corners) > 3:
                objpoints.append(charuco_board.chessboardCorners[charuco_ids.flatten()])
                imgpoints.append(charuco_corners)

                # Draw detected markers and Charuco corners
                cv.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)
                cv.imshow("Charuco Detection", img)
                cv.waitKey(500)

    cv.destroyAllWindows()

    # Perform camera calibration
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Save calibration parameters
    fs = cv.FileStorage(output_file, cv.FileStorage_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.release()

    print(f"Calibration complete. Parameters saved to {output_file}")


# Configuration for camera 1
calibrate_camera_charuco(
    images_pattern="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_images_charuco/*.jpg",
    squares_x=6,   # Number of squares along X-axis
    squares_y=8,   # Number of squares along Y-axis
    square_size=0.04,  # Size of squares in meters
    marker_size=0.03,  # Size of Aruco markers in meters
    output_file="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml",
)
