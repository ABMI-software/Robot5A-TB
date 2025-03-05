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
    fs.write("reprojection_error", calculate_reprojection_error(all_charuco_corners, all_charuco_ids, mtx, dist, rvecs, tvecs, charuco_board, square_size))
    fs.release()


    print(f"Calibration complete. Parameters saved to {output_file}")



def calculate_reprojection_error(all_charuco_corners, all_charuco_ids, mtx, dist, rvecs, tvecs, charuco_board, square_size):
    """
    Calculate the reprojection error for camera calibration
    Parameters:
    - all_charuco_corners: List of detected 2D corner points in images
    - all_charuco_ids: List of corner IDs
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
    for i in range(len(all_charuco_corners)):
        # Manually create 3D object points
        obj_points = []
        for corner_id in all_charuco_ids[i][:, 0]:
            # Calculate 3D coordinates based on corner ID and square size
            row = corner_id // charuco_board.getChessboardSize()[0]
            col = corner_id % charuco_board.getChessboardSize()[0]
            obj_points.append([col * square_size, row * square_size, 0.0])
        obj_points = np.float32(obj_points)
        
        # Reproject the 3D object points to 2D image points
        img_points, _ = cv2.projectPoints(
            obj_points,
            rvecs[i],
            tvecs[i],
            mtx,
            dist
        )
        
        # Calculate the Euclidean distance between detected and reprojected points
        error = cv2.norm(
            all_charuco_corners[i],
            img_points,
            cv2.NORM_L2
        ) / len(img_points)
        
        total_error += error
    
    # Return mean error divided by number of images
    return total_error / len(all_charuco_corners)



# Configuration for camera 1
calibrate_camera_charuco(
    images_pattern="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_images_charuco/*.jpg",
    squares_x=8,   # Number of squares along X-axis
    squares_y=6,   # Number of squares along Y-axis
    square_size=0.04,  # Size of squares in meters
    marker_size=0.03,  # Size of Aruco markers in meters
    output_file="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml",
)