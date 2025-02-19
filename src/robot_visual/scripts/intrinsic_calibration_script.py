import numpy as np
import cv2 as cv
import glob
import yaml

def calibrate_camera(images_pattern, chessboard_size, square_size, output_file):
    # Termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size  # Scale to square size

    # Arrays to store object points and image points
    objpoints = []  # 3D points
    imgpoints = []  # 2D points

    # Load calibration images
    images = glob.glob(images_pattern)

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Visualize corners
            cv.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv.imshow("Corners", img)
            cv.waitKey(500)

    cv.destroyAllWindows()

    # Perform camera calibration
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Save calibration parameters in OpenCV YAML format
    fs = cv.FileStorage(output_file, cv.FileStorage_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.write("reprojection_error", calculate_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist))
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
calibrate_camera(
    images_pattern="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_images/*.jpg",
    chessboard_size=(20, 13),# n-1
    
    square_size=0.02,
    output_file="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml",
)


