import numpy as np
import cv2 as cv
import glob

def load_camera_parameters(filename):
    """Load previously saved camera matrix and distortion coefficients."""
    fs = cv.FileStorage(filename, cv.FILE_STORAGE_READ)

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


def calibrate_camera_charuco_set_axis(images_pattern, squares_x, squares_y, square_size, marker_size, calibration_file):
    """Detect Charuco board and estimate pose with correct transformations."""
    
    # Load camera parameters
    mtx, dist = load_camera_parameters(calibration_file)

    dist = np.array(dist, dtype=np.float64)

    # Ensure it's a single-row or single-column vector
    if dist.shape == (1, 5):  # Convert to column vector
        dist = dist.T
    elif dist.shape == (5,):  # Reshape to (5,1)
        dist = dist.reshape(-1, 1)

    print(f"Fixed Distortion Coefficients Shape: {dist.shape}") 

    # Define Charuco board
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    charuco_board = cv.aruco.CharucoBoard_create(squares_x, squares_y, square_size, marker_size, aruco_dict)

    all_rvecs = []
    all_tvecs = []

    # Load images
    images = glob.glob(images_pattern)

    for fname in images:
        img = cv.imread(fname)
        height, width = img.shape[:2]  # Extract height and width
        camMatrixNew,roi = cv.getOptimalNewCameraMatrix(mtx, dist, (width,height), 1, (width,height))
        imgUndist = cv.undistort(img, mtx, dist, None, camMatrixNew)
        gray = cv.cvtColor(imgUndist, cv.COLOR_BGR2GRAY)

        # Detect Aruco markers
        corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict)

        if ids is not None and len(ids) > 0:
            # Refine marker detection
            valid, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(corners, ids, gray, charuco_board)

            if valid:
                # **Estimate pose of the Charuco board**
                success, rvecs, tvecs = cv.aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, charuco_board, mtx, dist, None, None
                )
                
                if success:
                    all_rvecs.append(rvecs)
                    all_tvecs.append(tvecs)
                    
                    # Draw the detected board and axis
                    cv.aruco.drawDetectedMarkers(img, corners, ids)
                    drawAxis(img, mtx, dist, rvecs, tvecs, marker_size * 1.5)
                    cv.imshow("Detected Charuco Board", img)
                    cv.waitKey(500)  # Show each image for 0.5s

            else:
                print(f"No valid Charuco corners in {fname}.")
        else:
            print(f"No Aruco markers detected in {fname}.")

    cv.destroyAllWindows()

    # Define a new origin transformation (example)
    new_origin_transform = np.array([
        [1, 0, 0, 0.36],  # Move along X (last column)
        [0, 1, 0, 0.03],  # Move along Y (last column)
        [0, 0, 1, 0],  # Move along Z (last column)
        [0, 0, 0, 1]
    ], dtype=np.float32)

    # Apply the transformation
    transformed_rvecs, transformed_tvecs = transform_camera_pose(all_rvecs, all_tvecs, new_origin_transform)

    # Compute average pose
    avg_rvec, avg_tvec, avg_transform = average_rotation_translation(transformed_rvecs, transformed_tvecs)

    # Draw the detected board and axis
    cv.aruco.drawDetectedMarkers(img, corners, ids)
    drawAxis(img, mtx, dist, avg_rvec, avg_tvec, marker_size * 1.5)
    cv.imshow("Detected Charuco Board", img)
    cv.waitKey(5000)  # Show each image for 5s

    print("\n=== Average Pose (World to Camera) ===")
    print(f"Rotation Vector (rvec):\n{avg_rvec}")
    print(f"Translation Vector (tvec):\n{avg_tvec}")
    print("4x4 Transformation Matrix:")
    for i in range(4):
        print(f"      - {avg_transform[i].tolist()}")

def drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.05):
    """Draws XYZ coordinate axes on the detected Charuco board."""
    axis_points = np.float32([
        [0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, -length]
    ]).reshape(-1, 3)
    
    img_points, _ = cv.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    img_points = np.int32(img_points).reshape(-1, 2)

    origin = tuple(img_points[0])
    cv.line(img, origin, tuple(img_points[1]), (0, 0, 255), 3)  # X-axis (Red)
    cv.line(img, origin, tuple(img_points[2]), (0, 255, 0), 3)  # Y-axis (Green)
    cv.line(img, origin, tuple(img_points[3]), (255, 0, 0), 3)  # Z-axis (Blue)

def average_rotation_translation(rvecs, tvecs):
    """Computes the average rotation and translation vectors."""
    R_mats = [cv.Rodrigues(rvec)[0] for rvec in rvecs]
    avg_tvec = np.mean(tvecs, axis=0)
    avg_R_mat = np.mean(R_mats, axis=0)
    avg_rvec, _ = cv.Rodrigues(avg_R_mat)

    transform_4x4 = np.eye(4, dtype=np.float32)
    transform_4x4[0:3, 0:3] = avg_R_mat
    transform_4x4[0:3, 3] = avg_tvec.flatten()

    return avg_rvec, avg_tvec, transform_4x4

def transform_camera_pose(rvecs, tvecs, new_origin_transform):
    """
    Transforms the camera pose (rvecs, tvecs) by applying a new origin transformation.
    
    Parameters:
    - rvecs (list of np.ndarray): List of rotation vectors.
    - tvecs (list of np.ndarray): List of translation vectors.
    - new_origin_transform (np.ndarray): 4x4 transformation matrix representing the new origin.

    Returns:
    - transformed_rvecs: List of transformed rotation vectors.
    - transformed_tvecs: List of transformed translation vectors.
    """
    transformed_rvecs = []
    transformed_tvecs = []

    R_new_origin = new_origin_transform[:3, :3]  # Rotation part
    T_new_origin = new_origin_transform[:3, 3]   # Translation part

    for rvec, tvec in zip(rvecs, tvecs):
        # Convert rotation vector to matrix
        R_cam, _ = cv.Rodrigues(rvec)

        # Transform the rotation
        R_transformed = R_new_origin @ R_cam  # New rotation = R_new_origin * R_cam
        rvec_transformed, _ = cv.Rodrigues(R_transformed)

        # Transform the translation
        tvec_transformed = R_new_origin @ tvec + T_new_origin.reshape(-1, 1)

        transformed_rvecs.append(rvec_transformed)
        transformed_tvecs.append(tvec_transformed)

    return transformed_rvecs, transformed_tvecs


# Run the function with your image set
calibrate_camera_charuco_set_axis(
    images_pattern="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/extrinsic_images_charuco/*.jpg",
    squares_x=8,
    squares_y=6,
    square_size=0.04,
    marker_size=0.03,
    calibration_file="/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml"
)
