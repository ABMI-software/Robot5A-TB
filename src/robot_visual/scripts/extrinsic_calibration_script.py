import cv2
import numpy as np

# Paths
image_path = "/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/extrinsic_images/camera_1_004.jpg"
calibration_path = "/home/chipmunk-151/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml"

# Marker info
marker_size = 0.03  # 3 cm
aruco_dict_type = cv2.aruco.DICT_6X6_250

# Positions of markers in world frame
marker_positions_world = {
    0: (0.0475 - 0, -0.0475 - 0, 0.0),# 11 repere axe 
    1: (0.3475 - 0, -0.0475 - 0, 0.0),
    2: (0.0475 - 0, -0.2475 - 0, 0.0),
    3: (0.3475 - 0, -0.2475 - 0, 0.0),
    4: (0.1975 - 0, -0.1475 - 0, 0.0)
}

def load_camera_calibration(calibration_file):
    fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Unable to open calibration file: {calibration_file}")

    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    fs.release()
    return camera_matrix, dist_coeffs

def get_marker_corners_3d(marker_center_world, size):
    half_size = size / 2.0
    # Corners in marker frame: top-left, top-right, bottom-right, bottom-left
    corners_local = np.array([
        [-half_size,  half_size, 0],
        [ half_size,  half_size, 0],
        [ half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)
    return corners_local + np.array(marker_center_world, dtype=np.float32)

def create_board(marker_positions, dictionary, marker_size):
    board_ids = list(marker_positions.keys())
    board_obj_points = []
    for mid in board_ids:
        marker_center = marker_positions[mid]
        corners_3d = get_marker_corners_3d(marker_center, marker_size).astype(np.float32)
        board_obj_points.append(corners_3d)

    # Convert ids to a NumPy array of type int32
    board_ids = np.array(board_ids, dtype=np.int32)

    # Create the board
    board = cv2.aruco.Board_create(board_obj_points, dictionary, board_ids)
    return board

def main():
    # Load camera intrinsics
    camera_matrix, dist_coeffs = load_camera_calibration(calibration_path)

    # Prepare ArUco detection
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    aruco_params = cv2.aruco.DetectorParameters_create()

    # Create the board
    board = create_board(marker_positions_world, dictionary, marker_size)

    # Load image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error loading image: {image_path}")
        return

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=aruco_params)

    if ids is None or len(ids) == 0:
        print("No markers detected.")
        return

    # Estimate the board pose
    retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, None, None)
    if retval == 0:
        print("Could not estimate the board pose.")
        return

    # Convert rotation vector to matrix
    R_mat, _ = cv2.Rodrigues(rvec)

    # Extrinsic matrix [R|t]
    extrinsic_matrix = np.hstack((R_mat, tvec))
    print("Rotation Vector (rvec):")
    print(rvec)
    print("Translation Vector (tvec):")
    print(tvec)
    print("Extrinsic Matrix [R|t]:")
    print(extrinsic_matrix)

    # Full 4x4 transformation (World to Camera)
    transform_4x4 = np.eye(4, dtype=np.float32)
    transform_4x4[0:3, 0:3] = R_mat
    transform_4x4[0:3, 3] = tvec.flatten()
    print("4x4 Transformation matrix (World to Camera):")
    print(transform_4x4)

    # Visualization (optional)
    cv2.aruco.drawDetectedMarkers(img, corners, ids)
    if retval > 0:
        cv2.aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, marker_size * 1.5)
    cv2.imshow("Detected Markers & Board Pose", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
