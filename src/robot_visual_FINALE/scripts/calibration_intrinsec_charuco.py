import cv2
import numpy as np
import glob
import os
import yaml
from config_charuco import get_config
from remove_distortion import correct_distortion_for_camera
from rapport_eval_intrinsec import evaluate_calibration, test_distortion_correction, show_calibration_visual

def create_charuco_board(cfg):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    try:
        return cv2.aruco.CharucoBoard((cfg['squares_x'], cfg['squares_y']),
                                       cfg['square_size'], cfg['marker_size'], aruco_dict)
    except AttributeError:
        return cv2.aruco.CharucoBoard_create(cfg['squares_x'], cfg['squares_y'],
                                             cfg['square_size'], cfg['marker_size'], aruco_dict)

def detect_and_refine_corners(img, board, detector_params):
    """Détecte et affine les coins Charuco"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    corners, ids, _ = cv2.aruco.detectMarkers(
        gray, board.getDictionary(), parameters=detector_params
    )
    
    if ids is None or len(ids) < 20:
        return None, None, 0
    
    ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board
    )
    
    if not ret or charuco_corners is None or len(charuco_corners) < 20:
        return None, None, 0
    
    # Raffinement sous-pixel
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    cv2.cornerSubPix(gray, charuco_corners, (5,5), (-1, -1), criteria)
    
    return charuco_corners, charuco_ids, len(charuco_corners)

def calibrate_one_camera(path_pattern, output_file, cfg, cam_name):
    """Calibre une caméra avec Charuco"""
    print(f"\n=== DÉBUT CALIBRATION {cam_name.upper()} ===")
    
    board = create_charuco_board(cfg)
    
    # Paramètres de détection
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.adaptiveThreshWinSizeMin = 3
    detector_params.adaptiveThreshWinSizeMax = 23
    detector_params.minMarkerPerimeterRate = 0.03
    detector_params.maxMarkerPerimeterRate = 4.0
    
    img_paths = glob.glob(path_pattern)
    if not img_paths:
        print(f"✗ Aucune image trouvée pour {cam_name}")
        return None, None, 0
    
    all_corners = []
    all_ids = []
    valid_count = 0
    
    print(f" Analyse de {len(img_paths)} images...")
    
    for i, fp in enumerate(img_paths):
        img = cv2.imread(fp)
        if img is None:
            continue
        
        # Filtrage flou
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.Laplacian(gray, cv2.CV_64F).var()
        if blur < 50:
            print(f"  Image {i+1}: trop floue, ignorée")
            continue
        
        corners, ids, corner_count = detect_and_refine_corners(img, board, detector_params)
        if corners is not None:
            all_corners.append(corners)
            all_ids.append(ids)
            valid_count += 1
            print(f"  Image {i+1}: {corner_count} coins détectés ✓")
        else:
            print(f"  Image {i+1}: pas assez de coins détectés")
    
    if valid_count < 10:
        print(f"✗ Pas assez d'images valides pour {cam_name} ({valid_count} < 10)")
        return None, None, 0
    
    
    
    # Calibration
    h, w = cv2.imread(img_paths[0]).shape[:2]
    img_size = (w, h)
    
    f_est = max(img_size) * 1.2
    camera_matrix_init = np.array([[f_est, 0, w/2], [0, f_est, h/2], [0, 0, 1]], dtype=np.float64)
    
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | 
             cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5)
    
    print(" Calcul des paramètres de calibration...")
    ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, img_size, camera_matrix_init, None, flags=flags
    )
    
    # Calcul erreur de reprojection
    tot_err = 0.0
    tot_pts = 0
    for corners, ids in zip(all_corners, all_ids):
        obj_pts = []
        img_pts = []
        for ci, cid in enumerate(ids.flatten()):
            row = cid // (cfg['squares_x'] - 1)
            col = cid % (cfg['squares_x'] - 1)
            obj_pts.append([col * cfg['square_size'], row * cfg['square_size'], 0.0])
            img_pts.append(corners[ci][0])
        
        obj_pts = np.array(obj_pts, dtype=np.float32)
        img_pts = np.array(img_pts, dtype=np.float32)
        
        if len(obj_pts) >= 4:
            succ, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, mtx, dist)
            if succ:
                proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, mtx, dist)
                err = cv2.norm(img_pts, proj.reshape(-1, 2), cv2.NORM_L2)
                tot_err += err
                tot_pts += len(obj_pts)
    
    avg_error = (tot_err / tot_pts) if tot_pts > 0 else float('inf')
    
    
    
    # Sauvegarde
    data = {
        'camera_matrix': mtx.tolist(),
        'distortion_coefficients': dist.tolist(),
        'reprojection_error_mean': float(avg_error),
        'image_size': [w, h],
        'valid_images_count': valid_count,
        'focal_length': {'fx': float(mtx[0,0]), 'fy': float(mtx[1,1])},
        'principal_point': {'cx': float(mtx[0,2]), 'cy': float(mtx[1,2])},
        'calibration_date': str(np.datetime64('now'))
    }
    
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    
    print(f" Calibration {cam_name} terminée: {avg_error:.4f} px ({valid_count} images)")
    return mtx, dist, avg_error

def main():
    cfg = get_config()
    cam1_dir = cfg['camera_1_path']
    cam2_dir = cfg['camera_2_path']
    
    # Calibration des deux caméras
    mtx1, dist1, err1 = calibrate_one_camera(
        os.path.join(cam1_dir, "*.jpg"), cfg['camera_1_calib'], cfg, "Caméra 1"
    )
    mtx2, dist2, err2 = calibrate_one_camera(
        os.path.join(cam2_dir, "*.jpg"), cfg['camera_2_calib'], cfg, "Caméra 2"
    )
    
    # Correction de distorsion et évaluations
    if mtx1 is not None:
        correct_distortion_for_camera(cam1_dir, cfg['camera_1_calib'], "Caméra 1")
        show_calibration_visual(cam1_dir, cfg['camera_1_calib'], "Caméra 1")
        evaluate_calibration(mtx1, dist1, err1, cfg['camera_1_calib'])
        test_distortion_correction(os.path.join(cam1_dir, "*.jpg"), mtx1, dist1, "Caméra 1")
    
    if mtx2 is not None:
        correct_distortion_for_camera(cam2_dir, cfg['camera_2_calib'], "Caméra 2")
        show_calibration_visual(cam2_dir, cfg['camera_2_calib'], "Caméra 2")
        evaluate_calibration(mtx2, dist2, err2, cfg['camera_2_calib'])
        test_distortion_correction(os.path.join(cam2_dir, "*.jpg"), mtx2, dist2, "Caméra 2")
    
    if mtx1 is not None and mtx2 is not None:
        print("\n Calibration complète des deux caméras réussie")
        return 0
    else:
        print("\n Calibration partielle ou échouée")
        return 1

if __name__ == "__main__":
    exit(main())