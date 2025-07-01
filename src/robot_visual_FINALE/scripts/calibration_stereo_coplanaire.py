#!/usr/bin/env python3
"""
Calibration stéréo avec plateau Charuco - Version corrigée
Détection des coins, visualisation des correspondances et évaluation
Corrections pour géométrie stéréo non-rectifiée et ROI valides
"""

import cv2
import numpy as np
import glob
import os
import yaml
from config_charuco import get_config

def load_calibration(calib_file):
    """Charge les paramètres de calibration depuis un fichier YAML"""
    try:
        with open(calib_file, 'r') as file:
            data = yaml.safe_load(file)
        camera_matrix = np.array(data['camera_matrix'])
        dist_coeffs = np.array(data['distortion_coefficients'])
        img_size = tuple(data['image_size'])
        return camera_matrix, dist_coeffs, img_size
    except Exception as e:
        print(f"Erreur lors du chargement de {calib_file}: {e}")
        return None, None, None

def create_charuco_board(cfg):
    """Crée le plateau Charuco"""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    try:
        return cv2.aruco.CharucoBoard((cfg['squares_x'], cfg['squares_y']),
                                       cfg['square_size'], cfg['marker_size'], aruco_dict)
    except AttributeError:
        return cv2.aruco.CharucoBoard_create(cfg['squares_x'], cfg['squares_y'],
                                             cfg['square_size'], cfg['marker_size'], aruco_dict)

def detect_charuco_corners(img, board, detector_params):
    """Détecte les coins Charuco dans une image avec amélioration de contraste"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    
    # Amélioration du contraste pour une meilleure détection
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)
    
    # Détection des marqueurs ArUco
    corners, ids, _ = cv2.aruco.detectMarkers(gray, board.getDictionary(), parameters=detector_params)
    
    if ids is None or len(ids) < 4:
        return None, None, None, None
    
    # Interpolation des coins Charuco
    ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board
    )
    
    if not ret or charuco_corners is None or len(charuco_corners) < 6:  # Minimum 6 pour robustesse
        return None, None, None, None
    
    # Raffinement sous-pixel amélioré
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)
    cv2.cornerSubPix(gray, charuco_corners, (7, 7), (-1, -1), criteria)
    
    return charuco_corners, charuco_ids, corners, ids

def get_charuco_corner_3d_position(corner_id, squares_x, squares_y, square_size):
    """
    Calcule la position 3D d'un coin Charuco selon la convention OpenCV
    """
    total_corners_x = squares_x - 1
    total_corners_y = squares_y - 1
    
    if corner_id >= total_corners_x * total_corners_y:
        return None
    
    # Calculer row et col à partir de l'ID
    row = corner_id // total_corners_x
    col = corner_id % total_corners_x
    
    # Position 3D (origine en haut-gauche du premier carré)
    x = (col + 1) * square_size
    y = (row + 1) * square_size
    z = 0.0
    
    return np.array([x, y, z], dtype=np.float32)

def filter_common_corners_robust(corners1, ids1, corners2, ids2, cfg, min_points=8):
    """
    Filtre les coins communs avec validation géométrique robuste
    """
    if corners1 is None or ids1 is None or corners2 is None or ids2 is None:
        return None, None, None
    
    obj_points = []
    img_points1 = []
    img_points2 = []
    
    # Convertir en dictionnaires pour faciliter la recherche
    corners1_dict = {ids1[i][0]: corners1[i][0] for i in range(len(ids1))}
    corners2_dict = {ids2[i][0]: corners2[i][0] for i in range(len(ids2))}
    
    # Trouver les IDs communs
    common_ids = set(corners1_dict.keys()) & set(corners2_dict.keys())
    
    # Valider la distribution spatiale des points
    if len(common_ids) < min_points:
        return None, None, None
    
    for corner_id in sorted(common_ids):
        pos_3d = get_charuco_corner_3d_position(corner_id, cfg['squares_x'], cfg['squares_y'], cfg['square_size'])
        if pos_3d is not None:
            obj_points.append(pos_3d)
            img_points1.append(corners1_dict[corner_id])
            img_points2.append(corners2_dict[corner_id])
    
    if len(obj_points) < min_points:
        return None, None, None
    
    # Validation de la répartition spatiale des points
    obj_array = np.array(obj_points)
    x_span = np.max(obj_array[:, 0]) - np.min(obj_array[:, 0])
    y_span = np.max(obj_array[:, 1]) - np.min(obj_array[:, 1])
    
    # Vérifier que les points couvrent une zone suffisante
    min_span = cfg['square_size'] * 3  # Au moins 3 carrés dans chaque direction
    if x_span < min_span or y_span < min_span:
        print(f"    Distribution spatiale insuffisante: {x_span:.1f}x{y_span:.1f}mm")
        return None, None, None
    
    return (np.array(obj_points, dtype=np.float32),
            np.array(img_points1, dtype=np.float32),
            np.array(img_points2, dtype=np.float32))

def validate_stereo_pair(obj_pts, img_pts1, img_pts2, mtx1, dist1, mtx2, dist2):
    """
    Valide la qualité d'une paire stéréo avant de l'inclure dans la calibration
    """
    # Test de cohérence géométrique avec solvePnP
    try:
        ret1, rvec1, tvec1 = cv2.solvePnP(obj_pts, img_pts1, mtx1, dist1)
        ret2, rvec2, tvec2 = cv2.solvePnP(obj_pts, img_pts2, mtx2, dist2)
        
        if not ret1 or not ret2:
            return False, "Échec solvePnP"
        
        # Calcul de l'erreur de reprojection
        proj1, _ = cv2.projectPoints(obj_pts, rvec1, tvec1, mtx1, dist1)
        proj2, _ = cv2.projectPoints(obj_pts, rvec2, tvec2, mtx2, dist2)
        
        error1 = cv2.norm(img_pts1, proj1.reshape(-1, 2), cv2.NORM_L2) / len(img_pts1)
        error2 = cv2.norm(img_pts2, proj2.reshape(-1, 2), cv2.NORM_L2) / len(img_pts2)
        
        # Rejeter les paires avec erreur trop élevée
        if error1 > 2.0 or error2 > 2.0:
            return False, f"Erreur reprojection élevée: {error1:.2f}, {error2:.2f}"
        
        return True, f"Erreurs: {error1:.2f}, {error2:.2f}"
        
    except Exception as e:
        return False, f"Exception: {str(e)}"
    
    
def visualize_stereo_correspondences(img1, img2, corners1, ids1, corners2, ids2, 
                                   aruco_corners1, aruco_ids1, aruco_corners2, aruco_ids2,
                                   pair_idx, show_images=True):
    """Visualise les correspondances entre les deux images stéréo"""
    
    # Copier les images pour dessiner dessus
    vis_img1 = img1.copy() if len(img1.shape) == 3 else cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    vis_img2 = img2.copy() if len(img2.shape) == 3 else cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    
    # Dessiner les marqueurs ArUco détectés
    if aruco_corners1 is not None and aruco_ids1 is not None:
        cv2.aruco.drawDetectedMarkers(vis_img1, aruco_corners1, aruco_ids1)
    if aruco_corners2 is not None and aruco_ids2 is not None:
        cv2.aruco.drawDetectedMarkers(vis_img2, aruco_corners2, aruco_ids2)
    
    # Dessiner les coins Charuco et leurs correspondances
    if corners1 is not None and ids1 is not None and corners2 is not None and ids2 is not None:
        # Convertir en dictionnaires pour faciliter la recherche
        corners1_dict = {ids1[i][0]: corners1[i][0] for i in range(len(ids1))}
        corners2_dict = {ids2[i][0]: corners2[i][0] for i in range(len(ids2))}
        
        # Trouver les correspondances communes
        common_ids = set(corners1_dict.keys()) & set(corners2_dict.keys())
        
        print(f"  Paire {pair_idx}: {len(common_ids)} correspondances communes trouvées")
        
        # Dessiner tous les coins détectés
        for corner_id, corner in corners1_dict.items():
            color = (0, 255, 0) if corner_id in common_ids else (0, 0, 255)
            cv2.circle(vis_img1, tuple(corner.astype(int)), 8, color, 2)
            cv2.putText(vis_img1, str(corner_id), tuple(corner.astype(int) + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        for corner_id, corner in corners2_dict.items():
            color = (0, 255, 0) if corner_id in common_ids else (0, 0, 255)
            cv2.circle(vis_img2, tuple(corner.astype(int)), 8, color, 2)
            cv2.putText(vis_img2, str(corner_id), tuple(corner.astype(int) + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Créer une image combinée pour montrer les correspondances
        h1, w1 = vis_img1.shape[:2]
        h2, w2 = vis_img2.shape[:2]
        combined_h = max(h1, h2)
        combined_w = w1 + w2
        combined_img = np.zeros((combined_h, combined_w, 3), dtype=np.uint8)
        
        # Placer les images côte à côte
        combined_img[:h1, :w1] = vis_img1
        combined_img[:h2, w1:w1+w2] = vis_img2
        
        # Dessiner les lignes de correspondance
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
        for i, corner_id in enumerate(sorted(common_ids)):
            color = colors[i % len(colors)]
            c1 = corners1_dict[corner_id]
            c2 = corners2_dict[corner_id]
            pt1 = tuple(c1.astype(int))
            pt2 = tuple((c2 + [w1, 0]).astype(int))
            cv2.line(combined_img, pt1, pt2, color, 2)
            cv2.circle(combined_img, pt1, 6, color, -1)
            cv2.circle(combined_img, pt2, 6, color, -1)
        
        if show_images:
            # Redimensionner pour l'affichage si nécessaire
            display_width = 1200
            if combined_w > display_width:
                scale = display_width / combined_w
                new_h = int(combined_h * scale)
                combined_img = cv2.resize(combined_img, (display_width, new_h))
            
            cv2.imshow(f"Correspondances Stereo - Paire {pair_idx}", combined_img)
            key = cv2.waitKey(0) & 0xFF
            cv2.destroyAllWindows()
            
            if key == ord('q'):
                return False  # Signal pour arrêter
        
        return len(common_ids)
    
    return 0


def stereo_calibration_charuco():
    """Effectue la calibration stéréo avec plateau Charuco"""
    cfg = get_config()
    
    print("=== CALIBRATION STÉRÉO CHARUCO ===")
    
    # Charger les calibrations intrinsèques
    mtx1, dist1, size1 = load_calibration(cfg['camera_1_calib'])
    mtx2, dist2, size2 = load_calibration(cfg['camera_2_calib'])
    
    if mtx1 is None or mtx2 is None:
        print("✗ Impossible de charger les calibrations intrinsèques")
        return False
    
    print(f"✓ Calibrations intrinsèques chargées")
    print(f"  Caméra 1: {size1}, focale: {mtx1[0,0]:.1f}px")
    print(f"  Caméra 2: {size2}, focale: {mtx2[0,0]:.1f}px")
    
    # Gestion des tailles d'images différentes
    if size1 != size2:
        print(f"⚠ Tailles d'images différentes ({size1} vs {size2})")
        # Utiliser la plus petite taille commune
        size_common = (min(size1[0], size2[0]), min(size1[1], size2[1]))
        print(f"  ➜ Utilisation de la taille commune: {size_common}")
    else:
        size_common = size1
    
    # Créer le plateau Charuco
    board = create_charuco_board(cfg)
    
    # Paramètres de détection optimisés pour stéréo
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.adaptiveThreshWinSizeMin = 5
    detector_params.adaptiveThreshWinSizeMax = 21
    detector_params.minMarkerPerimeterRate = 0.05
    detector_params.maxMarkerPerimeterRate = 3.0
    detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector_params.cornerRefinementWinSize = 5
    detector_params.cornerRefinementMaxIterations = 50
    detector_params.cornerRefinementMinAccuracy = 0.01
    
    # Chercher les paires d'images stéréo
    cam1_pattern = os.path.join(cfg['camera_1_stereo_path'], "camera_1_*.jpg")
    cam2_pattern = os.path.join(cfg['camera_2_stereo_path'], "camera_2_*.jpg")
    
    cam1_files = sorted(glob.glob(cam1_pattern))  
    cam2_files = sorted(glob.glob(cam2_pattern))
    
    if len(cam1_files) == 0 or len(cam2_files) == 0:
        print("✗ Aucune image stéréo trouvée")
        return False
    
    print(f"✓ Trouvé {len(cam1_files)} images caméra 1 et {len(cam2_files)} images caméra 2")
    
    # Associer les paires par numéro
    pairs = []
    for f1 in cam1_files:
        num1 = os.path.basename(f1).split('_')[-1].split('.')[0]
        f2_expected = os.path.join(cfg['camera_2_stereo_path'], f"camera_2_{num1}.jpg")
        if f2_expected in cam2_files:
            pairs.append((f1, f2_expected))
    
    print(f"✓ {len(pairs)} paires d'images trouvées")
    
    # Collecter les points pour la calibration stéréo
    object_points = []
    image_points1 = []
    image_points2 = []
    valid_pairs = 0
    
    print("\nAnalyse des paires d'images...")
    
    for i, (f1, f2) in enumerate(pairs):
        print(f"  Paire {i+1}/{len(pairs)}:", end=" ")
        
        img1 = cv2.imread(f1)
        img2 = cv2.imread(f2)
        
        if img1 is None or img2 is None:
            print("erreur de lecture")
            continue
        
        # Redimensionner si nécessaire
        if img1.shape[:2][::-1] != size_common:
            img1 = cv2.resize(img1, size_common)
        if img2.shape[:2][::-1] != size_common:
            img2 = cv2.resize(img2, size_common)
        
        # Détection des coins Charuco
        corners1, ids1, aruco_corners1, aruco_ids1 = detect_charuco_corners(img1, board, detector_params)
        corners2, ids2, aruco_corners2, aruco_ids2 = detect_charuco_corners(img2, board, detector_params)
        
        if corners1 is None or corners2 is None or ids1 is None or ids2 is None:
            print("détection insuffisante")
            continue
        
        # Filtrer les correspondances communes
        obj_pts, img_pts1, img_pts2 = filter_common_corners_robust(corners1, ids1, corners2, ids2, cfg)
        
        if obj_pts is None or len(obj_pts) < 8:
            count = len(obj_pts) if obj_pts is not None else 0
            print(f"correspondances insuffisantes ({count})")
            continue
        
        # Visualisation des correspondances
        num_correspondances = visualize_stereo_correspondences(
            img1, img2, corners1, ids1, corners2, ids2,
            aruco_corners1, aruco_ids1, aruco_corners2, aruco_ids2,
            i+1, show_images=(i < 3)  # Montrer seulement les 3 premières paires
        )
        
        # Validation de la paire
        is_valid, validation_msg = validate_stereo_pair(obj_pts, img_pts1, img_pts2, mtx1, dist1, mtx2, dist2)
        
        if not is_valid:
            print(f"rejetée ({validation_msg})")
            continue
        
        # Ajouter les points valides pour la calibration
        object_points.append(obj_pts)
        image_points1.append(img_pts1)
        image_points2.append(img_pts2)
        valid_pairs += 1
        print(f"✓ {len(obj_pts)} points ({validation_msg})")
    
    if valid_pairs < 5:
        print(f"\n✗ Pas assez de paires valides ({valid_pairs}). Minimum 5 requis.")
        return False
        
    print(f"\n✓ Calibration stéréo avec {valid_pairs} paires...")
    
    # Configuration de la calibration stéréo
    # Utiliser CALIB_FIX_INTRINSIC pour garder les paramètres intrinsèques fixes
    # Ajouter CALIB_RATIONAL_MODEL pour une meilleure modélisation de la distorsion
    flags = (cv2.CALIB_FIX_INTRINSIC | 
             cv2.CALIB_RATIONAL_MODEL |
             cv2.CALIB_FIX_PRINCIPAL_POINT)
    
    # Critères de convergence plus stricts
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8)
    
    try:
        print("  Lancement de cv2.stereoCalibrate...")
        ret, mtx1_new, dist1_new, mtx2_new, dist2_new, R, T, E, F = cv2.stereoCalibrate(
            object_points, image_points1, image_points2,
            mtx1, dist1, mtx2, dist2, size_common,
            criteria=criteria, flags=flags
        )
        
        if not ret or ret > 30.0:  # RMS trop élevé
            print(f"✗ Calibration de qualité insuffisante (RMS={ret:.3f})")
            return False
            
    except Exception as e:
        print(f"✗ Erreur lors de la calibration stéréo: {e}")
        return False
    
    print(f"  ✓ RMS error de stereoCalibrate: {ret:.4f}")
    
    # Rectification stéréo avec alpha optimisé
    print("  Calcul de la rectification stéréo...")
    
    # Alpha=1 conserve tous les pixels, alpha=0 ne conserve que les pixels valides
    # Alpha=-1 laisse OpenCV choisir automatiquement
    alpha = -1  
    
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        mtx1, dist1, mtx2, dist2, size_common, R, T, 
        alpha=alpha, newImageSize=size_common
    )
    
    # Calcul des métriques géométriques
    baseline = np.linalg.norm(T)
    
    # Analyse de la rotation - décomposition en angles d'Euler
    rotation_matrix = R
    # Conversion en angles d'Euler (ZYX convention)
    sy = np.sqrt(rotation_matrix[0,0] * rotation_matrix[0,0] +  rotation_matrix[1,0] * rotation_matrix[1,0])
    singular = sy < 1e-6
    
    if not singular:
        x_angle = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2])
        y_angle = np.arctan2(-rotation_matrix[2,0], sy)
        z_angle = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])
    else:
        x_angle = np.arctan2(-rotation_matrix[1,2], rotation_matrix[1,1])
        y_angle = np.arctan2(-rotation_matrix[2,0], sy)
        z_angle = 0
    
    # Conversion en degrés
    euler_angles = np.array([x_angle, y_angle, z_angle]) * 180.0 / np.pi
    
    # Analyse du vecteur de translation
    translation_analysis = {
        'tx_mm': float(T[0][0] * 1000),
        'ty_mm': float(T[1][0] * 1000),
        'tz_mm': float(T[2][0] * 1000),
        'baseline_mm': float(baseline * 1000),
        'euler_x_deg': float(euler_angles[0]),
        'euler_y_deg': float(euler_angles[1]), 
        'euler_z_deg': float(euler_angles[2])
    }
    
    # Validation des ROI
    roi1_valid = roi1[2] > 0 and roi1[3] > 0
    roi2_valid = roi2[2] > 0 and roi2[3] > 0
    
    # Sauvegarde des résultats
    stereo_data = {
        'rotation_matrix': R.tolist(),
        'translation_vector': T.tolist(),
        'essential_matrix': E.tolist(),
        'fundamental_matrix': F.tolist(),
        'rectification_R1': R1.tolist(),
        'rectification_R2': R2.tolist(),
        'projection_P1': P1.tolist(),
        'projection_P2': P2.tolist(),
        'disparity_to_depth_Q': Q.tolist(),
        'roi1': roi1,
        'roi2': roi2,
        'rms_error': float(ret),
        'baseline_mm': translation_analysis['baseline_mm'],
        'translation_analysis': translation_analysis,
        'valid_pairs': int(valid_pairs),
        'image_size': list(size_common),
        'calibration_flags': int(flags),
        'alpha_rectification': float(alpha),
        'roi_validity': {'roi1_valid': roi1_valid, 'roi2_valid': roi2_valid},
        'calibration_date': str(np.datetime64('now'))
    }
    
    os.makedirs(os.path.dirname(cfg['stereo_calib']), exist_ok=True)
    with open(cfg['stereo_calib'], 'w') as f:
        yaml.dump(stereo_data, f, default_flow_style=False)
    
    # Affichage des résultats
    print(f"\n=== RÉSULTATS CALIBRATION STÉRÉO ===")
    print(f"✓ Calibration réussie avec {valid_pairs} paires")
    print(f"  RMS error: {ret:.4f} pixels")
    print(f"  Baseline: {baseline*1000:.1f} mm")
    print(f"  Translation (mm):")
    print(f"    Tx: {translation_analysis['tx_mm']:.1f}")
    print(f"    Ty: {translation_analysis['ty_mm']:.1f}")
    print(f"    Tz: {translation_analysis['tz_mm']:.1f}")
    print(f"  Rotation (angles d'Euler en degrés):")
    print(f"    Roll (X):  {translation_analysis['euler_x_deg']:.1f}°")
    print(f"    Pitch (Y): {translation_analysis['euler_y_deg']:.1f}°")
    print(f"    Yaw (Z):   {translation_analysis['euler_z_deg']:.1f}°")
    print(f"  ROI Caméra 1: {roi1} {'✓' if roi1_valid else '✗'}")
    print(f"  ROI Caméra 2: {roi2} {'✓' if roi2_valid else '✗'}")
    print(f"  Fichier sauvegardé: {cfg['stereo_calib']}")
     
    
    return True

def main():
    """Fonction principale"""
    success = stereo_calibration_charuco()
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())