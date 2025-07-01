#!/usr/bin/env python3
"""
Calibration stéréo 3D pour caméras non-coplanaires
Calcule la transformation 3D complète entre les caméras
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
    """Détecte les coins Charuco dans une image"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    
    # Amélioration du contraste
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)
    
    # Détection des marqueurs ArUco
    corners, ids, _ = cv2.aruco.detectMarkers(gray, board.getDictionary(), parameters=detector_params)
    
    if ids is None or len(ids) < 4:
        return None, None
    
    # Interpolation des coins Charuco
    ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board
    )
    
    if not ret or charuco_corners is None or len(charuco_corners) < 6:
        return None, None
    
    # Raffinement sous-pixel
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)
    cv2.cornerSubPix(gray, charuco_corners, (7, 7), (-1, -1), criteria)
    
    return charuco_corners, charuco_ids

def get_board_3d_points(board, corner_ids):
    """Obtient les points 3D du plateau pour les IDs donnés"""
    obj_points = []
    for corner_id in corner_ids.flatten():
        # Récupération de la position 3D du coin depuis le board
        board_corners = board.getChessboardCorners()
        if corner_id < len(board_corners):
            obj_points.append(board_corners[corner_id])
    return np.array(obj_points, dtype=np.float32)

def estimate_pose_from_charuco(corners, ids, board, camera_matrix, dist_coeffs):
    """Estime la pose du plateau Charuco dans le repère caméra"""
    if corners is None or ids is None or len(corners) < 4:
        return None, None
    
    # Obtenir les points 3D correspondants
    obj_points = get_board_3d_points(board, ids)
    
    if len(obj_points) != len(corners):
        return None, None
    
    # Estimation de pose avec solvePnP
    success, rvec, tvec = cv2.solvePnP(
        obj_points, corners.reshape(-1, 2), 
        camera_matrix, dist_coeffs
    )
    
    if not success:
        return None, None
    
    return rvec, tvec

def compute_3d_transformation(rvec1, tvec1, rvec2, tvec2):
    """
    Calcule la transformation 3D de la caméra 1 vers la caméra 2
    """
    # Conversion des vecteurs de rotation en matrices
    R1, _ = cv2.Rodrigues(rvec1)
    R2, _ = cv2.Rodrigues(rvec2)
    
    # Positions du plateau dans chaque repère caméra
    T1 = tvec1.flatten()
    T2 = tvec2.flatten()
    
    # Transformation de cam1 vers cam2
    # Si P est un point 3D dans le repère du plateau:
    # P_cam1 = R1 * P + T1
    # P_cam2 = R2 * P + T2
    # Donc: P_cam2 = R2 * R1^(-1) * (P_cam1 - T1) + T2
    
    R1_inv = R1.T
    R_cam1_to_cam2 = R2 @ R1_inv
    T_cam1_to_cam2 = T2 - R_cam1_to_cam2 @ T1
    
    return R_cam1_to_cam2, T_cam1_to_cam2

def validate_transformation(R, T, min_baseline=30.0, max_rotation=145.0):
    """Valide la transformation 3D calculée"""
    # Vérifier la baseline (distance entre caméras)
    baseline = np.linalg.norm(T)
    if baseline < min_baseline / 1000.0:  # Convertir mm en m
        return False, f"Baseline trop faible: {baseline*1000:.1f}mm"
    
    # Vérifier que la rotation n'est pas trop importante
    rotation_angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    rotation_deg = np.degrees(rotation_angle)
    
    if rotation_deg > max_rotation:
        return False, f"Rotation excessive: {rotation_deg:.1f}°"
    
    return True, f"Baseline: {baseline*1000:.1f}mm, Rotation: {rotation_deg:.1f}°"

def create_3d_camera_visualization(R, T, cfg):
    """
    Crée une visualisation 3D de la configuration des caméras
    """
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Position des caméras
        cam1_pos = np.array([0, 0, 0])  # Caméra 1 à l'origine
        cam2_pos = T * 1000  # Caméra 2 (en mm)
        
        # Taille des caméras pour la visualisation
        cam_size = 50
        
        # Dessiner les caméras
        ax.scatter(*cam1_pos, color='blue', s=200, marker='s', label='Caméra 1')
        ax.scatter(*cam2_pos, color='red', s=200, marker='s', label='Caméra 2')
        
        # Dessiner la ligne de base
        ax.plot([cam1_pos[0], cam2_pos[0]], 
                [cam1_pos[1], cam2_pos[1]], 
                [cam1_pos[2], cam2_pos[2]], 'k--', linewidth=2, label='Baseline')
        
        # Dessiner les axes des caméras
        axis_length = 100  # mm
        
        # Axes caméra 1 (identité)
        axes1 = np.eye(3) * axis_length
        colors = ['red', 'green', 'blue']
        labels = ['X1', 'Y1', 'Z1']
        
        for i, (axis, color, label) in enumerate(zip(axes1, colors, labels)):
            ax.quiver(cam1_pos[0], cam1_pos[1], cam1_pos[2],
                     axis[0], axis[1], axis[2], color=color, alpha=0.7,
                     label=label if i == 0 else "")
        
        # Axes caméra 2 (transformés)
        axes2 = R @ axes1
        labels2 = ['X2', 'Y2', 'Z2']
        
        for i, (axis, color, label) in enumerate(zip(axes2, colors, labels2)):
            ax.quiver(cam2_pos[0], cam2_pos[1], cam2_pos[2],
                     axis[0], axis[1], axis[2], color=color, alpha=0.4, linestyle=':',
                     label=label if i == 0 else "")
        
        # Configuration du graphique
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('Configuration 3D des Caméras Stéréo')
        ax.legend()
        
        # Égaliser les axes
        max_range = max(np.abs(cam2_pos)) + axis_length
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range])
        
        # Ajouter des informations textuelles
        baseline = np.linalg.norm(T) * 1000
        rotation_angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)) * 180 / np.pi
        
        info_text = f"""Configuration Stéréo:
Baseline: {baseline:.1f} mm
Angle de rotation: {rotation_angle:.1f}°
Translation: [{T[0]*1000:.1f}, {T[1]*1000:.1f}, {T[2]*1000:.1f}] mm"""
        
        ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=10,
                 verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
        # Sauvegarder le graphique
        output_path = cfg.get('stereo_calib', 'calibration_stereo.yaml').replace('.yaml', '_3d_config.png')
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"  ✓ Schéma 3D sauvegardé: {output_path}")
        
        plt.show()
        
    except ImportError:
        print("  ⚠ Matplotlib non disponible pour la visualisation 3D")
    except Exception as e:
        print(f"  ⚠ Erreur lors de la visualisation 3D: {e}")

def stereo_calibration_3d():
    """Effectue la calibration stéréo 3D pour caméras non-coplanaires"""
    cfg = get_config()
    
    print("=== CALIBRATION STÉRÉO 3D ===")
    
    # Charger les calibrations intrinsèques
    mtx1, dist1, size1 = load_calibration(cfg['camera_1_calib'])
    mtx2, dist2, size2 = load_calibration(cfg['camera_2_calib'])
    
    if mtx1 is None or mtx2 is None:
        print("✗ Impossible de charger les calibrations intrinsèques")
        return False
    
    print(f"✓ Calibrations intrinsèques chargées")
    
    # Créer le plateau Charuco
    board = create_charuco_board(cfg)
    
    # Paramètres de détection
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.adaptiveThreshWinSizeMin = 5
    detector_params.adaptiveThreshWinSizeMax = 21
    detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    
    # Chercher les paires d'images
    cam1_pattern = os.path.join(cfg['camera_1_stereo_path'], "camera_1_*.jpg")
    cam2_pattern = os.path.join(cfg['camera_2_stereo_path'], "camera_2_*.jpg")
    
    cam1_files = sorted(glob.glob(cam1_pattern))
    cam2_files = sorted(glob.glob(cam2_pattern))
    
    if len(cam1_files) == 0 or len(cam2_files) == 0:
        print("✗ Aucune image stéréo trouvée")
        return False
    
    # Associer les paires par numéro
    pairs = []
    for f1 in cam1_files:
        num1 = os.path.basename(f1).split('_')[-1].split('.')[0]
        f2_expected = os.path.join(cfg['camera_2_stereo_path'], f"camera_2_{num1}.jpg")
        if f2_expected in cam2_files:
            pairs.append((f1, f2_expected))
    
    print(f"✓ {len(pairs)} paires d'images trouvées")
    
    # Collecter les transformations 3D
    transformations = []
    valid_pairs = 0
    
    print("\nAnalyse des transformations 3D...")
    
    for i, (f1, f2) in enumerate(pairs):
        print(f"  Paire {i+1}/{len(pairs)}:", end=" ")
        
        img1 = cv2.imread(f1)
        img2 = cv2.imread(f2)
        
        if img1 is None or img2 is None:
            print("erreur de lecture")
            continue
        
        # Détection des coins Charuco dans chaque image
        corners1, ids1 = detect_charuco_corners(img1, board, detector_params)
        corners2, ids2 = detect_charuco_corners(img2, board, detector_params)
        
        if corners1 is None or corners2 is None:
            print("détection insuffisante")
            continue
        
        # Estimation de pose pour chaque caméra
        rvec1, tvec1 = estimate_pose_from_charuco(corners1, ids1, board, mtx1, dist1)
        rvec2, tvec2 = estimate_pose_from_charuco(corners2, ids2, board, mtx2, dist2)
        
        if rvec1 is None or rvec2 is None:
            print("estimation de pose échouée")
            continue
        
        # Calcul de la transformation 3D entre caméras
        R_12, T_12 = compute_3d_transformation(rvec1, tvec1, rvec2, tvec2)
        
        # Validation de la transformation
        is_valid, msg = validate_transformation(R_12, T_12)
        
        if not is_valid:
            print(f"rejetée ({msg})")
            continue
        
        transformations.append((R_12, T_12))
        valid_pairs += 1
        print(f"✓ ({msg})")
    
    if valid_pairs < 3:
        print(f"\n✗ Pas assez de paires valides ({valid_pairs}). Minimum 3 requis.")
        return False
    
    # Moyenner les transformations pour plus de robustesse
    print(f"\n✓ Calcul de la transformation moyenne sur {valid_pairs} paires...")
    
    # Moyenne des matrices de rotation (approximation)
    R_mean = np.mean([R for R, T in transformations], axis=0)
    T_mean = np.mean([T for R, T in transformations], axis=0)
    
    # Orthogonalisation de la matrice de rotation moyenne
    U, _, Vt = np.linalg.svd(R_mean)
    R_mean = U @ Vt
    
    # Calcul des métriques finales
    baseline = np.linalg.norm(T_mean)
    rotation_angle = np.arccos(np.clip((np.trace(R_mean) - 1) / 2, -1, 1))
    rotation_deg = np.degrees(rotation_angle)
    
    # Analyse de la transformation
    euler_angles = cv2.Rodrigues(R_mean)[0].flatten() * 180.0 / np.pi
    
    # Sauvegarde des résultats
    stereo_3d_data = {
        'rotation_matrix': R_mean.tolist(),
        'translation_vector': T_mean.tolist(),
        'baseline_mm': float(baseline * 1000),
        'rotation_angle_deg': float(rotation_deg),
        'euler_angles_deg': euler_angles.tolist(),
        'camera_1_matrix': mtx1.tolist(),
        'camera_1_distortion': dist1.tolist(),
        'camera_2_matrix': mtx2.tolist(),
        'camera_2_distortion': dist2.tolist(),
        'image_size_1': list(size1),
        'image_size_2': list(size2),
        'valid_pairs': int(valid_pairs),
        'calibration_type': '3D_non_coplanar',
        'calibration_date': str(np.datetime64('now'))
    }
    
    os.makedirs(os.path.dirname(cfg['stereo_calib']), exist_ok=True)
    with open(cfg['stereo_calib'], 'w') as f:
        yaml.dump(stereo_3d_data, f, default_flow_style=False)
    
    # Affichage des résultats
    print(f"\n=== RÉSULTATS CALIBRATION STÉRÉO 3D ===")
    print(f"✓ Calibration réussie avec {valid_pairs} paires")
    print(f"  Baseline: {baseline*1000:.1f} mm")
    print(f"  Angle de rotation: {rotation_deg:.1f}°")
    print(f"  Translation (mm): [{T_mean[0]*1000:.1f}, {T_mean[1]*1000:.1f}, {T_mean[2]*1000:.1f}]")
    print(f"  Angles d'Euler (°): [{euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f}]")
    print(f"  Fichier sauvegardé: {cfg['stereo_calib']}")
    
    return True

def project_point_to_camera2(point_3d_cam1, R, T):
    """
    Projette un point 3D du repère caméra 1 vers le repère caméra 2
    """
    return R @ point_3d_cam1 + T

def triangulate_3d_point(point_cam1, point_cam2, mtx1, mtx2, R, T):
    """
    Triangule un point 3D à partir de ses projections dans les deux caméras
    """
    # Création des matrices de projection
    P1 = mtx1 @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = mtx2 @ np.hstack([R, T.reshape(-1, 1)])
    
    # Triangulation
    point_4d = cv2.triangulatePoints(P1, P2, point_cam1.T, point_cam2.T)
    point_3d = point_4d[:3] / point_4d[3]
    
    return point_3d.T

def evaluate_stereo_performance(pairs, board, detector_params, mtx1, dist1, mtx2, dist2, R_mean, T_mean):
    """
    Évalue les performances de la calibration stéréo
    """
    print("\n=== ÉVALUATION DES PERFORMANCES ===")
    
    reprojection_errors = []
    triangulation_errors = []
    correspondence_counts = []
    
    for i, (f1, f2) in enumerate(pairs[:5]):  # Tester sur 5 paires max
        img1 = cv2.imread(f1)
        img2 = cv2.imread(f2)
        
        if img1 is None or img2 is None:
            continue
        
        # Détection des coins
        corners1, ids1 = detect_charuco_corners(img1, board, detector_params)
        corners2, ids2 = detect_charuco_corners(img2, board, detector_params)
        
        if corners1 is None or corners2 is None:
            continue
        
        # Trouver les correspondances communes
        ids1_flat = ids1.flatten()
        ids2_flat = ids2.flatten()
        common_ids = set(ids1_flat) & set(ids2_flat)
        
        if len(common_ids) < 4:
            continue
        
        # Préparer les points correspondants
        points1 = []
        points2 = []
        obj_points = []
        
        for corner_id in common_ids:
            idx1 = np.where(ids1_flat == corner_id)[0][0]
            idx2 = np.where(ids2_flat == corner_id)[0][0]
            
            points1.append(corners1[idx1][0])
            points2.append(corners2[idx2][0])
            
            # Point 3D théorique du plateau
            board_corners = get_board_3d_points(board, np.array([[corner_id]]))
            if len(board_corners) > 0:
                obj_points.append(board_corners[0])
        
        if len(points1) < 4:
            continue
        
        points1 = np.array(points1, dtype=np.float32)
        points2 = np.array(points2, dtype=np.float32)
        obj_points = np.array(obj_points, dtype=np.float32)
        
        # 1. Erreur de reprojection
        # Estimer la pose pour chaque caméra
        ret1, rvec1, tvec1 = cv2.solvePnP(obj_points, points1, mtx1, dist1)
        ret2, rvec2, tvec2 = cv2.solvePnP(obj_points, points2, mtx2, dist2)
        
        if ret1 and ret2:
            # Reprojection
            proj1, _ = cv2.projectPoints(obj_points, rvec1, tvec1, mtx1, dist1)
            proj2, _ = cv2.projectPoints(obj_points, rvec2, tvec2, mtx2, dist2)
            
            error1 = np.mean(np.linalg.norm(points1 - proj1.reshape(-1, 2), axis=1))
            error2 = np.mean(np.linalg.norm(points2 - proj2.reshape(-1, 2), axis=1))
            
            reprojection_errors.append((error1 + error2) / 2)
        
        # 2. Erreur de triangulation
        triangulated_points = triangulate_3d_point(points1, points2, mtx1, mtx2, R_mean, T_mean)
        
        if len(triangulated_points) == len(obj_points):
            # Calculer l'erreur 3D
            tri_error = np.mean(np.linalg.norm(triangulated_points - obj_points, axis=1))
            triangulation_errors.append(tri_error * 1000)  # en mm
        
        correspondence_counts.append(len(common_ids))
    
    # Statistiques
    if reprojection_errors:
        print(f"  Erreur de reprojection:")
        print(f"    Moyenne: {np.mean(reprojection_errors):.2f} pixels")
        print(f"    Écart-type: {np.std(reprojection_errors):.2f} pixels")
        print(f"    Max: {np.max(reprojection_errors):.2f} pixels")
    
    if triangulation_errors:
        print(f"  Erreur de triangulation:")
        print(f"    Moyenne: {np.mean(triangulation_errors):.1f} mm")
        print(f"    Écart-type: {np.std(triangulation_errors):.1f} mm")
        print(f"    Max: {np.max(triangulation_errors):.1f} mm")
    
    if correspondence_counts:
        print(f"  Correspondances par paire:")
        print(f"    Moyenne: {np.mean(correspondence_counts):.1f} points")
        print(f"    Min: {np.min(correspondence_counts)} points")
    
    return {
        'reprojection_error_mean': np.mean(reprojection_errors) if reprojection_errors else 0,
        'triangulation_error_mean': np.mean(triangulation_errors) if triangulation_errors else 0,
        'correspondences_mean': np.mean(correspondence_counts) if correspondence_counts else 0
    }

def visualize_stereo_results(pairs, board, detector_params, mtx1, dist1, mtx2, dist2, R_mean, T_mean, cfg):
    """
    Visualise les résultats de la calibration stéréo
    """
    print("\n=== VISUALISATION DES RÉSULTATS ===")
    
    for i, (f1, f2) in enumerate(pairs[:3]):  # Visualiser 3 paires max
        img1 = cv2.imread(f1)
        img2 = cv2.imread(f2)
        
        if img1 is None or img2 is None:
            continue
        
        print(f"  Visualisation paire {i+1}...")
        
        # Détection des coins
        corners1, ids1 = detect_charuco_corners(img1, board, detector_params)
        corners2, ids2 = detect_charuco_corners(img2, board, detector_params)
        
        if corners1 is None or corners2 is None:
            continue
        
        # Dessiner les détections
        vis_img1 = img1.copy()
        vis_img2 = img2.copy()
        
        # Dessiner les coins détectés
        for j, corner in enumerate(corners1):
            cv2.circle(vis_img1, tuple(corner[0].astype(int)), 8, (0, 255, 0), 2)
            cv2.putText(vis_img1, str(ids1[j][0]), tuple(corner[0].astype(int) + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        for j, corner in enumerate(corners2):
            cv2.circle(vis_img2, tuple(corner[0].astype(int)), 8, (0, 255, 0), 2)
            cv2.putText(vis_img2, str(ids2[j][0]), tuple(corner[0].astype(int) + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Triangulation et reprojection pour validation
        ids1_flat = ids1.flatten()
        ids2_flat = ids2.flatten()
        common_ids = set(ids1_flat) & set(ids2_flat)
        
        if len(common_ids) >= 4:
            points1 = []
            points2 = []
            
            for corner_id in common_ids:
                idx1 = np.where(ids1_flat == corner_id)[0][0]
                idx2 = np.where(ids2_flat == corner_id)[0][0]
                points1.append(corners1[idx1][0])
                points2.append(corners2[idx2][0])
            
            points1 = np.array(points1, dtype=np.float32)
            points2 = np.array(points2, dtype=np.float32)
            
            # Triangulation
            triangulated = triangulate_3d_point(points1, points2, mtx1, mtx2, R_mean, T_mean)
            
            # Reprojection des points triangulés
            if len(triangulated) > 0:
                # Pose de la caméra 1 (origine)
                rvec1 = np.array([0., 0., 0.])
                tvec1 = np.array([0., 0., 0.])
                
                # Pose de la caméra 2
                rvec2, _ = cv2.Rodrigues(R_mean)
                tvec2 = T_mean
                
                # Reprojection
                proj1, _ = cv2.projectPoints(triangulated, rvec1, tvec1, mtx1, dist1)
                proj2, _ = cv2.projectPoints(triangulated, rvec2, tvec2, mtx2, dist2)
                
                # Dessiner les points reprojetés en rouge
                for pt in proj1:
                    cv2.circle(vis_img1, tuple(pt[0].astype(int)), 4, (0, 0, 255), -1)
                
                for pt in proj2:
                    cv2.circle(vis_img2, tuple(pt[0].astype(int)), 4, (0, 0, 255), -1)
        
        # Affichage côte à côte
        h1, w1 = vis_img1.shape[:2]
        h2, w2 = vis_img2.shape[:2]
        combined_h = max(h1, h2)
        combined_w = w1 + w2
        combined_img = np.zeros((combined_h, combined_w, 3), dtype=np.uint8)
        
        combined_img[:h1, :w1] = vis_img1
        combined_img[:h2, w1:w1+w2] = vis_img2
        
        # Texte d'information
        cv2.putText(combined_img, f"Paire {i+1} - Vert: Detections, Rouge: Reprojections", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Redimensionner pour l'affichage
        display_width = 1200
        if combined_w > display_width:
            scale = display_width / combined_w
            new_h = int(combined_h * scale)
            combined_img = cv2.resize(combined_img, (display_width, new_h))
        
        cv2.imshow(f"Calibration Stereo 3D - Paire {i+1}", combined_img)
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        
        if key == ord('q'):
            break
    
    # Visualisation 3D de la configuration des caméras
    print("  Génération du schéma 3D des caméras...")
    create_3d_camera_visualization(R_mean, T_mean, cfg)

def main():
    """Fonction principale"""
    success = stereo_calibration_3d()
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())