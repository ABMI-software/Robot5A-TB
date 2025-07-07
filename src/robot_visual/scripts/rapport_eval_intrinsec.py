#!/usr/bin/env python3
"""
Module d'évaluation de la calibration et visualisation des résultats
"""

import cv2
import numpy as np
import glob
import os
import yaml
from config_charuco import get_config

def load_camera_calibration(calibration_file):
    """Charger les paramètres de calibration"""
    with open(calibration_file, 'r') as file:
        data = yaml.safe_load(file)
    return np.array(data['camera_matrix']), np.array(data['distortion_coefficients'])

def create_charuco_board():
    """Créer le plateau Charuco"""
    cfg = get_config()
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    try:
        return cv2.aruco.CharucoBoard((cfg['squares_x'], cfg['squares_y']),
                                       cfg['square_size'], cfg['marker_size'], aruco_dict)
    except AttributeError:
        return cv2.aruco.CharucoBoard_create(cfg['squares_x'], cfg['squares_y'],
                                             cfg['square_size'], cfg['marker_size'], aruco_dict)

def detect_charuco_corners(img, board):
    """Détecter les coins Charuco dans une image"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    detector_params = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, board.getDictionary(), parameters=detector_params)
    
    corner_img = img.copy()
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(corner_img, corners, ids)
        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board
        )
        if ret:
            cv2.aruco.drawDetectedCornersCharuco(corner_img, charuco_corners, charuco_ids, (0, 255, 0))
    
    return corner_img

def show_calibration_visual(input_dir, calibration_file, camera_name, max_images=3):
    """Afficher la visualisation: original, détection coins, correction distorsion"""
    print(f"\n=== VISUALISATION {camera_name.upper()} ===")
    
    if not os.path.exists(calibration_file):
        print(f"   Fichier de calibration non trouvé")
        return
    
    camera_matrix, dist_coeffs = load_camera_calibration(calibration_file)
    board = create_charuco_board()
    
    # Trouver les images
    extensions = ['*.jpg']
    images = []
    for ext in extensions:
        images.extend(glob.glob(os.path.join(input_dir, ext)))
        if len(images) >= max_images:
            break
    
    if not images:
        print(f"   Aucune image trouvée dans {input_dir}")
        return
    
    images = images[:max_images]
    print(f" Affichage de {len(images)} image de démonstration")
    
    for i, img_path in enumerate(images):
        img = cv2.imread(img_path)
        if img is None:
            continue
        
        # Image avec détection des coins
        corner_img = detect_charuco_corners(img, board)
        
        # Image corrigée
        h, w = img.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (w, h), 1, (w, h)
        )
        corrected_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
        
        # Créer l'image de comparaison 3 colonnes
        comparison = np.zeros((h, w*3, 3), dtype=np.uint8)
        comparison[:, :w] = img
        comparison[:, w:2*w] = corner_img
        comparison[:, 2*w:] = corrected_img
        
        # Ajouter les titres
        cv2.putText(comparison, 'ORIGINAL', (10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(comparison, 'DETECTION COINS', (w + 10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        cv2.putText(comparison, 'CORRIGEE', (2*w + 10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(comparison, f'{camera_name} - {os.path.basename(img_path)}', 
                   (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Redimensionner pour l'affichage
        display_width = 1500
        if w*3 > display_width:
            scale = display_width / (w*3)
            new_w = int(w*3 * scale)
            new_h = int(h * scale)
            comparison = cv2.resize(comparison, (new_w, new_h))
        
        window_name = f'Visualisation {camera_name}'
        cv2.imshow(window_name, comparison)
        
        print(f"   Image {i+1}/{len(images)} - Appuyez sur une touche pour continuer")
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)

def evaluate_calibration(mtx, dist, reprojection_error, calibration_file):
    """Évaluer la qualité de la calibration"""
    print(f"\n=== RAPPORT D'ÉVALUATION CALIBRATION ===")
    
    # Charger les données supplémentaires
    with open(calibration_file, 'r') as file:
        data = yaml.safe_load(file)
    valid_images_count = data.get('valid_images_count', 0)
    
    # 1. Erreur de reprojection
    print(f" Erreur de reprojection: {reprojection_error:.4f} pixels")
    if reprojection_error <= 0.3:
        print("  ✓ EXCELLENT (< 0.3 px)")
    elif reprojection_error <= 0.5:
        print("  ✓ BON (< 0.5 px)")
    elif reprojection_error <= 1.0:
        print("  ⚠ ACCEPTABLE (< 1.0 px)")
    else:
        print("  ✗ MAUVAIS (> 1.0 px)")
    
    print ("=> évaluation de la précision de la calibration en fonction de distance moyenne entre les points projetés (selon la calibration) et les points réellement détectés.")
       
    # 3. Paramètres intrinsèques
    fx, fy = mtx[0, 0], mtx[1, 1]
    cx, cy = mtx[0, 2], mtx[1, 2]
    focal_ratio = abs(fx - fy) / max(fx, fy)
    
    print(f"\n Focales fx/fy: {fx:.1f}/{fy:.1f} px")
    print(f" Centre optique: ({cx:.1f}, {cy:.1f})")
    print(f" Différence focales: {focal_ratio*100:.2f}%")
        
    if focal_ratio < 0.01:
        print("  ✓ EXCELLENT équilibre focales < 0.01")
    elif focal_ratio < 0.05:
        print("  ✓ BON équilibre focales < 0.05")
    else:
        print("  ⚠ Différence focale importante")
    print ("=> évaluation des déformation géométrique")
    
    # 4. Distorsion
    k1, k2, p1, p2 = dist[0][:4]
    radial_dist = abs(k1) + abs(k2)
    tangential_dist = abs(p1) + abs(p2)
    
    print(f" \nDistorsion radiale: {radial_dist:.4f}")
    print(f" Distorsion tangentielle: {tangential_dist:.4f}")
    
    if radial_dist < 0.1:
        print("  ✓ Distorsion radiale faible < 0.1 ")
    elif radial_dist < 0.3:
        print("  ⚠ Distorsion radiale modérée < 0.3")
    else:
        print("  ⚠ Distorsion radiale importante")

def test_distortion_correction(images_pattern, mtx, dist, camera_name):
    """Tester l'efficacité de la correction de distorsion"""
    print(f"\n=== ANALYSE CORRECTION DISTORSION {camera_name.upper()} ===")
    
    images = glob.glob(images_pattern)
    if not images:
        print("  ✗ Aucune image trouvée pour le test")
        return
    
    test_images = images[:3]  # Tester sur 3 images
    total_diff = 0
    processed_count = 0
    
    for img_path in test_images:
        img = cv2.imread(img_path)
        if img is None:
            continue
        
        h, w = img.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistorted = cv2.undistort(img, mtx, dist, None, new_mtx)
        
        # Calcul de la différence
        diff = cv2.absdiff(img, undistorted)
        mean_diff = np.mean(diff)
        
        
        total_diff += mean_diff
        processed_count += 1
    
    if processed_count > 0:
        avg_correction = total_diff / processed_count
        print(f" Impact moyen de correction: {avg_correction:.2f}")
        if avg_correction > 8:
            print("  Correction de distorsion efficace > 8")
        else:
            print("  Correction de distorsion limitée")
    print ('''=> Represente la moyenne de la différence d'intensité d'un meme pixel avant et apres la correction sur le lot d'image ''')