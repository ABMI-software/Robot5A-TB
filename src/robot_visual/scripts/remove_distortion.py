#!/usr/bin/env python3
"""
Module de correction de distorsion pour les caméras calibrées
"""
import numpy as np
import cv2
import glob
import yaml
import os

def load_camera_calibration(calibration_file):
    """Charger les paramètres de calibration depuis un fichier YAML"""
    try:
        with open(calibration_file, 'r') as file:
            data = yaml.safe_load(file)
        
        camera_matrix = np.array(data['camera_matrix'])
        dist_coeffs = np.array(data['distortion_coefficients'])
        return camera_matrix, dist_coeffs, True
    except Exception as e:
        print(f"  ✗ Erreur chargement calibration: {e}")
        return None, None, False

def remove_distortion_from_image(img, camera_matrix, dist_coeffs):
    """Corriger la distorsion d'une image"""
    if img is None:
        return None
    
    height, width = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (width, height), 1, (width, height)
    )
    
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted_img

def process_images_in_directory(input_dir, output_dir, camera_matrix, dist_coeffs):
    """Traiter toutes les images d'un répertoire pour corriger la distorsion"""
    os.makedirs(output_dir, exist_ok=True)
    
    extensions = ['*.jpg']
    image_files = []
    
    for ext in extensions:
        image_files.extend(glob.glob(os.path.join(input_dir, ext)))
    
    if not image_files:
        print(f"  ⚠ Aucune image trouvée dans {input_dir}")
        return 0
    
    print(f" Correction de distorsion sur {len(image_files)} image(s)...")
    
    processed_count = 0
    for i, img_path in enumerate(image_files):
        img = cv2.imread(img_path)
        if img is None:
            continue
        
        undistorted_img = remove_distortion_from_image(img, camera_matrix, dist_coeffs)
        if undistorted_img is None:
            continue
        
        filename = os.path.basename(img_path)
        name, ext = os.path.splitext(filename)
        output_path = os.path.join(output_dir, f"{name}_undistorted{ext}")
        
        if cv2.imwrite(output_path, undistorted_img):
            processed_count += 1
            print(f"  ✓ [{i+1}/{len(image_files)}] {filename}")
    
    return processed_count

def correct_distortion_for_camera(input_dir, calibration_file, camera_name):
    """Corriger la distorsion pour une caméra"""
    print(f"\n=== CORRECTION DISTORSION {camera_name.upper()} ===")
    
    if not os.path.exists(calibration_file):
        print(f"   Fichier de calibration non trouvé: {calibration_file}")
        return False
    
    camera_matrix, dist_coeffs, success = load_camera_calibration(calibration_file)
    if not success:
        return False
    
    output_dir = f"{input_dir}_undistorted"
    print(f"   Source: {input_dir}")
    print(f"   Destination: {output_dir}")
    
    processed_count = process_images_in_directory(input_dir, output_dir, camera_matrix, dist_coeffs)
    
    if processed_count > 0:
        print(f"  ✅ Correction terminée: {processed_count} image(s) traitée(s)")
        return True
    else:
        print(f"  ⚠ Aucune image n'a pu être traitée")
        return False