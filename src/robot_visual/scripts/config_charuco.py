#!/usr/bin/env python3
"""
Configuration centralisée pour tous les scripts de calibration
"""

# Configuration principale pour tous les scripts
CONFIG = {
    # IDs des caméras (à ajuster selon votre système)
    'camera_1_id': 2,
    'camera_2_id': 1,
    
    # Chemins des répertoires d'images
    'camera_1_path': "/home/chloe/Robot5A-TB/src/robot_visual/config/camera_1_images_charuco",
    'camera_2_path': "/home/chloe/Robot5A-TB/src/robot_visual/config/camera_2_images_charuco",
    'camera_1_stereo_path': "/home/chloe/Robot5A-TB/src/robot_visual/config/images_stereo_charuco/camera_1",
    'camera_2_stereo_path': "/home/chloe/Robot5A-TB/src/robot_visual/config/images_stereo_charuco/camera_2",
    
    # Fichiers de calibration
    'camera_1_calib': "/home/chloe/Robot5A-TB/src/robot_visual/config/camera_1_calibration.yaml",
    'camera_2_calib': "/home/chloe/Robot5A-TB/src/robot_visual/config/camera_2_calibration.yaml",
    'stereo_calib': "/home/chloe/Robot5A-TB/src/robot_visual/config/stereo_calibration.yaml",
    
    'output_undistorted_base_dir': 'undistorted_images',
    
    # Paramètres du plateau Charuco
    'squares_x': 8,        # Nombre de carrés en X
    'squares_y': 6,        # Nombre de carrés en Y
    'square_size': 0.04,   # Taille des carrés en mètres
    'marker_size': 0.03,   # Taille des marqueurs ArUco en mètres
}

def get_config():
    """Retourner la configuration"""
    return CONFIG

def update_camera_ids(cam1_id, cam2_id):
    """Mettre à jour les IDs des caméras"""
    CONFIG['camera_1_id'] = cam1_id
    CONFIG['camera_2_id'] = cam2_id

def update_paths(base_path):
    """Mettre à jour les chemins avec un nouveau répertoire de base"""
    CONFIG['camera_1_path'] = f"{base_path}/camera_1_images_charuco"
    CONFIG['camera_2_path'] = f"{base_path}/camera_2_images_charuco"
    CONFIG['camera_1_calib'] = f"{base_path}/camera_1_calibration.yaml"
    CONFIG['camera_2_calib'] = f"{base_path}/camera_2_calibration.yaml"
    CONFIG['stereo_calib'] = f"{base_path}/stereo_calibration.yaml"

def update_charuco_params(squares_x, squares_y, square_size, marker_size):
    """Mettre à jour les paramètres du plateau Charuco"""
    CONFIG['squares_x'] = squares_x
    CONFIG['squares_y'] = squares_y
    CONFIG['square_size'] = square_size
    CONFIG['marker_size'] = marker_size



"""if __name__ == "__main__":
    print_config()"""
