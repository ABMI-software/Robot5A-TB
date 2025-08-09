import cv2
import numpy as np
import os
import yaml
import time
from config_charuco import get_config

def load_calibration(calib_file):
    """Charge les paramètres de calibration depuis un fichier YAML."""
    try:
        with open(calib_file, 'r') as file:
            data = yaml.safe_load(file)
        camera_matrix = np.array(data['camera_matrix'])
        dist_coeffs = np.array(data['distortion_coefficients'])
        img_size = tuple(data['image_size'])
        return camera_matrix, dist_coeffs, img_size
    except Exception as e:
        print(f"Erreur lors du chargement du fichier de calibration {calib_file}: {e}")
        return None, None, None

def undistort_image(img, camera_matrix, dist_coeffs):
    """Corrige la distorsion d'une image avec qualité optimale."""
    if camera_matrix is None or dist_coeffs is None:
        return None
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted_img

def get_next_index(folder_path, prefix):
    """Retourne le prochain index disponible pour nommer les images."""
    if not os.path.exists(folder_path):
        return 0
    files = [f for f in os.listdir(folder_path) if f.startswith(prefix) and f.endswith(".jpg")]
    if not files:
        return 0
    indices = [int(f.split('_')[-1].split('.')[0]) for f in files]
    return max(indices) + 1

def resize_for_display(frame, max_width=800):
    """Redimensionner pour l'affichage avec qualité optimale (pas pour la sauvegarde)"""
    height, width = frame.shape[:2]
    if width > max_width:
        ratio = max_width / width
        new_width = max_width
        new_height = int(height * ratio)
        # Utiliser INTER_AREA pour un meilleur redimensionnement
        return cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)
    return frame

def test_camera_connection(camera_id):
    """Tester si une caméra est disponible"""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        cap.release()
        return False
    
    # Essayer de lire une frame
    ret, frame = cap.read()
    cap.release()
    return ret and frame is not None

def setup_camera(cap, camera_id, target_size):
    """Configurer les paramètres de la caméra pour obtenir la meilleure qualité possible"""
    if not cap.isOpened():
        print(f"Erreur: Caméra {camera_id} non trouvée.")
        return False
    
    # Configuration avancée pour qualité maximale
    try:
        # Format de compression haute qualité
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Paramètres d'exposition et de gain
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Auto-exposition
        cap.set(cv2.CAP_PROP_AUTO_WB, 1)        # Balance des blancs automatique
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)      # Autofocus si disponible
        
        # Paramètres de qualité
        cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)     # Luminosité neutre
        cap.set(cv2.CAP_PROP_CONTRAST, 32)      # Contraste modéré
        cap.set(cv2.CAP_PROP_SATURATION, 32)    # Saturation normale
        cap.set(cv2.CAP_PROP_SHARPNESS, 32)     # Netteté améliorée
        
    except Exception as e:
        print(f"Certains paramètres avancés non supportés sur caméra {camera_id}: {e}")
    
    # Définir la résolution cible
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_size[1])
    
    # Laisser le temps à la caméra de s'ajuster
    time.sleep(0.1)
    
    # Vérifier la résolution obtenue
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    if (actual_w, actual_h) != target_size:
        print(f"Avertissement: Caméra {camera_id} résolution {actual_w}x{actual_h}, attendue {target_size}")
    else:
        print(f"Caméra {camera_id} - Résolution: {actual_w}x{actual_h}")
    
    # Optimiser le framerate
    cap.set(cv2.CAP_PROP_FPS, 30)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Caméra {camera_id} - FPS: {actual_fps}")
    
    # Vérifier quelques frames pour stabiliser la caméra
    for _ in range(5):
        ret, frame = cap.read()
        if not ret:
            print(f"Attention: Problème de lecture sur caméra {camera_id}")
            return False
    
    return True

def main():
    config = get_config()
    
    print("=== ENREGISTREMENT D'IMAGES STÉRÉO CORRIGÉES HAUTE QUALITÉ ===")

    # Charger les paramètres de calibration pour les deux caméras
    cam1_matrix, cam1_dist, cam1_size = load_calibration(config['camera_1_calib'])
    cam2_matrix, cam2_dist, cam2_size = load_calibration(config['camera_2_calib'])
    
    if cam1_matrix is None:
        print(f"Échec du chargement des paramètres de calibration pour caméra 1: {config['camera_1_calib']}")
    if cam2_matrix is None:
        print(f"Échec du chargement des paramètres de calibration pour caméra 2: {config['camera_2_calib']}")
    
    if cam1_matrix is None and cam2_matrix is None:
        print("ERREUR: Aucun paramètre de calibration valide trouvé!")
        return

    # Tester les caméras disponibles
    camera_1_available = test_camera_connection(config['camera_1_id'])
    camera_2_available = test_camera_connection(config['camera_2_id'])
    
    if not camera_1_available and not camera_2_available:
        print("ERREUR: Aucune caméra détectée!")
        print("Vérifiez les connexions USB et les IDs de caméra dans config_charuco.py")
        return
    
    if not camera_1_available:
        print(f"ATTENTION: Caméra 1 (ID {config['camera_1_id']}) non disponible")
    if not camera_2_available:
        print(f"ATTENTION: Caméra 2 (ID {config['camera_2_id']}) non disponible")

    # Initialiser les caméras disponibles
    cap1, cap2 = None, None
    
    if camera_1_available and cam1_matrix is not None:
        cap1 = cv2.VideoCapture(config['camera_1_id'])
        if not setup_camera(cap1, 1, cam1_size):
            cap1.release()
            cap1 = None
            camera_1_available = False
    
    if camera_2_available and cam2_matrix is not None:
        cap2 = cv2.VideoCapture(config['camera_2_id'])
        # Utiliser la même résolution que cam1 pour cohérence
        target_size = cam2_size if cam2_size else cam1_size
        if not setup_camera(cap2, 2, target_size):
            cap2.release()
            cap2 = None
            camera_2_available = False

    if not camera_1_available and not camera_2_available:
        print("ERREUR: Impossible d'initialiser les caméras")
        return

    # Créer les répertoires de sauvegarde s'ils n'existent pas
    os.makedirs(config['camera_1_stereo_path'], exist_ok=True)
    os.makedirs(config['camera_2_stereo_path'], exist_ok=True)

    # Obtenir les prochains indices pour les fichiers
    cam1_count = get_next_index(config['camera_1_stereo_path'], "camera_1") if camera_1_available else 0
    cam2_count = get_next_index(config['camera_2_stereo_path'], "camera_2") if camera_2_available else 0

    print("\nCommandes:")
    if camera_1_available and camera_2_available:
        print("- 's' : Sauvegarder une paire d'images corrigées synchronisées")
    if camera_1_available:
        print("- '1' : Sauvegarder image caméra 1 corrigée uniquement")
    if camera_2_available:
        print("- '2' : Sauvegarder image caméra 2 corrigée uniquement")
    print("- 'q' : Quitter")
    print("\nQualité maximale - Correction de distorsion active")

    while True:
        ret1, frame1 = (False, None)
        ret2, frame2 = (False, None)
        
        if cap1:
            ret1, frame1 = cap1.read()
        if cap2:
            ret2, frame2 = cap2.read()

        # Variables pour les images corrigées
        undistorted1, undistorted2 = None, None

        # Traitement caméra 1
        if ret1 and frame1 is not None and cam1_matrix is not None:
            # Convertir en niveaux de gris
            gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            # Corriger la distorsion
            undistorted1 = undistort_image(gray1, cam1_matrix, cam1_dist)

        # Traitement caméra 2
        if ret2 and frame2 is not None and cam2_matrix is not None:
            # Convertir en niveaux de gris
            gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            # Corriger la distorsion
            undistorted2 = undistort_image(gray2, cam2_matrix, cam2_dist)

        # Affichage
        display_frames = []
        labels = []

        if undistorted1 is not None:
            display1 = resize_for_display(undistorted1, max_width=500)
            display_frames.append(display1)
            labels.append("Caméra 1 (Corrigée)")

        if undistorted2 is not None:
            display2 = resize_for_display(undistorted2, max_width=500)
            display_frames.append(display2)
            labels.append("Caméra 2 (Corrigée)")

        # Afficher les images
        if len(display_frames) == 2:
            # Ajuster les tailles pour l'affichage côte à côte
            h1, w1 = display_frames[0].shape[:2]
            h2, w2 = display_frames[1].shape[:2]
            
            if h1 != h2:
                # Redimensionner pour avoir la même hauteur
                target_height = min(h1, h2)
                if h1 > target_height:
                    new_w1 = int(w1 * target_height / h1)
                    display_frames[0] = cv2.resize(display_frames[0], (new_w1, target_height), interpolation=cv2.INTER_AREA)
                if h2 > target_height:
                    new_w2 = int(w2 * target_height / h2)
                    display_frames[1] = cv2.resize(display_frames[1], (new_w2, target_height), interpolation=cv2.INTER_AREA)
            
            combined = np.hstack(display_frames)
            cv2.imshow("Caméra 1 (Corrigée) | Caméra 2 (Corrigée)", combined)
        elif len(display_frames) == 1:
            cv2.imshow(labels[0], display_frames[0])

        # Attendre une entrée clavier
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('s') and undistorted1 is not None and undistorted2 is not None:
            # Sauvegarder les images corrigées à leur taille d'origine avec qualité maximale
            filename1 = os.path.join(config['camera_1_stereo_path'], f"camera_1_{cam1_count:03d}.jpg")
            filename2 = os.path.join(config['camera_2_stereo_path'], f"camera_2_{cam2_count:03d}.jpg")
            
            cv2.imwrite(filename1, undistorted1, [cv2.IMWRITE_JPEG_QUALITY, 98])
            cv2.imwrite(filename2, undistorted2, [cv2.IMWRITE_JPEG_QUALITY, 98])
            
            print(f"✓ Paire corrigée sauvegardée:")
            print(f"  - Caméra 1: {filename1} ({undistorted1.shape[1]}x{undistorted1.shape[0]})")
            print(f"  - Caméra 2: {filename2} ({undistorted2.shape[1]}x{undistorted2.shape[0]})")
            cam1_count += 1
            cam2_count += 1
            
        elif key == ord('1') and undistorted1 is not None:
            # Sauvegarder caméra 1 uniquement
            filename1 = os.path.join(config['camera_1_stereo_path'], f"camera_1_{cam1_count:03d}.jpg")
            cv2.imwrite(filename1, undistorted1, [cv2.IMWRITE_JPEG_QUALITY, 98])
            print(f"✓ Caméra 1 corrigée sauvegardée: {filename1} ({undistorted1.shape[1]}x{undistorted1.shape[0]})")
            cam1_count += 1
            
        elif key == ord('2') and undistorted2 is not None:
            # Sauvegarder caméra 2 uniquement
            filename2 = os.path.join(config['camera_2_stereo_path'], f"camera_2_{cam2_count:03d}.jpg")
            cv2.imwrite(filename2, undistorted2, [cv2.IMWRITE_JPEG_QUALITY, 98])
            print(f"✓ Caméra 2 corrigée sauvegardée: {filename2} ({undistorted2.shape[1]}x{undistorted2.shape[0]})")
            cam2_count += 1
            
        elif key == ord('q'):
            print("Arrêt de l'enregistrement.")
            break

    # Libérer les ressources
    if cap1:
        cap1.release()
    if cap2:
        cap2.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()