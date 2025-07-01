import cv2
import numpy as np
import os
import yaml
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
    """Corrige la distorsion d'une image."""
    if camera_matrix is None or dist_coeffs is None:
        return None
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted_img

def get_next_index(folder_path, prefix):
    """Retourne le prochain index disponible pour nommer les images."""
    files = [f for f in os.listdir(folder_path) if f.startswith(prefix) and f.endswith(".jpg")]
    if not files:
        return 0
    indices = [int(f.split('_')[-1].split('.')[0]) for f in files]
    return max(indices) + 1

def resize_for_display(frame, max_width=800):
    """Redimensionner pour l'affichage (pas pour la sauvegarde)"""
    height, width = frame.shape[:2]
    if width > max_width:
        ratio = max_width / width
        new_width = max_width
        new_height = int(height * ratio)
        return cv2.resize(frame, (new_width, new_height))
    return frame

def main():
    config = get_config()

    # Charger les paramètres de calibration pour les deux caméras
    cam1_matrix, cam1_dist, cam1_size = load_calibration(config['camera_1_calib'])
    cam2_matrix, cam2_dist, cam2_size = load_calibration(config['camera_2_calib'])
    if cam1_matrix is None or cam2_matrix is None:
        print("Échec du chargement des paramètres de calibration pour une ou deux caméras.")
        return

    # Initialiser les caméras
    cap1 = cv2.VideoCapture(config['camera_1_id'])
    cap2 = cv2.VideoCapture(config['camera_2_id'])
    if not cap1.isOpened() or not cap2.isOpened():
        print("Erreur : Impossible d'ouvrir une ou deux caméras.")
        return

    # Définir les résolutions des caméras
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, cam1_size[0])
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, cam1_size[1])
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, cam1_size[0])  # Avant rotation, même résolution que cam1
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, cam1_size[1])

    # Vérifier les résolutions
    actual_w1 = int(cap1.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h1 = int(cap1.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_w2 = int(cap2.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h2 = int(cap2.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (actual_w1, actual_h1) != cam1_size:
        print(f"Avertissement : Résolution caméra 1 définie à {actual_w1}x{actual_h1}, attendue {cam1_size}")
    if (actual_w2, actual_h2) != (cam1_size[0], cam1_size[1]):
        print(f"Avertissement : Résolution caméra 2 définie à {actual_w2}x{actual_h2}, attendue {cam1_size}")

    # Créer les répertoires de sauvegarde s'ils n'existent pas
    os.makedirs(config['camera_1_stereo_path'], exist_ok=True)
    os.makedirs(config['camera_2_stereo_path'], exist_ok=True)

    # Obtenir les prochains indices pour les fichiers
    cam1_count = get_next_index(config['camera_1_stereo_path'], "camera_1")
    cam2_count = get_next_index(config['camera_2_stereo_path'], "camera_2")

    print("Démarrage de l'enregistrement des images corrigées.")
    print("Appuyez sur 's' pour sauvegarder les images corrigées simultanément.")
    print("Appuyez sur 'q' pour quitter.")

    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        if ret1 and ret2:
            # Convertir en niveaux de gris
            gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

            # Appliquer les rotations
            rotated_gray1 = cv2.rotate(gray1, cv2.ROTATE_180)  # Caméra 1 : rotation 180°
            rotated_gray2 = cv2.rotate(gray2, cv2.ROTATE_90_COUNTERCLOCKWISE)  # Caméra 2 : rotation 90°

            # Corriger la distorsion
            undistorted1 = undistort_image(rotated_gray1, cam1_matrix, cam1_dist)
            undistorted2 = undistort_image(rotated_gray2, cam2_matrix, cam2_dist)

            if undistorted1 is not None and undistorted2 is not None:
                # Redimensionner pour l'affichage
                display1 = resize_for_display(undistorted1, max_width=500)
                display2 = resize_for_display(undistorted2, max_width=500)

                # Afficher les images redimensionnées
                cv2.imshow("Caméra 1 (Corrigée)", display1)
                cv2.imshow("Caméra 2 (Corrigée)", display2)

        # Attendre une entrée clavier
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and ret1 and ret2 and undistorted1 is not None and undistorted2 is not None:
            # Sauvegarder les images corrigées à leur taille d'origine
            filename1 = os.path.join(config['camera_1_stereo_path'], f"camera_1_{cam1_count:03d}.jpg")
            filename2 = os.path.join(config['camera_2_stereo_path'], f"camera_2_{cam2_count:03d}.jpg")
            cv2.imwrite(filename1, undistorted1)
            cv2.imwrite(filename2, undistorted2)
            print(f"Sauvegardé : {filename1} et {filename2}")
            cam1_count += 1
            cam2_count += 1
        elif key == ord('q'):
            print("Arrêt de l'enregistrement.")
            break

    # Libérer les ressources
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()