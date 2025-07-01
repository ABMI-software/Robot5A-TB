"""
Script d'enregistrement d'images de calibration pour deux caméras
Modifié pour: caméra 1 à gauche, caméra 2 à droite, images noir et blanc, résolution maximale
"""

import cv2 
import numpy as np
import os

from config_charuco import get_config

class ImageRecorder:
    def __init__(self, config):
        self.config = config
        self.setup_directories()
        
    def setup_directories(self):
        """Créer les répertoires nécessaires"""
        for path in [self.config['camera_1_path'], self.config['camera_2_path']]:
            os.makedirs(path, exist_ok=True)
    
    def get_next_index(self, folder_path, prefix):
        """Obtenir le prochain index disponible pour les images"""
        files = [f for f in os.listdir(folder_path) if f.startswith(prefix) and f.endswith(".jpg")]
        if not files:
            return 0
        indices = [int(f.split('_')[-1].split('.')[0]) for f in files]
        return max(indices) + 1
    
    def setup_camera(self, cap, camera_id):
        """Configurer les paramètres de la caméra pour obtenir la meilleure qualité possible"""
        if not cap.isOpened():
            print(f"Erreur: Caméra {camera_id} non trouvée.")
            return False
        
        # Désactiver les ajustements automatiques pour une meilleure qualité
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Auto-exposition activée
        cap.set(cv2.CAP_PROP_AUTO_WB, 1)        # Balance des blancs automatique
        
        # Essayer de définir la résolution maximale possible
        # Tester plusieurs résolutions courantes par ordre décroissant
        resolutions_to_try = [
            (3840, 2160),  # 4K
            (2560, 1440),  # 2K
            (1920, 1080),  # Full HD
            
        ]
        
        best_resolution = None
        for width, height in resolutions_to_try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # Vérifier si la résolution a été acceptée
            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            if actual_width >= width * 0.9 and actual_height >= height * 0.9:  # Tolérance de 10%
                best_resolution = (actual_width, actual_height)
                break
        
        # Si aucune résolution spécifique n'a fonctionné, utiliser celle par défaut
        if best_resolution is None:
            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            best_resolution = (actual_width, actual_height)
        
        print(f"Caméra {camera_id} - Résolution configurée: {best_resolution[0]}x{best_resolution[1]}")
        
        # Paramètres pour améliorer la qualité
        cap.set(cv2.CAP_PROP_FPS, 30)  # 30 FPS
        
        # Essayer d'améliorer la qualité d'image si supporté
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        except:
            pass  # Ignorer si non supporté
        
        return True
    
    def convert_to_grayscale(self, frame):
        """Convertir une frame BGR en niveaux de gris"""
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
     
   
    
    def resize_for_display(self, frame, max_width=800):
        """Redimensionner pour l'affichage (pas pour la sauvegarde)"""
        height, width = frame.shape[:2]
        if width > max_width:
            ratio = max_width / width
            new_width = max_width
            new_height = int(height * ratio)
            return cv2.resize(frame, (new_width, new_height))
        return frame
    
    def record_calibration_images(self):
        """Enregistrer les images de calibration pour les deux caméras"""
        print("=== ENREGISTREMENT D'IMAGES DE CALIBRATION (NOIR ET BLANC) ===")
        
        # Obtenir les indices de départ
        camera_1_count = self.get_next_index(self.config['camera_1_path'], "camera_1")
        camera_2_count = self.get_next_index(self.config['camera_2_path'], "camera_2")
        
        print(f"Démarrage depuis l'index {camera_1_count} pour Caméra 1 (GAUCHE)")
        print(f"Démarrage depuis l'index {camera_2_count} pour Caméra 2 (DROITE)")
        
        # Ouvrir les caméras
        cap_1 = cv2.VideoCapture(self.config['camera_1_id'])
        cap_2 = cv2.VideoCapture(self.config['camera_2_id'])
        
        if not (self.setup_camera(cap_1, 1) and self.setup_camera(cap_2, 2)):
            cap_1.release()
            cap_2.release()
            return False
        
        print("\nCommandes:")
        print("- 's' : Sauvegarder une paire d'images synchronisées")
        print("- '1' : Sauvegarder image caméra 1 (GAUCHE pivotée 180°) uniquement")
        print("- '2' : Sauvegarder image caméra 2 (DROITE) uniquement")
        print("- 'q' : Quitter")
        print("\nCaméra 1 pivotée de 180° - Affichage redimensionné, sauvegarde taille réelle")
        
        while True:
            ret_1, frame_1 = cap_1.read()
            ret_2, frame_2 = cap_2.read()
            
            if ret_1 and ret_2:
                # Convertir en niveaux de gris
                gray_frame_1 = self.convert_to_grayscale(frame_1)
                gray_frame_2 = self.convert_to_grayscale(frame_2)
                
                
                # Redimensionner pour l'affichage seulement
                display_frame_1 = self.resize_for_display(gray_frame_1, 640)
                display_frame_2 = self.resize_for_display(gray_frame_2, 640)
                
                # Afficher les images côte à côte
                try:
                    combined = np.hstack((display_frame_1, display_frame_2))
                    cv2.imshow("Caméra 1 (GAUCHE pivotée 180°) | Caméra 2 (DROITE)", combined)
                except:
                    cv2.imshow("Caméra 1 (GAUCHE pivotée 180°)", display_frame_1)
                    cv2.imshow("Caméra 2 (DROITE)", display_frame_2)
                
            elif ret_1:
                gray_frame_1 = self.convert_to_grayscale(frame_1)
                display_frame_1 = self.resize_for_display(gray_frame_1, 640)
                cv2.imshow("Caméra 1 (GAUCHE pivotée 180°)", display_frame_1)
                print("Attention: Frame vide de la Caméra 2 (DROITE)")
            elif ret_2:
                gray_frame_2 = self.convert_to_grayscale(frame_2)
                display_frame_2 = self.resize_to_fit_display(gray_frame_2, 640)
                cv2.imshow("Caméra 2 (DROITE)", display_frame_2)
                print("Attention: Frame vide de la Caméra 1 (GAUCHE)")
            else:
                print("Attention: Frames vides des deux caméras")
                continue
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s') and ret_1 and ret_2:  # Sauvegarder paire synchronisée
                filename_1 = os.path.join(self.config['camera_1_path'], f"camera_1_{camera_1_count:03d}.jpg")
                filename_2 = os.path.join(self.config['camera_2_path'], f"camera_2_{camera_2_count:03d}.jpg")
                
                # Convertir en niveaux de gris
                gray_save_1 = self.convert_to_grayscale(frame_1)
                gray_save_2 = self.convert_to_grayscale(frame_2)
                
                
                # Sauvegarder en taille réelle
                cv2.imwrite(filename_1, gray_save_1)
                cv2.imwrite(filename_2, gray_save_2)
                
                print(f"Paire N&B sauvegardée:")
                print(f"  - GAUCHE (180°): {filename_1} ({gray_save_1.shape[1]}x{gray_save_1.shape[0]})")
                print(f"  - DROITE (180°): {filename_2} ({gray_save_2.shape[1]}x{gray_save_2.shape[0]})")
                camera_1_count += 1
                camera_2_count += 1
                
            elif key == ord('1') and ret_1:  # Sauvegarder caméra 1 (GAUCHE) uniquement
                filename_1 = os.path.join(self.config['camera_1_path'], f"camera_1_{camera_1_count:03d}.jpg")
                gray_save_1 = self.convert_to_grayscale(frame_1)
                cv2.imwrite(filename_1, gray_save_1)
                print(f"Sauvegardé GAUCHE (pivotée): {filename_1} ({gray_save_1.shape[1]}x{gray_save_1.shape[0]})")
                camera_1_count += 1
                
            elif key == ord('2') and ret_2:
                filename_2 = os.path.join(self.config['camera_2_path'], f"camera_2_{camera_2_count:03d}.jpg")
                gray_save_2 = self.convert_to_grayscale(frame_2)
                cv2.imwrite(filename_2, gray_save_2, [cv2.IMWRITE_JPEG_QUALITY, 95])
                print(f"✓ DROITE sauvegardée: {filename_2} ({gray_save_2.shape[1]}x{gray_save_2.shape[0]})")
                camera_2_count += 1
                
            elif key == ord('q'):
                print("Arrêt de l'enregistrement...")
                break
        
        cap_1.release()
        cap_2.release()
        cv2.destroyAllWindows()
        return True

def main():
    config = get_config()
    recorder = ImageRecorder(config)
    recorder.record_calibration_images()

if __name__ == "__main__":
    main()

