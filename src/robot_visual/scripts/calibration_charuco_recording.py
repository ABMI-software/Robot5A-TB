"""
Script d'enregistrement d'images de calibration pour deux caméras
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
        """Configurer les paramètres de la caméra pour obtenir l'image originale"""
        if not cap.isOpened():
            print(f"Erreur: Caméra {camera_id} non trouvée.")
            return False
        
        # Réinitialiser tous les paramètres aux valeurs par défaut
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Auto-exposition activée
        cap.set(cv2.CAP_PROP_AUTO_WB, 1)        # Balance des blancs automatique
        
        cap.set(cv2.CAP_PROP_FPS, 30) 
         
        max_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        max_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Caméra {camera_id} - Résolution détectée: {int(max_width)}x{int(max_height)}")
        
        
        
        # Vérifier la résolution effective après configuration
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Caméra {camera_id} - Résolution effective: {int(actual_width)}x{int(actual_height)}")
        
        return True
    
    def convert_to_grayscale_hq(self, frame):
        """Convertir en noir et blanc avec la meilleure qualité possible"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      
    def resize_for_display(self, frame, max_width=640):
        """Redimensionner uniquement pour l'affichage (pas pour la sauvegarde)"""
        height, width = frame.shape[:2]
        if width > max_width:
            ratio = max_width / width
            new_width = max_width
            new_height = int(height * ratio)
            return cv2.resize(frame, (new_width, new_height))
        return frame
    
    def record_calibration_images(self):
        """Enregistrer les images de calibration pour les deux caméras"""
        print("=== ENREGISTREMENT D'IMAGES DE CALIBRATION ===")
        
        # Obtenir les indices de départ
        camera_1_count = self.get_next_index(self.config['camera_1_path'], "camera_1")
        camera_2_count = self.get_next_index(self.config['camera_2_path'], "camera_2")
        
        print(f"Démarrage depuis l'index {camera_1_count} pour Caméra 1")
        print(f"Démarrage depuis l'index {camera_2_count} pour Caméra 2")
        
        # Ouvrir les caméras
        cap_1 = cv2.VideoCapture(self.config['camera_1_id'])
        cap_2 = cv2.VideoCapture(self.config['camera_2_id'])
        
        if not (self.setup_camera(cap_1, 1) and self.setup_camera(cap_2, 2)):
            return False
        
        print("\nCommandes:")
        print("- 's' : Sauvegarder une paire d'images synchronisées")
        print("- '1' : Sauvegarder image caméra 1 uniquement")
        print("- '2' : Sauvegarder image caméra 2 uniquement")
        print("- 'q' : Quitter")
        
        while True:
            ret_1, frame_1 = cap_1.read()
            ret_2, frame_2 = cap_2.read()
            
            if ret_1 and ret_2:
                # Redimensionner SEULEMENT pour l'affichage, pas pour la sauvegarde
                display_frame_1 = self.resize_for_display(frame_1, 640)
                display_frame_2 = self.resize_for_display(frame_2, 640)
                
                # Afficher les images côte à côte
                combined = np.hstack((display_frame_1, display_frame_2))
                cv2.imshow("Caméra 1 (gauche) | Caméra 2 (droite)", combined)
                
            elif ret_1:
                display_frame_1 = self.resize_for_display(frame_1, 640)
                cv2.imshow("Caméra 1", display_frame_1)
                print("Attention: Frame vide de la Caméra 2")
            elif ret_2:
                display_frame_2 = self.resize_for_display(frame_2, 640)
                cv2.imshow("Caméra 2", display_frame_2)
                print("Attention: Frame vide de la Caméra 1")
            else:
                print("Attention: Frames vides des deux caméras")
                continue
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s') and ret_1 and ret_2:  # Sauvegarder paire synchronisée
                filename_1 = os.path.join(self.config['camera_1_path'], f"camera_1_{camera_1_count:03d}.jpg")
                filename_2 = os.path.join(self.config['camera_2_path'], f"camera_2_{camera_2_count:03d}.jpg")
                
                # Sauvegarder les images ORIGINALES (pas redimensionnées)
                cv2.imwrite(filename_1, frame_1)
                cv2.imwrite(filename_2, frame_2)
                
                print(f"Paire sauvegardée: {filename_1} & {filename_2}")
                print(f"  - Caméra 1: {frame_1.shape[1]}x{frame_1.shape[0]}")
                print(f"  - Caméra 2: {frame_2.shape[1]}x{frame_2.shape[0]}")
                camera_1_count += 1
                camera_2_count += 1
                
            elif key == ord('1') and ret_1:  # Sauvegarder caméra 1 uniquement
                filename_1 = os.path.join(self.config['camera_1_path'], f"camera_1_{camera_1_count:03d}.jpg")
                cv2.imwrite(filename_1, frame_1)
                print(f"Sauvegardé: {filename_1} ({frame_1.shape[1]}x{frame_1.shape[0]})")
                camera_1_count += 1
                
            elif key == ord('2') and ret_2:  # Sauvegarder caméra 2 uniquement
                filename_2 = os.path.join(self.config['camera_2_path'], f"camera_2_{camera_2_count:03d}.jpg")
                cv2.imwrite(filename_2, frame_2)
                print(f"Sauvegardé: {filename_2} ({frame_2.shape[1]}x{frame_2.shape[0]})")
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

"""
# TESTS
# Pour tester ce script:
# 1. Connectez deux caméras aux IDs spécifiés dans config
# 2. Lancez: python calibration_charuco_recording.py
# 3. Vérifiez que les fenêtres de caméras s'ouvrent avec les bonnes résolutions
# 4. Testez les commandes clavier ('s', '1', '2', 'q')
# 5. Vérifiez que les images sauvegardées ont la résolution originale de vos caméras
# 6. Vérifiez les messages de résolution affichés dans la console
"""