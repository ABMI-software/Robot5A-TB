"""
Script d'enregistrement d'images de calibration pour une seule caméra
Modifié pour: une seule caméra, sans rotation, images noir et blanc, résolution maximale
"""

import cv2 
import numpy as np
import os

class ImageRecorder:
    def __init__(self, camera_id=0, output_path="calibration_images"):
        self.camera_id = camera_id
        self.output_path = output_path
        self.setup_directories()
        
    def setup_directories(self):
        """Créer le répertoire nécessaire"""
        os.makedirs(self.output_path, exist_ok=True)
    
    def get_next_index(self, folder_path, prefix):
        """Obtenir le prochain index disponible pour les images"""
        files = [f for f in os.listdir(folder_path) if f.startswith(prefix) and f.endswith(".jpg")]
        if not files:
            return 0
        indices = [int(f.split('_')[-1].split('.')[0]) for f in files]
        return max(indices) + 1
    
    def setup_camera(self, cap):
        """Configurer les paramètres de la caméra pour obtenir la meilleure qualité possible"""
        if not cap.isOpened():
            print(f"Erreur: Caméra {self.camera_id} non trouvée.")
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
            (1280, 720),   # HD
            (640, 480),    # VGA
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
        
        print(f"Caméra {self.camera_id} - Résolution configurée: {best_resolution[0]}x{best_resolution[1]}")
        
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
        """Enregistrer les images de calibration pour une caméra"""
        print("=== ENREGISTREMENT D'IMAGES DE CALIBRATION (NOIR ET BLANC) ===")
        
        # Obtenir l'indice de départ
        image_count = self.get_next_index(self.output_path, "image")
        
        print(f"Démarrage depuis l'index {image_count}")
        print(f"Dossier de sauvegarde: {self.output_path}")
        
        # Ouvrir la caméra
        cap = cv2.VideoCapture(self.camera_id)
        
        if not self.setup_camera(cap):
            cap.release()
            return False
        
        print("\nCommandes:")
        print("- 's' ou 'ESPACE' : Sauvegarder une image")
        print("- 'q' ou 'ESC' : Quitter")
        print("\nAffichage redimensionné, sauvegarde en taille réelle")
        
        while True:
            ret, frame = cap.read()
            
            if ret:
                # Convertir en niveaux de gris
                gray_frame = self.convert_to_grayscale(frame)
                
                # Redimensionner pour l'affichage seulement
                display_frame = self.resize_for_display(gray_frame, 800)
                
                # Afficher l'image
                cv2.imshow("Caméra - Appuyez sur 's' ou ESPACE pour sauvegarder", display_frame)
                
            else:
                print("Attention: Frame vide de la caméra")
                continue
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s') or key == ord(' '):  # Sauvegarder image (s ou espace)
                if ret:
                    filename = os.path.join(self.output_path, f"image_{image_count:03d}.jpg")
                    
                    # Convertir en niveaux de gris pour la sauvegarde
                    gray_save = self.convert_to_grayscale(frame)
                    
                    # Sauvegarder en taille réelle avec haute qualité
                    cv2.imwrite(filename, gray_save, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    
                    print(f"✓ Image sauvegardée: {filename} ({gray_save.shape[1]}x{gray_save.shape[0]})")
                    image_count += 1
                
            elif key == ord('q') or key == 27:  # Quitter (q ou ESC)
                print("Arrêt de l'enregistrement...")
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
        print(f"\nEnregistrement terminé. {image_count} images sauvegardées dans '{self.output_path}'")
        return True

def main():
    # Configuration par défaut
    camera_id = 0  # ID de la caméra (0 pour la première caméra)
    output_path = "calibration_images"  # Dossier de sortie
    
    # Vous pouvez modifier ces valeurs selon vos besoins
    print("=== CONFIGURATION ===")
    print(f"Caméra utilisée: {camera_id}")
    print(f"Dossier de sortie: {output_path}")
    print("=" * 20)
    
    recorder = ImageRecorder(camera_id, output_path)
    recorder.record_calibration_images()

if __name__ == "__main__":
    main()