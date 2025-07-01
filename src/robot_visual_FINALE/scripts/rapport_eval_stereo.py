#!/usr/bin/env python3
"""
Évaluation de la synchronisation des caméras stéréo avec marqueur ArUco
Mesure les écarts de position, rotation et dimension pour évaluer la qualité stéréo
"""

import cv2
import numpy as np
import yaml
import time
from config_charuco import get_config

class StereoSyncEvaluator:
    def __init__(self):
        self.cfg = get_config()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        
        # Paramètres optimisés pour détection en noir et blanc
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 7
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.005
        
        # Charger les calibrations
        self.mtx1, self.dist1 = self.load_camera_calibration(self.cfg['camera_1_calib'])
        self.mtx2, self.dist2 = self.load_camera_calibration(self.cfg['camera_2_calib'])
        self.stereo_params = self.load_stereo_calibration(self.cfg['stereo_calib'])
        
        # Statistiques d'évaluation
        self.stats = {
            'position_errors': [],
            'rotation_errors': [],
            'size_errors': [],
            'detection_sync': 0,
            'detection_total': 0
        }
    
    def load_camera_calibration(self, calib_file):
        """Charge les paramètres de calibration d'une caméra"""
        try:
            with open(calib_file, 'r') as f:
                data = yaml.safe_load(f)
            return np.array(data['camera_matrix']), np.array(data['distortion_coefficients'])
        except Exception as e:
            print(f"Erreur chargement {calib_file}: {e}")
            return None, None
    
    def load_stereo_calibration(self, stereo_file):
        """Charge les paramètres de calibration stéréo"""
        try:
            with open(stereo_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Erreur chargement {stereo_file}: {e}")
            return None
    
    def detect_aruco_pose(self, img, camera_matrix, dist_coeffs, marker_size=0.03):
        """Détecte et calcule la pose d'un marqueur ArUco en noir et blanc"""
        # Conversion en niveau de gris avec amélioration du contraste
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        
        # Amélioration du contraste pour une meilleure détection
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        # Détection des marqueurs avec paramètres optimisés pour B&W
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None and len(ids) > 0:
            # Prendre le premier marqueur détecté
            corner = corners[0]
            
            # Estimation de la pose
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, marker_size, camera_matrix, dist_coeffs
            )
            
            # Calculer la taille apparente du marqueur
            marker_corners = corner[0]
            side_lengths = [
                np.linalg.norm(marker_corners[1] - marker_corners[0]),
                np.linalg.norm(marker_corners[2] - marker_corners[1]),
                np.linalg.norm(marker_corners[3] - marker_corners[2]),
                np.linalg.norm(marker_corners[0] - marker_corners[3])
            ]
            apparent_size = np.mean(side_lengths)
            measured_distance = np.linalg.norm(tvec[0][0])
            
            return {
                'detected': True,
                'id': ids[0][0],
                'rvec': rvec[0][0],
                'tvec': tvec[0][0],
                'corners': corner[0],
                'apparent_size': apparent_size,
                'distance': measured_distance,
                'gray_image': gray  # Retourner l'image en niveaux de gris pour affichage
            }
        
        return {'detected': False}
    
    def compare_poses(self, pose1, pose2):
        """Compare deux poses ArUco et calcule les erreurs avec validation des distances attendues"""
        if not (pose1['detected'] and pose2['detected']):
            return None
        
        # Distances attendues (en mètres)
        expected_dist_cam1 = 1.12  # 1m12
        expected_dist_cam2 = 1.05  # 1m05
        
        # Erreur de position (distance euclidienne)
        pos_error = np.linalg.norm(pose1['tvec'] - pose2['tvec'])
        
        # Erreur de rotation (angle entre les vecteurs de rotation)
        rvec1_norm = pose1['rvec'] / (np.linalg.norm(pose1['rvec']) + 1e-6)
        rvec2_norm = pose2['rvec'] / (np.linalg.norm(pose2['rvec']) + 1e-6)
        dot_product = np.clip(np.dot(rvec1_norm, rvec2_norm), -1.0, 1.0)
        rot_error = np.arccos(abs(dot_product)) * 180.0 / np.pi
        
        # Erreur de taille apparente
        size_error = abs(pose1['apparent_size'] - pose2['apparent_size'])
        
        # Validation des distances mesurées vs attendues
        dist_error_cam1 = abs(pose1['distance'] - expected_dist_cam1)
        dist_error_cam2 = abs(pose2['distance'] - expected_dist_cam2)
        
        return {
            'position_error_mm': pos_error * 1000,
            'rotation_error_deg': rot_error,
            'size_error_px': size_error,
            'distance_cam1_mm': pose1['distance'] * 1000,
            'distance_cam2_mm': pose2['distance'] * 1000,
            'expected_dist_cam1_mm': expected_dist_cam1 * 1000,
            'expected_dist_cam2_mm': expected_dist_cam2 * 1000,
            'dist_error_cam1_mm': dist_error_cam1 * 1000,
            'dist_error_cam2_mm': dist_error_cam2 * 1000
        }
    
    def draw_pose_info(self, img, pose, camera_name):
        """Dessine les informations de pose sur l'image (en noir et blanc avec overlay couleur)"""
        # Convertir en BGR pour l'affichage si c'est en niveaux de gris
        if len(img.shape) == 2:
            img_display = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            img_display = img.copy()
        
        if not pose['detected']:
            cv2.putText(img_display, f"{camera_name}: ArUco non détecté", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return img_display
        
        # Dessiner le marqueur et ses axes
        corners_reshaped = pose['corners'].reshape(1, 4, 2)
        cv2.aruco.drawDetectedMarkers(img_display, [corners_reshaped])
        
        # Dessiner les axes 3D (taille réduite pour marqueur 3cm)
        cv2.drawFrameAxes(img_display, 
                         self.mtx1 if camera_name == "Cam1" else self.mtx2,
                         self.dist1 if camera_name == "Cam1" else self.dist2,
                         pose['rvec'], pose['tvec'], 0.02)  # Axes de 2cm
        
        # Distances attendues pour validation
        expected_dist = 1120 if camera_name == "Cam1" else 1050  # en mm
        measured_dist = pose['distance'] * 1000
        dist_error = abs(measured_dist - expected_dist)
        
        # Couleur selon la précision de distance
        dist_color = (0, 255, 0) if dist_error < 50 else (0, 165, 255) if dist_error < 100 else (0, 0, 255)
        
        # Informations textuelles
        info_text = [
            f"{camera_name} - ID: {pose['id']}",
            f"Distance: {measured_dist:.0f}mm (attendu: {expected_dist}mm)",
            f"Erreur: {dist_error:.0f}mm",
            f"Taille: {pose['apparent_size']:.1f}px"
        ]
        
        for i, text in enumerate(info_text):
            color = dist_color if i in [1, 2] else (255, 255, 255)
            cv2.putText(img_display, text, (10, 30 + i*25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return img_display
    
    def display_comparison_results(self, img_combined, comparison):
        """Affiche les résultats de comparaison sur l'image combinée"""
        if comparison is None:
            cv2.putText(img_combined, "Synchronisation: ÉCHEC", (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return
        
        # Seuils adaptés pour marqueur 3cm à ~1m
        pos_threshold = 30.0   # mm (plus strict pour marqueur proche)
        rot_threshold = 8.0    # degrés
        size_threshold = 15.0  # pixels
        dist_threshold = 50.0  # mm pour validation des distances attendues
        
        sync_quality = "EXCELLENTE"
        color = (0, 255, 0)
        
        # Validation distances attendues
        dist_valid = (comparison['dist_error_cam1_mm'] < dist_threshold and 
                     comparison['dist_error_cam2_mm'] < dist_threshold)
        
        if (comparison['position_error_mm'] > pos_threshold or 
            comparison['rotation_error_deg'] > rot_threshold or
            comparison['size_error_px'] > size_threshold or
            not dist_valid):
            sync_quality = "MOYENNE"
            color = (0, 165, 255)
        
        if (comparison['position_error_mm'] > pos_threshold * 2 or 
            comparison['rotation_error_deg'] > rot_threshold * 2 or
            not dist_valid):
            sync_quality = "MAUVAISE"
            color = (0, 0, 255)
        
        # Affichage des métriques
        metrics_text = [
            f"Synchronisation: {sync_quality}",
            f"Erreur position: {comparison['position_error_mm']:.1f}mm",
            f"Erreur rotation: {comparison['rotation_error_deg']:.1f}°",
            f"Erreur taille: {comparison['size_error_px']:.1f}px",
            f"C1: {comparison['distance_cam1_mm']:.0f}mm (err: {comparison['dist_error_cam1_mm']:.0f}mm)",
            f"C2: {comparison['distance_cam2_mm']:.0f}mm (err: {comparison['dist_error_cam2_mm']:.0f}mm)",
            f"Distances attendues: C1=1120mm, C2=1050mm"
        ]
        
        for i, text in enumerate(metrics_text):
            text_color = color if i == 0 else (255, 255, 255)
            cv2.putText(img_combined, text, (50, 50 + i*28),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.65, text_color, 2)
    
    def update_statistics(self, comparison):
        """Met à jour les statistiques d'évaluation"""
        self.stats['detection_total'] += 1
        
        if comparison is not None:
            self.stats['detection_sync'] += 1
            self.stats['position_errors'].append(comparison['position_error_mm'])
            self.stats['rotation_errors'].append(comparison['rotation_error_deg'])
            self.stats['size_errors'].append(comparison['size_error_px'])
    
    def print_final_statistics(self):
        """Affiche les statistiques finales"""
        if self.stats['detection_total'] == 0:
            print("Aucune mesure effectuée")
            return
        
        sync_rate = (self.stats['detection_sync'] / self.stats['detection_total']) * 100
        
        print(f"\n=== STATISTIQUES D'ÉVALUATION ===")
        print(f"Taux de synchronisation: {sync_rate:.1f}%")
        print(f"Détections synchrones: {self.stats['detection_sync']}/{self.stats['detection_total']}")
        
        if self.stats['position_errors']:
            print(f"\nErreurs de position (mm):")
            print(f"  Moyenne: {np.mean(self.stats['position_errors']):.2f}")
            print(f"  Médiane: {np.median(self.stats['position_errors']):.2f}")
            print(f"  Max: {np.max(self.stats['position_errors']):.2f}")
            
            print(f"\nErreurs de rotation (°):")
            print(f"  Moyenne: {np.mean(self.stats['rotation_errors']):.2f}")
            print(f"  Médiane: {np.median(self.stats['rotation_errors']):.2f}")
            print(f"  Max: {np.max(self.stats['rotation_errors']):.2f}")
    
    def run_evaluation(self, marker_size=0.03, duration=30):
        """Lance l'évaluation de synchronisation avec traitement noir et blanc optimisé"""
        if self.mtx1 is None or self.mtx2 is None:
            print("✗ Impossible de charger les calibrations")
            return False
        
        print(f"=== ÉVALUATION SYNCHRONISATION STÉRÉO ===")
        print(f"Durée: {duration}s, Marqueur: {marker_size*1000:.0f}mm")
        print("Traitement: Noir et Blanc avec amélioration de contraste")
        print("Appuyez sur 'q' pour quitter, 's' pour sauvegarder une image")
        print()
        
        # Initialiser les caméras
        cap1 = cv2.VideoCapture(self.cfg['camera_1_id'])
        cap2 = cv2.VideoCapture(self.cfg['camera_2_id'])
        
        if not cap1.isOpened() or not cap2.isOpened():
            print("✗ Impossible d'ouvrir les caméras")
            return False
        
        # Configuration des caméras
        cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
        cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
        cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        
        start_time = time.time()
        frame_count = 0
        
        try:
            while time.time() - start_time < duration:
                # Capture simultanée
                ret1, img1 = cap1.read()
                ret2, img2 = cap2.read()
                
                if not (ret1 and ret2):
                    continue
                
                frame_count += 1
                
                # Détection des poses ArUco
                pose1 = self.detect_aruco_pose(img1, self.mtx1, self.dist1, marker_size)
                pose2 = self.detect_aruco_pose(img2, self.mtx2, self.dist2, marker_size)
                
                # Comparaison des poses
                comparison = self.compare_poses(pose1, pose2)
                self.update_statistics(comparison)
                
                # Visualisation
                img1 = self.draw_pose_info(img1, pose1, "Cam1")
                img2 = self.draw_pose_info(img2, pose2, "Cam2")
                
                # Créer l'image combinée
                h = max(img1.shape[0], img2.shape[0])
                img_combined = np.zeros((h + 200, img1.shape[1] + img2.shape[1], 3), dtype=np.uint8)
                img_combined[:img1.shape[0], :img1.shape[1]] = img1
                img_combined[:img2.shape[0], img1.shape[1]:] = img2
                
                # Afficher les résultats de comparaison
                self.display_comparison_results(img_combined, comparison)
                
                # Affichage temps restant
                remaining = duration - (time.time() - start_time)
                cv2.putText(img_combined, f"Temps restant: {remaining:.1f}s", 
                           (img1.shape[1] + 50, img2.shape[0] + 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Redimensionner pour l'affichage
                display_width = 1400
                scale = display_width / img_combined.shape[1]
                display_height = int(img_combined.shape[0] * scale)
                img_display = cv2.resize(img_combined, (display_width, display_height))
                
                cv2.imshow("Évaluation Synchronisation Stéréo", img_display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    cv2.imwrite(f"sync_eval_{int(time.time())}.jpg", img_combined)
                    print("Image sauvegardée")
        
        finally:
            cap1.release()
            cap2.release()
            cv2.destroyAllWindows()
        
        # Affichage des statistiques finales
        self.print_final_statistics()
        
        print(f"\nÉvaluation terminée ({frame_count} frames analysées)")
        return True

def main():
    """Fonction principale"""
    evaluator = StereoSyncEvaluator()
    
    # Paramètres d'évaluation pour marqueur 3cm
    MARKER_SIZE = 0.03      # Marqueur de 3cm
    DURATION = 60           # Durée d'évaluation en secondes
    
    print("=== CONFIGURATION ===")
    print(f"Marqueur ArUco: {MARKER_SIZE*1000:.0f}mm")
    print("Distances attendues:")
    print("  Caméra 1: 1120mm")
    print("  Caméra 2: 1050mm")
    print("Traitement: Noir et Blanc optimisé")
    print()
    
    success = evaluator.run_evaluation(marker_size=MARKER_SIZE, duration=DURATION)
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())