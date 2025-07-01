#!/usr/bin/env python3
"""
Tests complémentaires pour l'évaluation stéréo
"""

import cv2
import numpy as np

class StereoAdditionalTests:
    
    def test_rectification_quality(self, img1, img2, stereo_params):
        """Test de la qualité de rectification"""
        R1 = np.array(stereo_params['rectification_R1'])
        R2 = np.array(stereo_params['rectification_R2'])
        P1 = np.array(stereo_params['projection_P1'])
        P2 = np.array(stereo_params['projection_P2'])
        
        # Créer les cartes de rectification
        map1x, map1y = cv2.initUndistortRectifyMap(mtx1, dist1, R1, P1, img1.shape[:2][::-1], cv2.CV_32FC1)
        map2x, map2y = cv2.initUndistortRectifyMap(mtx2, dist2, R2, P2, img2.shape[:2][::-1], cv2.CV_32FC1)
        
        # Rectifier les images
        rect1 = cv2.remap(img1, map1x, map1y, cv2.INTER_LINEAR)
        rect2 = cv2.remap(img2, map2x, map2y, cv2.INTER_LINEAR)
        
        # Mesurer l'alignement horizontal
        return self.measure_epipolar_alignment(rect1, rect2)
    
    def test_temporal_sync(self, num_frames=30):
        """Test de synchronisation temporelle avec objet en mouvement"""
        cap1 = cv2.VideoCapture(0)
        cap2 = cv2.VideoCapture(1)
        
        timestamps_diff = []
        
        for i in range(num_frames):
            t1 = time.time()
            ret1, img1 = cap1.read()
            t2 = time.time()
            ret2, img2 = cap2.read()
            t3 = time.time()
            
            if ret1 and ret2:
                # Analyser la cohérence temporelle des détections
                sync_score = self.analyze_motion_coherence(img1, img2)
                timestamps_diff.append(abs(t2-t1) - abs(t3-t2))
        
        return np.mean(timestamps_diff), np.std(timestamps_diff)
    
    def depth_accuracy_test(self, known_distances):
        """Test de précision de reconstruction 3D avec distances connues"""
        # Placer des objets à distances connues
        # Mesurer la disparité et calculer la profondeur
        # Comparer avec les distances réelles
        pass

# Usage rapide pour tests spécifiques
def quick_sync_test():
    """Test rapide de synchronisation (30 secondes)"""
    evaluator = StereoEvaluator()
    evaluator.evaluate_stereo_consistency(num_samples=5, delay=0.2)

if __name__ == "__main__":
    quick_sync_test()