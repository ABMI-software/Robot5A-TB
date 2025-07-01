#!/usr/bin/env python3
"""
Script de calibration complète - exécute séquentiellement tous les processus
"""
from config_charuco import get_config
import subprocess
import sys
import os
import time

class FullCalibration:
    def __init__(self):
        self.script_dir = os.path.dirname(__file__)
        
    def run_script(self, script_name, description):
        """Exécuter un script et attendre sa completion"""
        print(f"\n{'='*60}")
        print(f"ÉTAPE: {description}")
        print(f"{'='*60}")
        
        script_path = os.path.join(self.script_dir, script_name)
        
        if not os.path.exists(script_path):
            print(f"ERREUR: Script {script_name} non trouvé!")
            return False
        
        try:
            # Lancer le script et attendre sa completion
            result = subprocess.run([sys.executable, script_path], 
                                  capture_output=False, 
                                  text=True)
            
            if result.returncode == 0:
                print(f"\n✓ {description} terminé avec succès")
                return True
            else:
                print(f"\n✗ Erreur lors de {description}")
                return False
                
        except Exception as e:
            print(f"ERREUR lors de l'exécution de {script_name}: {str(e)}")
            return False
    
    def run_full_calibration(self):
        """Exécuter la calibration complète"""
        print("=== CALIBRATION STÉRÉO COMPLÈTE ===")
        print("Ce processus va exécuter les étapes suivantes:")
        print("1. Enregistrement d'images de calibration")
        print("2. Calibration intrinsèque des deux caméras")
        print("3. Calibration stéréo")
        print("\nAppuyez sur Entrée pour commencer ou Ctrl+C pour annuler...")
        
        try:
            input()
        except KeyboardInterrupt:
            print("\nProcessus annulé par l'utilisateur")
            return False
        
        start_time = time.time()
        
        # Étape 1: Enregistrement d'images
        if not self.run_script("calibration_charuco_recording.py", 
                              "Enregistrement d'images de calibration"):
            print("Échec de l'enregistrement d'images. Arrêt du processus.")
            return False
        
        print("\nAppuyez sur Entrée pour continuer avec la calibration intrinsèque...")
        try:
            input()
        except KeyboardInterrupt:
            print("\nProcessus annulé par l'utilisateur")
            return False
        
        # Étape 2: Calibration intrinsèque
        if not self.run_script("calibration_instrinsec_charuco.py", 
                              "Calibration intrinsèque des deux caméras"):
            print("Échec de la calibration intrinsèque. Arrêt du processus.")
            return False
        
        print("\nAppuyez sur Entrée pour continuer avec la calibration stéréo...")
        try:
            input()
        except KeyboardInterrupt:
            print("\nProcessus annulé par l'utilisateur")
            return False
        
        # Étape 3: Calibration stéréo
        if not self.run_script("calibration_stereo_charuco.py", 
                              "Calibration stéréo"):
            print("Échec de la calibration stéréo. Arrêt du processus.")
            return False
        
        # Résumé final
        end_time = time.time()
        duration = end_time - start_time
        
        print(f"\n{'='*60}")
        print("🎉 CALIBRATION COMPLÈTE TERMINÉE AVEC SUCCÈS!")
        print(f"{'='*60}")
        print(f"Durée totale: {duration/60:.1f} minutes")
        print("\nFichiers générés:")
        print("- camera_1_calibration.yaml (paramètres intrinsèques caméra 1)")
        print("- camera_2_calibration.yaml (paramètres intrinsèques caméra 2)")
        print("- stereo_calibration.yaml (paramètres stéréo pour triangulation)")
        print("\nVous pouvez maintenant utiliser ces paramètres pour la vision stéréo!")
        
        return True

def main():
    calibrator = FullCalibration()
    success = calibrator.run_full_calibration()
    
    if success:
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()

"""
# TESTS
# Pour tester ce script:
# 1. Assurez-vous que tous les autres scripts sont dans le même répertoire
# 2. Vérifiez que les caméras sont connectées et fonctionnelles
# 3. Préparez un plateau Charuco avec les bonnes dimensions
# 4. Lancez: python calibration_full_charuco.py
# 5. Suivez les instructions à l'écran pour chaque étape
# 6. Vérifiez que chaque étape se termine sans erreur
# 7. Contrôlez que tous les fichiers de calibration sont générés
# 8. Testez l'interruption du processus avec Ctrl+C
# 9. Vérifiez que les erreurs de calibration sont dans des limites acceptables
"""