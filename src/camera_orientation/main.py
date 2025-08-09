"""
Fichier principal pour l'application de contrôle multi-servomoteurs STS3032
Version avec limites d'angles personnalisées pour les IDs 2 et 4
"""

import tkinter as tk
from sts3032_gui import MultiSTS3032GUI

def main():
    """Fonction principale de l'application"""
    # Création de la fenêtre principale
    root = tk.Tk()
    
    # Création de l'interface graphique
    app = MultiSTS3032GUI(root)
    
    # Gestionnaire de fermeture propre
    def on_closing():
        """Nettoie les ressources avant de fermer l'application"""
        app.cleanup()
        root.destroy()
    
    # Association du gestionnaire de fermeture
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Lancement de la boucle principale
    root.mainloop()

if __name__ == "__main__":
    main()