import tkinter as tk
from tkinter import messagebox, ttk
import subprocess
import os
import sys

class CalibrationGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Système de calibration stéréo")
        self.root.geometry("600x450")  # Taille ajustée pour la phrase
        self.root.resizable(False, False)
        
        # Fond de l'interface
        self.root.configure(bg='#f5f6fa')
        
        # Frame principale pour centrer le contenu
        main_frame = tk.Frame(self.root, bg='#f5f6fa')
        main_frame.pack(expand=True, fill='both', padx=30, pady=20)
        
        # Titre
        title_label = tk.Label(
            main_frame, 
            text="Système de calibration stéréo", 
            font=("Helvetica", 22, "bold"),
            bg='#f5f6fa',
            fg='#2c3e50'
        )
        title_label.pack(pady=(0, 10))
        
        # Phrase ajoutée
        subtitle_label = tk.Label(
            main_frame,
            text="Sélectionnez la fonction que vous voulez lancer",
            font=("Helvetica", 12),
            bg='#f5f6fa',
            fg='#7f8c8d'
        )
        subtitle_label.pack(pady=(0, 30))
        
        # Frame pour les boutons
        button_frame = tk.Frame(main_frame, bg='#f5f6fa')
        button_frame.pack(expand=True)
        
        # Styles pour les boutons
        style = ttk.Style()
        style.theme_use('clam')  # Thème avec bords plus doux
        
        # Palette de couleurs
        colors = {
            'recording': '#3498db',
            'intrinsic': '#2ecc71',
            'stereo': '#e74c3c',
            'full': '#f39c12'
        }
        
        # Configuration des styles
        for btn_type, color in colors.items():
            style.configure(f'{btn_type}.TButton',
                font=("Helvetica", 14),
                background=color,
                foreground='white',
                borderwidth=0,
                focuscolor=color,
                padding=10,
                relief='flat'
            )
            style.map(f'{btn_type}.TButton',
                background=[('active', self.darken_color(color))],
                foreground=[('active', 'white')]
            )
        
        # Liste des boutons
        buttons = [
            ("🎥 - Enregistrement d'images - 🎥", self.run_recording, 'recording'),
            ("⚙️ -  Calibration intrinsèque- ⚙️", self.run_intrinsic, 'intrinsic'),
            ("🔍 - Calibration stéréo - 🔍", self.run_stereo, 'stereo'),
            ("🤖 - Calibration complète - 🤖", self.run_full, 'full')
        ]
        
        for text, command, btn_type in buttons:
            btn = ttk.Button(
                button_frame,
                text=text,
                command=command,
                style=f'{btn_type}.TButton',
                cursor='hand2'
            )
            btn.pack(fill='x', pady=10)
        
    def darken_color(self, color, amount=0.8):
        """Assombrir une couleur hexadécimale"""
        color = color.lstrip('#')
        rgb = tuple(int(color[i:i+2], 16) for i in (0, 2, 4))
        darkened = tuple(int(c * amount) for c in rgb)
        return f'#{darkened[0]:02x}{darkened[1]:02x}{darkened[2]:02x}'
    
    def run_script(self, script_name):
        """Exécuter un script Python"""
        try:
            script_path = os.path.join(os.path.dirname(__file__), script_name)
            if not os.path.exists(script_path):
                messagebox.showerror("Erreur", f"Script {script_name} non trouvé!")
                return
            subprocess.Popen([sys.executable, script_path])
            messagebox.showinfo("Information", f"Script {script_name} lancé!")
        except Exception as e:
            messagebox.showerror("Erreur", f"Erreur lors du lancement: {str(e)}")
    
    def run_recording(self):
        self.run_script("calibration_charuco_recording.py")
    
    def run_intrinsic(self):
        self.run_script("calibration_instrinsec_charuco.py")
    
    def run_stereo(self):
        self.run_script("calibration_stereo_charuco.py")
    
    def run_full(self):
        self.run_script("calibration_full_charuco.py")
    
    def run(self):
        self.root.mainloop()

def main():
    app = CalibrationGUI()
    app.run()

if __name__ == "__main__":
    main()