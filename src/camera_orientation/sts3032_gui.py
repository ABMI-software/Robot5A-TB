"""
Interface graphique pour le contrôleur multi-servomoteurs STS3032
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from sts3032_controller import MultiSTS3032Controller

class ServoControlFrame:
    """Frame de contrôle individuel pour un servomoteur"""
    def __init__(self, parent, servo_id, controller):
        self.parent = parent
        self.servo_id = servo_id
        self.controller = controller
        self.monitoring = False
        
        # Obtenir les limites pour ce servo
        self.limits = controller.get_servo_limits(servo_id)
        
        # Frame principal pour ce servo avec indication des limites
        limit_info = f" ({self.limits['min']}° à {self.limits['max']}°)" if self.limits != controller.get_servo_limits(None) else ""
        self.frame = ttk.LabelFrame(parent, text=f"Servo ID: {servo_id}{limit_info}", padding="5")
        
        # Variables
        self.speed_var = tk.IntVar(value=500)
        self.acc_var = tk.IntVar(value=0)
        
        # Position centrée dans la plage permise
        center_degrees = (self.limits['min'] + self.limits['max']) / 2
        self.degrees_var = tk.DoubleVar(value=center_degrees)
        center_position = controller.degrees_to_position(center_degrees, servo_id)
        self.position_var = tk.IntVar(value=center_position)
        
        self.monitor_var = tk.BooleanVar()
        
        self.create_widgets()
        
    def create_widgets(self):
        """Crée les widgets de contrôle pour ce servo"""
        
        # Status et contrôles de base
        status_frame = ttk.Frame(self.frame)
        status_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=2)
        
        self.status_label = ttk.Label(status_frame, text="Hors tension", foreground="red")
        self.status_label.pack(side="left")
        
        self.power_on_btn = ttk.Button(status_frame, text="ON", 
                                      command=self.power_on, width=6)
        self.power_on_btn.pack(side="right", padx=2)
        
        self.power_off_btn = ttk.Button(status_frame, text="OFF", 
                                       command=self.power_off, width=6)
        self.power_off_btn.pack(side="right", padx=2)
        
        # Vitesse
        ttk.Label(self.frame, text="Vitesse:").grid(row=1, column=0, sticky="w")
        speed_scale = ttk.Scale(self.frame, from_=0, to=1023, variable=self.speed_var,
                              orient="horizontal", length=150, command=self.update_speed)
        speed_scale.grid(row=1, column=1, padx=2)
        self.speed_label = ttk.Label(self.frame, text="500", width=4)
        self.speed_label.grid(row=1, column=2)
        
        # Position en degrés - LIMITÉE selon l'ID du servo
        ttk.Label(self.frame, text="Position:").grid(row=2, column=0, sticky="w")
        degrees_scale = ttk.Scale(self.frame, from_=self.limits['min'], to=self.limits['max'], 
                                variable=self.degrees_var, orient="horizontal", length=150, 
                                command=self.update_position_from_degrees)
        degrees_scale.grid(row=2, column=1, padx=2)
        self.degrees_label = ttk.Label(self.frame, text=f"{self.degrees_var.get():.1f}°", width=6)
        self.degrees_label.grid(row=2, column=2)
        
        # Position brute - Calculée selon les limites
        min_pos = self.controller.degrees_to_position(self.limits['min'], self.servo_id)
        max_pos = self.controller.degrees_to_position(self.limits['max'], self.servo_id)
        
        ttk.Label(self.frame, text="Position brute:").grid(row=3, column=0, sticky="w")
        pos_scale = ttk.Scale(self.frame, from_=min_pos, to=max_pos, variable=self.position_var,
                            orient="horizontal", length=150, command=self.update_position_from_raw)
        pos_scale.grid(row=3, column=1, padx=2)
        self.position_label = ttk.Label(self.frame, text=str(self.position_var.get()), width=6)
        self.position_label.grid(row=3, column=2)
        
        # Position exacte
        exact_frame = ttk.Frame(self.frame)
        exact_frame.grid(row=4, column=0, columnspan=3, pady=2)
        
        ttk.Label(exact_frame, text="Position exacte:").pack(side="left")
        self.exact_pos_var = tk.StringVar(value=f"{self.degrees_var.get():.1f}")
        self.exact_pos_entry = ttk.Entry(exact_frame, textvariable=self.exact_pos_var, width=8)
        self.exact_pos_entry.pack(side="left", padx=2)
        
        ttk.Button(exact_frame, text="Set (°)", command=self.set_exact_degrees, width=6).pack(side="left", padx=1)
        ttk.Button(exact_frame, text="Set (pos)", command=self.set_exact_position, width=8).pack(side="left", padx=1)
        
        # Info des limites
        limits_frame = ttk.Frame(self.frame)
        limits_frame.grid(row=5, column=0, columnspan=3, pady=2)
        
        limit_text = f"Limites: {self.limits['min']}° à {self.limits['max']}°"
        if self.limits != self.controller.get_servo_limits(None):
            ttk.Label(limits_frame, text=limit_text, foreground="blue", font=("TkDefaultFont", 8)).pack()
        
        # Monitoring
        monitor_frame = ttk.Frame(self.frame)
        monitor_frame.grid(row=6, column=0, columnspan=3, pady=2)
        
        self.monitor_check = ttk.Checkbutton(monitor_frame, text="Monitor", 
                                           variable=self.monitor_var, command=self.toggle_monitoring)
        self.monitor_check.pack(side="left")
        
        self.current_pos_label = ttk.Label(monitor_frame, text="--", width=10)
        self.current_pos_label.pack(side="left", padx=5)
        
    def update_status(self):
        """Met à jour le statut du servo"""
        if self.controller.is_servo_powered(self.servo_id):
            self.status_label.config(text="Sous tension", foreground="green")
        else:
            self.status_label.config(text="Hors tension", foreground="red")
    
    def power_on(self):
        """Met sous tension ce servo"""
        def power_thread():
            success = self.controller.power_on_sequence(self.servo_id)
            self.parent.after(0, lambda: self.update_status())
            if success:
                self.parent.after(0, lambda: self.parent.master.log_message(f"Servo {self.servo_id}: Sous tension"))
            else:
                self.parent.after(0, lambda: self.parent.master.log_message(f"Servo {self.servo_id}: Échec mise sous tension"))
        
        threading.Thread(target=power_thread, daemon=True).start()
    
    def power_off(self):
        """Met hors tension ce servo"""
        def power_off_thread():
            success = self.controller.power_off_sequence(self.servo_id)
            self.parent.after(0, lambda: self.update_status())
            if success:
                self.parent.after(0, lambda: self.parent.master.log_message(f"Servo {self.servo_id}: Hors tension"))
            else:
                self.parent.after(0, lambda: self.parent.master.log_message(f"Servo {self.servo_id}: Échec mise hors tension"))
        
        threading.Thread(target=power_off_thread, daemon=True).start()
    
    def update_speed(self, value=None):
        """Met à jour la vitesse"""
        speed = int(self.speed_var.get())
        self.speed_label.config(text=str(speed))
        if self.controller.is_servo_powered(self.servo_id):
            self.controller.set_speed(self.servo_id, speed)
    
    def update_position_from_degrees(self, value=None):
        """Met à jour la position depuis les degrés"""
        degrees = self.degrees_var.get()
        # Vérifier les limites
        degrees = max(self.limits['min'], min(self.limits['max'], degrees))
        self.degrees_var.set(degrees)
        
        position = self.controller.degrees_to_position(degrees, self.servo_id)
        
        self.degrees_label.config(text=f"{degrees:.1f}°")
        self.position_var.set(position)
        self.position_label.config(text=str(position))
        
        if self.controller.is_servo_powered(self.servo_id):
            speed = int(self.speed_var.get())
            self.controller.set_position_degrees(self.servo_id, degrees, speed=speed)
    
    def set_exact_degrees(self):
        """Applique une position exacte en degrés"""
        if not self.controller.is_servo_powered(self.servo_id):
            messagebox.showwarning("Attention", f"Servo {self.servo_id} doit être sous tension")
            return
        
        try:
            degrees = float(self.exact_pos_var.get())
            # Vérifier les limites
            if degrees < self.limits['min'] or degrees > self.limits['max']:
                messagebox.showwarning("Attention", 
                    f"Angle hors limites pour servo {self.servo_id}\n"
                    f"Limites: {self.limits['min']}° à {self.limits['max']}°\n"
                    f"Valeur demandée: {degrees}°")
                return
            
            self.degrees_var.set(degrees)
            self.update_position_from_degrees()
            self.exact_pos_var.set(f"{degrees:.1f}")
        except ValueError:
            messagebox.showerror("Erreur", "Valeur de degrés invalide")
    
    def set_exact_position(self):
        """Applique une position exacte en valeur brute"""
        if not self.controller.is_servo_powered(self.servo_id):
            messagebox.showwarning("Attention", f"Servo {self.servo_id} doit être sous tension")
            return
        
        try:
            position = int(float(self.exact_pos_var.get()))
            position = max(0, min(4095, position))
            
            # Vérifier que la position correspond aux limites d'angle
            degrees = self.controller.position_to_degrees(position)
            if degrees < self.limits['min'] or degrees > self.limits['max']:
                messagebox.showwarning("Attention", 
                    f"Position correspond à un angle hors limites pour servo {self.servo_id}\n"
                    f"Limites: {self.limits['min']}° à {self.limits['max']}°\n"
                    f"Angle correspondant: {degrees:.1f}°")
                return
            
            self.position_var.set(position)
            self.update_position_from_raw()
            self.exact_pos_var.set(str(position))
        except ValueError:
            messagebox.showerror("Erreur", "Valeur de position invalide")
    
    def update_position_from_raw(self, value=None):
        """Met à jour la position depuis la valeur brute"""
        position = int(self.position_var.get())
        degrees = self.controller.position_to_degrees(position)
        
        # Vérifier les limites
        if degrees < self.limits['min'] or degrees > self.limits['max']:
            # Remettre dans les limites
            degrees = max(self.limits['min'], min(self.limits['max'], degrees))
            position = self.controller.degrees_to_position(degrees, self.servo_id)
            self.position_var.set(position)
        
        self.position_label.config(text=str(position))
        self.degrees_var.set(degrees)
        self.degrees_label.config(text=f"{degrees:.1f}°")
        
        if self.controller.is_servo_powered(self.servo_id):
            speed = int(self.speed_var.get())
            self.controller.set_position(self.servo_id, position, speed=speed)
    
    def toggle_monitoring(self):
        """Active/désactive le monitoring"""
        self.monitoring = self.monitor_var.get()
        if self.monitoring and self.controller.is_servo_powered(self.servo_id):
            self.start_monitoring()
    
    def start_monitoring(self):
        """Démarre le monitoring"""
        def monitor_thread():
            while self.monitoring and self.controller.is_servo_powered(self.servo_id):
                try:
                    pos = self.controller.get_present_position(self.servo_id)
                    if pos is not None:
                        degrees = self.controller.position_to_degrees(pos)
                        self.parent.after(0, lambda: self.current_pos_label.config(text=f"{degrees:.1f}°"))
                    time.sleep(0.2)
                except:
                    break
        
        threading.Thread(target=monitor_thread, daemon=True).start()


class MultiSTS3032GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-STS3032 Servo Controller - Limites personnalisées")
        self.root.geometry("900x750")
        self.root.resizable(True, True)
        
        self.controller = MultiSTS3032Controller()
        self.servo_frames = {}
        self.scanning = False
        
        self.create_widgets()
        
    def create_widgets(self):
        """Crée l'interface utilisateur"""
        
        # Frame principal avec scrollbar
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Frame de connexion et scan
        conn_frame = ttk.LabelFrame(main_frame, text="Connexion et Scan", padding="10")
        conn_frame.pack(fill="x", pady=5)
        
        # Connexion
        conn_controls = ttk.Frame(conn_frame)
        conn_controls.pack(fill="x")
        
        self.connect_btn = ttk.Button(conn_controls, text="Connecter COM6", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5)
        
        self.scan_btn = ttk.Button(conn_controls, text="Scanner Servos (ID 1-20)", 
                                  command=self.scan_servos, state="disabled")
        self.scan_btn.pack(side="left", padx=5)
        
        # Plage de scan
        ttk.Label(conn_controls, text="Plage:").pack(side="left", padx=(20,5))
        self.scan_start_var = tk.IntVar(value=1)
        self.scan_end_var = tk.IntVar(value=20)
        
        ttk.Spinbox(conn_controls, from_=1, to=253, textvariable=self.scan_start_var, width=5).pack(side="left")
        ttk.Label(conn_controls, text="à").pack(side="left", padx=2)
        ttk.Spinbox(conn_controls, from_=1, to=253, textvariable=self.scan_end_var, width=5).pack(side="left")
        
        # Info sur les limites
        info_label = ttk.Label(conn_controls, text="• IDs 2,4: -35° à +110° • Autres: -180° à +180°", 
                              foreground="blue", font=("TkDefaultFont", 9))
        info_label.pack(side="right", padx=10)
        
        # Statut et barre de progression
        status_frame = ttk.Frame(conn_frame)
        status_frame.pack(fill="x", pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Déconnecté", foreground="red")
        self.status_label.pack(side="left")
        
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(status_frame, variable=self.progress_var, length=200)
        self.progress_bar.pack(side="right", padx=5)
        
        # Frame des contrôles globaux
        global_frame = ttk.LabelFrame(main_frame, text="Contrôles Globaux", padding="10")
        global_frame.pack(fill="x", pady=5)
        
        global_controls = ttk.Frame(global_frame)
        global_controls.pack()
        
        self.global_power_on_btn = ttk.Button(global_controls, text="Tout Sous Tension", 
                                             command=self.power_on_all, state="disabled")
        self.global_power_on_btn.pack(side="left", padx=5)
        
        self.global_power_off_btn = ttk.Button(global_controls, text="Tout Hors Tension", 
                                              command=self.power_off_all, state="disabled")
        self.global_power_off_btn.pack(side="left", padx=5)
        
        self.detected_label = ttk.Label(global_controls, text="Aucun servo détecté")
        self.detected_label.pack(side="left", padx=20)
        
        # Frame scrollable pour les servos
        self.create_servo_scroll_frame(main_frame)
        
        # Frame de logs
        log_frame = ttk.LabelFrame(main_frame, text="Logs", padding="5")
        log_frame.pack(fill="both", expand=True, pady=5)
        
        self.log_text = tk.Text(log_frame, height=6, font=("Courier", 9))
        log_scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        
        self.log_text.pack(side="left", fill="both", expand=True)
        log_scrollbar.pack(side="right", fill="y")
        
        # Message d'accueil avec info sur les limites
        self.log_message("Application démarrée - Limites: IDs 2,4 = -35° à +110°, autres = -180° à +180°")
        
    def create_servo_scroll_frame(self, parent):
        """Crée un frame scrollable pour les servos"""
        # Frame avec scrollbar pour les servos
        servo_container = ttk.LabelFrame(parent, text="Servomoteurs Détectés", padding="5")
        servo_container.pack(fill="both", expand=True, pady=5)
        
        # Canvas et scrollbar
        self.servo_canvas = tk.Canvas(servo_container, height=200)
        servo_scrollbar = ttk.Scrollbar(servo_container, orient="vertical", command=self.servo_canvas.yview)
        self.servo_scrollable_frame = ttk.Frame(self.servo_canvas)
        
        self.servo_scrollable_frame.bind(
            "<Configure>",
            lambda e: self.servo_canvas.configure(scrollregion=self.servo_canvas.bbox("all"))
        )
        
        self.servo_canvas.create_window((0, 0), window=self.servo_scrollable_frame, anchor="nw")
        self.servo_canvas.configure(yscrollcommand=servo_scrollbar.set)
        
        self.servo_canvas.pack(side="left", fill="both", expand=True)
        servo_scrollbar.pack(side="right", fill="y")
        
        # Bind mousewheel
        def _on_mousewheel(event):
            self.servo_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        self.servo_canvas.bind("<MouseWheel>", _on_mousewheel)
    
    def log_message(self, message):
        """Ajoute un message au log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.root.update_idletasks()
    
    def toggle_connection(self):
        """Bascule l'état de connexion"""
        if not self.controller.connected:
            if self.controller.connect():
                self.log_message("Connecté sur COM6 à 1000000 bauds")
                self.connect_btn.config(text="Déconnecter")
                self.scan_btn.config(state="normal")
                self.status_label.config(text="Connecté", foreground="green")
            else:
                self.log_message("Erreur de connexion sur COM6")
                messagebox.showerror("Erreur", "Impossible de se connecter au port COM6")
        else:
            # Mise hors tension de tous les servos avant déconnexion
            self.power_off_all()
            
            self.controller.disconnect()
            self.log_message("Déconnecté")
            self.connect_btn.config(text="Connecter COM6")
            self.scan_btn.config(state="disabled")
            self.global_power_on_btn.config(state="disabled")
            self.global_power_off_btn.config(state="disabled")
            self.status_label.config(text="Déconnecté", foreground="red")
            self.clear_servo_frames()
    
    def scan_servos(self):
        """Lance le scan des servomoteurs"""
        if self.scanning:
            return
        
        self.scanning = True
        self.scan_btn.config(state="disabled", text="Scan en cours...")
        self.progress_var.set(0)
        
        def scan_thread():
            try:
                start_id = self.scan_start_var.get()
                end_id = self.scan_end_var.get()
                
                def progress_callback(current_id, max_id):
                    progress = ((current_id - start_id) / (end_id - start_id + 1)) * 100
                    self.root.after(0, lambda: self.progress_var.set(progress))
                
                detected = self.controller.scan_servos((start_id, end_id), progress_callback)
                
                self.root.after(0, lambda: self.scan_complete(detected))
                
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"Erreur lors du scan: {e}"))
                self.root.after(0, self.scan_complete([]))
        
        threading.Thread(target=scan_thread, daemon=True).start()
    
    def scan_complete(self, detected_servos):
        """Appelé quand le scan est terminé"""
        self.scanning = False
        self.scan_btn.config(state="normal", text="Scanner Servos")
        self.progress_var.set(100)
        
        if detected_servos:
            # Identifier les servos avec limites spéciales
            limited_servos = [sid for sid in detected_servos if sid in [2, 4]]
            normal_servos = [sid for sid in detected_servos if sid not in [2, 4]]
            
            message = f"Scan terminé: {len(detected_servos)} servos détectés: {detected_servos}"
            if limited_servos:
                message += f"\nServos avec limites -35°/+110°: {limited_servos}"
            if normal_servos:
                message += f"\nServos standard -180°/+180°: {normal_servos}"
            
            self.log_message(message)
            self.detected_label.config(text=f"{len(detected_servos)} servos détectés")
            self.global_power_on_btn.config(state="normal")
            self.global_power_off_btn.config(state="normal")
            self.create_servo_controls(detected_servos)
        else:
            self.log_message("Scan terminé: Aucun servo détecté")
            self.detected_label.config(text="Aucun servo détecté")
            self.clear_servo_frames()
        
        # Reset progress bar after a delay
        self.root.after(2000, lambda: self.progress_var.set(0))
    
    def create_servo_controls(self, servo_ids):
        """Crée les contrôles pour chaque servo détecté"""
        self.clear_servo_frames()
        
        # Calculer le nombre de colonnes (max 3 pour plus d'espace)
        cols = min(3, len(servo_ids))
        
        for i, servo_id in enumerate(servo_ids):
            row = i // cols
            col = i % cols
            
            servo_frame = ServoControlFrame(self.servo_scrollable_frame, servo_id, self.controller)
            servo_frame.frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")
            
            self.servo_frames[servo_id] = servo_frame
        
        # Configure grid weights
        for col in range(cols):
            self.servo_scrollable_frame.columnconfigure(col, weight=1)
    
    def clear_servo_frames(self):
        """Supprime tous les frames de servos"""
        for frame in self.servo_frames.values():
            frame.frame.destroy()
        self.servo_frames = {}
    
    def power_on_all(self):
        """Met sous tension tous les servomoteurs"""
        def power_on_thread():
            results = self.controller.power_on_all()
            success_count = sum(1 for success in results.values() if success)
            total_count = len(results)
            
            self.root.after(0, lambda: self.log_message(f"Mise sous tension globale: {success_count}/{total_count} réussies"))
            
            # Met à jour les statuts
            for servo_id in results:
                if servo_id in self.servo_frames:
                    self.root.after(0, lambda sid=servo_id: self.servo_frames[sid].update_status())
        
        threading.Thread(target=power_on_thread, daemon=True).start()
    
    def power_off_all(self):
        """Met hors tension tous les servomoteurs"""
        def power_off_thread():
            results = self.controller.power_off_all()
            success_count = sum(1 for success in results.values() if success)
            total_count = len(results)
            
            self.root.after(0, lambda: self.log_message(f"Mise hors tension globale: {success_count}/{total_count} réussies"))
            
            # Met à jour les statuts
            for servo_id in results:
                if servo_id in self.servo_frames:
                    self.root.after(0, lambda sid=servo_id: self.servo_frames[sid].update_status())
        
        threading.Thread(target=power_off_thread, daemon=True).start()
    
    def cleanup(self):
        """Nettoie les ressources avant fermeture"""
        if self.controller.connected:
            self.power_off_all()
            time.sleep(0.5)  # Attendre que les servos se mettent hors tension
            self.controller.disconnect()