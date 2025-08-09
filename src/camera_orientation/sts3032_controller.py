"""
Classe de contrôle pour plusieurs servomoteurs STS3032
"""

import serial
import time
from config import *

class MultiSTS3032Controller:
    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
        """
        Initialise le contrôleur pour plusieurs servomoteurs STS3032
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connected = False
        self.detected_servos = {}  # {id: servo_info}
        
    def connect(self):
        """Établit la connexion série"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=DEFAULT_TIMEOUT
            )
            self.connected = True
            time.sleep(0.2)
            return True
        except Exception as e:
            self.connected = False
            print(f"Erreur de connexion: {e}")
            return False
    
    def disconnect(self):
        """Ferme la connexion série"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        self.detected_servos = {}
    
    def calculate_checksum(self, packet):
        """Calcule le checksum pour le protocole SCS"""
        checksum = 0
        for i in range(2, len(packet)):
            checksum += packet[i]
        return (~checksum) & 0xFF
    
    def write_data(self, servo_id, address, data):
        """Écrit des données dans un registre du servomoteur"""
        if not self.connected or not self.ser or not self.ser.is_open:
            return False
        
        packet = []
        packet.extend(HEADER)
        packet.append(servo_id)
        packet.append(len(data) + 3)
        packet.append(INST_WRITE)
        packet.append(address)
        packet.extend(data)
        
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)
        
        try:
            self.ser.flushInput()
            self.ser.write(bytearray(packet))
            self.ser.flush()
            time.sleep(0.02)
            return True
        except Exception as e:
            print(f"Erreur d'écriture servo {servo_id}: {e}")
            return False
    
    def read_data(self, servo_id, address, length):
        """Lit des données depuis un registre du servomoteur"""
        if not self.connected or not self.ser or not self.ser.is_open:
            return None
        
        packet = []
        packet.extend(HEADER)
        packet.append(servo_id)
        packet.append(4)
        packet.append(INST_READ)
        packet.append(address)
        packet.append(length)
        
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)
        
        try:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.write(bytearray(packet))
            self.ser.flush()
            time.sleep(0.02)
            
            expected_length = 6 + length
            response = self.ser.read(expected_length)
            
            if len(response) >= expected_length:
                return list(response[5:5+length])
            return None
        except Exception as e:
            print(f"Erreur de lecture servo {servo_id}: {e}")
            return None
    
    def ping(self, servo_id):
        """Teste la communication avec un servomoteur spécifique"""
        if not self.connected:
            return False
        
        packet = []
        packet.extend(HEADER)
        packet.append(servo_id)
        packet.append(2)
        packet.append(INST_PING)
        
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)
        
        try:
            self.ser.flushInput()
            self.ser.write(bytearray(packet))
            self.ser.flush()
            time.sleep(0.02)
            
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                if len(response) >= 6:
                    return True
            return False
        except:
            return False
    
    def scan_servos(self, id_range=(1, 20), progress_callback=None):
        """
        Scanne la plage d'IDs pour détecter les servomoteurs connectés
        """
        if not self.connected:
            return []
        
        detected = []
        self.detected_servos = {}
        
        for servo_id in range(id_range[0], id_range[1] + 1):
            if progress_callback:
                progress_callback(servo_id, id_range[1])
            
            if self.ping(servo_id):
                servo_info = {
                    'id': servo_id,
                    'powered': False,
                    'position': 0,
                    'speed': 500,
                    'acceleration': 0,
                    'limits': get_servo_limits(servo_id)  # Ajouter les limites
                }
                detected.append(servo_id)
                self.detected_servos[servo_id] = servo_info
                time.sleep(0.1)  # Délai entre les pings
        
        return detected
    
    def power_on_sequence(self, servo_id):
        """Séquence de mise sous tension d'un servomoteur"""
        if servo_id not in self.detected_servos:
            return False
        
        # Vérification communication
        if not self.ping(servo_id):
            return False
        
        # Configuration du mode position
        self.write_data(servo_id, ADDR_MODE, [0])  # Mode position
        time.sleep(0.1)
        
        # Configuration limite de torque
        self.write_data(servo_id, ADDR_MAX_TORQUE_L, [0xFF])  # 1023 = 0x3FF
        time.sleep(0.01)
        self.write_data(servo_id, ADDR_MAX_TORQUE_H, [0x03])
        time.sleep(0.1)
        
        # Activation du torque
        if not self.enable_torque(servo_id, True):
            return False
        
        self.detected_servos[servo_id]['powered'] = True
        return True
    
    def power_off_sequence(self, servo_id):
        """Séquence de mise hors tension d'un servomoteur"""
        if servo_id not in self.detected_servos:
            return False
        
        # Désactivation du torque
        success = self.enable_torque(servo_id, False)
        time.sleep(0.1)
        
        if success:
            self.detected_servos[servo_id]['powered'] = False
        
        return success
    
    def power_on_all(self):
        """Met sous tension tous les servomoteurs détectés"""
        results = {}
        for servo_id in self.detected_servos.keys():
            results[servo_id] = self.power_on_sequence(servo_id)
        return results
    
    def power_off_all(self):
        """Met hors tension tous les servomoteurs détectés"""
        results = {}
        for servo_id in self.detected_servos.keys():
            results[servo_id] = self.power_off_sequence(servo_id)
        return results
    
    def enable_torque(self, servo_id, enable=True):
        """Active ou désactive le couple du servomoteur"""
        value = 1 if enable else 0
        return self.write_data(servo_id, ADDR_TORQUE_ENABLE, [value])
    
    def set_acceleration(self, servo_id, acceleration):
        """Définit l'accélération du servomoteur (0-254)"""
        acceleration = max(0, min(254, acceleration))
        success = self.write_data(servo_id, ADDR_ACC, [acceleration])
        if success and servo_id in self.detected_servos:
            self.detected_servos[servo_id]['acceleration'] = acceleration
        return success
    
    def set_speed(self, servo_id, speed):
        """Définit la vitesse du servomoteur (0-1023)"""
        speed = max(0, min(1023, speed))
        speed_l = speed & 0xFF
        speed_h = (speed >> 8) & 0xFF
        
        success1 = self.write_data(servo_id, ADDR_GOAL_SPEED_L, [speed_l])
        time.sleep(0.01)
        success2 = self.write_data(servo_id, ADDR_GOAL_SPEED_H, [speed_h])
        
        success = success1 and success2
        if success and servo_id in self.detected_servos:
            self.detected_servos[servo_id]['speed'] = speed
        
        return success
    
    def get_servo_limits(self, servo_id):
        """Retourne les limites d'angle pour un servo"""
        return get_servo_limits(servo_id)
    
    def normalize_degrees(self, degrees, servo_id=None):
        """Normalise les degrés selon les limites du servo"""
        if servo_id is not None:
            limits = self.get_servo_limits(servo_id)
            # Clamper dans les limites spécifiques du servo
            return max(limits["min"], min(limits["max"], degrees))
        else:
            # Normalisation standard -180 à +180
            while degrees > 180:
                degrees -= 360
            while degrees <= -180:
                degrees += 360
            return degrees
    
    def degrees_to_position(self, degrees, servo_id=None):
        """Convertit des degrés en position du servomoteur"""
        if servo_id is not None:
            limits = self.get_servo_limits(servo_id)
            # Vérifier que l'angle est dans les limites
            degrees = max(limits["min"], min(limits["max"], degrees))
        else:
            degrees = self.normalize_degrees(degrees)
        
        # Conversion en position 0-4095
        position = int(((degrees + 180.0) / 360.0) * 4095)
        return max(0, min(4095, position))
    
    def position_to_degrees(self, position):
        """Convertit une position en degrés (-180 à +180)"""
        degrees = (position / 4095.0) * 360.0 - 180.0
        return self.normalize_degrees(degrees)
    
    def set_position(self, servo_id, position_or_degrees, speed=None, is_degrees=False):
        """Définit la position cible du servomoteur"""
        if is_degrees:
            # Vérifier les limites avant conversion
            degrees = self.normalize_degrees(position_or_degrees, servo_id)
            position = self.degrees_to_position(degrees, servo_id)
        else:
            position = max(0, min(4095, position_or_degrees))
        
        # Définir la vitesse si spécifiée
        if speed is not None:
            self.set_speed(servo_id, speed)
            time.sleep(0.01)
        
        # Définir la position
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        
        success1 = self.write_data(servo_id, ADDR_GOAL_POSITION_L, [pos_l])
        time.sleep(0.01)
        success2 = self.write_data(servo_id, ADDR_GOAL_POSITION_H, [pos_h])
        
        success = success1 and success2
        if success and servo_id in self.detected_servos:
            self.detected_servos[servo_id]['position'] = position
        
        return success
    
    def set_position_degrees(self, servo_id, degrees, speed=None):
        """Définit la position en degrés avec vérification des limites"""
        return self.set_position(servo_id, degrees, speed, is_degrees=True)
    
    def get_present_position(self, servo_id):
        """Lit la position actuelle du servomoteur"""
        data = self.read_data(servo_id, ADDR_PRESENT_POSITION_L, 2)
        if data and len(data) >= 2:
            position = data[0] | (data[1] << 8)
            return position
        return None
    
    def get_servo_status(self, servo_id):
        """Retourne le statut complet d'un servomoteur"""
        if servo_id not in self.detected_servos:
            return None
        
        status = self.detected_servos[servo_id].copy()
        current_pos = self.get_present_position(servo_id)
        if current_pos is not None:
            status['current_position'] = current_pos
            status['current_degrees'] = self.position_to_degrees(current_pos)
        
        return status
    
    def is_servo_powered(self, servo_id):
        """Vérifie si un servomoteur est sous tension"""
        return servo_id in self.detected_servos and self.detected_servos[servo_id]['powered']
    
    def get_detected_servos(self):
        """Retourne la liste des servomoteurs détectés"""
        return list(self.detected_servos.keys())