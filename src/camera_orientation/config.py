"""
Constantes et adresses des registres pour le servomoteur STS3032
Configuration pour deux moteurs sur URT-1
"""

# Constantes pour le protocole SCS
HEADER = [0xFF, 0xFF]

# Adresses des registres essentiels
ADDR_ID = 5  # Adresse pour modifier l'ID du moteur
ADDR_TORQUE_ENABLE = 40
ADDR_ACC = 41
ADDR_GOAL_POSITION_L = 42
ADDR_GOAL_POSITION_H = 43
ADDR_GOAL_SPEED_L = 46
ADDR_GOAL_SPEED_H = 47
ADDR_MODE = 26
ADDR_MAX_TORQUE_L = 14
ADDR_MAX_TORQUE_H = 15
ADDR_PRESENT_POSITION_L = 56
ADDR_PRESENT_POSITION_H = 57
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63

# Instructions du protocole
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

# Paramètres de conversion
POSITION_RESOLUTION = 4095
DEGREES_RANGE = 360.0  # -180° à +180°

# Configuration pour deux moteurs
MOTOR1_ID = 1
MOTOR2_ID = 2
BROADCAST_ID = 254  # ID pour commander tous les moteurs

# Valeurs par défaut
DEFAULT_PORT = "COM8"
DEFAULT_BAUDRATE = 1000000
DEFAULT_SERVO_ID = 1
DEFAULT_TIMEOUT = 0.5

# IDs possibles pour la détection
POSSIBLE_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# Configuration des plages d'angles par moteur
SERVO_ANGLE_LIMITS = {
    # Moteurs avec plage limitée
    2: {"min": -35, "max": 110},
    4: {"min": -35, "max": 110},
    # Les autres moteurs gardent la plage complète par défaut
    "default": {"min": -180, "max": 180}
}

def get_servo_limits(servo_id):
    """Retourne les limites d'angle pour un servo donné"""
    return SERVO_ANGLE_LIMITS.get(servo_id, SERVO_ANGLE_LIMITS["default"])