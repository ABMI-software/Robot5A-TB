import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # Configuration du port série
        self.PORT = "COM4"  # Port utilisé par la Nucleo
        self.BAUDRATE = 9600
        
        # Initialisation de la communication série
        try:
            self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout=1)
            self.get_logger().info(f"Connexion série établie avec {self.PORT} à {self.BAUDRATE} bps.")
            time.sleep(2)  # Temps pour s'assurer que la connexion est prête
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la connexion série : {e}")
            exit(1)

        # Variables globales pour le suivi
        self.positions_mode1 = []
        self.positions_mode2 = []
        self.aller_retours_mode3 = 0
        self.vitesse_actuelle = 0  # Stocke la vitesse actuelle
        self.debut_test = time.time()  # Temps de début pour le feedback

        # Create a timer to periodically check for commands
        self.timer = self.create_timer(1.0, self.check_for_commands)

    def envoyer_commande(self, commande):
        """Envoyer une commande à la Nucleo et lire la réponse."""
        self.ser.write((commande + "\n").encode())
        time.sleep(0.1)
        while self.ser.in_waiting > 0:
            response = self.ser.readline().decode().strip()
            self.get_logger().info(response)
            return response

    def check_for_commands(self):
        # Here you can implement a way to check for commands from a topic or service
        pass

    def mode_manuel(self):
        """Mode manuel pour définir des positions spécifiques."""
        self.get_logger().info("Mode manuel activé. Entrez une position en mm ou 'S' pour arrêter.")
        self.envoyer_commande('1')  # Activer le mode manuel
        # Implement a way to receive positions via a ROS2 topic or service

    def mode_aleatoire(self):
        """Mode aléatoire pour des déplacements aléatoires."""
        self.envoyer_commande('2')  # Activer le mode aléatoire
        # Implement a way to receive the number of random movements via a ROS2 topic or service

    def mode_aller_retour(self):
        """Mode aller-retour entre deux positions limites."""
        self.envoyer_commande('3')  # Activer le mode aller-retour
        # Implement a way to stop the mode via a ROS2 topic or service

    def ajuster_vitesse(self):
        """Ajuster la vitesse de déplacement."""
        # Implement a way to receive speed adjustments via a ROS2 topic or service

    def retourner_etat_initial(self):
        """Retourner le chariot à la position initiale."""
        self.envoyer_commande('R')

    def afficher_feedback(self):
        """Afficher un récapitulatif des actions effectuées."""
        self.get_logger().info(f"Positions atteintes en mode manuel : {self.positions_mode1}")
        self.get_logger().info(f"Positions atteintes en mode aléatoire : {self.positions_mode2}")
        self.get_logger().info(f"Nombre d'aller-retours effectués en mode 3 : {self.aller_retours_mode3}")
        self.get_logger().info(f"Vitesse actuelle : {self.vitesse_actuelle} mm/s")
        self.get_logger().info(f"Durée totale du test : {round(time.time() - self.debut_test, 2)} secondes")

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    
    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    finally:
        serial_node.ser.close()
        serial_node.get_logger().info("Connexion série fermée.")
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()