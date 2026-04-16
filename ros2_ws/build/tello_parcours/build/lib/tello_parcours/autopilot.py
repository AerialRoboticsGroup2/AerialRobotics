import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import time

class TelloAutoPilot(Node):
    def __init__(self):
        super().__init__('tello_autopilot')
        # We maken een 'publisher' aan die bewegingscommando's (Twist) naar de drone stuurt
        self.publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.get_logger().info("Autopilot Node is gestart!")

    def stuur_actie(self, commando):
        # We bootsen even jouw terminal commando na voor de zekerheid
        cmd = f'ros2 service call /drone1/tello_action tello_msgs/srv/TelloAction "{{cmd: \'{commando}\'}}"'
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL)

    def vlieg(self, x_snelheid, y_snelheid, z_snelheid, draai_snelheid, seconden):
        msg = Twist()
        msg.linear.x = float(x_snelheid)   # Vooruit/achteruit
        msg.linear.y = float(y_snelheid)   # Links/rechts schuiven
        msg.linear.z = float(z_snelheid)   # Omhoog/omlaag
        msg.angular.z = float(draai_snelheid) # Om as draaien
        
        self.publisher_.publish(msg)
        time.sleep(seconden)
        
        # Na de tijd stoppen we met bewegen (alles op 0)
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    autopilot = TelloAutoPilot()

    print("--- START MISSIE ---")
    
    # 1. Opstijgen
    autopilot.get_logger().info("1. Opstijgen...")
    autopilot.stuur_actie('takeoff')
    time.sleep(4) # Wacht 4 seconden tot hij stabiel in de lucht hangt

    # 2. Naar voren vliegen (snelheid 0.5 voor 3 seconden)
    autopilot.get_logger().info("2. Vlieg door de (denkbeeldige) eerste hoepel...")
    autopilot.vlieg(x_snelheid=0.5, y_snelheid=0.0, z_snelheid=0.0, draai_snelheid=0.0, seconden=3.0)
    time.sleep(1) # Even 1 seconde stilhangen

    # 3. Landen
    autopilot.get_logger().info("3. Missie voltooid, we gaan landen...")
    autopilot.stuur_actie('land')

    print("--- MISSIE EINDE ---")

    autopilot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
