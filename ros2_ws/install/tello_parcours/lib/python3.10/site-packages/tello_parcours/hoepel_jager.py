import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import time

class HoepelJager(Node):
    def __init__(self):
        super().__init__('hoepel_jager')
        
        # Ogen en Spieren koppelen
        self.camera_sub = self.create_subscription(Image, '/drone1/image_raw', self.camera_callback, 10)
        self.motor_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.bridge = CvBridge()

        # Het geheugen van de drone
        self.missie_staat = "OPSTIJGEN"
        self.afwijking_x = 0
        self.hoepel_gevonden = False
        self.hoepel_grootte = 0
        
        # Start het "denk-ritme" van de drone (10 keer per seconde)
        self.denk_timer = self.create_timer(0.1, self.brein_loop)

        self.get_logger().info("Missie gestart: Hoepel Jager is online!")

    def stuur_actie(self, commando):
        cmd = f'ros2 service call /drone1/tello_action tello_msgs/srv/TelloAction "{{cmd: \'{commando}\'}}"'
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL)

    def camera_callback(self, msg):
        # Dit is exact dezelfde visie-code als daarnet!
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            ondergrens_groen = np.array([40, 40, 40])
            bovengrens_groen = np.array([80, 255, 255])
            masker = cv2.inRange(hsv_image, ondergrens_groen, bovengrens_groen)
            
            contouren, _ = cv2.findContours(masker, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contouren) > 0:
                grootste_contour = max(contouren, key=cv2.contourArea)
                self.hoepel_grootte = cv2.contourArea(grootste_contour)
                
                if self.hoepel_grootte > 500:
                    self.hoepel_gevonden = True
                    M = cv2.moments(grootste_contour)
                    if M["m00"] != 0:
                        hoepel_x = int(M["m10"] / M["m00"])
                        scherm_midden_x = cv_image.shape[1] // 2
                        
                        # Update het geheugen voor de motoren!
                        self.afwijking_x = hoepel_x - scherm_midden_x
                        
                        # Teken feedback op scherm
                        cv2.circle(cv_image, (hoepel_x, int(M["m01"] / M["m00"])), 5, (0, 0, 255), -1)
                else:
                    self.hoepel_gevonden = False
            else:
                self.hoepel_gevonden = False

            # Teken de huidige missie-staat op het scherm
            cv2.putText(cv_image, f"Status: {self.missie_staat}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("Drone Vizier", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Camera fout: {e}")

    # --- HIER ZITTEN DE ECHTE HERSENS ---
    def brein_loop(self):
        msg = Twist() # Klaarmaken om bewegingscommando's te sturen

        if self.missie_staat == "OPSTIJGEN":
            self.get_logger().info("Opstijgen...")
            self.stuur_actie('takeoff')
            time.sleep(4) # Geef hem even tijd om stabiel te hangen
            self.missie_staat = "VLIEGEN"

        elif self.missie_staat == "VLIEGEN":
            if self.hoepel_gevonden:
                # 1. DE P-REGELAAR: Bereken hoe hard we moeten draaien
                # Formule: Snelheid = Afwijking * Constante
                draai_snelheid = float(self.afwijking_x) * -0.002 
                
                # Begrens de draaisnelheid (zodat hij niet misselijk wordt)
                msg.angular.z = max(-0.5, min(0.5, draai_snelheid)) 

                # 2. Bepaal de voorwaartse snelheid
                if abs(self.afwijking_x) < 40:
                    # Hoepel is mooi in het midden, gas erop!
                    msg.linear.x = 0.5 
                else:
                    # Hoepel is te ver opzij, vlieg langzamer en focus op draaien
                    msg.linear.x = 0.1 

                # 3. Zijn we er doorheen? (Als de hoepel héél groot is op het scherm)
                if self.hoepel_grootte > 120000:
                    self.get_logger().info("Hoepel gepasseerd! We gaan landen.")
                    self.missie_staat = "LANDEN"

            else:
                # Hoepel kwijt! Draai langzaam rondjes om hem te zoeken
                msg.angular.z = 0.3 

            # Stuur de berekende snelheden naar de motoren!
            self.motor_pub.publish(msg)

        elif self.missie_staat == "LANDEN":
            # Zet motoren stil
            self.motor_pub.publish(Twist()) 
            time.sleep(1)
            self.stuur_actie('land')
            self.missie_staat = "KLAAR"

def main(args=None):
    rclpy.init(args=args)
    node = HoepelJager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()  