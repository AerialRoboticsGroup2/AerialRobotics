import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np # Deze hebben we nodig voor het rekenwerk met kleuren!

class TelloCamera(Node):
    def __init__(self):
        super().__init__('tello_camera_viewer')
        self.subscription = self.create_subscription(Image, '/drone1/image_raw', self.camera_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Slimme Camera gestart! Aan het zoeken naar groene hoepels...")

    def camera_callback(self, msg):
        try:
            # 1. Haal het normale beeld op
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hoogte, breedte, _ = cv_image.shape
            
            # Midden van het scherm (het vizier)
            scherm_midden_x = breedte // 2
            scherm_midden_y = hoogte // 2

            # 2. Zet beeld om naar HSV voor betere kleurherkenning
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # 3. Definieer wat "Gazebo Groen" is in HSV
            ondergrens_groen = np.array([40, 40, 40])
            bovengrens_groen = np.array([80, 255, 255])

            # 4. Maak een "masker": Alles wat groen is wordt wit, de rest wordt zwart
            masker = cv2.inRange(hsv_image, ondergrens_groen, bovengrens_groen)

            # 5. Zoek de contouren (randen) van de witte vlekken in het masker
            contouren, _ = cv2.findContours(masker, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contouren) > 0:
                # Pak de grootste groene vlek (we gaan ervan uit dat dit de dichtstbijzijnde hoepel is)
                grootste_contour = max(contouren, key=cv2.contourArea)
                
                # Als de vlek groot genoeg is (geen ruis)
                if cv2.contourArea(grootste_contour) > 500:
                    # Bereken het exacte middelpunt van de hoepel
                    M = cv2.moments(grootste_contour)
                    if M["m00"] != 0:
                        hoepel_x = int(M["m10"] / M["m00"])
                        hoepel_y = int(M["m01"] / M["m00"])

                        # Teken een rode stip in het midden van de hoepel!
                        cv2.circle(cv_image, (hoepel_x, hoepel_y), 5, (0, 0, 255), -1)
                        
                        # Teken een kader om de hoepel
                        x, y, w, h = cv2.boundingRect(grootste_contour)
                        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                        # Bereken de afwijking (error): Hoe ver zit de hoepel van ons vizier af?
                        afwijking_x = hoepel_x - scherm_midden_x
                        
                        # Laat de afwijking zien op het scherm
                        cv2.putText(cv_image, f"Afwijking X: {afwijking_x} px", (20, 40), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        
                        # Teken een lijn van het midden van het scherm naar de hoepel
                        cv2.line(cv_image, (scherm_midden_x, scherm_midden_y), (hoepel_x, hoepel_y), (0, 255, 255), 2)

            # Vizier tekenen
            cv2.line(cv_image, (scherm_midden_x, 0), (scherm_midden_x, hoogte), (255, 0, 0), 1)
            cv2.line(cv_image, (0, scherm_midden_y), (breedte, scherm_midden_y), (255, 0, 0), 1)

            # Laat beide schermen zien (het normale en hoe de drone 'denkt' in zwart-wit)
            cv2.imshow("Tello Drone Ogen", cv_image)
            cv2.imshow("Wat de drone ziet (Masker)", masker)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Fout: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelloCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()