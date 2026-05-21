# Import OpenCV library
# Used for image processing and computer vision
import cv2

# Import NumPy library
# Used for numerical arrays and matrix operations
import numpy as np


# Define GateDetector class
# This class handles gate detection using computer vision
class GateDetector:

    # Function to detect a gate from one camera frame
    # Input:
    #   frame -> image captured from drone camera
    # Output:
    #   dictionary containing gate information
    #   OR None if no gate is detected
    def detect_gate(self, frame):

        # Convert image from BGR color space to HSV color space
        # HSV is better for color-based object detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define lower HSV boundary for red color (range 1)
        # HSV values:
        # H = hue
        # S = saturation
        # V = brightness
        lower_red1 = np.array([0, 120, 70])

        # Define upper HSV boundary for red color (range 1)
        upper_red1 = np.array([10, 255, 255])

        # Define second lower HSV boundary for red color
        # Red wraps around HSV hue range, so two ranges are needed
        lower_red2 = np.array([170, 120, 70])

        # Define second upper HSV boundary for red color
        upper_red2 = np.array([180, 255, 255])

        # Create binary mask for first red range
        # Pixels inside red range become white
        # Other pixels become black
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        # Create binary mask for second red range
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # Combine both masks into one final mask
        # Detects all red regions in image
        mask = mask1 + mask2

        # Find contours (object boundaries) in the mask
        # RETR_EXTERNAL:
        #   retrieve only outer contours
        #
        # CHAIN_APPROX_SIMPLE:
        #   compress contour points to save memory
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # If no contours are found,
        # no gate is detected
        if len(contours) == 0:

            # Return None to indicate failure
            return None

        # Find contour with largest area
        # Assumes the gate is the largest red object
        largest = max(contours, key=cv2.contourArea)

        # Compute contour area
        # Used to filter out small noisy detections
        area = cv2.contourArea(largest)

        # Ignore very small contours
        # Helps remove noise and false detections
        if area < 2000:

            # Return None if detected object is too small
            return None

        # Compute bounding rectangle around contour
        #
        # x = left coordinate
        # y = top coordinate
        # w = width
        # h = height
        x, y, w, h = cv2.boundingRect(largest)

        # Compute horizontal center of gate
        center_x = x + w // 2

        # Compute vertical center of gate
        center_y = y + h // 2

        # Return gate information as dictionary
        return {

            # Horizontal center coordinate
            "x": center_x,

            # Vertical center coordinate
            "y": center_y,

            # Contour area
            "area": area,

            # Bounding box coordinates
            # Format:
            # (x, y, width, height)
            "bbox": (x, y, w, h)
        }
