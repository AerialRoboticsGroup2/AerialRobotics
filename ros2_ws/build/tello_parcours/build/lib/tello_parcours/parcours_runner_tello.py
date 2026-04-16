"""
Parcours Runner — Autonomous green-hoop navigation for a Tello drone.

This ROS 2 node flies a DJI Tello (simulated via tello_ros in Gazebo)
through a parcours of 4 green hoops arranged in a half-circle arc.

State machine
─────────────
  TAKEOFF → STABILIZE → SEARCH → APPROACH → TRAVERSE ─┐
                           ▲                            │
                           └────────────────────────────┘
                                (repeat per hoop)
  After all hoops: → LAND → DONE

Vision
──────
  - HSV thresholding for green detection
  - Morphological cleanup to remove noise
  - Largest-contour tracking (nearest hoop)
  - Centroid error computed in both X (yaw) and Y (altitude)

Control
───────
  - Proportional controller for yaw   (angular.z)
  - Proportional controller for altitude (linear.z)
  - Forward speed gated by alignment quality
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess


class ParcoursRunnerTello(Node):

    # ── Tuneable parameters ─────────────────────────────────────────── #

    TOTAL_HOOPS = 4

    # HSV range for real-world green gate (Calibrated via tuner)
    HSV_LOWER = np.array([35, 74, 43])
    HSV_UPPER = np.array([98, 255, 255])

    # Contour filtering
    MIN_CONTOUR_AREA = 500        # Ignore noise below this
    PASS_THROUGH_AREA = 450000    # Hoop fills screen (bbox) → we flew through
    PASS_CENTER_TOL = 120         # Relaxed error (px) to accept a pass-through
    PASS_CONFIRM_TICKS = 3        # Must stay centered+large for 0.3 s
    MAX_SEARCH_AREA = 450000      # Ignore detections bigger than this while
                                  # searching — prevents re-detecting the
                                  # hoop we just passed through
    SEARCH_COOLDOWN_TICKS = 15    # 1.5 s: only apply area filter for this
                                  # long after entering SEARCH, then accept
                                  # any detection regardless of size

    # Morphological close kernel for contour detection
    # Larger kernel merges segments of dashed hoops into one blob
    MORPH_CLOSE_KERNEL = 15

    # PD-controller gains
    KP_YAW = 0.002
    KD_YAW = 0.005              # Derivative dampens oscillation
    KP_ALT = 0.003              # Slightly higher than yaw for crisp alt
    KD_ALT = 0.004              # Derivative dampens altitude oscillation
    EMA_ALPHA = 0.4             # Error smoothing (0=ignore new, 1=no filter)

    # Speed limits
    MAX_YAW_SPEED = 0.16
    MAX_ALT_SPEED = 0.16
    FORWARD_FAST = 0.12           # Increased (was 0.04): 12% power is more realistic
    FORWARD_SLOW = 0.08           # Increased (was 0.03)
    TRAVERSE_SPEED = 0.20         # Increased (was 0.15)
    SEARCH_SPEED = 0.05           # Increased (was 0.02)

    # Alignment tolerance (pixels from frame center)
    ALIGN_TOL_X = 70              # Relaxed
    ALIGN_TOL_Y = 60              # Relaxed
    COARSE_TOL_X = 200            # Relaxed
    COARSE_TOL_Y = 150            # Relaxed (Was 80): Don't stop moving forward
                                  # just because we are slightly high/low.

    # Proximity braking — slow down as hoop gets bigger (= closer)
    BRAKE_AREA_SOFT = 150000      # Start slowing down
    BRAKE_AREA_HARD = 250000      # Almost stopped, just drifting through

    # Timing — all values are ticks at 10 Hz  (1 tick = 0.1 s)
    STABILIZE_TICKS = 40          # 4 s hover after takeoff
    LAND_DELAY_TICKS = 10         # 1 s hover before landing
    LOST_PATIENCE_TICKS = 20      # 2 s before declaring "hoop lost"
    TRAVERSE_TICKS = 12           # 1.2 s extra gentle push to clear hoop
    SWEEP_LEG_1 = 40              # ~24 degrees first leg
    SWEEP_LEG_2 = 80              # ~48 degrees back
    MAX_SEARCH_TICKS = 400        # 40 s: plenty of time for calm sweeping

    # ── Constructor ─────────────────────────────────────────────────── #

    def __init__(self):
        super().__init__('parcours_runner_tello')

        # Define QoS profile compatible with the real drone (High frequency, Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # ROS interfaces (using generic topics, usually overridable by namespaces)
        self.camera_sub = self.create_subscription(
            Image, '/image_raw', self._camera_cb, qos_profile)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # Check if we can show GUI windows (not available in headless WSL)
        self.gui_available = self._check_gui()

        # Vision state (written by camera callback, read by control loop)
        self.hoop_detected = False
        self.error_x = 0          # >0 → hoop is right of center
        self.error_y = 0          # >0 → hoop is below center
        self.hoop_area = 0

        # Throttle for console logging (avoid flooding)
        self._log_counter = 0

        # Mission state
        self.state = 'INIT'
        self.hoops_passed = 0
        self.ticks = 0            # Multipurpose tick counter
        self.search_ticks = 0     # Dedicated sweep timer

        # Search direction: +1 = rotate right, −1 = rotate left
        self.search_dir = 1
        self.last_successful_search_dir = 1  # Updated when a hoop is found

        # Counter for frames without detection in APPROACH (avoids
        # flipping back to SEARCH on a single dropped frame)
        self.lost_ticks = 0

        # Multi-frame pass-through confirmation counter
        self.pass_confirm_count = 0

        # Previous errors for derivative terms in PD controllers
        self.prev_ctrl_error_x = 0
        self.prev_ctrl_error_y = 0

        # EMA-filtered errors for smooth control output
        self.filt_error_x = 0.0
        self.filt_error_y = 0.0

        # Search cooldown counter (ticks since entering SEARCH)
        self.search_cooldown = 0

        # 10 Hz control loop
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info('══════════════════════════════════')
        self.get_logger().info('   Parcours Runner — ONLINE')
        self.get_logger().info(f'   Doel: {self.TOTAL_HOOPS} hoepels')
        if not self.gui_available:
            self.get_logger().info('   GUI: niet beschikbaar (headless modus)')
        self.get_logger().info('══════════════════════════════════')

    # ── Tello action (non-blocking) ─────────────────────────────────── #

    def _tello_action(self, command: str):
        """Send takeoff / land via the tello_action service."""
        cmd = (
            "ros2 service call /tello_action "
            f"tello_msgs/srv/TelloAction \"{{cmd: '{command}'}}\""
        )
        subprocess.Popen(
            cmd, shell=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # ── GUI availability check ───────────────────────────────────────── #

    @staticmethod
    def _check_gui() -> bool:
        """Try to open a tiny test window.  Returns False on headless systems."""
        try:
            cv2.namedWindow('__test__', cv2.WINDOW_NORMAL)
            cv2.destroyWindow('__test__')
            cv2.waitKey(1)
            return True
        except cv2.error:
            return False

    # ── Camera callback (vision only, no actuation) ─────────────────── #

    def _camera_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2

            # --- green mask ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)
            # Small open to remove speckle noise
            k_open = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
            # --- Robust Bounding Box Isolation ---
            # Downscale mask by 4 to cleanly and rapidly bridge massive
            # gaps between dashed hoop segments (scale=4 -> 15x15 kernel = 60px bridge)
            scale = 4
            small_mask = cv2.resize(mask, (w // scale, h // scale), interpolation=cv2.INTER_NEAREST)
            
            k_fuse = np.ones((self.MORPH_CLOSE_KERNEL, self.MORPH_CLOSE_KERNEL), np.uint8)
            fused_mask = cv2.morphologyEx(small_mask, cv2.MORPH_CLOSE, k_fuse)
            
            # Find the fused blobs (each blob represents a fully isolated hoop)
            contours, _ = cv2.findContours(
                fused_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            detected = False
            if contours:
                # Find the fused hoop with the absolute largest BOUNDING BOX area.
                best_c = max(contours, key=lambda c: cv2.boundingRect(c)[2] * cv2.boundingRect(c)[3])
                bx, by, bw, bh = cv2.boundingRect(best_c)
                
                # Scale the bounding box back up perfectly
                bx *= scale
                by *= scale
                bw *= scale
                bh *= scale
                
                bbox_area = bw * bh
                
                if bbox_area > self.MIN_CONTOUR_AREA * 10:
                    hx = bx + bw // 2
                    hy = by + bh // 2
                    
                    self.error_x = int(hx - cx)
                    self.error_y = int(hy - cy)
                    self.hoop_area = bbox_area
                    detected = True

            self.hoop_detected = detected

            # --- Visual output (GUI or console) ---
            if self.gui_available:
                self._draw_gui(frame, mask, cx, cy, h, w, detected)
            else:
                self._log_vision(detected)

        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')

    # ── GUI drawing (only called when display is available) ─────────── #

    def _draw_gui(self, frame, mask, cx, cy, h, w, detected):
        """Draw HUD overlays and show debug windows."""
        if detected:
            hx, hy = cx + self.error_x, cy + self.error_y
            cv2.circle(frame, (hx, hy), 6, (0, 0, 255), -1)
            cv2.line(frame, (cx, cy), (hx, hy), (0, 255, 255), 2)

        cv2.line(frame, (cx, 0), (cx, h), (255, 0, 0), 1)
        cv2.line(frame, (0, cy), (w, cy), (255, 0, 0), 1)
        hud = (f'{self.state}  |  '
               f'Hoepels: {self.hoops_passed}/{self.TOTAL_HOOPS}')
        cv2.putText(frame, hud, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        if detected:
            info = (f'err_x:{self.error_x:+d}  '
                    f'err_y:{self.error_y:+d}  '
                    f'area:{int(self.hoop_area)}')
            cv2.putText(frame, info, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50,
                        (255, 255, 255), 1)

        cv2.imshow('Parcours Runner', frame)
        cv2.imshow('Green Mask', mask)
        cv2.waitKey(1)

    # ── Console logging fallback (headless mode) ───────────────────── #

    def _log_vision(self, detected):
        """Print vision state to the terminal every ~0.5 s (every 5th frame)."""
        self._log_counter += 1
        if self._log_counter % 5 != 0:
            return
        if detected:
            self.get_logger().info(
                f'[VISION] hoepel: err_x={self.error_x:+d}  '
                f'err_y={self.error_y:+d}  area={int(self.hoop_area)}')
        else:
            self.get_logger().info('[VISION] geen hoepel gedetecteerd')

    # ── Control loop (state machine) ────────────────────────────────── #

    def _control_loop(self):
        cmd = Twist()

        # ---- INIT --------------------------------------------------- #
        if self.state == 'INIT':
            self.ticks += 1
            if self.ticks >= 20:
                self.state = 'TAKEOFF'
                self.ticks = 0

        # ---- TAKEOFF ------------------------------------------------ #
        elif self.state == 'TAKEOFF':
            self.get_logger().info('Opstijgen...')
            self._tello_action('takeoff')
            self.state = 'STABILIZE'
            self.ticks = 0

        # ---- STABILIZE ---------------------------------------------- #
        elif self.state == 'STABILIZE':
            self.ticks += 1
            if self.ticks >= self.STABILIZE_TICKS:
                self.get_logger().info(
                    'Gestabiliseerd — zoeken naar hoepel 1')
                self.state = 'SEARCH'
                self.ticks = 0
                self.search_ticks = 0
                self.search_cooldown = 0

        # ---- SEARCH ------------------------------------------------- #
        elif self.state == 'SEARCH':
            self.search_cooldown += 1

            # Time-decaying area filter: only reject large detections
            # during the first SEARCH_COOLDOWN_TICKS after entering
            # SEARCH.  After the cooldown, accept any size — this
            # prevents permanently filtering out a nearby next hoop.
            if self.search_cooldown <= self.SEARCH_COOLDOWN_TICKS:
                new_hoop = (self.hoop_detected and
                            self.hoop_area < self.MAX_SEARCH_AREA)
            else:
                new_hoop = self.hoop_detected

            if new_hoop:
                # We see a hoop — but is it centered enough for APPROACH?
                if abs(self.error_x) < 120:
                    # Hoop is roughly centered → start approaching
                    self.get_logger().info(
                        f'Hoepel {self.hoops_passed + 1} uitgelijnd '
                        f'(area={int(self.hoop_area)}, '
                        f'err_x={self.error_x:+d}) — naderen')
                    self.last_successful_search_dir = self.search_dir
                    self.state = 'APPROACH'
                    self.lost_ticks = 0
                    self.pass_confirm_count = 0
                    # Seed PD and EMA with current reading
                    self.prev_ctrl_error_x = self.error_x
                    self.prev_ctrl_error_y = self.error_y
                    self.filt_error_x = float(self.error_x)
                    self.filt_error_y = float(self.error_y)
                else:
                    # Hoop spotted but far off-center — rotate toward it
                    # WITHOUT forward speed.  Proportional rotation so
                    # we don't overshoot center during the coarse align.
                    rotate_speed = min(0.3, abs(self.error_x) * 0.0012)
                    rotate_dir = -1 if self.error_x > 0 else 1
                    cmd.angular.z = rotate_speed * rotate_dir
            else:
                # No hoop in sight — oscillating sweep search
                cmd.angular.z = self.SEARCH_SPEED * self.search_dir

                # Drone should remain perfectly in place while sweeping
                # (Forward drift removed to prevent flying past hoops)
                cmd.linear.x = 0.0

                self.search_ticks += 1

                # Sweep logic: scan up to ~17 degrees one way, then 34 degrees back.
                # Keeps the drone looking mostly forward for a straight/wavy line of hoops.
                if self.search_ticks == self.SWEEP_LEG_1:
                    self.search_dir *= -1
                    self.get_logger().info('Zoekrichting omgedraaid (sweep rand bereikt)')
                elif self.search_ticks >= self.SWEEP_LEG_2:
                    self.search_dir *= -1
                    self.search_ticks = self.SWEEP_LEG_1  # Keep bounding the sweep
                    self.get_logger().info('Zoekrichting omgedraaid (sweep rand bereikt)')

                # Search timeout: if we've swept for MAX_SEARCH_TICKS
                # without finding anything, there's nothing in range.
                if self.search_ticks >= self.MAX_SEARCH_TICKS:
                    self.get_logger().info(
                        f'Geen hoepel gevonden na {self.MAX_SEARCH_TICKS / 10:.0f}s '
                        f'— landen (hoepels doorgevlogen: {self.hoops_passed})')
                    self.state = 'LAND'
                    self.ticks = 0

        # ---- APPROACH ----------------------------------------------- #
        elif self.state == 'APPROACH':
            if not self.hoop_detected:
                self.lost_ticks += 1
                self.pass_confirm_count = 0  # Reset confirmation
                if self.lost_ticks >= self.LOST_PATIENCE_TICKS:
                    self.get_logger().info(
                        'Hoepel kwijt — opnieuw zoeken')
                    self.state = 'SEARCH'
                    self.search_cooldown = 0
                    self.ticks = 0
                # While "lost" we keep the last command → coast
            else:
                self.lost_ticks = 0

                # --- Pass-through detection (multi-frame) ---
                # Require BOTH massive area AND roughly centered. Since we merge bounding boxes,
                # the center is highly accurate even when clipping.
                centered = (abs(self.error_x) < self.PASS_CENTER_TOL and
                            abs(self.error_y) < self.PASS_CENTER_TOL)
                if self.hoop_area > self.PASS_THROUGH_AREA and centered:
                    self.pass_confirm_count += 1
                else:
                    self.pass_confirm_count = 0

                if self.pass_confirm_count >= self.PASS_CONFIRM_TICKS:
                    self.hoops_passed += 1
                    self.pass_confirm_count = 0
                    self.get_logger().info(
                        f'★ Hoepel {self.hoops_passed}/{self.TOTAL_HOOPS} '
                        f'doorgevlogen! (area={int(self.hoop_area)}, '
                        f'err_x={self.error_x:+d}, err_y={self.error_y:+d})')

                    self.state = 'TRAVERSE'
                    self.ticks = 0
                    # Start next search from the direction that worked
                    self.search_dir = self.last_successful_search_dir
                    return  # skip publishing below

                # --- EMA-filtered errors for control ---
                a = self.EMA_ALPHA
                self.filt_error_x = a * self.error_x + (1 - a) * self.filt_error_x
                self.filt_error_y = a * self.error_y + (1 - a) * self.filt_error_y
                fe_x = self.filt_error_x
                fe_y = self.filt_error_y

                # --- PD-controller: yaw (on filtered signal) ---
                d_error_x = fe_x - self.prev_ctrl_error_x
                yaw = (-self.KP_YAW * fe_x - self.KD_YAW * d_error_x)
                yaw = max(-self.MAX_YAW_SPEED, min(self.MAX_YAW_SPEED, yaw))
                self.prev_ctrl_error_x = fe_x

                # --- PD-controller: altitude (on filtered signal) ---
                d_error_y = fe_y - self.prev_ctrl_error_y
                alt = (-self.KP_ALT * fe_y - self.KD_ALT * d_error_y)
                alt = max(-self.MAX_ALT_SPEED, min(self.MAX_ALT_SPEED, alt))
                self.prev_ctrl_error_y = fe_y

                cmd.angular.z = yaw
                cmd.linear.z = alt

                # --- Forward speed (alignment + proximity braking) ---
                # Use filtered error for alignment assessment
                far_off_x = abs(fe_x) > self.COARSE_TOL_X
                far_off_y = abs(fe_y) > self.COARSE_TOL_Y
                well_aligned = (abs(fe_x) < self.ALIGN_TOL_X and
                                abs(fe_y) < self.ALIGN_TOL_Y)

                # If vertically far off, STOP forward motion entirely
                # and let the altitude controller fix the height first.
                if far_off_y:
                    fwd = 0.0
                elif far_off_x:
                    fwd = 0.03
                elif well_aligned:
                    fwd = self.FORWARD_FAST
                else:
                    fwd = self.FORWARD_SLOW

                # 2. Proximity brake — the closer the hoop, the slower
                #    we fly so we have time to align before passing through
                if self.hoop_area > self.BRAKE_AREA_HARD:
                    fwd = min(fwd, 0.03)
                elif self.hoop_area > self.BRAKE_AREA_SOFT:
                    fwd = min(fwd, 0.06)

                cmd.linear.x = fwd

        # ---- TRAVERSE ----------------------------------------------- #
        elif self.state == 'TRAVERSE':
            # Fly straight through the hoop.
            # STRICTLY no turning so we don't crash into the ring.
            # Gentle push to comfortably clear the rim based on TRAVERSE_SPEED.
            self.ticks += 1
            cmd.linear.x = self.TRAVERSE_SPEED

            # (No altitude correction here, as clipping creates a false center on the rim)            
            # The final hoop needs a longer straight push (3.5s) to guarantee
            # the tail of the drone clears the ring before we land. Intermediate
            # hoops use a 2.5 s push to clear the ring before searching
            # for the next one.
            req_ticks = 35 if self.hoops_passed >= self.TOTAL_HOOPS else self.TRAVERSE_TICKS

            if self.ticks >= req_ticks:
                if self.hoops_passed >= self.TOTAL_HOOPS:
                    self.state = 'LAND'
                    self.ticks = 0
                else:
                    self.get_logger().info(
                        f'Hoepel gepasseerd! Zoeken naar hoepel {self.hoops_passed + 1}...')
                    self.state = 'SEARCH'
                    self.search_ticks = 0
                    self.search_cooldown = 0
                    self.ticks = 0

        # ---- LAND --------------------------------------------------- #
        elif self.state == 'LAND':
            self.ticks += 1
            if self.ticks >= self.LAND_DELAY_TICKS:
                self.get_logger().info(
                    '══ Alle hoepels doorgevlogen!  Landen... ══')
                self._tello_action('land')
                self.state = 'DONE'

        # ---- DONE --------------------------------------------------- #
        elif self.state == 'DONE':
            return  # Nothing left to do

        # Publish velocity command
        if self.state not in ('TAKEOFF', 'DONE'):
            self.cmd_pub.publish(cmd)


# ── Entry point ──────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = ParcoursRunnerTello()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the drone stops on shutdown
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        if node.gui_available:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
