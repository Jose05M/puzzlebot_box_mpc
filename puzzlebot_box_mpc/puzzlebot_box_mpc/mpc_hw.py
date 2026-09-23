import math
import time
from enum import Enum

import cv2
import numpy as np

from rclpy import qos
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import csv

class State(Enum):
    WAIT_COMMAND = 1
    GO_TO_VIEWPOINT = 2
    TRACK_TARGET = 3
    COLLECTING = 4
    RETURN_HOME = 5
    FINISHED = 6
    WAIT_NEXT_ACTION = 7
    GO_TO_WAYPOINT = 8


class PuzzlebotBoxMPCNode(Node):
    def __init__(self):
        super().__init__("puzzlebot_box_mpc_hw_node")

        # ============================================================
        # PARÁMETROS ROS
        # ============================================================

        self.log_file = open("mpc_results.csv", "w", newline="")
        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            "time",
            "rho",
            "alpha",
            "v_cmd",
            "w_cmd",
            "cost",
            "x",
            "y",
            "theta",
            "state",
            "target_detected"
        ])

        self.start_time = time.time()

        
        self.rho_history =[]
        self.rho_filter_n = 8

        self.declare_parameter("image_topic", "/video_source/raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("color_command_topic", "/box_color_command")
        self.declare_parameter("odom_topic", "/odom")

        # Seguridad: si está en False, calcula MPC pero NO mueve el robot.
        self.declare_parameter("enable_motion", True)

        # Si está en True, después de recibir color avanza por tiempo
        # hacia la zona de observación.
        self.declare_parameter("viewpoint_drive_enabled", False)
        self.declare_parameter("viewpoint_drive_time", 3.0)
        self.declare_parameter("viewpoint_drive_speed", 0.08)

        # Parámetros de cámara / caja.
        self.declare_parameter("focal_px", 1300.0)
        self.declare_parameter("box_width_m", 0.06)

        # Distancia deseada para detenerse frente a la caja.
        self.declare_parameter("rho_ref", 0.10)

        # Parámetros de detección.
        self.declare_parameter("min_area", 800)

        # Constantes calibradas con su CSV.
        self.declare_parameter("k_alpha", 0.5247550903726591)
        self.declare_parameter("k_rho", 0.9653389089777799)

        # Tiempos y límites del MPC.
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("horizon", 10)

        self.declare_parameter("v_max", 0.12)
        self.declare_parameter("w_max", 0.30)

        # ============================================================
        # PARÁMETROS PARA REGRESO A HOME / WAYPOINT POR ODOMETRÍA
        # ============================================================

        self.declare_parameter("return_enabled", True)

        self.declare_parameter("return_distance_tolerance", 0.08)
        self.declare_parameter("return_angle_tolerance_deg", 8.0)

        self.declare_parameter("return_v_max", 0.10)
        self.declare_parameter("return_w_max", 0.60)

        self.declare_parameter("k_return_distance", 0.55)
        self.declare_parameter("k_return_angle", 1.60)
        self.declare_parameter("k_return_final_angle", 1.20)

        # ============================================================
        # LECTURA DE PARÁMETROS
        # ============================================================

        self.image_topic = self.get_parameter("image_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.color_command_topic = self.get_parameter("color_command_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value

        self.enable_motion = bool(self.get_parameter("enable_motion").value)

        self.viewpoint_drive_enabled = bool(
            self.get_parameter("viewpoint_drive_enabled").value
        )
        self.viewpoint_drive_time = float(
            self.get_parameter("viewpoint_drive_time").value
        )
        self.viewpoint_drive_speed = float(
            self.get_parameter("viewpoint_drive_speed").value
        )

        self.focal_px = float(self.get_parameter("focal_px").value)
        self.box_width_m = float(self.get_parameter("box_width_m").value)
        self.rho_ref = float(self.get_parameter("rho_ref").value)
        self.min_area = int(self.get_parameter("min_area").value)

        self.k_alpha = float(self.get_parameter("k_alpha").value)
        self.k_rho = float(self.get_parameter("k_rho").value)

        self.dt = float(self.get_parameter("dt").value)
        self.horizon = int(self.get_parameter("horizon").value)

        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.return_enabled = bool(self.get_parameter("return_enabled").value)

        self.return_distance_tolerance = float(
            self.get_parameter("return_distance_tolerance").value
        )

        self.return_angle_tolerance = math.radians(
            float(self.get_parameter("return_angle_tolerance_deg").value)
        )

        self.return_v_max = float(self.get_parameter("return_v_max").value)
        self.return_w_max = float(self.get_parameter("return_w_max").value)

        self.k_return_distance = float(
            self.get_parameter("k_return_distance").value
        )
        self.k_return_angle = float(
            self.get_parameter("k_return_angle").value
        )
        self.k_return_final_angle = float(
            self.get_parameter("k_return_final_angle").value
        )

        # ============================================================
        # ROS
        # ============================================================

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.color_sub = self.create_subscription(
            String,
            self.color_command_topic,
            self.color_command_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos.qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # Timer para imprimir pose en vivo cada segundo
        self.create_timer(1.0, self.print_pose_callback)

        # ============================================================
        # ESTADO DEL SISTEMA
        # ============================================================

        self.state = State.WAIT_COMMAND
        self.target_color = None

        self.state_start_time = time.time()

        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0

        self.target_lost_counter = 0
        self.max_lost_frames = 10

        # Odometría actual
        self.current_pose = None  # (x, y, theta)

        # Home siempre es (0, 0, 0)
        self.home_pose = (0.0, 0.0, 0.0)

        # Waypoint objetivo para GO_TO_WAYPOINT
        self.waypoint_pose = None  # (x, y) — sin corrección de ángulo final

        # ============================================================
        # COLORES HSV: verde, rosa, amarillo
        # ============================================================

        self.color_ranges = {
            "green": [
                (np.array([35, 70, 100]), np.array([90, 255, 255]))
            ],
            "pink": [
                (np.array([140, 60, 100]), np.array([175, 255, 255]))
            ],
            "yellow": [
                (np.array([20, 80, 120]), np.array([38, 255, 255]))
            ]
        }

        self.color_draw = {
            "green": (0, 255, 0),
            "pink": (255, 0, 255),
            "yellow": (0, 255, 255)
        }

        self.get_logger().info("Nodo Puzzlebot Box MPC iniciado.")
        self.get_logger().info(f"Imagen: {self.image_topic}")
        self.get_logger().info(f"Cmd_vel: {self.cmd_vel_topic}")
        self.get_logger().info(f"Comando color: {self.color_command_topic}")
        self.get_logger().info(f"Odometría: {self.odom_topic}")
        self.get_logger().info(f"enable_motion = {self.enable_motion}")
        self.get_logger().info(f"return_enabled = {self.return_enabled}")
        self.get_logger().info(f"focal_px = {self.focal_px}")
        self.get_logger().info(f"box_width_m = {self.box_width_m}")
        self.get_logger().info(f"K_alpha = {self.k_alpha}")
        self.get_logger().info(f"K_rho = {self.k_rho}")
        self.get_logger().info("Home fijo: (0.0, 0.0, 0.0)")

        print("\n==========================================")
        print("PUZZLEBOT BOX MPC NODE + RETURN HOME")
        print("==========================================")
        print("Teclas dentro de la ventana OpenCV:")
        print("g = objetivo verde")
        print("p = objetivo rosa")
        print("y = objetivo amarillo")
        print("c = cancelar / esperar nuevo comando")
        print("x = detener robot")
        print("m = activar/desactivar movimiento real")
        print("q = salir")
        print("==========================================\n")

    # ============================================================
    # UTILIDADES
    # ============================================================

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    # ============================================================
    # CALLBACK DE ODOMETRÍA
    # ============================================================

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.yaw_from_quaternion(msg.pose.pose.orientation)

        self.current_pose = (x, y, theta)

    # ============================================================
    # PRINT EN VIVO DE POSE
    # ============================================================

    def print_pose_callback(self):
        if self.current_pose is not None:
            x, y, theta = self.current_pose
            print(
                f"[POSE] x={x:.3f} m  y={y:.3f} m  "
                f"theta={math.degrees(theta):.1f} deg  "
                f"| estado={self.state.name}"
            )

    # ============================================================
    # CALLBACK DE COMANDO DE COLOR / HOME / WAYPOINT
    # ============================================================

    def color_command_callback(self, msg):
        data = msg.data.strip()

        # --- comando: home ---
        if data == "home":
            if self.state == State.WAIT_NEXT_ACTION:
                self.state = State.RETURN_HOME
                self.state_start_time = time.time()
                self.get_logger().info("Comando HOME recibido. Regresando a (0,0).")
            else:
                self.get_logger().warn(
                    f"Comando 'home' ignorado en estado {self.state.name}."
                )
            return

        # --- comando: waypoint x y ---
        if data.startswith("waypoint"):
            parts = data.split()
            if len(parts) == 3:
                try:
                    wx = float(parts[1])
                    wy = float(parts[2])
                except ValueError:
                    self.get_logger().warn(
                        f"Waypoint inválido: '{data}'. Formato: waypoint x y"
                    )
                    return

                self.waypoint_pose = (wx, wy)

                if self.state == State.WAIT_NEXT_ACTION:
                    self.state = State.GO_TO_WAYPOINT
                    self.state_start_time = time.time()
                    self.get_logger().info(
                        f"Waypoint recibido: ({wx:.3f}, {wy:.3f}). Navegando."
                    )
                else:
                    self.get_logger().warn(
                        f"Waypoint guardado pero estado actual es {self.state.name}."
                    )
            else:
                self.get_logger().warn(
                    f"Formato incorrecto: '{data}'. Usa: waypoint x y"
                )
            return

        # --- comando: cancel ---
        if data == "cancel":
            self.stop_robot()
            self.target_color = None
            self.state = State.WAIT_COMMAND
            self.target_lost_counter = 0
            self.get_logger().info("Sistema reiniciado. Esperando comando.")
            return

        # --- comando: color ---
        color = data.lower()
        if color not in self.color_ranges:
            self.get_logger().warn(
                f"Comando no reconocido: '{data}'. Usa green, pink, yellow, home, waypoint x y o cancel."
            )
            return

        self.set_target_color(color)

    def set_target_color(self, color):
        self.target_color = color

        if self.viewpoint_drive_enabled:
            self.state = State.GO_TO_VIEWPOINT
        else:
            self.state = State.TRACK_TARGET

        self.state_start_time = time.time()
        self.target_lost_counter = 0

        self.get_logger().info(f"Nuevo objetivo: {self.target_color}")

    # ============================================================
    # DETECCIÓN DE COLOR
    # ============================================================

    def get_mask(self, hsv, color_name):
        final_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in self.color_ranges[color_name]:
            mask = cv2.inRange(hsv, lower, upper)
            final_mask = cv2.bitwise_or(final_mask, mask)

        kernel = np.ones((5, 5), np.uint8)
        final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
        final_mask = cv2.morphologyEx(final_mask, cv2.MORPH_CLOSE, kernel)

        return final_mask

    def detect_boxes(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        for color_name in self.color_ranges.keys():
            mask = self.get_mask(hsv, color_name)

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)

                if area < self.min_area:
                    continue

                M = cv2.moments(contour)

                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                x, y, w, h = cv2.boundingRect(contour)

                detections.append({
                    "color": color_name,
                    "area": area,
                    "cx": cx,
                    "cy": cy,
                    "bbox": (x, y, w, h),
                    "contour": contour
                })

        return detections

    def choose_closest_target(self, detections):
        """
        Si hay varias cajas del mismo color, escoge la de mayor área.
        Mayor área aparente = caja más cercana.
        """

        if self.target_color is None:
            return None, 0

        candidates = []

        for detection in detections:
            if detection["color"] == self.target_color:
                candidates.append(detection)

        if len(candidates) == 0:
            return None, 0

        closest_target = max(candidates, key=lambda d: d["area"])

        return closest_target, len(candidates)

    # ============================================================
    # ESTIMACIÓN DE RHO Y ALPHA
    # ============================================================

    def estimate_rho_alpha(self, target, image_width):
        x, y, bbox_w, bbox_h = target["bbox"]

        image_center_x = image_width / 2.0
        error_x = target["cx"] - image_center_x

        alpha = math.atan2(error_x, self.focal_px)

        if bbox_w <= 0:
            rho = None
        else:
            rho = (self.focal_px * self.box_width_m) / bbox_w
        
        if rho is not None:
            self.rho_history.append(rho)
            if len(self.rho_history) > self.rho_filter_n:
                self.rho_history.pop(0)
            rho = sum(self.rho_history) / len(self.rho_history)

        return rho, alpha, error_x

    # ============================================================
    # MPC REAL CALIBRADO
    # ============================================================

    def mpc_controller(self, rho, alpha):
        """
        Estado:
            x = [rho, alpha]

        Referencia:
            rho -> rho_ref
            alpha -> 0

        Control:
            u = [v, w]

        Modelo calibrado:
            rho(k+1) = rho(k) - K_rho * v(k) * cos(alpha(k)) * dt
            alpha(k+1) = alpha(k) - K_alpha * w(k) * dt
        """

        v_candidates = np.linspace(0.0, self.v_max, 7)
        w_candidates = np.linspace(-self.w_max, self.w_max, 17 )

        q_rho = 12.0
        q_alpha = 22.0

        r_v = 1.0
        r_w = 1.2

        r_dv = 2.0
        r_dw = 5.0

        best_cost = float("inf")
        best_v = 0.0
        best_w = 0.0

        for v in v_candidates:
            for w in w_candidates:

                rho_pred = rho
                alpha_pred = alpha

                prev_v = self.last_v_cmd
                prev_w = self.last_w_cmd

                cost = 0.0

                for _ in range(self.horizon):
                    rho_pred = (
                        rho_pred
                        - self.k_rho * v * math.cos(alpha_pred) * self.dt
                    )

                    alpha_pred = (
                        alpha_pred
                        - self.k_alpha * w * self.dt
                    )

                    if rho_pred < self.rho_ref:
                        rho_pred = self.rho_ref

                    rho_error = rho_pred - self.rho_ref
                    alpha_error = alpha_pred

                    cost += q_rho * (rho_error ** 2)
                    cost += q_alpha * (alpha_error ** 2)

                    cost += r_v * (v ** 2)
                    cost += r_w * (w ** 2)

                    cost += r_dv * ((v - prev_v) ** 2)
                    cost += r_dw * ((w - prev_w) ** 2)

                    prev_v = v
                    prev_w = w

                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_w = w

        # Seguridad 1:
        # Si está muy desalineado, no avanzar.
        if abs(alpha) > math.radians(15):
            best_v = 0.0
        elif abs(alpha) > math.radians(6) and self.last_v_cmd == 0.0:
            best_v = 0.0

        # Seguridad 2:
        # Si ya llegó a la distancia deseada, detener.
        if rho <= self.rho_ref:
            best_v = 0.0
            best_w = 0.0

        return best_v, best_w, best_cost

    # ============================================================
    # CONTROLADOR GENÉRICO POR ODOMETRÍA (home y waypoint)
    # ============================================================

    def go_to_pose_controller(self, target_x, target_y, target_theta=None):
        """
        Control proporcional hacia (target_x, target_y).
        Si target_theta no es None, corrige orientación al llegar.

        Retorna: (v_cmd, w_cmd, arrived, msg)
        """

        if self.current_pose is None:
            return 0.0, 0.0, False, "Sin odometria"

        x, y, theta = self.current_pose

        dx = target_x - x
        dy = target_y - y

        distance_error = math.sqrt(dx ** 2 + dy ** 2)

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - theta)

        # Fase 1: llegar a la posición
        if distance_error > self.return_distance_tolerance:
            w_cmd = self.k_return_angle * heading_error
            w_cmd = self.clamp(w_cmd, -self.return_w_max, self.return_w_max)

            if abs(heading_error) > math.radians(25):
                v_cmd = 0.0
            else:
                v_cmd = self.k_return_distance * distance_error
                v_cmd = self.clamp(v_cmd, 0.0, self.return_v_max)

            msg = (
                f"Navegando | dist={distance_error:.3f}m "
                f"heading_err={math.degrees(heading_error):.1f}deg"
            )
            return v_cmd, w_cmd, False, msg

        # Fase 2 (opcional): corregir orientación final
        if target_theta is not None:
            final_angle_error = self.normalize_angle(target_theta - theta)

            if abs(final_angle_error) > self.return_angle_tolerance:
                v_cmd = 0.0
                w_cmd = self.k_return_final_angle * final_angle_error
                w_cmd = self.clamp(w_cmd, -self.return_w_max, self.return_w_max)

                msg = (
                    f"Alineando orientacion | "
                    f"theta_err={math.degrees(final_angle_error):.1f}deg"
                )
                return v_cmd, w_cmd, False, msg

        return 0.0, 0.0, True, "Destino alcanzado"

    # ============================================================
    # CONTROL DE REGRESO A HOME POR ODOMETRÍA
    # ============================================================

    def return_home_controller(self):
        hx, hy, ht = self.home_pose
        v, w, arrived, msg = self.go_to_pose_controller(hx, hy, ht)

        if not arrived:
            msg = "Regresando home | " + msg

        return v, w, arrived, msg

    # ============================================================
    # CONTROL DE WAYPOINT POR ODOMETRÍA
    # ============================================================

    def go_to_waypoint_controller(self):
        if self.waypoint_pose is None:
            return 0.0, 0.0, False, "Waypoint no definido"

        wx, wy = self.waypoint_pose
        v, w, arrived, msg = self.go_to_pose_controller(wx, wy, target_theta=None)

        if not arrived:
            msg = "Hacia waypoint | " + msg

        return v, w, arrived, msg

    # ============================================================
    # PUBLICACIÓN DE VELOCIDAD
    # ============================================================

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)

        if self.enable_motion:
            self.cmd_pub.publish(cmd)
        else:
            # Modo seguro: no mueve el robot.
            stop = Twist()
            self.cmd_pub.publish(stop)

        self.last_v_cmd = float(v)
        self.last_w_cmd = float(w)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0

    # ============================================================
    # DIBUJO
    # ============================================================

    def draw_detection(self, frame, detection, is_target=False):
        color_name = detection["color"]
        draw_color = self.color_draw[color_name]

        x, y, w, h = detection["bbox"]
        cx = detection["cx"]
        cy = detection["cy"]
        area = detection["area"]

        thickness = 4 if is_target else 2

        cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, thickness)
        cv2.circle(frame, (cx, cy), 5, draw_color, -1)

        label = f"{color_name} | A={int(area)} | Wpx={w}"

        if is_target:
            label = "TARGET CLOSEST: " + label

        cv2.putText(
            frame,
            label,
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            draw_color,
            2
        )

    def draw_panel(
        self,
        frame,
        target,
        candidates_count,
        rho,
        alpha,
        error_x,
        v_cmd,
        w_cmd,
        cost,
        action_text
    ):
        h, w, _ = frame.shape

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 170), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.2, frame, 0.6, 0, frame)

        cv2.putText(
            frame,
            f"STATE: {self.state.name}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"TARGET: {self.target_color if self.target_color else 'None'} | Candidates: {candidates_count}",
            (15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2
        )

        motion_text = "ON" if self.enable_motion else "OFF"

        cv2.putText(
            frame,
            f"MOTION: {motion_text} | ACTION: {action_text}",
            (15, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"rho_ref={self.rho_ref:.2f}m | focal={self.focal_px:.1f}px | box_width={self.box_width_m:.3f}m",
            (15, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            2
        )

        odom_text = "None"
        if self.current_pose is not None:
            ox, oy, ot = self.current_pose
            odom_text = f"x={ox:.2f}, y={oy:.2f}, th={math.degrees(ot):.1f}deg"

        home_text = "HOME=(0.00, 0.00)"
        wp_text = "None"
        if self.waypoint_pose is not None:
            wpx, wpy = self.waypoint_pose
            wp_text = f"({wpx:.2f}, {wpy:.2f})"

        cv2.putText(
            frame,
            f"ODOM: {odom_text} | {home_text} | WP: {wp_text}",
            (15, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            2
        )

        panel_y = h - 160
        overlay = frame.copy()
        cv2.rectangle(frame, (0, panel_y), (w, h), (25, 25, 25), -1)
        cv2.addWeighted(overlay, 0.2, frame, 0.6, 0, frame)

        if target is None or rho is None:
            cv2.putText(
                frame,
                "Objetivo no detectado: sin estado confiable para MPC.",
                (15, panel_y + 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2
            )
            return

        cv2.putText(
            frame,
            f"Vision: e_x={error_x:.1f}px | alpha={alpha:.4f}rad ({math.degrees(alpha):.2f}deg) | rho={rho:.3f}m",
            (15, panel_y + 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"MPC input: x=[rho, alpha]=[{rho:.3f}, {alpha:.4f}]",
            (15, panel_y + 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"MPC output: v_cmd={v_cmd:.3f} m/s | w_cmd={w_cmd:.3f} rad/s",
            (15, panel_y + 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"Cost={cost:.3f} | K_rho={self.k_rho:.3f} | K_alpha={self.k_alpha:.3f}",
            (15, panel_y + 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

    # ============================================================
    # CALLBACK PRINCIPAL DE IMAGEN
    # ============================================================

    def image_callback(self, msg):

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        frame = frame.reshape((msg.height, msg.width, 3))

        # Si en tu cámara real la imagen no debe voltearse, comenta esta línea.
        frame = cv2.flip(frame, 1)

        height, width, _ = frame.shape
        image_center_x = int(width / 2)

        cv2.line(
            frame,
            (image_center_x, 0),
            (image_center_x, height),
            (255, 255, 255),
            2
        )

        detections = self.detect_boxes(frame)
        target, candidates_count = self.choose_closest_target(detections)

        for detection in detections:
            is_target = target is not None and detection is target
            self.draw_detection(frame, detection, is_target)

        rho = None
        alpha = 0.0
        error_x = 0.0

        if target is not None:
            rho, alpha, error_x = self.estimate_rho_alpha(target, width)

        v_cmd = 0.0
        w_cmd = 0.0
        cost = 0.0
        action_text = "Esperando"

        # ========================================================
        # MÁQUINA DE ESTADOS
        # ========================================================

        if self.state == State.WAIT_COMMAND:
            action_text = "Esperando comando de color"
            v_cmd = 0.0
            w_cmd = 0.0
            self.publish_cmd(v_cmd, w_cmd)

        elif self.state == State.GO_TO_VIEWPOINT:
            elapsed = time.time() - self.state_start_time

            if elapsed < self.viewpoint_drive_time:
                action_text = "Avanzando a zona de observacion"
                v_cmd = self.viewpoint_drive_speed
                w_cmd = 0.0
                self.publish_cmd(v_cmd, w_cmd)
            else:
                self.stop_robot()
                self.state = State.TRACK_TARGET
                action_text = "Zona de observacion alcanzada"

        elif self.state == State.TRACK_TARGET:
            if target is None or rho is None:
                self.target_lost_counter += 1
                action_text = "Objetivo no detectado"

                v_cmd = 0.0
                w_cmd = 0.0
                self.publish_cmd(v_cmd, w_cmd)

                if self.target_lost_counter > self.max_lost_frames:
                    action_text = "Objetivo perdido. Esperando deteccion"

            else:
                self.target_lost_counter = 0

                v_cmd, w_cmd, cost = self.mpc_controller(rho, alpha)

                if rho <= self.rho_ref:
                    action_text = "Caja alcanzada"
                    v_cmd = 0.0
                    w_cmd = 0.0
                    self.publish_cmd(v_cmd, w_cmd)
                    self.state = State.COLLECTING
                    self.state_start_time = time.time()
                else:
                    action_text = "MPC controlando hacia caja"
                    self.publish_cmd(v_cmd, w_cmd)

        elif self.state == State.COLLECTING:
            action_text = "RECOLECTAR caja / activar mecanismo"
            v_cmd = 0.0
            w_cmd = 0.0
            self.publish_cmd(v_cmd, w_cmd)

            elapsed = time.time() - self.state_start_time

            if elapsed > 1.0:
                self.stop_robot()
                self.state = State.WAIT_NEXT_ACTION
                self.get_logger().info(
                    "Recolección completa. Esperando siguiente acción: "
                    "'home' o 'waypoint x y'."
                )

        elif self.state == State.WAIT_NEXT_ACTION:
            action_text = "Esperando: 'home' o 'waypoint x y'"
            v_cmd = 0.0
            w_cmd = 0.0
            self.publish_cmd(v_cmd, w_cmd)

        elif self.state == State.RETURN_HOME:
            v_cmd, w_cmd, arrived_home, return_msg = self.return_home_controller()
            action_text = return_msg

            self.publish_cmd(v_cmd, w_cmd)

            if arrived_home:
                self.stop_robot()
                self.state = State.FINISHED
                self.get_logger().info("Home alcanzado. Misión terminada.")

        elif self.state == State.GO_TO_WAYPOINT:
            v_cmd, w_cmd, arrived, wp_msg = self.go_to_waypoint_controller()
            action_text = wp_msg

            self.publish_cmd(v_cmd, w_cmd)

            if arrived:
                self.stop_robot()
                self.state = State.WAIT_NEXT_ACTION
                self.get_logger().info(
                    f"Waypoint {self.waypoint_pose} alcanzado. "
                    "Esperando siguiente acción."
                )

        elif self.state == State.FINISHED:
            action_text = "Mision terminada. Presiona c para reiniciar"
            v_cmd = 0.0
            w_cmd = 0.0
            self.publish_cmd(v_cmd, w_cmd)

        self.draw_panel(
            frame=frame,
            target=target,
            candidates_count=candidates_count,
            rho=rho,
            alpha=alpha,
            error_x=error_x,
            v_cmd=v_cmd,
            w_cmd=w_cmd,
            cost=cost,
            action_text=action_text
        )

        #cv2.imshow("Puzzlebot Box MPC", frame)

        current_time = time.time() - self.start_time

        x = y = theta = 0.0

        if self.current_pose is not None:
            x, y, theta = self.current_pose

        target_detected = 1 if target is not None else 0

        rho_value = rho if rho is not None else -1.0

        self.csv_writer.writerow([
            current_time,
            rho_value,
            alpha,
            v_cmd,
            w_cmd,
            cost,
            x,
            y,
            theta,
            self.state.name,
            target_detected
        ])

        self.log_file.flush()
        
        key = cv2.waitKey(1) & 0xFF

        if key == ord("g"):
            self.set_target_color("green")

        elif key == ord("p"):
            self.set_target_color("pink")

        elif key == ord("y"):
            self.set_target_color("yellow")

        elif key == ord("m"):
            self.enable_motion = not self.enable_motion
            self.get_logger().info(f"enable_motion = {self.enable_motion}")

        elif key == ord("x"):
            self.stop_robot()
            self.get_logger().info("Robot detenido manualmente.")

        elif key == ord("c"):
            self.stop_robot()
            self.target_color = None
            self.state = State.WAIT_COMMAND
            self.target_lost_counter = 0
            self.waypoint_pose = None
            self.get_logger().info("Sistema reiniciado. Esperando comando.")

        elif key == ord("q"):
            self.stop_robot()
            rclpy.shutdown()

    def destroy_node(self):
        self.stop_robot()
        cv2.destroyAllWindows()
        super().destroy_node()
        self.log_file.close()


def main(args=None):
    rclpy.init(args=args)

    node = PuzzlebotBoxMPCNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
