import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import csv
import time
import math


class PuzzlebotVisionCalibration(Node):
    def __init__(self):
        super().__init__("puzzlebot_vision_calibration")

        # =========================
        # PARAMETROS ROS
        # =========================

        self.declare_parameter("image_topic", "/video_source/raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("csv_path", "calibration_data.csv")

        self.declare_parameter("box_width_m", 0.06)
        self.declare_parameter("known_distance_m", 1)
        self.declare_parameter("fov_horizontal_deg", 60.0)

        self.declare_parameter("min_area", 800)
        self.declare_parameter("linear_calib_v", 0.04)
        self.declare_parameter("angular_calib_w", 0.20)
        self.declare_parameter("trial_duration", 3.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.csv_path = self.get_parameter("csv_path").value

        self.box_width_m = float(self.get_parameter("box_width_m").value)
        self.known_distance_m = float(self.get_parameter("known_distance_m").value)
        self.fov_horizontal_deg = float(self.get_parameter("fov_horizontal_deg").value)

        self.min_area = int(self.get_parameter("min_area").value)
        self.linear_calib_v = float(self.get_parameter("linear_calib_v").value)
        self.angular_calib_w = float(self.get_parameter("angular_calib_w").value)
        self.trial_duration = float(self.get_parameter("trial_duration").value)

        # =========================
        # ROS
        # =========================

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # =========================
        # ESTADO DEL SISTEMA
        # =========================

        self.target_color = None
        self.mode = "IDLE"

        self.trial_active = False
        self.trial_start_time = None
        self.current_v_cmd = 0.0
        self.current_w_cmd = 0.0

        self.focal_px = None

        # =========================
        # COLORES HSV
        # =========================

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

        # =========================
        # CSV
        # =========================

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "timestamp",
            "elapsed_trial",
            "mode",
            "target_color",
            "target_found",
            "cx",
            "cy",
            "bbox_w",
            "bbox_h",
            "area",
            "error_x_px",
            "alpha_rad",
            "rho_m",
            "v_cmd",
            "w_cmd",
            "focal_px"
        ])

        self.get_logger().info("Nodo de calibracion iniciado.")
        self.get_logger().info(f"Escuchando imagen en: {self.image_topic}")
        self.get_logger().info(f"Publicando velocidades en: {self.cmd_vel_topic}")
        self.get_logger().info(f"Guardando datos en: {self.csv_path}")

        print("\n==========================================")
        print("CALIBRACION VISION + PUZZLEBOT")
        print("==========================================")
        print("Teclas:")
        print("g = seleccionar caja verde")
        print("p = seleccionar caja rosa")
        print("y = seleccionar caja amarilla")
        print("k = calibrar focal con distancia conocida")
        print("a = prueba angular positiva")
        print("d = prueba angular negativa")
        print("v = prueba lineal hacia adelante")
        print("x = detener robot")
        print("c = cancelar color")
        print("q = salir")
        print("==========================================\n")

    # =========================
    # VISION
    # =========================

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
        if self.target_color is None:
            return None, 0

        candidates = []

        for detection in detections:
            if detection["color"] == self.target_color:
                candidates.append(detection)

        if len(candidates) == 0:
            return None, 0

        # Mayor area aparente = caja mas cercana
        closest = max(candidates, key=lambda d: d["area"])

        return closest, len(candidates)

    # =========================
    # ESTIMACION DE ANGULO Y DISTANCIA
    # =========================

    def get_focal_from_fov(self, image_width):
        fov_rad = math.radians(self.fov_horizontal_deg)
        focal = image_width / (2.0 * math.tan(fov_rad / 2.0))
        return focal

    def estimate_alpha_and_rho(self, target, image_width):
        x, y, bbox_w, bbox_h = target["bbox"]
        image_center_x = image_width / 2.0

        error_x = target["cx"] - image_center_x

        if self.focal_px is None:
            focal = self.get_focal_from_fov(image_width)
        else:
            focal = self.focal_px

        alpha = math.atan2(error_x, focal)

        if bbox_w <= 0:
            rho = None
        else:
            rho = (focal * self.box_width_m) / bbox_w

        return error_x, alpha, rho

    def calibrate_focal(self, target):
        x, y, bbox_w, bbox_h = target["bbox"]

        if bbox_w <= 0:
            self.get_logger().warn("No se puede calibrar focal: ancho de caja invalido.")
            return

        self.focal_px = (bbox_w * self.known_distance_m) / self.box_width_m

        self.get_logger().info(
            f"Focal calibrada: {self.focal_px:.2f} px "
            f"usando distancia={self.known_distance_m:.2f} m, "
            f"ancho_real={self.box_width_m:.3f} m, "
            f"ancho_px={bbox_w}"
        )

    # =========================
    # CONTROL DE PRUEBAS
    # =========================

    def start_trial(self, mode, v_cmd, w_cmd):
        if self.target_color is None:
            self.get_logger().warn("Primero selecciona un color objetivo.")
            return

        self.mode = mode
        self.trial_active = True
        self.trial_start_time = time.time()
        self.current_v_cmd = v_cmd
        self.current_w_cmd = w_cmd

        self.get_logger().info(
            f"Iniciando prueba {mode}: v={v_cmd:.3f}, w={w_cmd:.3f}, "
            f"duracion={self.trial_duration:.2f}s"
        )

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

        self.current_v_cmd = 0.0
        self.current_w_cmd = 0.0
        self.trial_active = False
        self.mode = "IDLE"

        self.get_logger().info("Robot detenido.")

    def update_robot_command(self):
        if not self.trial_active:
            self.stop_cmd_only()
            return

        elapsed = time.time() - self.trial_start_time

        if elapsed >= self.trial_duration:
            self.stop_robot()
            return

        cmd = Twist()
        cmd.linear.x = self.current_v_cmd
        cmd.angular.z = self.current_w_cmd
        self.cmd_pub.publish(cmd)

    def stop_cmd_only(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # =========================
    # DIBUJO
    # =========================

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

    def draw_panel(self, frame, target, candidates_count, error_x, alpha, rho):
        h, w, _ = frame.shape

        cv2.rectangle(frame, (0, 0), (w, 140), (30, 30, 30), -1)

        cv2.putText(
            frame,
            f"MODE: {self.mode}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"TARGET COLOR: {self.target_color if self.target_color else 'None'} | Candidates: {candidates_count}",
            (15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"Command: v={self.current_v_cmd:.3f} m/s | w={self.current_w_cmd:.3f} rad/s",
            (15, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2
        )

        focal_text = f"{self.focal_px:.2f}px" if self.focal_px else "FOV approx"

        cv2.putText(
            frame,
            f"Focal: {focal_text} | box_width={self.box_width_m:.3f}m",
            (15, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2
        )

        panel_y = h - 130
        cv2.rectangle(frame, (0, panel_y), (w, h), (25, 25, 25), -1)

        if target is None:
            cv2.putText(
                frame,
                "Objetivo no detectado.",
                (15, panel_y + 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )
            return

        cv2.putText(
            frame,
            f"e_x={error_x:.1f} px | alpha={alpha:.4f} rad | alpha={math.degrees(alpha):.2f} deg",
            (15, panel_y + 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2
        )

        rho_text = f"{rho:.3f} m" if rho is not None else "None"

        cv2.putText(
            frame,
            f"rho={rho_text} | area={target['area']:.1f} px^2",
            (15, panel_y + 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            "Estado para MPC real: x = [rho, alpha], u = [v, w]",
            (15, panel_y + 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2
        )

    # =========================
    # CSV
    # =========================

    def log_data(self, target, target_found, error_x, alpha, rho):
        timestamp = time.time()

        if self.trial_start_time is None:
            elapsed = 0.0
        else:
            elapsed = timestamp - self.trial_start_time

        if target is None:
            cx = None
            cy = None
            bbox_w = None
            bbox_h = None
            area = None
        else:
            x, y, bbox_w, bbox_h = target["bbox"]
            cx = target["cx"]
            cy = target["cy"]
            area = target["area"]

        self.csv_writer.writerow([
            timestamp,
            elapsed,
            self.mode,
            self.target_color,
            int(target_found),
            cx,
            cy,
            bbox_w,
            bbox_h,
            area,
            error_x,
            alpha,
            rho,
            self.current_v_cmd,
            self.current_w_cmd,
            self.focal_px
        ])

        self.csv_file.flush()

    # =========================
    # CALLBACK PRINCIPAL
    # =========================

    def image_callback(self, msg):

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        
        frame = frame.reshape(
            (msg.height, msg.width, 3)
        
        )
        
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

        target_found = target is not None
        error_x = 0.0
        alpha = 0.0
        rho = None

        if target_found:
            error_x, alpha, rho = self.estimate_alpha_and_rho(target, width)

        self.update_robot_command()

        self.log_data(
            target=target,
            target_found=target_found,
            error_x=error_x,
            alpha=alpha,
            rho=rho
        )

        self.draw_panel(frame, target, candidates_count, error_x, alpha, rho)

        cv2.imshow("Puzzlebot Vision Calibration", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("g"):
            self.target_color = "green"
            self.get_logger().info("Color objetivo: verde")

        elif key == ord("p"):
            self.target_color = "pink"
            self.get_logger().info("Color objetivo: rosa")

        elif key == ord("y"):
            self.target_color = "yellow"
            self.get_logger().info("Color objetivo: amarillo")

        elif key == ord("k"):
            if target is None:
                self.get_logger().warn("No hay objetivo visible para calibrar focal.")
            else:
                self.calibrate_focal(target)

        elif key == ord("a"):
            self.start_trial(
                mode="ANGULAR_POSITIVE",
                v_cmd=0.0,
                w_cmd=self.angular_calib_w
            )

        elif key == ord("d"):
            self.start_trial(
                mode="ANGULAR_NEGATIVE",
                v_cmd=0.0,
                w_cmd=-self.angular_calib_w
            )

        elif key == ord("v"):
            self.start_trial(
                mode="LINEAR_FORWARD",
                v_cmd=self.linear_calib_v,
                w_cmd=0.0
            )

        elif key == ord("x"):
            self.stop_robot()

        elif key == ord("c"):
            self.target_color = None
            self.stop_robot()
            self.get_logger().info("Color cancelado. Sistema en espera.")

        elif key == ord("q"):
            self.stop_robot()
            rclpy.shutdown()

    def destroy_node(self):
        self.stop_cmd_only()

        if hasattr(self, "csv_file") and self.csv_file:
            self.csv_file.close()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PuzzlebotVisionCalibration()

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
