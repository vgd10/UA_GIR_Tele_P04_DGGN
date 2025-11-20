#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

import pygame

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class KeyboardToTwist(Node):
    def __init__(self):
        super().__init__("keyboard_to_twist")

        # -------- ROS params --------
        self.declare_parameter("topic_twist", "/tool_twist_vel")
        self.declare_parameter("topic_gripper", "/gripper_cmd")
        self.declare_parameter("rate_hz", 20.0)

        # Solo informativo para el usuario
        self.declare_parameter("ee_frame", "tool")

        # Velocidades actuales configurables con +/-
        self.declare_parameter("current_v_max", 0.1)   # m/s
        self.declare_parameter("current_w_max", 0.5)   # rad/s

        # Paso de incremento para +/-
        self.declare_parameter("step_v", 0.005)        # m/s -> 0.5 cm/s
        self.declare_parameter("step_w", 0.005)        # rad/s

        self.declare_parameter("hold_delay_s", 0.30)
        self.declare_parameter("repeat_rate_hz", 10.0)

        self.declare_parameter("gripper_step", 0.05)

        # Leer parámetros
        self.topic_twist = self.get_parameter("topic_twist").get_parameter_value().string_value
        self.topic_gripper = self.get_parameter("topic_gripper").get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value

        self.current_v_max = float(self.get_parameter("current_v_max").value)
        self.current_w_max = float(self.get_parameter("current_w_max").value)

        self.step_v = float(self.get_parameter("step_v").value)
        self.step_w = float(self.get_parameter("step_w").value)

        self.hold_delay_s = float(self.get_parameter("hold_delay_s").value)
        self.repeat_rate_hz = float(self.get_parameter("repeat_rate_hz").value)

        self.gripper_step = float(self.get_parameter("gripper_step").value)

        # Límites de seguridad reales
        self.v_min = 0.0
        self.w_min = 0.0

        self.limit_v_max = 0.2                # Límite lineal absoluto m/s
        self.limit_w_max = 0.78539816339      # Límite angular absoluto rad/s

        # Escala seleccionada con TAB
        self.adjust_target = "linear"

        # Estado de pinza
        self.gripper = 0.0

        # Estado para pantalla
        self.last_twist = Twist()

        # -------- Inicializar pygame --------
        pygame.init()
        self.screen = pygame.display.set_mode((900, 700))
        pygame.display.set_caption("Keyboard Teleop for EE Twist (ROS2)")

        self.font = pygame.font.SysFont("Courier New", 18)
        self.small = pygame.font.SysFont("Courier New", 16)
        self.clock = pygame.time.Clock()

        # -------- ROS publishers --------
        self.pub_twist = self.create_publisher(Twist, self.topic_twist, 10)
        self.pub_gripper = self.create_publisher(Float64, self.topic_gripper, 10)

        # Estado de repetición +/- con retardo
        self.plus_held = False
        self.minus_held = False
        self.plus_next_time = 0.0
        self.minus_next_time = 0.0

        self.get_logger().info(
            f"KeyboardToTwist iniciado. Publicando Twist (EE frame={self.ee_frame}) en {self.topic_twist}, "
            f"gripper en {self.topic_gripper}"
        )

    # -------- Helpers --------
    def now(self):
        return pygame.time.get_ticks() / 1000.0

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _apply_step(self, sign: int):
        """Incremento/decremento de velocidad con +/-."""
        if self.adjust_target == "linear":
            self.current_v_max = self.clamp(
                self.current_v_max + sign * self.step_v,
                self.v_min,
                self.limit_v_max
            )
        else:
            self.current_w_max = self.clamp(
                self.current_w_max + sign * self.step_w,
                self.w_min,
                self.limit_w_max
            )

    # -------- Event handling --------
    def handle_key_events(self, event):
        if event.type == pygame.QUIT:
            self.get_logger().info("Ventana cerrada, saliendo...")
            rclpy.shutdown()
            return

        if event.type == pygame.KEYDOWN:

            if event.key == pygame.K_ESCAPE:
                self.get_logger().info("ESC pulsado, saliendo...")
                rclpy.shutdown()
                return

            if event.key == pygame.K_TAB:
                self.adjust_target = "angular" if self.adjust_target == "linear" else "linear"
                return

            # + pulsado
            if event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self._apply_step(+1)
                self.plus_held = True
                self.plus_next_time = self.now() + self.hold_delay_s
                return

            # - pulsado
            if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self._apply_step(-1)
                self.minus_held = True
                self.minus_next_time = self.now() + self.hold_delay_s
                return

        if event.type == pygame.KEYUP:
            if event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self.plus_held = False
            if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self.minus_held = False

    def handle_hold_repeat(self):
        t = self.now()
        step_period = 1.0 / self.repeat_rate_hz

        if self.plus_held and t >= self.plus_next_time:
            self._apply_step(+1)
            self.plus_next_time += step_period

        if self.minus_held and t >= self.minus_next_time:
            self._apply_step(-1)
            self.minus_next_time += step_period

    # -------- UI --------
    def build_text_lines(self, twist: Twist):

        v = self.current_v_max
        w = self.current_w_max
        tgt = "LINEAR (m/s)" if self.adjust_target == "linear" else "ANGULAR (rad/s)"

        linx = twist.linear.x
        liny = twist.linear.y
        linz = twist.linear.z

        hold_info = f"(hold delay {self.hold_delay_s:.2f}s, repeat {self.repeat_rate_hz:.1f} Hz)"

        lines = [
            "CLICK EN ESTA VENTANA PARA CAPTURAR EL TECLADO | ESC: salir",
            "",
            f"Publicando: {self.topic_twist} @ {self.rate_hz:.1f} Hz   [Frame EE: {self.ee_frame}]",
            "",
            f"VEL. ACTUALES ->  lineal: {v:.3f} m/s (límite {self.limit_v_max} m/s)",
            f"                   angular: {w:.3f} rad/s (límite {self.limit_w_max:.3f} rad/s)",
            f"AJUSTE CON +/-  ->  Objetivo: {tgt}   (TAB para cambiar) {hold_info}",
            "",
            "CONTROLES (Frame del EE):",
            f"  LINEAL (m/s):    W:+X({v:+.3f})   S:-X({-v:+.3f})",
            f"                   A:+Y({v:+.3f})   D:-Y({-v:+.3f})",
            f"                   Q:+Z({v:+.3f})   E:-Z({-v:+.3f})",
            "",
            f"  ANGULAR (rad/s): LEFT:+yaw({w:+.3f}) RIGHT:-yaw({-w:+.3f})",
            f"                   UP:+roll({w:+.3f})  DOWN:-roll({-w:+.3f})",
            f"                   Z:+pitch({w:+.3f})  C:-pitch({-w:+.3f})",
            "",
            "PINZA:",
            "  F: abrir     G: cerrar",
            f"  gripper_cmd: {self.gripper:.2f}  [0=open, 1=closed]",
            "",
            "VELOCIDADES ENVIADAS (Twist):",
            f"  linear:  x={linx:+.3f}  y={liny:+.3f}  z={linz:+.3f}   m/s",
            f"  angular: x={twist.angular.x:+.3f}  y={twist.angular.y:+.3f}  z={twist.angular.z:+.3f}   rad/s",
        ]
        return lines

    def draw_text_block(self, lines):
        self.screen.fill((0, 0, 0))
        y = 24

        for line in lines:
            surf = self.font.render(line, True, (255, 255, 255))
            rect = surf.get_rect(topleft=(20, y))
            self.screen.blit(surf, rect)
            y += 26

        footer = self.small.render(
            "Tip: Mantén pulsadas las teclas de movimiento. +/- tienen retardo y repetición.",
            True, (180, 180, 180)
        )
        self.screen.blit(footer, (20, y + 6))
        pygame.display.flip()

    # -------- Main loop --------
    def run(self):
        try:
            while rclpy.ok():

                rclpy.spin_once(self, timeout_sec=0.0)

                for event in pygame.event.get():
                    self.handle_key_events(event)

                self.handle_hold_repeat()

                twist = Twist()
                keys = pygame.key.get_pressed()

                # Lineal
                if keys[pygame.K_w]:
                    twist.linear.x = self.current_v_max
                if keys[pygame.K_s]:
                    twist.linear.x = -self.current_v_max
                if keys[pygame.K_a]:
                    twist.linear.y = self.current_v_max
                if keys[pygame.K_d]:
                    twist.linear.y = -self.current_v_max
                if keys[pygame.K_q]:
                    twist.linear.z = self.current_v_max
                if keys[pygame.K_e]:
                    twist.linear.z = -self.current_v_max

                # Angular
                if keys[pygame.K_LEFT]:
                    twist.angular.z = self.current_w_max
                if keys[pygame.K_RIGHT]:
                    twist.angular.z = -self.current_w_max
                if keys[pygame.K_UP]:
                    twist.angular.x = self.current_w_max
                if keys[pygame.K_DOWN]:
                    twist.angular.x = -self.current_w_max
                if keys[pygame.K_z]:
                    twist.angular.y = self.current_w_max
                if keys[pygame.K_c]:
                    twist.angular.y = -self.current_w_max

                # Pinza
                if keys[pygame.K_g]:  # cerrar
                    self.gripper = self.clamp(self.gripper + self.gripper_step, 0.0, 1.0)
                if keys[pygame.K_f]:  # abrir
                    self.gripper = self.clamp(self.gripper - self.gripper_step, 0.0, 1.0)

                self.pub_twist.publish(twist)

                msg_g = Float64()
                msg_g.data = float(self.gripper)
                self.pub_gripper.publish(msg_g)

                self.last_twist = twist

                lines = self.build_text_lines(self.last_twist)
                self.draw_text_block(lines)

                self.clock.tick(self.rate_hz)

        finally:
            pygame.quit()
            self.get_logger().info("Saliendo de KeyboardToTwist")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToTwist()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
