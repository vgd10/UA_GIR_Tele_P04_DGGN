#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateHomeSender(Node):
    """
    Nodo ROS2 que:
      - Publica una pose HOME fija para el Kinova Gen3 de 7 DOF + pinza
        en el tópico /joint_states.
      - Mantiene esa publicación durante unos instantes
        (para asegurarse de que quien suscribe la reciba).
      - Luego se cierra automáticamente.

    No usa TF, ni Twist, ni comandos de pinza ni go_home.
    Solo sirve para mandar el robot a HOME y salir.
    """

    def __init__(self):
        super().__init__('joint_state_home_sender')

        # Parámetros
        self.declare_parameter('dt', 0.1)        # periodo de publicación (s)
        self.declare_parameter('duration', 3.0)  # tiempo total publicando HOME (s)

        self.dt = float(self.get_parameter('dt').value)
        self.duration = float(self.get_parameter('duration').value)

        # Número de iteraciones que publicaremos
        self.max_iters = int(self.duration / self.dt)
        if self.max_iters < 1:
            self.max_iters = 1

        self.iter = 0

        # Nombres de las 13 articulaciones (7 brazo + 6 pinza)
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
            'finger_joint',
            'left_inner_knuckle_joint',
            'left_inner_finger_joint',
            'right_outer_knuckle_joint',
            'right_inner_knuckle_joint',
            'right_inner_finger_joint'
        ]

        # Pose HOME del brazo (la que nos diste)
        self.arm_home = np.array([
            -7.599639974209538e-06,      # joint_1
            0.26020410210757117,         # joint_2
            3.140033165279362,           # joint_3
            -2.2701140639266875,         # joint_4
            5.324503843162631e-06,       # joint_5
            0.9596723955392745,          # joint_6
            1.569996636583614            # joint_7
        ], dtype=float)

        # Pinza abierta: todas las articulaciones de la pinza a 0
        self.gripper_open = np.zeros(6, dtype=float)

        # Publicador de /joint_states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Timer para enviar la pose home varias veces y luego cerrar
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f"JointStateHomeSender: enviando HOME durante {self.duration:.2f} s "
            f"cada {self.dt:.2f} s, luego se cerrará."
        )
        self.get_logger().info(f"arm_home = {self.arm_home.tolist()} (pinza abierta)")

    def timer_callback(self):
        # Construir mensaje JointState con la pose home
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.arm_home.tolist() + self.gripper_open.tolist()
        msg.velocity = []
        msg.effort = []

        self.publisher_.publish(msg)
        self.iter += 1

        if self.iter >= self.max_iters:
            self.get_logger().info(
                f"HOME publicado {self.iter} veces. Cerrando nodo joint_state_home_sender."
            )
            # Apagamos ROS2 desde aquí
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = JointStateHomeSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info("KeyboardInterrupt: cerrando nodo.")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

