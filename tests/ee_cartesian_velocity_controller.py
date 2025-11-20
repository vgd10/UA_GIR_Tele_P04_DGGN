#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64

import tf2_ros

from kinova_gen3.kinematics.jacobian import jacobian as kinova_jacobian


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    Convierte un cuaternión (x,y,z,w) en una matriz de rotación 3x3.
    """
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    R = np.zeros((3, 3))
    R[0, 0] = 1.0 - 2.0 * (yy + zz)
    R[0, 1] = 2.0 * (xy - wz)
    R[0, 2] = 2.0 * (xz + wy)

    R[1, 0] = 2.0 * (xy + wz)
    R[1, 1] = 1.0 - 2.0 * (xx + zz)
    R[1, 2] = 2.0 * (yz - wx)

    R[2, 0] = 2.0 * (xz - wy)
    R[2, 1] = 2.0 * (yz + wx)
    R[2, 2] = 1.0 - 2.0 * (xx + yy)

    return R


class JointStatePublisher(Node):
    """
    Control cinemático para Kinova Gen3 de 7 DOF + pinza:

      - Lee una vez la pose inicial del brazo de /joint_states y la usa como home.
      - Recibe un Twist en /tool_twist_vel que representa la velocidad
        de un punto virtual (tool_frame) desplazado respecto al EE.
      - Convierte ese twist del punto tool al origen del EE:
            v_ee = v_tool - r x w
        donde r es el vector (EE -> tool_frame) en el frame del EE.
      - Usa TF para obtener T_base_ee y calcula:
            v_base = Ad(T_base_ee) * v_ee
      - Convierte v_base a dq con Jacobiano + DLS.
      - Integra q y publica /joint_states.
      - Pinza controlada con /gripper_cmd en [0..1].
      - Publica un frame TF adicional: parent = ee_frame, child = "tool_frame".
    """

    def __init__(self):
        super().__init__('joint_state_publisher_gen3_gripper')

        # Parámetros de control
        self.declare_parameter('damping', 0.01)
        self.declare_parameter('max_joint_speed', 1.0)
        self.declare_parameter('dt', 0.01)  # 100 Hz
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'end_effector_link')

        # Offset del punto tool respecto al EE (en el frame del EE)
        self.declare_parameter('tool_offset_x', 0.0)
        self.declare_parameter('tool_offset_y', 0.0)
        self.declare_parameter('tool_offset_z', -0.2)  # modifcado para tener la pose de control en la pinza 

        self.damping = self.get_parameter('damping').get_parameter_value().double_value
        self.max_joint_speed = self.get_parameter('max_joint_speed').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value

        ox = self.get_parameter('tool_offset_x').get_parameter_value().double_value
        oy = self.get_parameter('tool_offset_y').get_parameter_value().double_value
        oz = self.get_parameter('tool_offset_z').get_parameter_value().double_value
        # r: vector desde EE hasta tool_frame, expresado en el frame del EE
        self.tool_offset = np.array([ox, oy, oz], dtype=float)

        # TF2: buffer + listener para base<-ee
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF broadcaster para publicar ee->tool_frame
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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

        # Estado interno del brazo
        self.arm_home = np.zeros(7, dtype=float)
        self.q = np.zeros(7, dtype=float)

        # Flag de pose inicial
        self.has_initial_state = False

        # Pinza: [finger, left_knuckle, left_finger, right_outer, right_inner_knuckle, right_inner_finger]
        self.gripper_closed = np.array([0.7, -0.7, 0.7, -0.7, -0.7, 0.7], dtype=float)
        self.gripper_factor = 0.0  # 0=open, 1=closed

        # Twist en el frame del punto tool (offset)
        self.v_tool_body = np.zeros(6)

        # Publisher de /joint_states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Subs
        self.sub_twist = self.create_subscription(
            Twist,
            '/tool_twist_vel',     
            self.cartesian_vel_cb,
            10
        )

        self.sub_gripper = self.create_subscription(
            Float64,
            '/gripper_cmd',
            self.gripper_cb,
            10
        )

        self.sub_js_init = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_init_cb,
            10
        )

        # Timer de control
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.log_step = 0
        self.get_logger().info(
            f"JointStatePublisher Gen3 + gripper inicializado. "
            f"Esperando pose inicial de /joint_states. "
            f"base_frame={self.base_frame}, ee_frame={self.ee_frame}, "
            f"tool_offset={self.tool_offset}"
        )

    # ----------------- Callbacks -----------------

    def joint_state_init_cb(self, msg: JointState):
        """
        Lee una vez la pose inicial del brazo desde /joint_states y la usa como home.
        Ignora la pinza; solo usa joint_1..joint_7.
        """
        if self.has_initial_state:
            return

        name_to_pos = dict(zip(msg.name, msg.position))
        q_tmp = np.zeros(7, dtype=float)

        for i, jname in enumerate(self.joint_names[:7]):  # solo los 7 del brazo
            if jname not in name_to_pos:
                self.get_logger().warn(
                    f"joint_state_init_cb: no se encontró {jname} en /joint_states, "
                    f"esperando siguiente mensaje..."
                )
                return
            q_tmp[i] = name_to_pos[jname]

        self.arm_home = q_tmp.copy()
        self.q = q_tmp.copy()
        self.has_initial_state = True
        self.get_logger().info(f"Pose inicial recibida, arm_home = {self.arm_home}")

    def cartesian_vel_cb(self, msg: Twist):
        """
        Twist del punto herramienta virtual (tool_frame) expresado en su propio frame.
        """
        self.v_tool_body = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ])

    def gripper_cb(self, msg: Float64):
        """Comando escalar de la pinza [0..1]."""
        val = float(msg.data)
        self.gripper_factor = max(0.0, min(1.0, val))

    # -------------- TF: adjunta base<-ee --------------

    def get_adjoint_base_ee(self):
        """
        Devuelve la matriz adjunta Ad_{T_base_ee} (6x6) tal que:
            v_base = Ad(T_base_ee) * v_ee
        donde v_ee es un twist expresado en el frame del EE.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(
                f"No se pudo obtener TF {self.base_frame} -> {self.ee_frame}: {ex}"
            )
            return None

        # Traslación
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        p = np.array([tx, ty, tz])

        # Rotación como matriz 3x3
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        R = quaternion_to_rotation_matrix(qx, qy, qz, qw)

        # Matriz skew-simétrica de p
        px = np.array([
            [0.0, -p[2],  p[1]],
            [p[2],  0.0, -p[0]],
            [-p[1], p[0], 0.0]
        ])

        # Matriz adjunta 6x6
        Ad = np.zeros((6, 6))
        Ad[0:3, 0:3] = R
        Ad[3:6, 3:6] = R
        Ad[3:6, 0:3] = px.dot(R)

        return Ad

    # -------------- tool -> ee: cambio de punto --------------

    def tool_twist_to_ee_twist(self, v_tool6: np.ndarray) -> np.ndarray:
        """
        Convierte un twist definido en el punto 'tool' (offset) al twist en el origen del EE.
        v_tool6 = [v_tool; w]  (6,)
        Devuelve v_ee6 = [v_ee; w], con:
            v_ee = v_tool - r x w
        donde r = vector desde EE hasta tool (self.tool_offset, en frame EE).
        """
        v_tool = v_tool6[0:3]
        w = v_tool6[3:6]
        r = self.tool_offset

        r_cross = np.array([
            [0.0,    -r[2],  r[1]],
            [r[2],   0.0,   -r[0]],
            [-r[1],  r[0],   0.0]
        ])

        # v_ee = v_tool - r x w = v_tool - r^x * w
        v_ee = v_tool - r_cross.dot(w)

        v_ee6 = np.zeros(6)
        v_ee6[0:3] = v_ee
        v_ee6[3:6] = w
        return v_ee6

    # -------------- Publicar TF ee -> tool_frame --------------

    def publish_tool_tf(self):
        """
        Publica un frame adicional en TF:
          parent = ee_frame
          child  = "tool_frame"
          traslación = self.tool_offset (en frame EE)
          rotación = identidad
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ee_frame
        t.child_frame_id = "tool_frame"

        t.transform.translation.x = float(self.tool_offset[0])
        t.transform.translation.y = float(self.tool_offset[1])
        t.transform.translation.z = -float(self.tool_offset[2])

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    # -------------- Cinemática: IK --------------

    def compute_dq_from_twist(self):
        """
        dq = J^T (J J^T + λ² I)^(-1) v_base

        donde:
          - v_tool_body: twist del punto offset (tool) en su frame.
          - v_ee_body   : mismo twist, pero en el origen del EE:
                          v_ee_body = tool_twist_to_ee_twist(v_tool_body)
          - v_base      : v_ee_body transformado al frame base:
                          v_base = Ad(T_base_ee) * v_ee_body
          - J           : Jacobiano espacial en base (6x7)
        """
        v_tool = self.v_tool_body.copy().reshape((6,))
        if np.linalg.norm(v_tool) < 1e-6:
            return np.zeros(7)

        #tool -> ee (cambio de punto)
        v_ee = self.tool_twist_to_ee_twist(v_tool)

        # ee -> base con TF
        Ad_base_ee = self.get_adjoint_base_ee()
        if Ad_base_ee is None:
            return np.zeros(7)

        v_base = Ad_base_ee.dot(v_ee)

        q = self.q.copy()

        # Jacobiano (6x7)
        J = np.asarray(kinova_jacobian(q)).reshape((6, 7))

        lam2 = self.damping ** 2
        JJt = J.dot(J.T)
        A = JJt + lam2 * np.eye(6)

        # Resolver (J J^T + λ² I) x = v_base
        x = np.linalg.solve(A, v_base)

        # dq = J^T x
        dq = J.T.dot(x)

        # Saturación de velocidad articular
        dq = np.clip(dq, -self.max_joint_speed, self.max_joint_speed)

        return dq

    # -------------- Timer / bucle --------------

    def timer_callback(self):
        if not self.has_initial_state:
            return

        dq = self.compute_dq_from_twist()
        self.q = self.q + dq * self.dt

        gripper_positions = (self.gripper_factor * self.gripper_closed).tolist()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.q.tolist() + gripper_positions
        msg.velocity = []
        msg.effort = []

        self.publisher_.publish(msg)

        # Publicar TF extra ee->tool_frame
        self.publish_tool_tf()

        self.log_step += 1
        if self.log_step >= int(1.0 / self.dt):
            self.log_step = 0
            self.get_logger().info(
                f"q[0:3]=[{self.q[0]:.2f}, {self.q[1]:.2f}, {self.q[2]:.2f}], "
                f"gripper_factor={self.gripper_factor:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

