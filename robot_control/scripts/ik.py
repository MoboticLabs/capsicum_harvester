#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

from visualization_msgs.msg import Marker


def skew(v):
    return np.array([
        [0,     -v[2],  v[1]],
        [v[2],   0,    -v[0]],
        [-v[1],  v[0],  0   ]
    ])

def matrix_exp(S, theta):
    w = S[:3]
    v = S[3:]
    if np.linalg.norm(w) < 1e-6:
        T = np.eye(4)
        T[:3, 3] = v * theta
        return T

    so3 = skew(w)
    so3_sq = np.dot(so3, so3)

    R_exp = np.eye(3) + np.sin(theta) * so3 + (1 - np.cos(theta)) * so3_sq
    G = np.eye(3) * theta + (1 - np.cos(theta)) * so3 + (theta - np.sin(theta)) * so3_sq

    T = np.eye(4)
    T[:3, :3] = R_exp
    T[:3, 3] = np.dot(G, v)
    return T

def log_exp(T):
    Rm = T[:3, :3]
    p = T[:3, 3]
    tr = np.trace(Rm)

    if tr >= 3.0 - 1e-6:
        return np.r_[0, 0, 0, p]

    theta = np.arccos(np.clip((tr - 1) / 2, -1, 1))
    so3 = (Rm - Rm.T) / (2 * np.sin(theta))
    w = np.array([so3[2, 1], so3[0, 2], so3[1, 0]])

    so3_sq = np.dot(so3, so3)
    G_inv = (np.eye(3) / theta
             - 0.5 * so3
             + (1 / theta - 0.5 / np.tan(theta / 2)) * so3_sq)

    v = np.dot(G_inv, p)
    return np.r_[w * theta, v * theta]

def adjoint(T):
    Rm = T[:3, :3]
    p = T[:3, 3]
    p_sk = skew(p)

    Adj = np.zeros((6, 6))
    Adj[:3, :3] = Rm
    Adj[3:, 3:] = Rm
    Adj[3:, :3] = np.dot(p_sk, Rm)
    return Adj


class InverseKinematics(Node):
    def __init__(self):
        super().__init__("inverse_kinematics")

        self.err_log = []
        self.l1 = 0.166
        self.l2 = 0.221
        self.l3 = 0.128
        self.l4 = 0.095
        self.l5 = 0.041

        self.M = np.eye(4)
        self.M[2, 3] = self.l1 + self.l2 + self.l3 + self.l4 + self.l5

        self.S_list = np.array([
            [0, 0, 1, 0, 0, 0],
            [1, 0, 0, 0, self.l1, 0],
            [-1, 0, 0, 0, -(self.l1 + self.l2), 0],
            [0, 0, 1, 0, 0, 0],
            [-1, 0, 0, 0, -(self.l1 + self.l2 + self.l3 + self.l4), 0]
        ]).T

        self.ndof = 5
        self.current_joints = np.zeros(self.ndof)

        self.create_subscription(JointState, "/joint_states", self.joint_callback, 10)
        self.create_subscription(PoseStamped, "/target_pose", self.solve_ik_callback, 10)
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/arm_controller/commands",
            10,
        )
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.js_timer = self.create_timer(0.02, self.publish_joint_states)

        self.get_logger().info("IK solver (body frame) ready.")
        self.get_logger().info(str(self.fk(np.zeros(self.ndof))))

    def joint_callback(self, msg):
        if len(msg.position) >= self.ndof:
            self.current_joints = np.array(msg.position[:self.ndof])

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        js.position = list(self.current_joints)
        self.joint_state_pub.publish(js)

    def fk(self, theta):
        T = np.eye(4)
        for i in range(self.ndof):
            T = np.dot(T, matrix_exp(self.S_list[:, i], theta[i]))
        T = np.dot(T, self.M)
        return T

    def jacobian_space(self, theta):
        J = np.zeros((6, self.ndof))
        T = np.eye(4)
        J[:, 0] = self.S_list[:, 0]

        for i in range(1, self.ndof):
            T = np.dot(T, matrix_exp(self.S_list[:, i - 1], theta[i - 1]))
            J[:, i] = np.dot(adjoint(T), self.S_list[:, i])
        return J

    def publish_marker(self, position, marker_id, r, g, b):
        marker = Marker()
        marker.header.frame_id = "link1"   # your base frame
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0


        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        self.marker_pub.publish(marker)


    def solve_ik_callback(self, msg):
        pos = np.array([msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z])

        quat = [msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w]

        T_sd = np.eye(4)
        T_sd[:3, :3] = R.from_quat(quat).as_matrix()
        T_sd[:3, 3] = pos

        self.publish_marker(pos, marker_id=1, r=1.0, g=0.0, b=0.0)   # red target


        theta = self.current_joints.copy()

        max_iters = 200
        eps_w = 0.01
        eps_v = 0.001
        alpha = 0.5  

        for i in range(max_iters):
            
            T_sb = self.fk(theta)
            ee_pos = T_sb[:3,3]
            self.publish_marker(ee_pos, marker_id = 2, r=0.0, g=1.0, b=0.0)
            
            T_bd = np.dot(np.linalg.inv(T_sb), T_sd)
            
            V_b = log_exp(T_bd)
            
            v_err = V_b[3:]
            w_err = V_b[:3]
            #self.err_log.append(v_err.copy())
            
            if np.linalg.norm(w_err) < eps_w and np.linalg.norm(v_err) < eps_v:
                self.publish_positions(theta)
                self.get_logger().info(f"IK converged in {i} iterations.")
                self.get_logger().info(f"Final angular error: {np.linalg.norm(w_err):.6f}")
                self.get_logger().info(f"Final linear error: {np.linalg.norm(v_err):.6f}")
                self.get_logger().info(f"final transfomration: {T_sb}")
                return

            
            J_s = self.jacobian_space(theta)
            J_b = np.dot(np.linalg.inv(adjoint(T_sb)), J_s)
            
            JJT = np.dot(J_b, J_b.T)
            lambda_damp = 0.01
            damp = lambda_damp**2 * np.eye(6)
            
            
            x = np.linalg.solve(JJT + damp, V_b)
            theta_dot = np.dot(J_b.T, x)
            
            
            theta = theta + alpha * theta_dot
            
            if i % 20 == 0:
                self.get_logger().info(f"Iter {i}: ||w_err||={np.linalg.norm(w_err):.4f}, ||v_err||={np.linalg.norm(v_err):.4f}")

        self.get_logger().warn("IK failed to converge.")
        #self.err_log = np.array(self.err_log)
        #np.save('/tmp/ik_err_log.npy', self.err_log)

    def publish_positions(self, theta):
        self.current_joints = theta.copy()
        msg = Float64MultiArray()
        msg.data = list(theta)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent joint command: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()