import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.serialization import serialize_message
import numpy as np

import rosbag2_py


class ControlStrategy(Node):
    def __init__(self, delta_t,):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(
            Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/hagen/odom", self.set_pose, 20)

        self.vel_sub
        self.odom_sub
        self.i = 0
        self.set_q_init = None
        self.q: np.ndarray = np.array([0, 0, 0])
        self.r = 0.14  # Wheel radius
        self.L = 0.46  # Axle length
        self.Ts = delta_t  # Sampling time
        self.duration = 10  # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)

        self.time_utilized = 0.0

        self.path_publisher = self.create_publisher(Path, "/path", 20)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)

        w = cy * cr * cp + sy * sr * sp
        x = cy * sr * cp - sy * cr * sp
        y = cy * cr * sp + sy * sr * cp
        z = sy * cr * cp - cy * sr * sp

        return x, y, z, w

    def listener_callback(self, msg):
        pass

    def wheels_to_diff(self, vl, vr):
        """
        Convert wheel velocities to differential drive velocities
        """
        v = (vr + vl)/2
        w = (vr - vl)/self.L
        return v, w

    def stop_vehicle(self, ):
        self.v = 0.0
        self.w = 0.0
        for _ in range(100):
            self.send_vel(0.0, 0.0)

    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap) > np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]

    def timer_callback(self, ):
        # uncomment if path has some problems appearing in rviz
        self.send_path_rviz()

        # we want to simulate up to self.duration
        if self.time_utilized >= self.duration or self.ref_idx + 1 >= len(self.control_points):
            self.stop_vehicle()
            self.get_logger().info("Simulation ended")
            self.end_controller = True
            return

        # logic for control strategy

        # direction vector along T_i
        v = self.control_points[self.ref_idx+1] - \
            self.control_points[self.ref_idx]
        # orthogonal vector to v
        v_orth = np.array([-v[1], v[0]])

        # vector from T_i to current position
        r = self.q[:2] - self.control_points[self.ref_idx]

        # check switch of segment
        if np.dot(v, v) != 0:
            u = np.dot(v, r) / np.dot(v, v)
        else:
            # just to move next segment if v is zero vector
            u = 1.1

        if u > 1:
            # need to switch to next segment
            self.ref_idx += 1
            self.get_logger().info("Switched to next segment")
            self.get_logger().info(
                f"Current position: {self.q[:2]}, reference point: {self.control_points[self.ref_idx]} ")
            return

        # normalized orthogonal distance between current pose and the line segment
        d = np.dot(v_orth, r) / np.linalg.norm(v_orth)

        # orientation of the line segment
        phi_lin = np.arctan2(v[1], v[0])

        # small positive coefficient to move follow perpendicular to the line when we are far
        k_r = 0.1
        phi_rot = np.arctan(k_r * d)

        phi_ref = phi_lin + phi_rot
        e_phi = self.wrap_to_pi(phi_ref - self.q[2])

        v = self.k_p * np.cos(e_phi)
        w = self.k_theta * e_phi

        self.send_vel(v, w)

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if (self.set_q_init is None):
            self.set_q_init = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.set_q_init

    def send_vel(self, v, w):
        # create message to be published
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher_.publish(msg)

        # update estimated odometry
        dq = np.array([
            v*np.cos(self.q[2]+self.Ts*w/2),
            v*np.sin(self.q[2]+self.Ts*w/2),
            w,
        ])
        self.q = self.q + self.Ts*dq  # Integration
        # Map orientation angle to [-pi, pi]
        self.q[2] = self.wrap_to_pi(self.q[2])
        self.time_utilized += self.Ts

    def reference_path_follower_diff_drive(
            self,
            duration=50,
            control_points=np.array([[3, 0], [6, 4], [3, 4], [3, 1], [0, 3]]),
            k_p=0.4,
            k_theta=3,
    ):
        self.duration = duration
        self.control_points = np.array([[0, 0], *control_points])
        self.k_p = k_p
        self.k_theta = k_theta
        self.ref_idx = 0

        self.send_path_rviz()

    def send_path_rviz(self,):
        # self.get_logger().info("Sending path to rviz")
        # create path for rviz
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        for i, point in enumerate(self.control_points):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])

            if i + 1 < len(self.control_points):
                v = self.control_points[i+1] - self.control_points[i]
                phi = np.arctan2(v[1], v[0])
                q = self.quaternion_from_euler(0, 0, phi)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            else:
                # take previous orientation
                pose.pose.orientation.x = path.poses[-1].pose.orientation.x
                pose.pose.orientation.y = path.poses[-1].pose.orientation.y
                pose.pose.orientation.z = path.poses[-1].pose.orientation.z
                pose.pose.orientation.w = path.poses[-1].pose.orientation.w

            path.poses.append(pose)

        self.path_publisher.publish(path)


def main(args=None):
    rclpy.init()
    control_strategy = ControlStrategy(delta_t=0.033)

    control_strategy.reference_path_follower_diff_drive()

    control_strategy.get_logger().info(f"c: {control_strategy.control_points}")

    while control_strategy.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            control_strategy.stop_vehicle()
            break
    control_strategy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
