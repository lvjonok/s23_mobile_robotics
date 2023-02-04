import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
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
        self.pose_publisher = self.create_publisher(
            PoseStamped, "/estimated_pose", 20)

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

    def init_diff_speed(self, name, v, w):
        self.v = float(v)
        self.w = float(w)

        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=f'hw1_{name}.bag', storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/estimated_pose',
            type='geometry_msgs/msg/PoseStamped',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.pose_subscriber = self.create_subscription(
            PoseStamped, "/estimated_pose", self.pose_callback, 20)

    def timer_callback(self, ):
        # we want to simulate up to self.duration
        if self.time_utilized < self.duration:
            self.send_vel(self.v, self.w)
            return

        self.stop_vehicle()
        self.get_logger().info("Simulation ended")
        self.end_controller = True

    def set_pose(self, msg):
        # self.get_logger().info("Setting initial pose")
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

        # send to topic with estimated pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = self.q[0]
        pose.pose.position.y = self.q[1]
        pose.pose.position.z = self.q[2]

        self.pose_publisher.publish(pose)

    def pose_callback(self, msg):
        self.writer.write(
            '/estimated_pose',
            serialize_message(msg),
            self.get_clock().now().nanoseconds,
        )


def main(args=None):
    rclpy.init()
    control_strategy = ControlStrategy(delta_t=0.033)

    configurations = [
        (0.5, 0.0),
        (1, 2),
        (0, 2),
        control_strategy.wheels_to_diff(
            20 * control_strategy.r,
            18 * control_strategy.r,
        ),
    ]

    # change index to run different configurations
    idx = 0
    print(f"Running configuration {idx}", configurations[idx])
    control_strategy.init_diff_speed(f"config{idx}", *configurations[idx])

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
