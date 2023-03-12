import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.serialization import serialize_message
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, Image, CameraInfo
import numpy as np

import rosbag2_py


class PurePursuit:
    def __init__(self, L=1.517363 + 1.013637):
        self.L = L
        self.k = 0.3
        self.kp = 3
        self.target_vel = 1

    def control(self, back_axle, cur_v, ref_pose):
        """
        control computes linear velocity and steering angle required for current step

        back_axle - middle of back axle
        """
        x, y, theta = back_axle
        t_x, t_y = ref_pose

        alpha = np.arctan2(t_y - y, t_x - x) - theta
        delta = np.arctan(2 * self.L * np.sin(alpha) /
                          (self.k * (cur_v + 1e-5) + self.L))

        delta_min = -0.7
        delta_max = -delta_min
        if delta > delta_max:
            delta = delta_max
        elif delta < delta_min:
            delta = delta_min

        return delta, self.kp * (self.target_vel - cur_v)


class StanleyControl:
    def __init__(self, L=1.517363 + 1.013637):
        self.L = L
        self.k = 0.5
        self.kp = 1

        self.target_vel = 0.5

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def control(self, front_axle, cur_v, ref_pose, prev_pose):
        # calculate cross track error

        # direction of front axle
        fa_dir = np.array([np.cos(front_axle[2]), np.sin(front_axle[2]), 0])
        # direction of current path
        ref_dir = np.array([np.cos(ref_pose[2]), np.sin(ref_pose[2]), 0])

        # closest point on current path segment
        self.closest_path_point = (np.dot(front_axle[:2] - prev_pose[:2], ref_dir[:2]) / np.dot(
            ref_dir[:2], ref_dir[:2])) * ref_dir

        err = front_axle[:2] - prev_pose[:2] - self.closest_path_point[:2]
        # print(err)

        err_scalar = np.linalg.norm(err)
        if np.dot(np.cross(front_axle - np.array([*prev_pose, 0]), self.closest_path_point), np.array([0, 0, 1])) > 0:
            # we have to turn left
            ...
        else:
            # we have to turn right
            err_scalar *= -1

        e_delta = np.arctan2((self.k * err_scalar), (self.kp * cur_v + 0.01))

        # calculate target error
        e_phi = self.normalize_angle(ref_pose[2] - front_axle[2])
        # print(f"{err_scalar:3.3} {e_delta:3.3}, {e_phi:3.3} {err}")

        delta = e_delta + e_phi
        delta_min = -0.7
        delta_max = -delta_min
        if delta > delta_max:
            delta = delta_max
        elif delta < delta_min:
            delta = delta_min

        return delta, self.kp * (self.target_vel - cur_v)


class ControlStrategy(Node):
    def __init__(self, delta_t, controller):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(
            Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/hagen/odom", self.set_pose, 20)
        self.camera_sub = self.create_subscription(
            Image, "/depth_camera/image_raw", self.new_camera_shot, 20
        )
        self.proj_pub = self.create_publisher(
            Image, "/depth_camera/projected", 30
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/hagen/scan", self.laser, 20
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/depth_camera/camera_info", self.new_camera_info, 20
        )
        self.last_scan = None
        self.last_camera_info = None

        self.vel_sub
        self.odom_sub
        self.i = 0
        self.set_q_init = None
        self.q: np.ndarray = np.array([0, 0, 0])
        self.q_real = np.array([0, 0, 0])

        # data taken from URDF
        self.r = 0.325  # Wheel radius
        self.L_back = 1.01364   # distance from chassis link to back axle
        self.L_front = 1.51736  # distance from chassis link to front axle
        self.L = self.L_back + self.L_front

        self.Ts = delta_t  # Sampling time
        self.duration = 30  # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)

        self.time_utilized = 0.0
        self.controller = controller

        self.path_publisher = self.create_publisher(
            Path, "/autocar/viz_path", 20)
        self.marker_publisher = self.create_publisher(
            Marker, '/autocar/viz_goals', 20)

        self.w, self.v = 0.0, 0.0

        self.metric = 0.0
        self.metric_real = 0.0
        self.metric_counter = 0
        self.q_history = [[*self.q, self.time_utilized]]
        self.q_history_real = [[*self.q, self.time_utilized]]

    def laser(self, msg):
        self.last_scan = msg

    def lidar2pos(self, scan: LaserScan) -> np.ndarray:
        # generate array of angles
        angles = np.arange(scan.angle_min, scan.angle_max,
                           scan.angle_increment)

        poses = []
        for angle, range in zip(angles, scan.ranges):
            if range == np.inf:
                continue

            poses.append(
                np.array([
                    np.cos(angle) * range,
                    np.sin(angle) * range,
                    0,
                ]),
            )

        return np.array(poses)

    def new_camera_shot(self, msg: Image):
        if not self.last_scan or not self.last_camera_info:
            return

        img = np.array(msg.data, dtype=np.uint8)
        img = np.reshape(img, (msg.height, msg.width, 3))

        res3d = self.lidar2pos(self.last_scan)

        image_points = []
        for p3d in res3d:
            x, y, z = p3d[0], p3d[1], p3d[2]
            # image_points.append(camera.project3dToPixel((-y, -z, x)))

            u = self.proj_matrix @ np.array([-y, -z, x, 1])
            x, y = u[0] / u[2], u[1] / u[2]
            image_points.append(np.array([x, y]))

        image_points = np.array(image_points)

        copyimg = img.copy()

        def draw_circ(y, x):
            for dx, dy in zip(list(range(-2, 3)), list(range(-2, 3))):
                nx, ny = x + dx, y + dy
                if 0 <= nx <= msg.width and 0 <= ny <= msg.height:
                    copyimg[int(ny), int(nx)] = np.array(
                        [255, 0, 0], dtype=np.uint8)

        for (x, y) in image_points:
            if 0 <= x <= msg.width and 0 <= y <= msg.height:
                draw_circ(y, x)
                # copyimg[int(y), int(x)] = np.array([255, 0, 0], dtype=np.uint8)

        copyimg = np.array(copyimg.flatten(), dtype=np.int)

        newimg = Image()
        newimg.width = msg.width
        newimg.height = msg.height
        newimg.header = msg.header
        newimg.encoding = msg.encoding
        print('pub2', copyimg)
        newimg.data = [int(v) for v in copyimg]

        self.proj_pub.publish(newimg)

    def new_camera_info(self, msg):
        if not self.last_camera_info:
            self.last_camera_info: CameraInfo = msg
            self.proj_matrix = np.array(
                self.last_camera_info.p).reshape((3, 4))

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
        # return
        # uncomment if path has some problems appearing in rviz
        # self.send_path_rviz()

        # we want to simulate up to self.duration
        if self.time_utilized >= self.duration or self.ref_idx + 1 >= len(self.targets):
            self.stop_vehicle()
            self.get_logger().info("Simulation ended")
            self.end_controller = True
            return

        # logic for control strategy

        # vector from current point to next
        v = self.targets[self.ref_idx+1] - \
            self.targets[self.ref_idx]

        # current position relative to current point
        qv = self.q[:2] - self.targets[self.ref_idx]
        self.metric += np.linalg.norm(qv - v)
        self.metric_real += np.linalg.norm(
            self.q_real[:2] - self.targets[self.ref_idx] - v)
        self.metric_counter += 1

        # vector from T_i to current position
        r = self.q[:2] - self.targets[self.ref_idx]

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
                f"POS: {self.q}, REF: {self.targets[self.ref_idx]} ")
            return

        next_point = self.targets[self.ref_idx + 1]

        # uncomment to use Pure Pursuit controller
        # back_axle = self.q - \
        #     np.array([np.cos(self.q[2]), np.sin(self.q[2]), 0]) * self.L_back
        # steering, a = self.controller.control(back_axle, self.v, next_point)

        # uncomment for stanley control
        front_axle = self.q + \
            np.array([np.cos(self.q[2]), np.sin(self.q[2]), 0]) * self.L_front

        v = self.targets[self.ref_idx+1] - self.targets[self.ref_idx]
        phi = np.arctan2(v[1], v[0])

        steering, a = self.controller.control(
            front_axle, self.v, np.array([*next_point, phi]), self.targets[self.ref_idx])

        self.send_vel(self.v, steering)
        self.v += a * self.Ts

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if (self.set_q_init is None):
            self.set_q_init = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw + np.pi / 2])
            self.q = self.set_q_init

            # uncomment for pure pursuit
            # we have to set position at rear axle
            # self.q = self.set_q_init - \
            #     np.array([
            #         np.cos(self.q[2]),
            #         np.sin(self.q[2]),
            #         0,
            #     ]) * self.L_back

            # uncomment for stanley control
            self.q = self.set_q_init - \
                np.array([
                    np.cos(self.q[2]),
                    np.sin(self.q[2]),
                    0,
                ]) * self.L_front

            self.q_real = self.q
        else:
            # uncomment for pure pursuit update
            # self.q_real = np.array([
            #     msg.pose.pose.position.x,
            #     msg.pose.pose.position.y,
            #     yaw + np.pi / 2,
            # ]) - \
            #     np.array([
            #         np.cos(self.q[2]),
            #         np.sin(self.q[2]),
            #         0,
            #     ]) * self.L_back

            # uncomment for stanley control update
            self.q_real = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw + np.pi / 2,
            ]) + \
                np.array([
                    np.cos(self.q[2]),
                    np.sin(self.q[2]),
                    0,
                ]) * self.L_front
            
            # self.q = self.q_real

    def send_vel(self, v, w):
        # create message to be published
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.publisher_.publish(msg)

        x = self.q[0] + v * np.cos(self.q[2]) * self.Ts
        y = self.q[1] + v * np.sin(self.q[2]) * self.Ts
        theta = self.q[2] + self.v * np.tan(w) / self.L * self.Ts
        theta = self.wrap_to_pi(np.fmod(theta, np.pi * 2))

        self.q = np.array([x, y, theta])
        # just send where we are really
        self.send_marker_rviz(x, y)
        self.q_history.append([x, y, theta, self.time_utilized])
        self.q_history_real.append([*self.q_real, self.time_utilized])
        self.time_utilized += self.Ts

    def path_follower_car(
            self,
            duration=240,
            control_points=np.array([[3, 3], [3, 6], [0, 9]]),
    ):
        self.duration = duration
        self.targets = np.array([[0, 0], *control_points])
        self.ref_idx = 0

        self.send_path_rviz()

    def send_marker_rviz(self, x, y):
        # send marker of current lookahead selected point
        m = Marker()
        m.header.frame_id = "odom"
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.color.a = 1.0
        m.color.r = 1.0
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 1.0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.id = 0
        m.ns = "my_namespace"

        self.marker_publisher.publish(m)

    def send_path_rviz(self,):
        # create path for rviz
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        for i, point in enumerate(self.targets):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])

            if i + 1 < len(self.targets):
                v = self.targets[i+1] - self.targets[i]
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
    c = StanleyControl()
    # c = PurePursuit()
    control_strategy = ControlStrategy(delta_t=0.033, controller=c)

    sine = []
    for t in np.linspace(0, 40, 200):
        sine.append([3 * np.sin(0.1 * t), t])

    control_strategy.path_follower_car(control_points=sine)

    control_strategy.get_logger().info(
        f"control_points: {control_strategy.targets}")

    while control_strategy.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            control_strategy.stop_vehicle()
            break
    control_strategy.destroy_node()
    rclpy.shutdown()

    # # save results
    # print(f"metric: {control_strategy.metric} {control_strategy.metric_real}")

    # base = "/home/leo/study/humble_ws/src/s23_mobile_robotics/s23_mobile_robotics/data/hw2/"
    # prefix = "pure_pursuit_" + str(c.target_vel)
    # with open(f'{base}{prefix}_metric.txt', 'w') as f:
    #     f.write(f"{control_strategy.metric}\n")
    #     f.write(f"{control_strategy.metric_real}\n")
    #     f.write(f"{control_strategy.metric_counter}\n")
    # np.save(f'{base}{prefix}_history.npy', np.array(
    #     control_strategy.q_history))
    # np.save(f'{base}{prefix}_real.npy', np.array(
    #     control_strategy.q_history_real))


if __name__ == '__main__':
    main()
