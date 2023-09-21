# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
import math
from socket import TIPC_SUBSCR_TIMEOUT

import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# from std_msgs.msg import String


def cal_avg(numbers):
    chuck_num = []
    for i in range(0, 60, 10):
        chuck_num.append(numbers[i : i + 10])
    for i in range(60, 300, 10):
        chuck_num.append(numbers[i : i + 10])
    for i in range(300, 360, 10):
        chuck_num.append(numbers[i : i + 10])

    averages = []
    for chuck in chuck_num:
        valid_num = [num for num in chuck if num != 0.0]
        if valid_num:
            avg = sum(valid_num) / len(valid_num)
            averages.append(avg)
        else:
            averages.append(4.0)
    return averages


class Turtlebot3Controller(Node):
    def __init__(self):
        super().__init__("turtlebot3_controller")  # node name
        self.cmdVelPublisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.scanSubscriber = self.create_subscription(
            LaserScan, "scan", self.scanCallback, qos_profile=qos_profile_sensor_data
        )
        self.batteryStateSubscriber = self.create_subscription(
            BatteryState, "battery_state", self.batteryStateCallback, 1
        )
        self.odomSubscriber = self.create_subscription(
            Odometry, "odom", self.odomCallback, 1
        )
        self.valueLaserRaw = {
            "range_min": 0.0,
            "range_max": 0.0,
            "ranges": [0.0] * 360,
        }
        self.valueBatteryState = None
        self.valueOdometry = {
            "position": None,  # Datatype: geometry_msg/Point   (x,y,z)
            "orientation": None,  # Datatype: geometry_msg/Quaternion (x,y,z,w)
            "linearVelocity": None,  # Datatype: geometry_msg/Vector3 (x,y,z)
            "angularVelocity": None,  # Datatype: geometry_msg/Vector3 (x,y,z)
        }

        self.state = 0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_rad = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_rad = 0.0
        self.step = 1
        self.get_key_state = False
        self.init_odom_state = False
        self.turn_side = 1
        # Use this timer for the job that should be looping until interrupted
        self.timer = self.create_timer(0.25, self.timerCallback)

    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        self.cmdVelPublisher.publish(msg)
        # self.get_logger().info('Publishing cmd_vel: "%s", "%s"' % linearVelocity, angularVelocity)

    def scanCallback(self, msg):
        self.valueLaserRaw = {
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "ranges": list(msg.ranges),
        }

    def batteryStateCallback(self, msg):
        self.valueBatteryState = msg

    def odomCallback(self, msg):
        self.valueOdometry = {
            "position": msg.pose.pose.position,
            "orientation": msg.pose.pose.orientation,
            "linearVelocity": msg.twist.twist.linear,
            "angularVelocity": msg.twist.twist.angular,
        }
        _, _, self.last_rad = self.euler_from_quaternion(
            self.valueOdometry["orientation"]
        )

    def avoid_hit(self):
        cal = cal_avg(self.valueLaserRaw["ranges"])
        front = [cal[0], cal[35]]
        left = cal[1:3]
        right = cal[33:35]

        if min(front) < 0.25:
            print("Stop front detect!")
            self.publishVelocityCommand(0.0, 0.0)
            self.state += 1
        elif min(left) < 0.2:
            print("Stop left detect!")
            self.publishVelocityCommand(0.0, 0.0)
            self.state += 1
        elif min(right) < 0.2:
            print("Stop right detect!")
            self.publishVelocityCommand(0.0, 0.0)
            self.state += 1
        else:
            self.publishVelocityCommand(0.2, 0.0)

    def turn(self):
        cal = cal_avg(self.valueLaserRaw["ranges"])

        max_index = cal.index(max(cal))
        print(max_index)
        if max_index < 18 and max_index > 0:
            if max_index < 3:
                self.publishVelocityCommand(0.0, 0.2)
            else:
                self.publishVelocityCommand(0.0, 0.9)
        elif max_index > 18:
            if max_index > 32:
                self.publishVelocityCommand(0.0, -0.2)
            else:
                self.publishVelocityCommand(0.0, -0.9)
        else:
            self.publishVelocityCommand(0.0, 0.0)
            self.step += 1

    def normalize_angle(self, angle):
        if angle > math.pi:
            n = (angle / math.pi) % 2
            angle = angle % math.pi
            if int(n) != 0:
                angle = angle - math.pi
        if angle < -math.pi:
            n = (angle / -math.pi) % 2
            angle = angle % -math.pi
            if int(n) != 0:
                angle = angle + math.pi
        return angle

    def TurnTo(self, angle):
        if not self.get_key_state:
            input_rad = angle
            if input_rad > 0:
                if int((input_rad / 180)) % 2 == 0:
                    self.turn_side = 1
                else:
                    self.turn_side = -1
            else:
                if int((input_rad / -180)) % 2 == 0:
                    self.turn_side = -1
                else:
                    self.turn_side = 1

            self.goal_rad = self.normalize_angle(
                self.last_rad + numpy.deg2rad(input_rad)
            )
            self.total_angle = self.normalize_angle(self.goal_rad - self.last_rad)
            print("Goal", self.goal_rad)
            self.get_key_state = True

        else:
            if self.step == 1:
                self.angle = self.normalize_angle(self.goal_rad - self.last_rad)
                print(self.angle)
                if math.fabs(self.angle) < 0.5:
                    angular_velocity = 0.1
                else:
                    angular_velocity = (math.fabs(self.angle) / 10) + 0.2

                if math.fabs(self.angle) > 0.05:
                    self.publishVelocityCommand(0.0, angular_velocity * self.turn_side)

                else:
                    self.publishVelocityCommand(0.0, 0.0)
                    self.step += 1

            elif self.step == 2:
                self.step = 1
                self.get_key_state = False
                self.state += 1

    def timerCallback(self):
        print("-----------------------------------------------------------")
        cal = cal_avg(self.valueLaserRaw["ranges"])
        closed_angle = cal.index(min(cal))
        if self.state == 0:
            self.avoid_hit()
        elif self.state == 1:
            if closed_angle < 18 and closed_angle > 0:
                self.TurnTo(closed_angle * 10)
            else:
                self.TurnTo(closed_angle / 2 * -10)
        elif self.state == 2:
            self.state = 0

    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        # sinr_cosp = 2 * (w * x + y * z)
        # cosr_cosp = 1 - 2 * (x * x + y * y)
        # roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        # sinp = 2 * (w * y - z * x)
        # pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return None, None, yaw


def robotStop():
    node = rclpy.create_node("tb3Stop")
    publisher = node.create_publisher(Twist, "cmd_vel", 1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    tb3ControllerNode = Turtlebot3Controller()
    print("tb3ControllerNode created")
    try:
        rclpy.spin(tb3ControllerNode)
    except KeyboardInterrupt:
        print("Done")

    tb3ControllerNode.publishVelocityCommand(0.0, 0.0)
    tb3ControllerNode.destroy_node()
    robotStop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
