# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
from socket import TIPC_SUBSCR_TIMEOUT
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import numpy
import sys
import termios

# from std_msgs.msg import String


terminal_msg = """
Turtlebot3 Control
----------------------------------------------------
From the current pose, please enter input!
----------------------------------------------------
"""


def cal_avg(numbers):
    chuck_num = []

    for i in range(0, 360, 10):
        chuck_num.append(numbers[i : i + 10])

    averages = []
    for chuck in chuck_num:
        valid_num = [num for num in chuck if num != 0.0]
        if valid_num:
            avg = sum(valid_num) / len(valid_num)
            averages.append(avg)
        else:
            averages.append(2.0)
    return averages


class Turtlebot3Controller(Node):
    def __init__(self):
        super().__init__("turtlebot3_controller")  # node name
        qos = QoSProfile(depth=10)
        self.cmdVelPublisher = self.create_publisher(Twist, "cmd_vel", qos)
        self.scanSubscriber = self.create_subscription(
            LaserScan, "scan", self.scanCallback, qos_profile=1
        )
        self.batteryStateSubscriber = self.create_subscription(
            BatteryState, "battery_state", self.batteryStateCallback, 1
        )
        self.odomSubscriber = self.create_subscription(
            Odometry, "odom", self.odomCallback, qos
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
        self.timer = self.create_timer(0.1, self.timerCallback)

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
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_rad = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state = True

    def ProgressBar(self):
        progress = 1.0 - (self.angle / self.total_angle)
        progress_bar = (
            "[" + "=" * int(progress * 20) + " " * (20 - int(progress * 20)) + "]"
        )
        print(f"Turning progress: {progress_bar} {progress:.2%}", end="\r")

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
                self.ProgressBar()
                if math.fabs(self.angle) < 0.5:
                    angular_velocity = 0.05
                else:
                    angular_velocity = (math.fabs(self.angle) / 10) + 0.1

                if math.fabs(self.angle) > 0.008:
                    self.publishVelocityCommand(0.0, angular_velocity * self.turn_side)

                else:
                    self.publishVelocityCommand(0.0, 0.0)
                    self.step += 1

            elif self.step == 2:
                self.step = 1
                self.state += 1
                self.get_key_state = False

    def GoTo(self, distance):
        if self.get_key_state is False:
            self.distance_m = distance / 100
            self.start_x = self.last_pose_x
            self.start_y = self.last_pose_y
            self.done = 0
            self.get_key_state = True

        else:
            if self.step == 1:
                walk_distance = math.sqrt(
                    (self.last_pose_x - self.start_x) ** 2
                    + (self.last_pose_y - self.start_y) ** 2
                )
                remain_distance = self.distance_m - walk_distance
                print(f"remain_distance : {remain_distance}", end="\r")
                if remain_distance < 0.4:
                    if remain_distance < 0.2:
                        linear_velocity = 0.1
                    else:
                        linear_velocity = remain_distance
                else:
                    linear_velocity = 0.5

                if remain_distance > 0.008:
                    if self.done == 0:
                        self.publishVelocityCommand(linear_velocity, 0.0)

                else:
                    self.publishVelocityCommand(0.0, 0.0)
                    self.step += 1

            elif self.step == 2:
                self.step = 1
                self.state += 1
                self.get_key_state = False

    def get_key(self):
        # Print terminal message and get inputs
        print(terminal_msg)
        input_theta = float(input("Choose State : "))
        settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        return input_theta

    def walk(self):
        if self.init_odom_state is True:
            if self.state == 0:
                self.GoTo(300)
            elif self.state == 1:
                self.TurnTo(-1210)
            elif self.state == 2:
                self.GoTo(250)
            elif self.state == 3:
                self.TurnTo(270)
            elif self.state == 4:
                self.GoTo(50)
            elif self.state == 5:
                self.TurnTo(-450)
            elif self.state == 6:
                self.GoTo(50)
            elif self.state == 7:
                self.TurnTo(-650)
            elif self.state == 8:
                self.GoTo(100)
            elif self.state == 9:
                self.TurnTo(850)
            elif self.state == 10:
                self.GoTo(100)
            elif self.state == 11:
                self.TurnTo(-500)
            elif self.state == 12:
                self.GoTo(120)
            elif self.state == 13:
                self.state = self.get_key()

    def timerCallback(self):
        self.walk()

        # self.publishVelocityCommand(linearVelocity,angularVelocity)

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

    tb3ControllerNode.destroy_node()
    robotStop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
