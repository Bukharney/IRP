# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
import math
from socket import TIPC_SUBSCR_TIMEOUT
from unittest import case

import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# from std_msgs.msg import String


def cal_avg(numbers):
    # slide to 36 chuck
    chuck_num = []
    for i in range(0, 360, 10):
        chuck_num.append(numbers[i : i + 10])

    # calculate average in each chuck
    averages = []
    for chuck in chuck_num:
        # remove 0.0 value in chuck
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
        self.get_key_state_1 = False
        self.get_key_state_2 = False
        self.get_key_state_3 = False
        self.init_odom_state = False
        self.init_fuzzy = False
        self.turn_side = 1
        self.seen = []
        self.walk = 0
        self.detect = 0
        self.see = 0
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
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_rad = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.init_odom_state = True

    def group_sensor(self):
        cal = cal_avg(self.valueLaserRaw["ranges"])
        sensor = {
            "front": [cal[0], cal[35]],
            "back": [cal[17], cal[18]],
            "front_left": cal[3:6],
            "front_right": cal[29:32],
            "left": cal[8:10],
            "right": cal[26:28],
            "back_left": cal[12:14],
            "back_right": cal[21:23],
        }
        return sensor

    def find_min(self, sensor):
        min_sensor = {}
        # find min value in each sensor
        for key, value in sensor.items():
            min_sensor[key] = min(value)
        return min_sensor

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
                self.walk = 0
                self.get_key_state = False
                self.state += 1

    def WhatDoISee(self):
        class detect:
            front = False
            back = False
            left = False
            right = False

        sensor = self.group_sensor()
        sensor = self.find_min(sensor)

        count = 4
        if sensor["front"] <= 0.3:
            count -= 1
            detect.front = True
        if sensor["back"] <= 0.3:
            count -= 1
            detect.back = True
        if sensor["left"] <= 0.3:
            count -= 1
            detect.left = True
        if sensor["right"] <= 0.3:
            count -= 1
            detect.right = True

        return count, detect

    def keep_distance(self):
        sensor = self.group_sensor()
        sensor = self.find_min(sensor)
        if sensor["left"] - sensor["right"] > 0.1:
            return 0.1
        elif sensor["left"] - sensor["right"] < -0.1:
            return -0.1
        else:
            return 0.0

    def GTNN(self, n_node):
        distance = 29.0 * n_node
        if self.get_key_state is False:
            self.distance_m = distance / 100
            self.start_x = self.last_pose_x
            self.start_y = self.last_pose_y
            self.get_key_state = True

        else:
            if self.step == 1:
                walk_distance = math.sqrt(
                    (self.last_pose_x - self.start_x) ** 2
                    + (self.last_pose_y - self.start_y) ** 2
                )
                remain_distance = self.distance_m - walk_distance
                print(f"remain_distance : {remain_distance}", end="\r")
                linear_velocity = 0.1
                angular_velocity = 0.0
                if remain_distance > 0.01:
                    self.publishVelocityCommand(linear_velocity, angular_velocity)
                else:
                    self.publishVelocityCommand(0.0, 0.0)
                    self.step += 1
            elif self.step == 2:
                self.step = 1
                self.walk += 1
                self.get_key_state = False
                self.state += 1

    def seen_history(self, see):
        if len(self.seen) <= 3:
            self.seen.append(see)
        else:
            self.seen.pop(0)
            self.seen.append(see)

    def walk_dicide(self, detect):
        print("dicide")
        if detect.front == False:
            self.GTNN(1)
        else:
            if detect.left == True and detect.right == True:
                input()
            elif detect.left == True:
                self.TurnTo(-91)
            elif detect.right == True:
                self.TurnTo(91)
            else:
                self.TurnTo(-91)

    def win(self, see):
        print("win")
        print("walk: ", self.walk)
        if self.walk == 3:
            print("victory")
            if self.state == 0:
                if see == 3:
                    self.TurnTo(-91)
                    if self.state == 1:
                        self.GTNN(1)
                    elif self.state == 2:
                        input()
                else:
                    if self.state == 1:
                        self.TurnTo(180)
                    elif self.state == 2:
                        self.GTNN(2)
                    elif self.state == 3:
                        self.TurnTo(-91)
                    elif self.state == 4:
                        self.GTNN(1)
                    elif self.state == 5:
                        input()
        elif self.seen == [3, 3, 3]:
            if self.state == 0:
                self.TurnTo(-90)
            elif self.state == 1:
                self.GTNN(1)
            elif self.state == 3:
                self.get_key_state_2 = True
        else:
            self.get_key_state_2 = True

    def walk_to(self):
        if self.init_odom_state is True:
            if self.get_key_state_1 is False:
                self.state = 0
                if self.get_key_state_2 is False:
                    if self.get_key_state_3 is False:
                        self.see, _ = self.WhatDoISee()
                        self.get_key_state_3 = True
                    else:
                        self.win(self.see)
                self.state = 0
                see, self.detect = self.WhatDoISee()
                self.seen_history(see)
                self.get_key_state_1 = True
            else:
                if self.state == 0:
                    self.walk_dicide(self.detect)
                elif self.state == 1:
                    self.get_key_state_2 = False
                    self.get_key_state_1 = False

    def timerCallback(self):
        self.walk_to()
        print("state: ", self.state)

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
