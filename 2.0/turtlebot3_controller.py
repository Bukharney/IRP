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
        self.init_odom_state = False
        self.init_fuzzy = False
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

    def membership(self):
        def membership_close(x):
            if x == 4.00:
                x = 0.1
            if x <= 0.1:
                return 1.0
            elif 0.1 < x <= 1.5:
                return (1.5 - x) / 1.5
            else:
                return 0.0

        sensor_membership = []
        sensor = self.group_sensor()
        sensor = self.find_min(sensor)
        for _, value in sensor.items():
            sensor_membership.append(membership_close(value))

        print(sensor)
        return sensor_membership

    def WhatDoISee(self):
        sensor = self.group_sensor()
        sensor = self.find_min(sensor)
        front = sensor["front"]
        back = sensor["back"]
        left = sensor["left"]
        right = sensor["right"]
        print("front: " + front, "back:" + back, "left" + left, "right" + right)
        return front, back, left, right

    def timerCallback(self):
        print("-----------------------------------------------------------")
        if self.init_odom_state == True:
            WhatDoISee = self.WhatDoISee()
            input("Press any key to continue...")

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
