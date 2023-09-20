# Authors : Nopphakorn Subs. Niwatchai Wang. Supasate Wor. Narith Tha. Tawan Thaep. Napatharak Muan.
from dis import dis
from socket import TIPC_SUBSCR_TIMEOUT
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
    for i in range(310, 360, 10):
        chuck_num.append(numbers[i : i + 10])
    for i in range(60, 310, 10):
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

    def avoid_hit(self):
        cal = cal_avg(self.valueLaserRaw["ranges"])

        l = min(cal[0:5])
        r = min(cal[31:36])
        print(r)
        print(l)

        if l < 0.25 or r < 0.25:
            self.publishVelocityCommand(0.0, 0.0)
        else:
            self.publishVelocityCommand(0.05, 0.0)

    def turn(self):
        cal = cal_avg(self.valueLaserRaw["ranges"])

        min_index = cal.index(min(cal))
        print(min_index)
        if min_index < 18 and min_index > 0:
            if min_index < 3:
                self.publishVelocityCommand(0.0, 0.2)
            else:
                self.publishVelocityCommand(0.0, 0.9)
        elif min_index > 18:
            if min_index > 32:
                self.publishVelocityCommand(0.0, -0.2)
            else:
                self.publishVelocityCommand(0.0, -0.9)
        else:
            self.publishVelocityCommand(0.0, 0.0)

    def timerCallback(self):
        print("-----------------------------------------------------------")
        print("Turn")

        # self.publishVelocityCommand(linearVelocity,angularVelocity)
        self.turn()


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
    except:
        KeyboardInterrupt
    print("Done")

    tb3ControllerNode.publishVelocityCommand(0.0, 0.0)
    tb3ControllerNode.destroy_node()
    robotStop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
