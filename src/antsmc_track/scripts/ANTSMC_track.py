import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Pose2D, Point
from visualization_msgs.msg import Marker
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import math


class waypoint:
    def __init__(self):
        self.ID = int
        self.x = float
        self.y = float
        self.yaw = float


class VehicleState:
    def __init__(self):
        self.x = float
        self.y = float
        self.yaw = float
        self.v = float
        self.kesi = float


class CtrlValue:
    def __init__(self):
        self.v = float
        self.kesi = float


def update_state(ctrl, car, T, L):
    car.v = ctrl.v
    car.kesi = ctrl.kesi
    car.x += car.v * math.cos(car.yaw) * T
    car.y += car.v * math.sin(car.yaw) * T
    car.yaw += car.v / L * math.tan(car.kesi) * T
    return car


class Path_:
    def __init__(self):
        self.waypoints = []
        self.path = []

    def add_new_point(self, waypoint):
        self.waypoints.append(waypoint)

    def add_path(self):
        self.path = self.waypoints

    def path_size(self):
        return len(self.waypoints)

    def get_waypoint(self, index):
        p = waypoint()
        p.ID = self.path[index].ID
        p.x = self.path[index].x
        p.y = self.path[index].y
        p.yaw = self.path[index].yaw
        return p

    def find_target_index(self, car_x, car_y):
        min = abs(np.sqrt(pow(car_x - self.path[0].x, 2) + pow(car_y - self.path[0].y, 2)))
        ind = 0
        for i in range(self.path_size()):
            d = abs(np.sqrt(pow(car_x - self.path[i].x, 2) + pow(car_y - self.path[i].y, 2)))
            if d < min:
                min = d
                ind = i
        if (ind + 1) < self.path_size():
            current_x = self.path[ind].x
            current_y = self.path[ind].y
            next_x = self.path[ind + 1].x
            next_y = self.path[ind + 1].y
            L_ = abs(np.sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)))
            L_1 = abs(np.sqrt(pow(car_x - next_x, 2) + pow(car_y - next_y, 2)))

            if L_ > L_1:
                ind += 1
        return ind

    def callback(self, msg):
        pass

    def test(self):
        rospy.init_node('node')
        path_sub = rospy.Subscriber('path', Path, self.callback)
        rospy.spin()


class ANTSMC_node:
    def __init__(self, initial_x, initial_y, initial_yaw, initial_v, initial_kesi):
        self.car_state = VehicleState()
        self.ctrl = CtrlValue()
        self.last_ind = None
        self.last_waypoint = None
        self.action = "the car is tracking!!"

        self.path_sub = rospy.Subscriber('path', Path, self.path_callback)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.visual_state_pub = rospy.Publisher('visualization_pose', Marker, queue_size=10)
        self.actual_state_pub = rospy.Publisher('ANTSMC_Pose', Pose2D, queue_size=10)

        self.visual_state_pose = None
        self.visual_state_trajectory = Marker()
        self.actual_pose = None
        self.vel_msg = None

        self.path = Path_()

        self.car_state.x = initial_x
        self.car_state.y = initial_y
        self.car_state.yaw = initial_yaw
        self.car_state.v = initial_v
        self.car_state.kesi = initial_kesi

    def path_callback(self, msg):
        waypoints = []
        for i in range(len(msg.poses)):
            wp = waypoint()
            wp.ID = msg.poses[i].header.seq
            wp.x = msg.poses[i].pose.position.x
            wp.y = msg.poses[i].pose.position.y

            orientation = msg.poses[i].pose.orientation
            quat = quaternion_from_euler(orientation.x, orientation.y, orientation.z, orientation.w)

            yaw = euler_from_quaternion(quat)[2]
            waypoints.append(wp)

        self.last_ind = self.path.path_size() - 1
        self.last_waypoint = self.path.get_waypoint(self.last_ind)

    def marker_set(self):
        self.visual_state_trajectory.header.frame_id = "map"
        self.visual_state_trajectory.header.stamp = rospy.Time.now()
        self.visual_state_trajectory.action = Marker.ADD()
        self.visual_state_trajectory.ns = "ANTSMC"

        self.visual_state_trajectory.id = 0
        self.visual_state_trajectory.type = Marker.POINTS()
        self.visual_state_trajectory.scale.x = 0.02
        self.visual_state_trajectory.scale.y = 0.02
        self.visual_state_trajectory.color.r = 1.0
        self.visual_state_trajectory.color.a = 1.0

    def node_ctrl(self, freq):
        rate = rospy.Rate(freq)
        self.marker_set()

        # Set tf transformation
        br = tf.TransformBroadcaster()
        transform = tf.Transform()

        while not rospy.is_shutdown():
            pass


if __name__ == '__main__':
    rospy.init_node('antsmc_track_node')
    ctrl_freq = 20
    L = 0.2
    initial_x = 0.0
    initila_y = 0.0
    initial_yaw = 0.0
    initial_v = 0
    initial_kesi = 0.1
    node = ANTSMC_node(initial_x=initial_x,
                       initial_y=initila_y,
                       initial_yaw=initial_yaw,
                       initial_v=initial_v,
                       initial_kesi=initial_kesi)

    node.node_ctrl()
