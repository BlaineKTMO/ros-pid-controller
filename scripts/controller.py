#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

HZ = rospy.get_param("/local_controller/turtlebot_pid_controller/HZ")
PATH_FRAME = rospy.get_param("/local_controller/turtlebot_pid_controller/PATH_FRAME")
NODE_NAME = rospy.get_param("/local_controller/turtlebot_pid_controller/NODE_NAME")
P_GAIN = rospy.get_param("/local_controller/turtlebot_pid_controller/P_GAIN")
D_GAIN = rospy.get_param("/local_controller/turtlebot_pid_controller/D_GAIN")
I_GAIN = rospy.get_param("/local_controller/turtlebot_pid_controller/I_GAIN")
PATH_TOPIC = rospy.get_param("/local_controller/turtlebot_pid_controller/PATH_TOPIC")
ODOM_TOPIC = rospy.get_param("/local_controller/turtlebot_pid_controller/ODOM_TOPIC")
CMD_VEL_TOPIC = rospy.get_param("/local_controller/turtlebot_pid_controller/CMD_VEL_TOPIC")
SPEED = rospy.get_param("/local_controller/turtlebot_pid_controller/SPEED")

DT = 1/HZ


class Controller:
    """
    PID Controller

    Attributes:
        posUpdated: odom flag
        pGain: Proportion parameter
        dGain: Derivative parameter
        iGain: Integral Parameter
        prevError: Error at previous timestep
        path: Path to follow
        drivePub: Command velocity publisher
        rate: ROS frequency controller
        tfBuffer: Message container
        tfListener: Transform subscriber
        pose: Current position
        error: Current error term
    """
    def __init__(self, pGain, dGain, iGain, path):
        self.rate = rospy.Rate(HZ)

        self.pGain = pGain
        self.dGain = dGain
        self.iGain = iGain

        self.prevError = 0

        self.path = path

        rospy.Subscriber(PATH_TOPIC, Path, self.pathCallback)
        rospy.Subscriber(ODOM_TOPIC, Odometry, self.posCallback)
        self.posUpdated = False

        self.drivePub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def posCallback(self, data: Odometry):
        self.pose = data.pose.pose

        self.posUpdated = True

    def pathCallback(self, data: Path):
        self.path = data
        self.pathUpdated = True

    def controllerLoop(self):
        """
        Main controller loop
        """
        self.rate.sleep()

        # Guard
        if self.posUpdated is not True:
            rospy.logwarn("Odometry information not updated . . .")
            return

        # Output = (P * e) + (D * de) + (I * ∫e)
        steerAngle = self.calcProportionalTerm()
        steerAngle += self.calcDerivativeTerm()
        steerAngle += self.calcIntegralTerm()
        print(f'Steering angle: {steerAngle:.3f}')

        # Save error value
        self.prevError = self.error

        # Create and publish message
        msg = Twist()
        msg.linear.x = SPEED
        msg.angular.z = steerAngle
        self.drivePub.publish(msg)

        # Reset flags
        self.posUpdated = False

    def calcDistance(self, start, end):
        """

        Args:
            start (Pose): Beginning position
            end (Pose): Ending position

        Returns: Distance between two poses

        """
        return math.sqrt((end.position.x - start.position.x) ** 2 +
                         (end.position.y - start.position.y) ** 2)

    def getShortestDist(self):
        """
        Returns: Shortest distance to path
        """
        minDist = 1e9
        for pose in self.path.poses:
            dist = self.calcDistance(pose.pose, self.pose)

            transform = self.tfBuffer.lookup_transform("base_footprint",
                                                       pose.header,
                                                       rospy.Time(0),
                                                       rospy.Duration(1.0))
            poseTransformed = tf2_geometry_msgs.do_transform_pose(pose,
                                                                  transform)
            
            # Determine if left or right of path
            if poseTransformed.pose.position.y < 0:
                dist = -dist

            if abs(dist) < abs(minDist):
                minDist = dist
                print(poseTransformed)

        return minDist

    def calcProportionalTerm(self):
        self.error = self.getShortestDist()
        return self.pGain * self.error

    def calcDerivativeTerm(self):
        dTerm = self.error - self.prevError
        dTerm /= DT

        return dTerm * self.dGain

    def calcIntegralTerm(self):
        return self.error * DT * self.iGain


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)

    path = Path()
    val = 0
    for i in range(100):
        val = val + 0.25
        pos = PoseStamped()
        pos.header = PATH_FRAME
        pos.pose.position.x = val
        pos.pose.position.y = val

        path.poses.append(pos)

    for pos in path.poses:
        print(pos)

    # controller = Controller(0.3, 0.35, 0.1, path)
    controller = Controller(P_GAIN, D_GAIN, I_GAIN, path)

    while not rospy.is_shutdown():
        controller.controllerLoop()
