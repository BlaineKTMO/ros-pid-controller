#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class Controller:
    def __init__(self, pGain, dGain, iGain, path):
        self.posUpdated = False

        self.pGain = pGain
        self.dGain = dGain
        self.iGain = iGain

        self.prevError = 0

        self.path = path

        rospy.Subscriber("/odom", Odometry, self.posCallback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def posCallback(self, data: Odometry):
        self.pose = data.pose.pose 

        self.posUpdated = True

    def controllerLoop(self):
        self.rate.sleep()
        if self.posUpdated is not True:
            return

        steerAngle = self.calcProportionalTerm()
        steerAngle += self.calcDerivativeTerm()
        steerAngle += self.calcIntegralTerm()
        print(steerAngle)

        self.prevError = self.error

        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = steerAngle 

        self.pub.publish(msg)

    def calcDistance(self, start, end):
        return math.sqrt((end.position.x - start.position.x) ** 2 + (end.position.y - start.position.y) ** 2)

    def getShortestDist(self):
        minDist = 1e9
        for pose in self.path.poses:
            dist = self.calcDistance(pose.pose, self.pose)

            transform = self.tfBuffer.lookup_transform("base_footprint", pose.header, rospy.Time(0), rospy.Duration(1.0))
            poseTransformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
            
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
        dTerm /= 0.1
        
        return dTerm * self.dGain 

    def calcIntegralTerm(self):
        return self.error * 0.1 * self.iGain

if __name__ == '__main__':
    rospy.init_node('controller')

    path = Path()
    val = 0
    for i in range(100):
        val = val + 0.25
        pos = PoseStamped()
        pos.header="odom"
        pos.pose.position.x = val
        pos.pose.position.y = val

        path.poses.append(pos)

    for pos in path.poses:
        print(pos)

    controller = Controller(0.3, 0.35, 0.1, path)

    while not rospy.is_shutdown():
        controller.controllerLoop()
