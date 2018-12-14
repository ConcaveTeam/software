import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from sensor_msgs.point_cloud2 import read_points


class FindTarget:
    def __init__(self):
        self.targetSub = rospy.Subscriber(
            '/target', PointStamped, self.targetCb)
        self.pointsSub = rospy.Subscriber('/stereo/points2')
        self.pointPub = rospy.Publisher(
            '/target3d', PointStamped, queue_size=1)
        self.target = np.array([])

    def targetCb(self, data):
        self.target = np.array([data.point.x, data.point.y, data.point.z])

    def pointsSub(self, data):
        points = np.array(read_points(data))
        print(points)


if __name__ == '__main__':
    rospy.init_node('find_ta')
    findTarget = FindTarget()
    rospy.spin()
