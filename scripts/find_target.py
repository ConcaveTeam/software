#!/usr/bin/env python
import rospy
import numpy as np
from concaveteam.msg import Spherical
from geometry_msgs.msg import PointStamped
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.msg import PointCloud2


class FindTarget:
    def __init__(self):
        self.targetSub = rospy.Subscriber(
            '/point2d_to_spherical/aim', Spherical, self.targetCb)
        self.pointsSub = rospy.Subscriber(
            '/stereo/points2', PointCloud2, self.pointsCb)
        self.pointPub = rospy.Publisher(
            '/target3d', PointStamped, queue_size=1)
        self.u = np.array([])

    def targetCb(self, data):
        azim = data.azimuth * np.pi / 180
        polar = data.polar * np.pi / 180

        # Compute unit vector for direction of target
        self.u = np.array(
            [np.sin(polar) * np.cos(azim),
             np.sin(polar) * np.sin(azim),
             np.cos(polar)])

    def pointsCb(self, data):
        if self.u.size == 0:
            return

        points = np.array(list(read_points(data, skip_nans=True)))[:, :3]

        # Orient points to z up, x right, and y forward
        swapTmp = np.copy(points[:, 2])
        points[:, 2] = points[:, 1]
        points[:, 1] = swapTmp
        points[:, 2] = -points[:, 2]

        # Project the points onto the ray
        proj = self.u * np.dot(points, self.u)[:, np.newaxis]

        # Calculate the perpendicular component
        perp = points - proj
        dists = np.linalg.norm(perp, 1)

        # Publish the closest point
        closest = points[dists.argmin()]
        # For some reason y is inverted
        point = PointStamped()
        point.header.frame_id = "left_cam"
        point.point.x, point.point.y, point.point.z = closest
        self.pointPub.publish(point)


if __name__ == '__main__':
    rospy.init_node('find_target', anonymous=True)
    findTarget = FindTarget()
    rospy.spin()
