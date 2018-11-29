#!/usr/bin/env python
import rospy
import numpy as np
from concaveteam.msg import Spherical
from geometry_msgs.msg import Point
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.msg import PointCloud2


class FindTarget:
    def __init__(self):
        self.targetSub = rospy.Subscriber(
            '/point2d_to_spherical/aim', Spherical, self.targetCb)
        self.pointsSub = rospy.Subscriber(
            '/stereo/points2', PointCloud2, self.pointsCb)
        self.pointPub = rospy.Publisher(
            '/target3d', Point, queue_size=1)
        self.u = np.array([])

    def targetCb(self, data):
        azim = data.azimuth
        polar = data.polar

        # Compute unit vector for direction of target
        self.u = np.array(
            [np.sin(polar) * np.cos(azim),
             np.sin(polar) * np.sin(azim),
             np.cos(polar)])

    def pointsCb(self, data):
        points = np.array(list(read_points(data, skip_nans=True)))[:, :3]
        if self.u.size == 0:
            return

        # Project the points onto the ray
        proj = self.u * np.dot(points, self.u)[:, np.newaxis]
        # print(proj.shape)

        # Calculate the perpendicular component
        perp = points - proj
        perp = perp / np.linalg.norm(perp, axis=1)[:, np.newaxis]
        dists = (points * perp).sum(1)

        # Publish the closest point
        closest = points[dists.argmin()]
        # For some reason y is inverted
        closest[:, 1] = -closest[:, 1]
        point = Point()
        point.x, point.y, point.z = closest
        self.pointPub.publish(point)
        print(closest)


if __name__ == '__main__':
    rospy.init_node('find_target', anonymous=True)
    findTarget = FindTarget()
    rospy.spin()
