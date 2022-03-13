from random import random
import numpy as np

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def get_centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length


class Rviz:

    @staticmethod
    def get_msg_markers_frontiers(frontiers):
        msg = MarkerArray()
        count = 0
        for points in frontiers:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = count
            count += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            mx, my = get_centroid(points)
            marker.pose.position.x = mx
            marker.pose.position.y = my
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            msg.markers.append(marker)

            color = random(), random(), random()
            for point in points:
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.id = count
                count += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.025
                marker.scale.y = 0.025
                marker.scale.z = 0.025
                marker.color.a = 1.0
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                msg.markers.append(marker)

        return msg

    @staticmethod
    def get_msg_markers_delete_all():
        msg = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = 0
        marker.action = Marker.DELETEALL
        msg.markers.append(marker)
        return msg
