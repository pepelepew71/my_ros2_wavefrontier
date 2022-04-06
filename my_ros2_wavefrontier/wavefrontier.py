#! /usr/bin/env python3
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum
import time
from typing import List

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from . import utils
from . import frontier


class Costmap2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 253
        LethalObstacle = 254
        NoInformation = 255

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.metadata.size_x, self.map.metadata.size_y)

    def getSizeX(self):
        return self.map.metadata.size_x

    def getSizeY(self):
        return self.map.metadata.size_y

    def __getIndex(self, mx, my):
        return my * self.map.metadata.size_x + mx


class WaypointFollower(Node):

    def __init__(self):
        super().__init__(node_name='wave_frontier_follower', namespace='')
        self.waypoints = None
        self.readyToMove = True
        self.currentPose = None
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.lastWaypoint = None
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.pub_markers_frontiers = self.create_publisher(MarkerArray, 'markers_frontiers', 1)
        self.pub_markers_queue_map = self.create_publisher(MarkerArray, 'queue_map', 1)
        self.pub_markers_queue_frontier = self.create_publisher(MarkerArray, 'queue_frontier', 1)
        self.pub_markers_demo = self.create_publisher(MarkerArray, 'demo', 1)

        self.costmapClient = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.costmapClient.wait_for_service(timeout_sec=1.0):
            self.info_msg('service not available, waiting again...')

        self.initial_pose_received = False
        self.goal_handle = None
        self.model_pose_sub = self.create_subscription(Odometry, '/odom', self.poseCallback, 1)
        self.costmapSub = self.create_subscription(OccupancyGrid(), '/map', self.occupancyGridCallback, 1)
        self.costmap = None

        self.get_logger().info('Running Waypoint Test')

    def occupancyGridCallback(self, msg):
        self.costmap = frontier.OccupancyGrid2d(msg)

    def _clear_rviz_markers(self):
        msg = utils.Rviz.get_msg_markers_delete_all()
        self.pub_markers_frontiers.publish(msg)
        self.pub_markers_queue_map.publish(msg)
        self.pub_markers_queue_frontier.publish(msg)

    def moveToFrontiers(self):
        self._clear_rviz_markers()

        frontiers = frontier.getFrontier(master=self, is_pub_queue=False)

        self._clear_rviz_markers()

        msg_markers_frontiers = utils.Rviz.get_msg_markers_frontiers(frontiers=frontiers)
        self.pub_markers_frontiers.publish(msg_markers_frontiers)

        if len(frontiers) == 0:
            self.info_msg('No More Frontiers')
            return

        count = [len(points) for points in frontiers]
        index = count.index(max(count))
        location = None
        location = [utils.get_centroid(frontiers[index])]

        self.info_msg(f'World points {location}')
        self.setWaypoints(location)

        action_request = FollowWaypoints.Goal()
        action_request.poses = self.waypoints

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return None

        self.info_msg('Goal accepted')

        get_result_future = self.goal_handle.get_result_async()

        self.info_msg("Waiting for 'FollowWaypoints' action to complete")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            result = get_result_future.result().result
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg('Goal failed with status code: {0}'.format(status))
            # return False

        if len(result.missed_waypoints) > 0:
            self.info_msg('Goal failed to process all waypoints, missed {0} wps.'.format(len(result.missed_waypoints)))
            # return False

        self.moveToFrontiers()

    def costmapCallback(self, msg):
        self.costmap = Costmap2d(msg)

        unknowns = 0
        for x in range(0, self.costmap.getSizeX()):
            for y in range(0, self.costmap.getSizeY()):
                if self.costmap.getCost(x, y) == 255:
                    unknowns = unknowns + 1
        self.get_logger().info(f'Unknowns {unknowns}')
        self.get_logger().info(f'Got Costmap {len(frontier.getFrontier(None, self.costmap, self.get_logger()))}')

    def dumpCostmap(self):
        costmapReq = GetCostmap.Request()
        self.get_logger().info('Requesting Costmap')
        costmap = self.costmapClient.call(costmapReq)
        self.get_logger().info(f'costmap resolution {costmap.specs.resolution}')

    def setInitialPose(self, pose):
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.currentPose = self.init_pose.pose.pose
        self.publishInitialPose()
        time.sleep(1)

    def poseCallback(self, msg):
        self.currentPose = msg.pose.pose
        self.initial_pose_received = True

    def setWaypoints(self, waypoints):
        self.waypoints = list()
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def run(self, block):
        if not self.waypoints:
            rclpy.error_msg('Did not set valid waypoints before running test!')
            return False

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'FollowWaypoints' action server not available, waiting...")

        action_request = FollowWaypoints.Goal()
        action_request.poses = self.waypoints

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        if not block:
            return True

        get_result_future = self.goal_handle.get_result_async()

        self.info_msg("Waiting for 'FollowWaypoints' action to complete")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            result = get_result_future.result().result
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg('Goal failed with status code: {0}'.format(status))
            return False
        if len(result.missed_waypoints) > 0:
            self.info_msg('Goal failed to process all waypoints,'
                          ' missed {0} wps.'.format(len(result.missed_waypoints)))
            return False

        self.info_msg('Goal succeeded!')
        return True

    def publishInitialPose(self):
        self.initial_pose_pub.publish(self.init_pose)

    def shutdown(self):
        self.info_msg('Shutting down')

        self.action_client.destroy()
        self.info_msg('Destroyed FollowWaypoints action client')

        transition_service = 'lifecycle_manager_navigation/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:
            self.error_msg('%s service call failed %r' % (transition_service, e,))

        self.info_msg('{} finished'.format(transition_service))

        transition_service = 'lifecycle_manager_localization/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:
            self.error_msg('%s service call failed %r' % (transition_service, e,))

        self.info_msg('{} finished'.format(transition_service))

    def cancel_goal(self):
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


def main():
    rclpy.init()

    starting_pose: List[float, float] = [0.0, 0.0]
    wf = WaypointFollower()

    retry_count: int = 0
    retries: int = 2
    while not wf.initial_pose_received and retry_count <= retries:
        retry_count += 1
        wf.info_msg('Setting initial pose')
        wf.setInitialPose(starting_pose)
        wf.info_msg('Waiting for amcl_pose to be received')
        rclpy.spin_once(wf, timeout_sec=1.0)  # wait for poseCallback

    while wf.costmap == None:
        wf.info_msg('Getting initial map')
        rclpy.spin_once(wf, timeout_sec=1.0)

    wf.moveToFrontiers()

    rclpy.spin(wf)


if __name__ == '__main__':
    main()
