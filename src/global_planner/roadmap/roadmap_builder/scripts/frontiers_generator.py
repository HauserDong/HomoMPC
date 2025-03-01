#！/usr/bin/env python3

import time
import math
import rospy
import random
import numpy as np
from sklearn.cluster import MeanShift
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from roadmap_builder.msg import Frontier, FrontierArray
from roadmap_builder.srv import GetFrontiers, GetFrontiersResponse

class FrontierGenerator:
    def __init__(self):
        """
        Class for frontier generator, based on rospy,
        inlcuding map preprocessing, frontier detection and centroid clustering.
        """
        self.frontier_clusters = dict()

        ## ROS parameters setting
        self.distance_obs = rospy.get_param('~dis_obs', 0.5)
        self.map_max_x = rospy.get_param('~map_max_x')
        self.map_max_y = rospy.get_param('~map_max_y')
        self.map_min_x = rospy.get_param('~map_min_x')
        self.map_min_y = rospy.get_param('~map_min_y')
        # self.robot_id = rospy.get_param('~robot_id', 0)
        # self.robotPrefix = rospy.get_param('~robot_ns', "robot_")

        ## ROS interfaces setting
        # topic = "/"+self.robotPrefix + str(self.robot_id) + '/frontierlist'
        # self.frontiers_pub = rospy.Publisher('/frontiers', FrontierArray, queue_size=1)
        # topic = "/"+self.robotPrefix + str(self.robot_id) + '/frontier_markers'
        self.centroid_markers_pub = rospy.Publisher('/centroid_markers', MarkerArray, queue_size=10)
        # self.pruned_map_pub = rospy.Publisher('/pruned_map', OccupancyGrid, queue_size=10)
        # topic = "/"+self.robotPrefix + str(self.robot_id) + '/move_base/global_costmap/costmap'
        # rospy.Subscriber(topic, OccupancyGrid, self.map_callback)


        while not rospy.is_shutdown():
            service = '/get_frontiers'
            # service = "/"+self.robotPrefix + str(self.robot_id) + '/get_frontiers'
            rospy.Service(service, GetFrontiers, self.get_frontiers_service)
            rospy.loginfo("Server start: /get_frontiers")
            rospy.spin()

    #=================================
    ## Service setting
    #=================================
    def get_frontiers_service(self, req):
        try:
            self.set_map_size_info(req.map)
            self.inflate_map(5)
            self.generate_frontiers()
            frontiers_msg, _ = self.publish_frontiers()

            if frontiers_msg is None:
                return GetFrontiersResponse(None)
            else:
                return GetFrontiersResponse(frontiers_msg.frontiers)

        except Exception as e:
            rospy.logerr("Service error: %s", str(e))


    #=======================================
    ## Frontiers generation and clustering
    #=======================================
    def generate_frontiers(self, method='mean_shift', expand=10, frontier_search=False):
        """
        Generate frontiers according to the occupancy map,
        ----------
        Parameters:
            method: mean_shift or group_mid
        """
        frontiers = set()
        if self.occupancy_map is None:
            rospy.LogWarning('No map info!')
            return None, None
        ##detect frontiers in two ways: first search and incremental search
        rospy.loginfo('Start detect the frontiers!')

        rospy.loginfo('Initial detection.')
        ##extract frontier points from the whole occupancy map
        for raw in self.grid_raws:
            for col in self.grid_cols:
                if self.occupancy_map[raw][col] == 0:  # Unoccupied cell
                    # Check if the cell has unexplored neighbors
                    if self._has_unexplored_neighbors(raw, col):
                        point_x = round(col*self.res+self.map_origin_x, 2)
                        point_y = round(raw*self.res+self.map_origin_y, 2)
                        frontiers.add((point_x, point_y))
        if frontier_search:
            rospy.loginfo('Expand frontiers to continue detection.')
            ##expand frontiers from the past frontiers
            for cluster in self.frontier_clusters.values():
                col_min = int((np.min(cluster['frontiers'][:, 0])-self.map_origin_x)/self.res)
                col_max = int((np.max(cluster['frontiers'][:, 0])-self.map_origin_x)/self.res)
                raw_min = int((np.min(cluster['frontiers'][:, 1])-self.map_origin_y)/self.res)
                raw_max = int((np.max(cluster['frontiers'][:, 1])-self.map_origin_y)/self.res)
                for raw in range(raw_min-expand, raw_max+expand+1):
                    for col in range(col_min-expand, col_max+expand+1):
                        if self.occupancy_map[raw][col] == 0:  # Unoccupied cell
                            # Check if the cell has unexplored neighbors
                            if self._has_unexplored_neighbors(raw, col):
                                point_x = round(col*self.res+self.map_origin_x, 2)
                                point_y = round(raw*self.res+self.map_origin_y, 2)
                                frontiers.add((point_x, point_y))
        ##cluster the set of frontiers
        if len(frontiers) > 0:
            rospy.loginfo(f'Get frontiers, total number {len(frontiers)}!')
            if method == 'mean_shift':
                self.frontier_clusters = self._mean_shift(frontiers)
                return self.frontier_clusters
            else:
                pass
        else:
            rospy.logwarn('There are no frontiers!')
            self.frontier_clusters = dict()
            return None

    def set_map_size_info(self, map_msg):
        """
        Get size of occupancy-grid map according to the input map and original axises.
        """
        self.res = map_msg.info.resolution
        self.occupancy_map = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        
        self.grid_max_raw = map_msg.info.height
        self.grid_max_col = map_msg.info.width
        self.map_origin_x = map_msg.info.origin.position.x
        self.map_origin_y = map_msg.info.origin.position.y

        self.grid_ori_col = int((0-self.map_origin_x)/self.res)
        self.grid_ori_raw = int((0-self.map_origin_y)/self.res)

        self.grid_cols = range(0, self.grid_max_col)
        self.grid_raws = range(0, self.grid_max_raw)
        self.safe_grid_dis = int(self.distance_obs/self.res)

    def inflate_map(self, radius):
        if self.occupancy_map is None:
            return
        temp = np.array(self.occupancy_map, copy=True)
        for i in self.grid_raws:
            for j in self.grid_cols:
                if temp[i][j] == 100:
                    # inflate
                    for di in range(-radius, radius):
                        for dj in range(-radius, radius):
                            _i = i + di
                            _j = j + dj
                            if _i>=0 and _i<self.grid_max_raw \
                                    and _j>=0 and _j<self.grid_max_col:
                                if self.occupancy_map[_i][_j] != -1 and self.occupancy_map[_i][_j]!= 100:
                                    self.occupancy_map[_i][_j]=100

    def _has_unexplored_neighbors(self, row, col, grids=4):
        # Check for unexplored neighbors
        if grids == 4:
            neighbor_shift = [(0,1), (1,0), (-1,0), (0,-1)]
        else:
            neighbor_shift = [(1, 1), (0,1), (-1,1), (1,0), (-1,0), (1,-1), (0,-1), (-1,-1)]
        for c, r in neighbor_shift:
            if (row+r in self.grid_raws and col+c in self.grid_cols and
                self.occupancy_map[row+r][col+c] == -1):
                return True
        return False

    def _mean_shift(self, frontiers, bandwidth=2.0, update=True):
        """
        Use Mean Shift clustering to filter and group frontier points.
        """
        frontier_clusters = dict()
        ##transfer list to numpy array
        frontiers = np.array(list(frontiers))
        ##initialize Mean Shift and execute clustering
        ms = MeanShift(bandwidth=bandwidth)
        ms.fit_predict(frontiers)
        labels = ms.labels_
        ##process each cluster
        for label in np.unique(labels):
            if label == -1:
                continue
            cluster_indices = np.where(labels == label)[0]
            cluster_frontiers = frontiers[cluster_indices]
            ##calculate the centroid for the cluster of frontiers
            centroid = np.mean(cluster_frontiers, axis=0)
            if self._close_to_obstacles(centroid):
                centroid = min(cluster_frontiers, key=lambda p:math.dist(p, centroid))
            ##add new centroid and frontiers to frontier_clusters
            frontier_clusters[label] = {
                'centroid': centroid,
                'frontiers': cluster_frontiers,
            }
            
        return frontier_clusters

    def _close_to_obstacles(self, centroid):
        col = int(centroid[0]/self.res)
        row = int(centroid[1]/self.res)
        safe_grid_range = np.arange(-self.safe_grid_dis, self.safe_grid_dis+1)
        for r in safe_grid_range:
            for c in safe_grid_range:
                if row+r in self.grid_raws and col+c in self.grid_cols:
                    if self.occupancy_map[row+r][col+c] == 1:
                        return True
        return False

    #=================================
    ## message preparing and publish
    #=================================
    def publish_frontiers(self):
        frontiers_msg = FrontierArray()
        centroid_markers_msg = MarkerArray()
        ## prepare the message type
        for id, cluster in self.frontier_clusters.items():
            frontiers = cluster['frontiers']
            centroid = cluster['centroid']
            # Set the frontiers
            frontiers_msg.frontiers.append(self._set_frontier_msg(frontiers, centroid))
            # Set the frontier markers
            centroid_markers_msg.markers.append(self._set_frontier_marker_msg(id, centroid))

        # self.frontiers_pub.publish(frontiers_msg)
        self.centroid_markers_pub.publish(centroid_markers_msg)

        return frontiers_msg, centroid_markers_msg

    def _set_frontier_msg(self, frontiers, centroid):
        temp_frontier = Frontier()
        temp_frontier.size = len(frontiers)
        temp_frontier.centroid.x = centroid[0]
        temp_frontier.centroid.y = centroid[1]
        temp_frontier.centroid.z = 0.0
        # points_list = []
        for frontier in frontiers:
            point = Point()
            point.x = frontier[0]
            point.y = frontier[1]
            point.z = 0.0
            temp_frontier.points.append(point)
        return temp_frontier

    def _set_frontier_marker_msg(self, id, centroid):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(10.0)
        return marker


if __name__ == '__main__':
    rospy.init_node('frontier_generator')
    frontier_generator = FrontierGenerator()
