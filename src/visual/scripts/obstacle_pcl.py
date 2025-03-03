################################################################
# 
# Author: Hauser Dong
# From Peking university
# Last update: 2025.03.03
# 
###############################################################

import rospy
import pcl
import sys
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '../../local_planner/planner/scripts'))
import SET
from geometry import *

class Point2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def point_compare(p1, p2):
    return p1.y < p2.y


def fill_polygon(vertices, cloud, resolution=0.01, height=0.5):
    if len(vertices) < 3:
        return

    min_y = min(vertices, key=lambda p: p.y).y
    max_y = max(vertices, key=lambda p: p.y).y

    for y in np.arange(min_y, max_y + resolution, resolution):
        intersections = []
        for i in range(len(vertices)):
            j = (i + 1) % len(vertices)
            x1, y1 = vertices[i].x, vertices[i].y
            x2, y2 = vertices[j].x, vertices[j].y

            if (y1 < y and y2 >= y) or (y2 < y and y1 >= y):
                x = x1 + (y - y1) / (y2 - y1) * (x2 - x1)
                intersections.append(x)

        intersections.sort()

        for k in range(0, len(intersections), 2):
            if k + 1 >= len(intersections):
                break
            x_start = intersections[k]
            x_end = intersections[k + 1]

            for x in np.arange(x_start, x_end + resolution, resolution):
                for z in np.arange(0.0, height + resolution, resolution):
                    cloud.append([x, y, z])


def pcl_to_ros(cloud):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    
    return pc2.create_cloud(header, fields, cloud)


if __name__ == '__main__':
    rospy.init_node('obstacle_pcl_vis')

    pcl_pub = rospy.Publisher('ob_pcl_vis', PointCloud2, queue_size=1)
    pcl_pub_inflated = rospy.Publisher('ob_pcl_vis_inflated', PointCloud2, queue_size=1)
    
    resolution = 0.05
    height = 0.4
    
    obstacle_cloud = []
    obstacles_cloud_inflated = []

    obs_env_idx = rospy.get_param('obs_env_idx', default=4)
    obstacles = SET.ini_obstacle_list[obs_env_idx]

    obstacles_inflated, _ = Build_ExtensionZone(obstacles , 0.2)

    # Add polygons and convert to ROS message
    polygons = [[Point2D(vertex[0], vertex[1]) for vertex in ob.vertex_list] for ob in obstacles]
    polygons_inflated = [[Point2D(vertex[0], vertex[1]) for vertex in ob.vertex_list] for ob in obstacles_inflated]

    for polygon in polygons:
        fill_polygon(polygon, obstacle_cloud, resolution, height)
        
    for polygon in polygons_inflated:
        fill_polygon(polygon, obstacles_cloud_inflated, resolution, height)

    # Convert PCL cloud to ROS message
    output = pcl_to_ros(obstacle_cloud)
    output_inflated = pcl_to_ros(obstacles_cloud_inflated)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pcl_pub.publish(output)
        pcl_pub_inflated.publish(output_inflated)
        rate.sleep()
