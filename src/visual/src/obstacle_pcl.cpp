#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
using namespace std;

struct Point2D {
    float x, y;
};

bool pointCompare(const Point2D &p1, const Point2D &p2) {
    return p1.y < p2.y;
}

void fillPolygon(const std::vector<Point2D> &vertices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution = 0.01, float height = 0.5) {
    if (vertices.size() < 3) return;

    auto [minIt, maxIt] = std::minmax_element(vertices.begin(), vertices.end(), pointCompare);
    float minY = minIt->y;
    float maxY = maxIt->y;

    for (float y = minY; y <= maxY; y += resolution) {
        std::vector<float> intersections;
        for (size_t i = 0; i < vertices.size(); ++i) {
            size_t j = (i + 1) % vertices.size();
            float y1 = vertices[i].y, y2 = vertices[j].y;
            float x1 = vertices[i].x, x2 = vertices[j].x;

            if ((y1 < y && y2 >= y) || (y2 < y && y1 >= y)) {
                float x = x1 + (y - y1) / (y2 - y1) * (x2 - x1);
                intersections.push_back(x);
            }
        }

        std::sort(intersections.begin(), intersections.end());

        for (size_t k = 0; k < intersections.size(); k += 2) {
            if (k + 1 >= intersections.size()) break;
            float xStart = intersections[k];
            float xEnd = intersections[k + 1];

            for (float x = xStart; x <= xEnd; x += resolution) {
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                for (float z = 0.0; z <= height; z+= resolution){
                    point.z = z;
                    cloud->points.push_back(point);
                }
            }
        }
    }

    cloud->header.frame_id = "map";
    cloud->width = cloud->points.size();
    cloud->height = 1;
}

int main (int argc, char** argv){

    ros::init(argc,argv,"obstacle_pcl_vis");
    ros::NodeHandle nh;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("ob_pcl_vis",1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // construct cloud point data
    float height = 0.4;
    float resolution = 0.05;
    std::vector<Point2D> empty;
    std::vector<Point2D> vertices;

    vertices = {
        {1.0, 1.0}, {2.0, 1.0}, {2.0,2.0}, {1.0,2.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);
    
    // generate a new obstacle point cloud 
    vertices.swap(empty);
    vertices = {
        {3.0, 1.0}, {4.0, 1.0}, {4.0,3.0}, {3.0,3.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);
    
    vertices.swap(empty);
    vertices = {
        {1.0, 4.0}, {3.0, 4.0}, {3.0,5.0}, {1.0,5.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {4.0, 4.0}, {5.0, 4.0}, {5.0,8.0}, {4.0,8.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {1.0, 8.0}, {2.0, 8.0}, {2.0,9.0}, {1.0,9.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {6.0, 3.0}, {8.0, 3.0}, {8.0,5.0}, {6.0,5.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {6.0, 1.0}, {7.0, 1.0}, {7.0,2.0}, {6.0,2.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {7.0, 8.0}, {8.0, 8.0}, {8.0,9.0}, {7.0,9.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {7.0, 6.0}, {9.0, 6.0}, {9.0,7.0}, {7.0,7.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {0.0, 6.0}, {1.0, 6.0}, {1.0,7.0}, {0.0,7.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {2.0, 6.0}, {3.0, 6.0}, {3.0,7.0}, {2.0,7.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    vertices.swap(empty);
    vertices = {
        {4.0, 2.0}, {5.0, 2.0}, {5.0, 3.0}, {4.0, 3.0}
    };
    fillPolygon(vertices, obstacle_ptr, resolution, height);

    // convert to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*obstacle_ptr,output);

    ros::Rate rate(1);
    while(ros::ok()){
        pcl_pub.publish(output);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}