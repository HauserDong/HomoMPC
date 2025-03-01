/**
 * Modified by: Hauser Dong
 * From Peking university
 * Last update: 2024.10.23
 * Brief Intro: This file is about homotopic path planning, 
 *              partly modified from https://github.com/HuangJingGitHub/HPSP
 *              and https://github.com/SimoneTinella/Path-Planning-Robot-over-Voronoi-Diagram.
*/

#include <random>
#include <chrono>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz/types.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include "../include/obstacles.hpp"
#include "../include/AStar.hpp"
#include "../include/Voronoi.hpp"
#include <roadmap_builder/GenVoronoi.h>
#include <tuw_multi_robot_msgs/Graph.h>

using namespace cv;
using namespace std;
namespace py = pybind11;

void DrawPath(Mat img, const vector<Point2f>& path, 
            Scalar color = Scalar(255, 0, 0), 
            int thickness = 2,
            Point2f shift = Point2f(0, 0)) {
    for (int i = 0; i < path.size() - 1; i++)
        line(img, convert_opencv_point(path[i] + shift), convert_opencv_point(path[i + 1] + shift), color, thickness);
}

void DrawPath(Mat img, const vector<PosNode*>& path, 
            Scalar color = Scalar(255, 0, 0), 
            int thickness = 2,
            Point2f shift = Point2f(0, 0)) {
    for (int i = 0; i < path.size() - 1; i++)
        line(img, convert_opencv_point(path[i]->pos + shift), convert_opencv_point(path[i + 1]->pos + shift), color, thickness);
}

void CalculatePassageMidPts(const vector<vector<Point2f>> & passage_pairs, vector<pair<int, Point2f>>& passage_mid_pts) {
    for (int i = 0; i< passage_pairs.size(); i++) {
        Point2f mid_pt = (passage_pairs[i][0] + passage_pairs[i][1]) / 2;
        passage_mid_pts.push_back(make_pair(i, mid_pt));
    }
}

void CalculateObsCentroids(const vector<PolygonObstacle>& obs_vec, vector<pair<int, Point2f>>& obs_centroids) {
    for (int i = 0; i < obs_vec.size(); i++) {
        Point2f centroid(0, 0);
        for (const auto& vertex : obs_vec[i].vertices)
            centroid += vertex;
        centroid = centroid / static_cast<float>(obs_vec[i].vertices.size());
        obs_centroids.push_back(make_pair(i, centroid));
    }
}

vector<vector<pair<float,float>>> convert_obstacles(py::list obstacles) {
    vector<vector<pair<float,float>>> obstacles_cpp;

    for (auto obstacle : obstacles){
        vector<pair<float,float>> obstacle_cpp;
        for (auto vertex : obstacle.cast<py::list>()){
            py::array_t<float> vertex_np = vertex.cast<py::array_t<float>>();
            auto v = vertex_np.unchecked<1>();

            if (v.shape(0)!=2){
                throw runtime_error("Each vertex should have 2 coordinates.");
            }else{
                obstacle_cpp.emplace_back(v(0), v(1));
            }
        }
        obstacles_cpp.push_back(obstacle_cpp);
    }
    return obstacles_cpp;
}

vector<PolygonObstacle> process_obstacles(const vector<vector<pair<float,float>>>& obstacles, const float& resolution, const pair<int,int>& config_size) {
    vector<PolygonObstacle> PolygonObstacles_vec(4);

    // By default, add environment 4 walls as obstacles
    vector<Point2f> top_obs_vertices{Point2f(0, 0), Point2f(config_size.first, 0), Point2f(10, -10)},
                    bottom_obs_vertices{Point2f(0, config_size.second), Point2f(config_size.first, config_size.second), 
                               Point2f(10, config_size.second + 10)},
                    left_obs_vertices{Point2f(0, 0), Point2f(0, config_size.second), Point2f(-10, 10)},
                    right_obs_vertices{Point2f(config_size.first, 0), Point2f(config_size.first, config_size.second), 
                              Point2f(config_size.first + 10, 10)};
    PolygonObstacle top_obs(top_obs_vertices), bottom_obs(bottom_obs_vertices), 
                    left_obs(left_obs_vertices), right_obs(right_obs_vertices);
    PolygonObstacles_vec[0] = top_obs;
    PolygonObstacles_vec[1] = bottom_obs;
    PolygonObstacles_vec[2] = left_obs;
    PolygonObstacles_vec[3] = right_obs;

    for (auto obstacle : obstacles){
        PolygonObstacle cur_obs;
        for (auto vertex : obstacle){
            cur_obs.vertices.push_back(Point2f(vertex.first/resolution, vertex.second/resolution));
        }
        PolygonObstacles_vec.push_back(cur_obs);
    }

    return PolygonObstacles_vec;
}

Point2f generate_random_point(float x_min, float x_max, float y_min, float y_max) {
    random_device rd;
    default_random_engine eng(rd());
    uniform_real_distribution<float> x_rand(x_min, x_max);
    uniform_real_distribution<float> y_rand(y_min, y_max);

    float x = x_rand(eng);
    float y = y_rand(eng);
    return Point2f(x, y);
}

void convert_path_format(const vector<PosNode*>& path, vector<vector<float>>& path_to_return, vector<float>& path_vel, float resolution){
    for (int i = 0; i < path.size(); i++){
        PosNode* node = path[i];
        path_to_return.push_back(vector<float>{node->pos.x * resolution, node->pos.y * resolution});

        path_vel[i] = path_vel[i] * resolution;
    }
}

int generate_random_number(int min, int max) {

    std::random_device rd;  
    std::mt19937 eng(rd()); 
    std::uniform_int_distribution<> distr(min, max); 

    return distr(eng);
}

vector<vector<vector<vector<float>>>> convert_other_path_set(py::list other_path_set, float resolution){
    vector<vector<vector<vector<float>>>> other_path_set_cpp;

    if (other_path_set.empty()){
        cout << "No other path set provided." << endl;
        return {};
    }
    
    for (auto agent : other_path_set){
        vector<vector<vector<float>>> agent_path_set;
        for (auto path : agent.cast<py::list>()){
            vector<vector<float>> agent_path;
            for (auto node : path.cast<py::list>()){
                vector<float> agent_path_node;
                for (auto coord : node.cast<py::list>()){
                    agent_path_node.push_back(coord.cast<float>()/resolution);
                }
                agent_path.push_back(agent_path_node);
            }
            agent_path_set.push_back(agent_path);
        }
        other_path_set_cpp.push_back(agent_path_set);
    }

    return other_path_set_cpp;
}

 vector<vector<double>> convert_other_passage_passing_time(py::list other_passage_passing_time){
    vector<vector<double>> other_passage_passing_time_cpp;

    if (other_passage_passing_time.empty()){
        cout << "No other passage passing time provided." << endl;
        return {};
    }
    
    for (auto agent : other_passage_passing_time){
        vector<double> agent_passage_passing_time;
        for (auto passage_time : agent.cast<py::list>()){
            agent_passage_passing_time.push_back(passage_time.cast<double>());
        }
        other_passage_passing_time_cpp.push_back(agent_passage_passing_time);
    }

    return other_passage_passing_time_cpp;
}

bool find_connector(const Point2f& p1, const Point2f& p2, Point2f& connector, vector<PolygonObstacle>& obs_vec_){
    // if conflict, try to find connectors [inspired by RAPTOR]
    int MAX_SAMPLE_NUM = 10;
    Point2f m = (p1 + p2)/2;
    float r = cv::norm(p1 - p2)/2;
    bool connector_found = false;
    bool connector_visible = true;
    float min_dist = 1e6;
    for (int i = 0; i < MAX_SAMPLE_NUM; i++){
        Point2f sample_pt = generate_random_point(m.x - r, m.x + r, m.y - r, m.y + r);
        for (auto ob : obs_vec_){
            if (ObstacleFree(ob, p1, sample_pt)== false || ObstacleFree(ob, p2, sample_pt) == false){
                connector_visible = false;
                break;
            }
        }
        if (connector_visible){
            float dist = cv::norm(sample_pt - p1) + cv::norm(sample_pt - p2);
            if (dist < min_dist){
                min_dist = dist;
                connector = sample_pt;
                connector_found = true;
            }
        }
        connector_visible = true;
    }

    return connector_found;
}


class HomoPathPlanner {
    private:
        int agent_idx_;
        vector<PolygonObstacle> obs_vec_;
        // pair<vector<vector<int>>, vector<vector<Point2f>>> visibility_check_res_;
        pair<vector<vector<int>>, vector<vector<Point2f>>> extended_visibility_check_res_;
        vector<vector<Point2f>> extended_visibility_check_whole_passage_;
        vector<pair<int, Point2f>> passage_mid_pts_;
        kdTree passage_kd_tree_;
        vector<pair<int, Point2f>> obs_centroids_;
        kdTree obs_kd_tree_;
        Mat back_img_;
        float resolution_;
        bool visualize_;
        float Vmax;     // used to evaluate the time to reach each passages
        float alpha_; // used to evaluate the difference of the time to reach each passages
        vector<PosNode*> tmp;
        int test_mode_;
        Voronoi voronoi_;
        vector<PosNode*> voronoi_graph_;

        tuw_multi_robot_msgs::Graph voronoi_map_;
        ros::Publisher map_pub;
        ros::Publisher voronoi_vis_pub;
        ros::ServiceClient voronoi_client;

    public:
        HomoPathPlanner(int agent_idx, py::list obstacles, py::dict map_range, float resolution, int test_mode, float alpha = -0.1, bool visualize = true);

        vector<vector<pair<float, float>>> GetExtendedVisibilityCheck();

        vector<float> CalculatePassageTimeMap(vector<vector<float>> path, float v, vector<float> pptm_old, py::array_t<float> pos_py);

        vector<float> CalculatePathTime(vector<vector<float>> path, float v, vector<float> path_time_old, py::array_t<float> pos_py);
        
        tuple<vector<vector<float>>, vector<float>, vector<float> , vector<float>> generate_homotopic_path(py::array_t<float> start_py, py::array_t<float> end_py, py::list other_passage_passing_time, float V_aver, bool replan);

        nav_msgs::OccupancyGrid convertToOccupancyGrid(int width, int height, const geometry_msgs::Point& origin);

        void restart_voronoi_graph();
        
        ~HomoPathPlanner(){
            for (int i = 0; i < tmp.size(); i++){
                delete tmp[i];
            }

            for (int i = 0; i < voronoi_graph_.size(); i++){
                delete voronoi_graph_[i];
            }
        }
};

visualization_msgs::Marker create_point_marker(const geometry_msgs::Point& point, int id, string ns, float scale, vector<float> color){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.a = 1.0;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];

    return marker;
}

visualization_msgs::Marker create_line_marker(const geometry_msgs::Point& start, const geometry_msgs::Point& end, int id, string ns, float scale, vector<float> color){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = scale;
    marker.color.a = 1.0;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];

    return marker;
}

HomoPathPlanner::HomoPathPlanner(int agent_idx, py::list obstacles, py::dict map_range, float resolution, int test_mode, float alpha, bool visualize): agent_idx_(agent_idx), resolution_(resolution), test_mode_(test_mode), alpha_(alpha), visualize_(visualize){
    /*
    INPUTS:
    obstacles: list of obstacles, each obstacle is a list of vertices
    map_range: dictionary containing the range of the map in x and y axes
    resolution: resolution of the map
    test_mode:
        0: single path planning
        1: homo-path planning (no restriction on passage)
        2: homo-path planning (pass as much passages as possible; do not consider others' results)
        3: homo-path planning (pass as much passages as possible; consider others' results; penalty on other homo-path passing passages)
        4: homo-path planning (pass as much passages as possible; consider others' results; penalty on other homo-path passing passages with time-stamp)
    visualize: whether to visualize the homotopic paths by opencv
    */

   Vmax = 0.6/resolution_;  // how many pixels per second

   // checking map range
    for (auto item : map_range){
        string axis = item.first.cast<string>();
        py::list range_list = item.second.cast<py::list>();
        if (range_list.size() != 2){
            throw runtime_error("Each axis should have 2 bound values.");
        }
        float lower_bound = range_list[0].cast<float>();
        float upper_bound = range_list[1].cast<float>();

        if (axis == "x")
            range_x = static_cast<int>(round((upper_bound - lower_bound) / resolution_));
        else if (axis == "y")
            range_y = static_cast<int>(round((upper_bound - lower_bound) / resolution_));
        else
            throw runtime_error("Invalid axis in map range. Only 'x' and 'y' are allowed.");
    }

    if (range_x == -1 || range_y == -1){
        throw runtime_error("Both x and y range should be provided.");
    }
    
    back_img_ = Mat(Size(range_x, range_y), CV_64FC3, Scalar(255, 255, 255));

    vector<vector<pair<float,float>>> obstalce_cpp = convert_obstacles(obstacles);
    obs_vec_ = process_obstacles(obstalce_cpp, resolution_, make_pair(range_x, range_y));
    for (int i = 4; i < obs_vec_.size(); i++) {
        PolygonObstacle cur_obs = obs_vec_[i];
        int cur_vertex_num = cur_obs.vertices.size();
        for (int j = 0; j < cur_vertex_num; j++)
            if (visualize_){
                line(back_img_, convert_opencv_point(cur_obs.vertices[j]), convert_opencv_point(cur_obs.vertices[(j + 1) % cur_vertex_num]), Scalar(0, 0, 0), 2);
            }
    }

    // Visibility check
    auto visibility_check_start = chrono::system_clock::now();
    // visibility_check_res_ = PureVisibilityPassageCheck(obs_vec_);
    pair<pair<vector<vector<int>>, vector<vector<Point2f>>>, vector<vector<Point2f>>> extended_visibility_check_res_tmp = ExtendedVisibilityPassageCheck(obs_vec_, resolution_);
    extended_visibility_check_res_ = extended_visibility_check_res_tmp.first;
    extended_visibility_check_whole_passage_ = extended_visibility_check_res_tmp.second;
    if (visualize_){
        for (int i = 0; i < extended_visibility_check_res_.first.size(); i++){
            line(back_img_, convert_opencv_point(extended_visibility_check_res_.second[i][0]), convert_opencv_point(extended_visibility_check_res_.second[i][1]), Scalar(0, 0, 0), 1);

            // whole passage visualization
            line(back_img_, convert_opencv_point(extended_visibility_check_whole_passage_[2*i][0]), convert_opencv_point(extended_visibility_check_whole_passage_[2*i][1]), Scalar(0, 0, 1), 1);

            line(back_img_, convert_opencv_point(extended_visibility_check_whole_passage_[2*i+1][0]), convert_opencv_point(extended_visibility_check_whole_passage_[2*i+1][1]), Scalar(0, 0, 1), 1);
        }
    }
    // cout << "Visibility check passage res: " << visibility_check_res_.first.size() 
    cout << "\nExtended visibility check passage res: " << extended_visibility_check_res_.first.size() << '\n';
    auto visibility_check_end = chrono::system_clock::now();
    chrono::duration<double> visibility_check_time = visibility_check_end - visibility_check_start;
    cout << "Visibility check time: " << visibility_check_time.count() << "s\n";

    // Get the passage middle points
    CalculatePassageMidPts(extended_visibility_check_res_.second, passage_mid_pts_);

    // Construct passage middle points kd-tree
    vector<pair<int, Point2f>> passage_mid_pts_copy(passage_mid_pts_);
    buildkdTree(passage_kd_tree_, passage_mid_pts_copy);

    // Get the obstacle centroids
    CalculateObsCentroids(obs_vec_, obs_centroids_);

    // Construct obstacle centroid kd-tree
    buildkdTree(obs_kd_tree_, obs_centroids_);

    auto time_voronoi_start = chrono::system_clock::now();
    // voronoi_ = Voronoi(range_x, range_y, obs_vec_, &back_img_, visualize_);
    // voronoi_graph_ = voronoi_.CreateVoronoiGraph();

    // Convert obs_vec_(vector<PolygonObstacle>) to nav_msgs/OccupancyGrid type
    geometry_msgs::Point origin;
    origin.x = 0.0; origin.y = 0.0;
    nav_msgs::OccupancyGrid map = convertToOccupancyGrid(range_x, range_y, origin);
    
    int argc = 1;
    char* argv[] = {const_cast<char*>("Voronoi_generator")};
    char** argv_ = {argv};
    ros::init(argc, argv_, "Voronoi_generator_agent"+to_string(agent_idx_));
    ros::NodeHandle nh;

    // Visualize the OccupancyGrid map
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid_map_vis"+to_string(agent_idx_), 1);

    // Call the service to generate Voronoi graph
    ros::service::waitForService("/gen_voronoi");
    voronoi_client = nh.serviceClient<roadmap_builder::GenVoronoi>("/gen_voronoi");

    roadmap_builder::GenVoronoi srv;
    srv.request.occupancy_map = map;

    // Visualize the Voronoi graph
    voronoi_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("voronoi_vis"+to_string(agent_idx_), 1);
    visualization_msgs::MarkerArray voronoi_vis;

    if (voronoi_client.call(srv)){
        voronoi_map_ = srv.response.voronoi_map;
        ROS_INFO("Voronoi graph generated successfully.");

        // geometry_msgs::Point voronoi_origin = voronoi_map_.origin.position;
        // int i = 0;
        // int j = 0;
        // for (int k = 0; k < voronoi_map_.vertices.size(); k++){
        //     int kk = 16;
        //     auto segment = voronoi_map_.vertices[kk];
        //     geometry_msgs::Point s;
        //     s.x = round((segment.path[0].x + voronoi_origin.x)*100)/100.0;
        //     s.y = round((segment.path[0].y + voronoi_origin.y)*100)/100.0;

        //     visualization_msgs::Marker marker = create_point_marker(s, i, "vertex", 0.1, {0.0, 0.0, 1.0});
        //     voronoi_vis.markers.push_back(marker);
        //     i++;

        //     geometry_msgs::Point e;
        //     e.x = round((segment.path.back().x + voronoi_origin.x)*100)/100.0;
        //     e.y = round((segment.path.back().y + voronoi_origin.y)*100)/100.0;

        //     visualization_msgs::Marker marker2 = create_point_marker(e, i, "vertex", 0.1, {1.0, 0.0, 0.0});
        //     voronoi_vis.markers.push_back(marker2);
        //     i++;

        //     for(int m = 0; m < segment.path.size()-1; m++){
        //         geometry_msgs::Point p1, p2;
        //         p1.x = round((segment.path[m].x + voronoi_origin.x)*100)/100.0;
        //         p1.y = round((segment.path[m].y + voronoi_origin.y)*100)/100.0;

        //         p2.x = round((segment.path[m+1].x + voronoi_origin.x)*100)/100.0;
        //         p2.y = round((segment.path[m+1].y + voronoi_origin.y)*100)/100.0;

        //         visualization_msgs::Marker marker3 = create_line_marker(p1, p2, j, "edge", 0.02, {0.0, 0.0, 1.0});
        //         voronoi_vis.markers.push_back(marker3);
        //         j++;
        //     }

        //     // print successors
        //     cout << "Segment " << kk << " successors: ";
        //     for (int m = 0; m < segment.successors.size(); m++){
        //         cout << segment.successors[m] << " ";
        //     }
        //     cout << endl;
        //     // print predecessors
        //     cout << "Segment " << kk << " predecessors: ";
        //     for (int m = 0; m < segment.predecessors.size(); m++){
        //         cout << segment.predecessors[m] << " ";
        //     }
        //     cout << endl;
        //     break;
        // }

        voronoi_graph_ = CreateVoronoiGraph(voronoi_map_, resolution_, voronoi_vis, &back_img_);

    }else{
        ROS_ERROR("Failed to call service gen_voronoi.");
    }

    auto time_voronoi_end = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds_voronoi = time_voronoi_end - time_voronoi_start;
    cout << "Generate Voronoi graph: " << elapsed_seconds_voronoi.count() << "s\n";

    // while(ros::ok()){
    //     ros::spinOnce();
    //     voronoi_vis_pub.publish(voronoi_vis);
    //     map_pub.publish(map);
    // }
    
}

vector<vector<pair<float, float>>> HomoPathPlanner::GetExtendedVisibilityCheck(){
    vector<vector<pair<float, float>>> res;
    
    for (const auto& point_pair: extended_visibility_check_whole_passage_){
        vector<pair<float, float>> pair_res;
        for (const auto& point: point_pair){
            pair_res.push_back(make_pair(point.x * resolution_, point.y * resolution_));
        }
        res.push_back(pair_res);
    }

    return res;
}

bool isPointInPolygon(const std::vector<Point2f>& polygon, int x, int y, const geometry_msgs::Point& origin) {
    int n = polygon.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon[i].x - origin.x, yi = polygon[i].y - origin.y;
        double xj = polygon[j].x - origin.x, yj = polygon[j].y - origin.y;
        bool intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

nav_msgs::OccupancyGrid HomoPathPlanner::convertToOccupancyGrid(int width, int height, const geometry_msgs::Point& origin){
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = resolution_;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position = origin;
    grid.info.origin.orientation.w = 1.0; // 单位四元数

    // 初始化网格数据
    grid.data.resize(width * height, 0); // 0 表示空闲

    // 将地图四周标位障碍物
    for (int x = 0; x < width; ++x) {
        grid.data[x] = 100; // 100 表示占用
        grid.data[(height - 1) * width + x] = 100;
    }
    for (int y = 0; y < height; ++y) {
        grid.data[y * width] = 100;
        grid.data[y * width + width - 1] = 100;
    }

    // 遍历每个多边形障碍物
    for (const auto& obstacle : obs_vec_) {
        // 遍历网格中的每个点
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // 检查点是否在多边形内部
                if (isPointInPolygon(obstacle.vertices, x, y, origin)) {
                    // 标记为占用状态
                    grid.data[y * width + x] = 100; // 100 表示占用
                }
            }
        }
    }

    return grid;
}

void DiminishPath(vector<PosNode*>& path, vector<float>& path_vel, vector<float>& path_time, float res){
    vector<PosNode*> path_tmp;
    vector<float> path_vel_tmp;
    vector<float> path_time_tmp;
    int i = 1;
    path_tmp.push_back(path[0]);
    path_vel_tmp.push_back(path_vel[0]);
    path_time_tmp.push_back(path_time[0]);
    bool end_in_path = false;
    while (i < path.size()){
        PosNode* n = path[i];
        PosNode* n_last = path_tmp.back();
        if (norm(n->pos - n_last->pos) > res){
            float time_mod = path_time_tmp.back() + norm(n->pos - n_last->pos) / path_vel_tmp.back();

            path_tmp.push_back(n);
            path_vel_tmp.push_back(path_vel[i]);
            path_time_tmp.push_back(time_mod);
            if (i==path.size()-1){
                end_in_path = true;
            }
        }
        i++;
    }
    if (!end_in_path){
        PosNode* n_last = path_tmp.back();
        PosNode* n = path[path.size()-1];
        float time_mod = path_time_tmp.back() + norm(n->pos - n_last->pos) / path_vel_tmp.back();

        path_tmp.push_back(n);
        path_vel_tmp.push_back(path_vel[path.size()-1]);
        path_time_tmp.push_back(time_mod);
    }
    // cout << "Length of path: " << path.size() << " New length: " << path_tmp.size() << endl;
    // cout << "Length of path_vel: " << path_vel.size() << " New length: " << path_vel_tmp.size() << endl;
    path = path_tmp;
    path_vel = path_vel_tmp;
    path_time = path_time_tmp;
}

tuple<vector<vector<float>>, vector<float>, vector<float> , vector<float>> HomoPathPlanner::generate_homotopic_path(py::array_t<float> start_py, py::array_t<float> end_py, py::list other_passage_passing_time, float V_aver, bool replan=false){ 

    /* [meters to pixels to meters]
    coordinates sent in are in meters
    changed to pixels by dividing by resolution
    convert to opencv version for quick visualization, but not change the original coordinates
    change back to meters when returning the path
    */

    /*
    INPUTS:
    start_py: start point of the path
    end_py: end point of the path
    other_passage_passing_time: others passage passing time stamps

    OUTPUTS:
    a pair containing: 1. the index of path chosen 2. the homotopic paths set
    */

    // Convert other agents' paths set [from meters to pixels]
    // vector<vector<vector<vector<float>>>> other_path_set_cpp = convert_other_path_set(other_path_set, resolution_);
    vector<vector<double>> other_passage_passing_time_cpp;

    // vector<unordered_map<int,float>> passage_time_map_other;

    float V;
    if (V_aver > 0){
        V = V_aver / resolution_;
    }else{
        V = Vmax;
    }

    if (test_mode_ == 4 || test_mode_ == 1 || test_mode_ == 2 || test_mode_ == 3){
        // Construct unordered map to process other agents' homo-paths' passing passage time
        // passage_time_map_other = ConstructTimeMap(other_passage_passing_time_cpp);

        other_passage_passing_time_cpp = convert_other_passage_passing_time(other_passage_passing_time);
    }
    
    auto start_time_path = chrono::system_clock::now();

    // Different homotopy class paths generation
    auto start_np = start_py.unchecked<1>();
    auto end_np = end_py.unchecked<1>();
    if(start_np.shape(0)!=2 || end_np.shape(0)!=2){
        throw runtime_error("Start and end points should have 2 coordinates.");
    }

    Point2f start = Point2f(start_np(0)/resolution_, start_np(1)/resolution_), end = Point2f(end_np(0)/resolution_, end_np(1)/resolution_);
    int path_num = 0;
    int success_path_num = 0;

    // vector<vector<vector<float>>> path_set_meter;
    // vector<vector<PosNode*>> path_set_pixel;
    vector<PosNode*> path;
    vector<vector<float>> path_to_return;
    vector<float> path_vel;
    vector<float> path_time;
    vector<vector<vector<int>>> passage_passing_info;
    bool path_found = false;  

    if (test_mode_ == 0){
        auto start_ = new PosNode(start);
        tmp.push_back(start_);
        auto end_ = new PosNode(end);
        tmp.push_back(end_);

        AStarPlanner planner(voronoi_graph_, passage_kd_tree_, obs_kd_tree_, obs_vec_, extended_visibility_check_res_, extended_visibility_check_whole_passage_, other_passage_passing_time_cpp, test_mode_, resolution_, alpha_, V, replan, 0.0, 0.0);

        bool success = planner.Plan(start_, end_);

        if(success){
            auto result = planner.GetPath();
            path = get<0>(result);
            path_vel = get<1>(result);
            path_time = get<2>(result);
            

            float res = 0.1/resolution_;
            // DiminishPath(path, path_vel, path_time, res);
            
            if (visualize_){
                DrawPath(back_img_, path, Scalar(0, 255, 0));
            }
            
            success_path_num++;
            
            convert_path_format(path, path_to_return, path_vel, resolution_);
            // path_set_meter.push_back(path_to_return);
            // path_set_pixel.push_back(path);

            path_found = true;
        }

        restart_voronoi_graph();

    }
    else if(test_mode_ == 1 || test_mode_ == 2 || test_mode_ == 3 || test_mode_ == 4){
        // homotopic paths set
        vector<PolygonObstacle> obs_vec_tmp(obs_vec_);

        // Search point near start
        // vector<PosNode*> intermediate_targets;
        // vector<PosNode*> no_access_corridor;

        // NearPassageFinder finder(voronoi_graph_, extended_visibility_check_res_, resolution_);

        // auto start_node = new PosNode(start);
        // tmp.push_back(start_node);
        // finder.FindNearPassages(start_node);
        // intermediate_targets = finder.GetIntermediateTargets();
        // cout << "intermediate_targets size: " << intermediate_targets.size() << endl;
        // for (PosNode* intermediate_target: intermediate_targets){
        //     if (intermediate_target == nullptr) {
        //         cout << "Intermediate target is nullptr\n";
        //         continue;
        //     }
        //     if (intermediate_target->passage_parent == nullptr){
        //         cout << "No passage parent\n";
        //         continue;
        //     }

        //     intermediate_target->passage_parent->closed = true;
            
        //     no_access_corridor.push_back(intermediate_target->passage_parent);
            
        // }

        auto start_ = new PosNode(start);
        tmp.push_back(start_);
        auto end_ = new PosNode(end);
        tmp.push_back(end_);

        AStarPlanner planner(voronoi_graph_, passage_kd_tree_, obs_kd_tree_, obs_vec_, extended_visibility_check_res_, extended_visibility_check_whole_passage_, other_passage_passing_time_cpp, test_mode_, resolution_, alpha_, V, replan);
        // if (replan){
        //     AStarPlanner planner(voronoi_graph_, passage_kd_tree_, obs_kd_tree_, obs_vec_, extended_visibility_check_res_, extended_visibility_check_whole_passage_, other_passage_passing_time_cpp, test_mode_, resolution_, alpha_, V, replan, 2000.0);
        // }
        

        bool success = planner.Plan(start_, end_);

        if (success){
            auto result = planner.GetPath();
            path = std::get<0>(result);
            path_vel = std::get<1>(result);
            path_time = std::get<2>(result);

            if (visualize_){
                DrawPath(back_img_, path, Scalar(0, 255, 0));
            }

            float res = 0.1/resolution_;
            // DiminishPath(path, path_vel, path_time, res);
            
            convert_path_format(path, path_to_return, path_vel, resolution_);
            // path_set_meter.push_back(path_to_return);
            // path_set_pixel.push_back(path);

            path_found = true;
        }

        restart_voronoi_graph();
        
    //     if (!path_set_found){
    //         AStarPlanner planner(voronoi_graph_, passage_kd_tree_, obs_kd_tree_, obs_vec_, extended_visibility_check_res_, passage_time_map_other, resolution_, alpha_, V, 0.0, 0.0, 0.0);
            
    //         auto start_ = new PosNode(start);
    //         tmp.push_back(start_);
    //         auto end_ = new PosNode(end);
    //         tmp.push_back(end_);

    //         bool success = planner.Plan(start_, end_);

    //         if(success){
    //             vector<PosNode*> path = planner.GetPath();

    //             float res = 0.1/resolution_;
    //             DiminishPath(path, res);
                
    //             if (visualize_){
    //                 DrawPath(back_img_, path, Scalar(0, 255, 0));
    //             }
                
    //             success_path_num++;
                
    //             vector<vector<float>> path_to_return;
    //             convert_path_format(path, path_to_return, path_vel resolution_);
    //             path_set_meter.push_back(path_to_return);
    //             path_set_pixel.push_back(path);
    //         }

    //         restart_voronoi_graph();
    //     }

    //     for (auto node: no_access_corridor){
    //         node->closed = false;
    //     }
    }

    auto end_time_path = chrono::system_clock::now();
    chrono::duration<double> elapsed_seconds_path = end_time_path - start_time_path;
    cout << "Passage-aware Homotopic Paths Generation Time: " << elapsed_seconds_path.count() << "s\n";
    // cout << "Path number: " << path_num << '\n';
    // cout << "Successful path number: " << success_path_num << '\n';
    // if(path_num > 0)
    //     cout << "Average time for each path: " << elapsed_seconds_path.count() / path_num << "s\n";

    if (visualize_){
        circle(back_img_, convert_opencv_point(start), 4, cv::viz::Color::black(), FILLED);
        circle(back_img_, convert_opencv_point(end), 4, cv::viz::Color::black(), FILLED);
        imshow("Passage-aware Homotopic Paths Generation", back_img_);
        waitKey(0);
    }

    vector<float> self_pptm;
    if (path_found){
        // Generate passage passing time map (pptm)
        int num;
        if (test_mode_ == 4 || test_mode_ == 2 || test_mode_ == 3){
            // complete passages detection
            num = extended_visibility_check_whole_passage_.size();
        }else{
            num = extended_visibility_check_res_.first.size();
        }

        vector<float> pptm_old(num, -1.0);

        self_pptm = CalculatePassageTimeMap(path_to_return, V*resolution_, pptm_old, start_py);
    }

    return make_tuple(path_to_return, path_vel, path_time, self_pptm);
}

vector<float> HomoPathPlanner::CalculatePathTime(vector<vector<float>> path, float v, vector<float> path_time_old, py::array_t<float> pos_py){

    auto pos_np = pos_py.unchecked<1>();
    Point2f pos = Point2f(pos_np(0), pos_np(1));

    vector<float> path_time(path.size(), 0.0);

    float min_dist = 1e6;
    int min_idx = -1;
    for (int j = 0; j < path.size(); j++){
        Point2f node = Point2f(path[j][0], path[j][1]);
        float dist = norm(node - pos);
        if (dist < min_dist){
            min_dist = dist;
            min_idx = j;
        }
    }

    float dist = 0.0f;
    for (int j = 0; j < path.size() - 1; j++){
        Point2f node1, node2;
        node1.x = path[j][0];
        node1.y = path[j][1];
        node2.x = path[j+1][0];
        node2.y = path[j+1][1];

        if (j < min_idx){
            // dist += cv::norm(node1 - node2);
            path_time[j+1] = path_time_old[j+1];
        }else{
            dist = cv::norm(node1 - node2);
            path_time[j+1] = path_time[j] + dist / v;
        }
    }

    return path_time;
}

vector<float> HomoPathPlanner::CalculatePassageTimeMap(vector<vector<float>> path, float v, vector<float> pptm_old, py::array_t<float> pos_py){
    auto start_pptm_path = chrono::system_clock::now();

    auto pos_np = pos_py.unchecked<1>();
    Point2f pos = Point2f(pos_np(0), pos_np(1));

    float min_dist = 1e6;
    int min_idx = -1;
    for (int j = 0; j < path.size(); j++){
        Point2f node = Point2f(path[j][0], path[j][1]);
        float dist = norm(node - pos);
        if (dist < min_dist){
            min_dist = dist;
            min_idx = j;
        }
    }

    vector<vector<Point2f>> passage_tmp;
    vector<float> self_pptm = pptm_old;

    if (test_mode_ == 4 || test_mode_ == 2 || test_mode_ == 3){
        // complete passages detection
        passage_tmp = extended_visibility_check_whole_passage_;
    }else{
        passage_tmp = extended_visibility_check_res_.second;
    }

    float dist = 0.0f;
    for (int j = 0; j < path.size() - 1; j++){

        Point2f node1, node2;
        node1.x = path[j][0];
        node1.y = path[j][1];
        node2.x = path[j+1][0];
        node2.y = path[j+1][1];

        if (j < min_idx){
            dist += cv::norm(node1 - node2);
            continue;
        }

        for (int k = 0; k < passage_tmp.size(); k++){
            float ext = 0.1;
            Point2f p1 = passage_tmp[k][0] * resolution_;
            Point2f p2 = passage_tmp[k][1] * resolution_;
            Point2f p1_ext = p1 + ext * (p1 - p2) / cv::norm(p1 - p2);
            Point2f p2_ext = p2 + ext * (p2 - p1) / cv::norm(p2 - p1);

            if (SegmentIntersection(node1, node2, p1_ext, p2_ext)==true){
                Point2f passage_mid_pts_ = (p1 + p2) / 2;
                float dist_tmp = dist + norm(node1 - passage_mid_pts_);
                self_pptm[k] = dist_tmp / v;
            }
        }

        dist += cv::norm(node1 - node2);
    }

    auto end_pptm_path = chrono::system_clock::now();
    // cout << "Passage Passing Time Map Generation Time: " << chrono::duration<double>(end_pptm_path - start_pptm_path).count() << "s\n";

    return self_pptm;
}

void HomoPathPlanner::restart_voronoi_graph(){

    for (auto root : voronoi_graph_){
        root->parent = nullptr;
        root->f_cost = 0;
        root->g_cost = 0;
        root->min_passage_width = 10000;
        root->cur_passage_width = -1;
        root->dist = 0.0;
        root->time = 0.0;
        root->vel = 0.0;
        root->passage_parent = nullptr;
        root->passage_start_time.clear();
    }
}

PYBIND11_MODULE(generate_homotopic_path, m) {
    py::class_<HomoPathPlanner>(m, "HomoPathPlanner")
        .def(py::init<int, py::list, py::dict, float, int, float, bool>())
        .def("generate_homotopic_path", &HomoPathPlanner::generate_homotopic_path, "A function that generates homotopic paths", py::arg("start"), py::arg("end"), py::arg("other_passage_passing_time"), py::arg("V_aver"),py::arg("replan"))
        .def("GetExtendedVisibilityCheck", &HomoPathPlanner::GetExtendedVisibilityCheck, "A function that returns the extended visibility check result")
        .def("CalculatePassageTimeMap", &HomoPathPlanner::CalculatePassageTimeMap, "A function that calculate passage time map", py::arg("path"), py::arg("v"),  py::arg("pptm_old"), py::arg("pos"))
        .def("CalculatePathTime", &HomoPathPlanner::CalculatePathTime, "A function that calculate path time", py::arg("path"), py::arg("v"), py::arg("path_time_old"), py::arg("pos"));
}