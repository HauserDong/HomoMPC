/**
 * Modified by: Hauser Dong
 * From Peking university
 * Last update: 2025.03.03
 * Brief Intro: This file is partly modified from https://github.com/SimoneTinella/Path-Planning-Robot-over-Voronoi-Diagram.
*/

#include <math.h>
#include <unordered_map>
#include <opencv2/imgproc.hpp>
#include "obstacles.hpp"
#include "kd_tree.hpp"
#include <tuw_multi_robot_msgs/Graph.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

const float dist_tol = sqrt(2);
const float radius = 5 * sqrt(2); 

int range_x = -1, range_y = -1;

Point2f convert_opencv_point(const Point2f& point) {
    return Point2f(point.x, range_y - point.y);
}

class Voronoi{
    private:
        int range_x_, range_y_;
        vector<PolygonObstacle> obs_vec_;
        vector<Point2f> voronoi_pts_;
        unordered_map<Point2f*, int> voronoi_intersect_pts;
        
        Mat* back_img_;
        bool visualize_;

    public:
        Voronoi(){}
        Voronoi(int r_x, int r_y, vector<PolygonObstacle>& obs_vec, Mat* back_img, bool visualize);
        vector<PosNode*> CreateVoronoiGraph();
        ~Voronoi(){}
};

Voronoi::Voronoi(int r_x, int r_y, vector<PolygonObstacle>& obs_vec, Mat* back_img, bool visualize): range_x_(r_x), range_y_(r_y), obs_vec_(obs_vec), back_img_(back_img), visualize_(visualize){

    // search for every point in the map
    for (int i = 0; i < range_x_; i++){

        for(int j = 0; j < range_y_; j++){
            float i_f = i, j_f = j;
            Point2f cur_pt(i_f, j_f);
            vector<float> dists;

            if(DetectPointInObstacleList(obs_vec_, cur_pt)){
                continue;
            }

            // Calculate the distance to every obstacle
            for(auto ob : obs_vec_){
                float dist = MinDistanceToObstacle(ob, cur_pt);
                dists.push_back(dist);
            }

            sort(dists.begin(), dists.end());

            float min1 = dists[0], min2 = dists[1];
            if(fabs(min1-min2) <= dist_tol){
                voronoi_pts_.push_back(cur_pt);

                if(obs_vec_.size() > 2){
                    // check for intersection points and count the number of intersections
                    float min3 = dists[2];
                    if(fabs(min2-min3)<=dist_tol && fabs(min1-min3) <= dist_tol){
                        int count = 3;

                        for(int k = 3; k < dists.size(); k++){
                            bool inter_flag = true;
                            for(int l = k-1; l >= 0; l--){
                                if(fabs(dists[k]-dists[l])>dist_tol){
                                    inter_flag = false;
                                    break;
                                }
                            }

                            if(inter_flag){
                                count++;
                            }else{
                                break;
                            }
                        }

                        voronoi_intersect_pts[&voronoi_pts_.back()] = count;
                    }
                }
            }
        }
    }

}

vector<PosNode*> Voronoi::CreateVoronoiGraph(){

    vector<PosNode*> voronoi_graph;
    for(auto pt : voronoi_pts_){
        voronoi_graph.push_back(new PosNode(pt));
    }

    for(int i = 0 ; i < voronoi_pts_.size(); i++){
        // int intersect_num = -1;
        // if(voronoi_intersect_pts.find(&voronoi_pts_[i]) != voronoi_intersect_pts.end()){
        //     intersect_num = voronoi_intersect_pts[&voronoi_pts_[i]];
        // }

        for(int j = 0; j < voronoi_pts_.size(); j++){
            if (i == j)
                continue;
            if (norm(voronoi_pts_[i] - voronoi_pts_[j]) > radius)
                continue;
            
            if (norm(voronoi_pts_[i] - voronoi_pts_[j]) <= dist_tol){
                // cout << "Voronoi pts size: " << voronoi_pts_.size() << endl;
                // cout << "Voronoi graph size: " << voronoi_graph.size() << endl;

                // cout << "i: " << i << " j: " << j << endl;

                voronoi_graph[i]->neighbor.push_back(voronoi_graph[j]);

                if(visualize_){
                    line(*back_img_, convert_opencv_point(voronoi_graph[i]->pos), convert_opencv_point(voronoi_graph[j]->pos), Scalar(255, 0, 0), 1);
                }
            }
        }
    }

    return voronoi_graph;
}

visualization_msgs::Marker create_point_marker(const Point2f& point, int id, string ns, float scale, vector<float> color){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    geometry_msgs::Point p;
    p.x = point.x;
    p.y = point.y;
    marker.pose.position = p;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.a = 1.0;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];

    return marker;
}

visualization_msgs::Marker create_line_marker(const Point2f& start, const Point2f& end, int id, string ns, float scale, vector<float> color){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    
    geometry_msgs::Point s, e;
    s.x = start.x;
    s.y = start.y;
    e.x = end.x;
    e.y = end.y;

    marker.points.push_back(s);
    marker.points.push_back(e);
    marker.scale.x = scale;
    marker.color.a = 1.0;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];

    return marker;
}


vector<PosNode*> CreateVoronoiGraph(const tuw_multi_robot_msgs::Graph& voronoi_map, float resolution, visualization_msgs::MarkerArray& voronoi_vis, Mat* back_img){
    vector<PosNode*> voronoi_graph;
    unordered_map<int, int> segIDX_to_vecIDX_head;    // check whether the first element of segment is already in the voronoi_graph vector
    unordered_map<int, int> segIDX_to_vecIDX_tail;    // check whether the last element of segment is already in the voronoi_graph vector

    geometry_msgs::Point voronoi_origin = voronoi_map.origin.position;
    int vertex_id = 0, edge_id = 0;

    for (int i = 0; i < voronoi_map.vertices.size(); i++){
        auto segment = voronoi_map.vertices[i];
        
        // check whether the first element of segment is already in the voronoi_graph vector
        if (segIDX_to_vecIDX_head.find(i)==segIDX_to_vecIDX_head.end()){
            // add segment.path to voronoi_graph one by one
            for (int j = 0; j < segment.path.size(); j++){
                geometry_msgs::Point p = segment.path[j];
                Point2f pos;
                pos.x = round((p.x + voronoi_origin.x)*100)/100.0;
                pos.y = round((p.y + voronoi_origin.y)*100)/100.0;
                pos.x = pos.x / resolution;
                pos.y = pos.y / resolution;
                voronoi_graph.push_back(new PosNode(pos));

                if(j==0){
                    segIDX_to_vecIDX_head[i] = voronoi_graph.size()-1;
                }
                if(j==segment.path.size()-1){
                    segIDX_to_vecIDX_tail[i] = voronoi_graph.size()-1;
                }
                if(j>0){
                    // add bidirectional edge
                    voronoi_graph[voronoi_graph.size()-2]->neighbor.push_back(voronoi_graph.back());
                    voronoi_graph.back()->neighbor.push_back(voronoi_graph[voronoi_graph.size()-2]);

                    // add visualization
                    Point2f p1, p2;
                    p1.x = voronoi_graph[voronoi_graph.size()-2]->pos.x * resolution;
                    p1.y = voronoi_graph[voronoi_graph.size()-2]->pos.y * resolution;
                    p2.x = voronoi_graph.back()->pos.x * resolution;
                    p2.y = voronoi_graph.back()->pos.y * resolution;
                    visualization_msgs::Marker marker = create_line_marker(p1, p2, edge_id, "edge", 0.02, {0.0, 0.0, 1.0});
                    voronoi_vis.markers.push_back(marker);
                    edge_id++;

                    line(*back_img, convert_opencv_point(voronoi_graph[voronoi_graph.size()-2]->pos), convert_opencv_point(voronoi_graph.back()->pos), Scalar(255, 0, 0), 1);
                }
            }

            // add successors
            for (int j = 0; j < segment.successors.size(); j++){
                int succ = segment.successors[j];
                if (segIDX_to_vecIDX_head.find(succ) != segIDX_to_vecIDX_head.end()){
                    int succ_head = segIDX_to_vecIDX_head[succ];
                    int succ_tail = segIDX_to_vecIDX_tail[succ];

                    Point2f head_pos = voronoi_graph[succ_head]->pos;
                    Point2f tail_pos = voronoi_graph[succ_tail]->pos;
                    Point2f cur_tail = voronoi_graph[segIDX_to_vecIDX_tail[i]]->pos;

                    if(norm(head_pos-cur_tail) <= norm(tail_pos-cur_tail)){
                        voronoi_graph[segIDX_to_vecIDX_tail[i]]->neighbor.push_back(voronoi_graph[succ_head]);
                        voronoi_graph[succ_head]->neighbor.push_back(voronoi_graph[segIDX_to_vecIDX_tail[i]]);

                        Point2f p1, p2;
                        p1.x = voronoi_graph[segIDX_to_vecIDX_tail[i]]->pos.x * resolution;
                        p1.y = voronoi_graph[segIDX_to_vecIDX_tail[i]]->pos.y * resolution;
                        p2.x = voronoi_graph[succ_head]->pos.x * resolution;
                        p2.y = voronoi_graph[succ_head]->pos.y * resolution;
                        visualization_msgs::Marker marker = create_line_marker(p1, p2, edge_id, "edge", 0.02, {0.0, 0.0, 1.0});
                        voronoi_vis.markers.push_back(marker);
                        edge_id++;

                        line(*back_img, convert_opencv_point(voronoi_graph[segIDX_to_vecIDX_tail[i]]->pos), convert_opencv_point(voronoi_graph[succ_head]->pos), Scalar(255, 0, 0), 1);

                    }else{
                        voronoi_graph[segIDX_to_vecIDX_tail[i]]->neighbor.push_back(voronoi_graph[succ_tail]);
                        voronoi_graph[succ_tail]->neighbor.push_back(voronoi_graph[segIDX_to_vecIDX_tail[i]]);

                        Point2f p1, p2;
                        p1.x = voronoi_graph[segIDX_to_vecIDX_tail[i]]->pos.x * resolution;
                        p1.y = voronoi_graph[segIDX_to_vecIDX_tail[i]]->pos.y * resolution;
                        p2.x = voronoi_graph[succ_tail]->pos.x * resolution;
                        p2.y = voronoi_graph[succ_tail]->pos.y * resolution;
                        visualization_msgs::Marker marker = create_line_marker(p1, p2, edge_id, "edge", 0.02, {0.0, 0.0, 1.0});
                        voronoi_vis.markers.push_back(marker);
                        edge_id++;

                        line(*back_img, convert_opencv_point(voronoi_graph[segIDX_to_vecIDX_tail[i]]->pos), convert_opencv_point(voronoi_graph[succ_tail]->pos), Scalar(255, 0, 0), 1);
                    }
                }
            }

            // add predecessors
            for (int j = 0; j < segment.predecessors.size(); j++){
                int pred = segment.predecessors[j];
                if (segIDX_to_vecIDX_head.find(pred) != segIDX_to_vecIDX_head.end()){
                    int pred_head = segIDX_to_vecIDX_head[pred];
                    int pred_tail = segIDX_to_vecIDX_tail[pred];

                    Point2f head_pos = voronoi_graph[pred_head]->pos;
                    Point2f tail_pos = voronoi_graph[pred_tail]->pos;
                    Point2f cur_head = voronoi_graph[segIDX_to_vecIDX_head[i]]->pos;

                    if(norm(tail_pos-cur_head) <= norm(head_pos-cur_head)){
                        voronoi_graph[segIDX_to_vecIDX_head[i]]->neighbor.push_back(voronoi_graph[pred_tail]);
                        voronoi_graph[pred_tail]->neighbor.push_back(voronoi_graph[segIDX_to_vecIDX_head[i]]);

                        Point2f p1, p2;
                        p1.x = voronoi_graph[segIDX_to_vecIDX_head[i]]->pos.x * resolution; 
                        p1.y = voronoi_graph[segIDX_to_vecIDX_head[i]]->pos.y * resolution;
                        p2.x = voronoi_graph[pred_tail]->pos.x * resolution;
                        p2.y = voronoi_graph[pred_tail]->pos.y * resolution;
                        visualization_msgs::Marker marker = create_line_marker(p1, p2, edge_id, "edge", 0.02, {0.0, 0.0, 1.0});
                        voronoi_vis.markers.push_back(marker);
                        edge_id++;

                        line(*back_img, convert_opencv_point(voronoi_graph[segIDX_to_vecIDX_head[i]]->pos), convert_opencv_point(voronoi_graph[pred_tail]->pos), Scalar(255, 0, 0), 1);
                    }else{
                        voronoi_graph[segIDX_to_vecIDX_head[i]]->neighbor.push_back(voronoi_graph[pred_head]);
                        voronoi_graph[pred_head]->neighbor.push_back(voronoi_graph[segIDX_to_vecIDX_head[i]]);

                        Point2f p1, p2;
                        p1.x = voronoi_graph[segIDX_to_vecIDX_head[i]]->pos.x * resolution;
                        p1.y = voronoi_graph[segIDX_to_vecIDX_head[i]]->pos.y * resolution;
                        p2.x = voronoi_graph[pred_head]->pos.x * resolution;
                        p2.y = voronoi_graph[pred_head]->pos.y * resolution;
                        visualization_msgs::Marker marker = create_line_marker(p1, p2, edge_id, "edge", 0.02, {0.0, 0.0, 1.0});
                        voronoi_vis.markers.push_back(marker);
                        edge_id++;

                        line(*back_img, convert_opencv_point(voronoi_graph[segIDX_to_vecIDX_head[i]]->pos), convert_opencv_point(voronoi_graph[pred_head]->pos), Scalar(255, 0, 0), 1);
                    }
                }
            }
        }
    }

    return voronoi_graph;
}