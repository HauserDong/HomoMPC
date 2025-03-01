#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include "kd_tree.hpp"

using namespace cv;
using namespace std;

bool VISUALIZE_ASTAR_SEARCH = false;    // if planning fails, set this to true to visualize the A* search process for debugging

struct ComparePosNode{
    bool operator()(PosNode* a, PosNode* b){
        return a->f_cost > b->f_cost;
    }
};

Point2f convert_opencv_point_(const Point2f& point) {
    return Point2f(point.x, 500 - point.y);
}

class NearPassageFinder{
    public:

        vector<PosNode*> intermediate_targets;

        NearPassageFinder(vector<PosNode*>& voronoi_graph, pair<vector<vector<int>>, vector<vector<Point2f>>> extended_visibility_check_res, float resolution){
            voronoi_graph_ = voronoi_graph;
            extended_visibility_passage_pair_ = extended_visibility_check_res.first;
            extended_visibility_passage_pts_ = extended_visibility_check_res.second;
            resolution_ = resolution;

            for (const auto& passage_pair : extended_visibility_passage_pts_){
                passage_mid_pts_.push_back((passage_pair[0] + passage_pair[1])/2);
            }
        }

        ~NearPassageFinder(){}
        void FindNearPassages(PosNode* start){
            // Clear intermediate targets
            intermediate_targets.clear();
            
            // Find nearest node to the start point
            float min_dist = 10000;
            PosNode* nearest_node_start = nullptr;
            for (auto node:voronoi_graph_){
                float dist = norm(node->pos - start->pos);
                if (dist < min_dist){
                    min_dist = dist;
                    nearest_node_start = node;
                }
            }
            if (nearest_node_start == nullptr){
                cout << "Failed to find the nearest node to the start point.\n";
                return;
            }

            // Find all nearest passages to the start point using breadth-first search
            float threshold = 1e-1/resolution_;

            queue<PosNode*> open_set;
            unordered_map<PosNode*, bool> closed_list;

            open_set.push(nearest_node_start);
            while(!open_set.empty()){
                PosNode* cur_node = open_set.front();
                open_set.pop();
                closed_list[cur_node] = true;

                // Check if the current node is a passage middle point
                bool is_passage_mid_pt = false;
                for (auto const& mid_pt : passage_mid_pts_){
                    if (norm(cur_node->pos - mid_pt) < threshold){

                        is_passage_mid_pt = true;

                        bool already_in_intermediate_targets = false;
                        for (auto node : intermediate_targets){
                            if (node->pos == cur_node->pos){
                                already_in_intermediate_targets = true;
                                break;
                            }
                        }

                        if (already_in_intermediate_targets) break;

                        intermediate_targets.push_back(cur_node);
                        cout << "Found a passage middle point near the start point.\n";
                        cout << "Middle point: " << mid_pt * resolution_ <<endl;
                        // cout << "Current node: " << cur_node->pos * resolution_ <<endl;
                        break;
                    }
                }
                if (is_passage_mid_pt) continue;

                // Add neighbors to the open set
                for (auto neighbor : cur_node->neighbor){
                    if (closed_list.find(neighbor) == closed_list.end()){
                        open_set.push(neighbor);
                        neighbor->passage_parent = cur_node;
                    }
                }
            }
        }

        vector<PosNode*> GetIntermediateTargets(){
            return intermediate_targets;
        }

    private:
        vector<PosNode*> voronoi_graph_;
        vector<vector<int>> extended_visibility_passage_pair_;
        vector<vector<Point2f>> extended_visibility_passage_pts_;
        vector<Point2f> passage_mid_pts_;
        float resolution_;

};

class AStarPlanner{
    public:
        PosNode* start_node_;
        PosNode* target_node_;
        vector<PosNode*> voronoi_graph_;
        kdTree* passage_graph_;
        kdTree* obstacle_graph_;
        vector<PolygonObstacle> obs_vec_;
        vector<vector<double>> other_passage_time_;
        priority_queue<PosNode*, vector<PosNode*>, ComparePosNode> open_set;
        unordered_map<PosNode*, bool> closed_list;
        vector<vector<int>> extended_visibility_passage_pair_;
        vector<vector<Point2f>> extended_visibility_passage_pts_;
        vector<vector<Point2f>> extended_visibility_check_whole_passage_;
        float passage_passing_weight_other_;
        float passage_width_weight_;
        float resolution_;
        float alpha_;
        float Vmax_;
        bool replan_;
        int start_in_passage_idx = -1;
        int test_mode_;

        AStarPlanner(vector<PosNode*>& voronoi_graph, kdTree& passage_graph, kdTree& obstacle_graph, const vector<PolygonObstacle>& obs_vec, pair<vector<vector<int>>, vector<vector<Point2f>>> extended_visibility_check_res, vector<vector<Point2f>> extended_visibility_check_whole_passage, const  vector<vector<double>>& other_passage_time, int test_mode, float resolution, float alpha, float V, bool replan, float passage_passing_weight_other = 500.0, float passage_width_weight= 50.0):start_node_(nullptr),target_node_(nullptr), voronoi_graph_(voronoi_graph), passage_graph_(&passage_graph), obstacle_graph_(&obstacle_graph), obs_vec_(obs_vec), other_passage_time_(other_passage_time), test_mode_(test_mode), resolution_(resolution), alpha_(alpha), Vmax_(V), replan_(replan), passage_passing_weight_other_(passage_passing_weight_other), passage_width_weight_(passage_width_weight) {
            extended_visibility_passage_pair_ = extended_visibility_check_res.first;
            extended_visibility_passage_pts_ = extended_visibility_check_res.second;
            extended_visibility_check_whole_passage_ = extended_visibility_check_whole_passage;
        }
        ~AStarPlanner(){}
        bool Plan(PosNode* start, PosNode* target, PolygonObstacle* tmp_obs);
        tuple<vector<PosNode*>, vector<float>, vector<float>> GetPath();
        vector<float> CalPassageTimeDuration(PosNode* node1, PosNode* node2, int passage_idx, float ext);
        void restartPlanner();
    
    private:
        double Heuristic(const PosNode* a, const PosNode* b){
            double h_cost = 0.0;
            h_cost += norm(a->pos - b->pos);            // length cost
            h_cost += norm(a->pos - b->pos)/Vmax_;      // time cost
            return h_cost;
        }

        bool OneShotToTarget(PosNode* cur_node);

        float GetPassagePenalty(PosNode* node1, PosNode* node2);
};

void AStarPlanner::restartPlanner(){
    closed_list.clear();
    while(!open_set.empty()){
        open_set.pop();
    }
}

bool AStarPlanner::Plan(PosNode* start, PosNode* target, PolygonObstacle* tmp_obs=nullptr){
    start_node_ = start;
    target_node_ = target;

    if (DetectPointInObstacleList(obs_vec_, start_node_->pos)){
        cout << "Start point is colliding with obstacle.\n";
        return false;
    }

    if (DetectPointInObstacleList(obs_vec_, target_node_->pos)){
        cout << "Target point is colliding with obstacle.\n";
        return false;
    }

    // find nearest voronoi node to the start point
    float min_dist = 10000;
    PosNode* nearest_node_start = nullptr;
    for (auto node:voronoi_graph_){
        float dist = norm(node->pos - start_node_->pos);
        if (dist < min_dist){
            min_dist = dist;
            nearest_node_start = node;
        }
    }
    if (nearest_node_start != nullptr){
        if(nearest_node_start->pos == start_node_->pos){
            start_node_ = nearest_node_start;
        }else{
            start_node_->neighbor.push_back(nearest_node_start);
            // cout << "Nearest node to the start point: " << nearest_node_start->pos << '\n';
        }
    }else{
        cout << "Failed to find the nearest node to the start point.\n";
        return false;
    }

    // find nearest voronoi node to the target point
    min_dist = 10000;
    PosNode* nearest_node_target = nullptr;
    bool nearest_node_target_add_neighbor = false;
    for (auto node:voronoi_graph_){
        float dist = norm(node->pos - target_node_->pos);
        if (dist < min_dist){
            min_dist = dist;
            nearest_node_target = node;
        }
    }
    if (nearest_node_target != nullptr){
        if(nearest_node_target->pos == target_node_->pos){
            target_node_ = nearest_node_target;
            // cout << "Target node is the nearest node to the target point.\n";
        }else{
            nearest_node_target->neighbor.push_back(target_node_);
            // cout << "Nearest node to the target point added into neighbor list: " << nearest_node_target->pos << '\n';
            nearest_node_target_add_neighbor = true;
        }
    }else{
        cout << "Failed to find the nearest node to the target point.\n";
        return false;
    }

    start_node_->g_cost = 0;
    start_node_->f_cost = start_node_->g_cost + Heuristic(start_node_, target_node_);
    start_node_->vel = Vmax_;
    open_set.push(start_node_);

    Mat back_img_ = Mat(Size(500, 500), CV_64FC3, Scalar(255, 255, 255));

    // Check if start is in a passage (only for replanning)
    if (replan_){
        for (int i = 0; i < extended_visibility_passage_pts_.size(); i++){
            vector<Point2f> passage_1 = extended_visibility_check_whole_passage_[2*i];
            vector<Point2f> passage_2 = extended_visibility_check_whole_passage_[2*i+1];
            Point2f mid_pt_1 = (passage_1[0] + passage_1[1]) / 2;
            Point2f mid_pt_2 = (passage_2[0] + passage_2[1]) / 2;

            Point2f d1 = start_node_->pos - mid_pt_1;
            Point2f d2 = start_node_->pos - mid_pt_2;
            if (d1.x * d2.x + d1.y * d2.y < 0){
                // start is in the passage
                start_in_passage_idx = i;
            }
        }

        // if(start_in_passage_idx != -1){   
        //     for (int i = 0; i < other_passage_time_.size(); i++){
        //         int block_idx = -1; 
        //         double other_passage_time_1 = other_passage_time_[i][2*start_in_passage_idx];
        //         double other_passage_time_2 = other_passage_time_[i][2*start_in_passage_idx+1];
        //         if (other_passage_time_1 > 0 && other_passage_time_2 > 0){
        //             // other agent passes the passage
        //             if (other_passage_time_1 < other_passage_time_2){
        //                 block_idx = 2*start_in_passage_idx;
        //             }else{
        //                 block_idx = 2*start_in_passage_idx+1;
        //             }
        //         }
        //         if (block_idx != -1){
        //             // block the passage
        //             PosNode* start_on_graph = start_node_->neighbor;

        //         }

        //     }
        // }
        
    }

    int expand_num = 0;
    while (!open_set.empty()){
        PosNode* cur_node = open_set.top();
        open_set.pop();
        expand_num ++;
        
        if(VISUALIZE_ASTAR_SEARCH){
            circle(back_img_, convert_opencv_point_(cur_node->pos), 1, cv::viz::Color::black(), FILLED);
        }
        
        if (cur_node->pos == target_node_->pos){
            // cout << "Last node of nearest node target: " << nearest_node_target->neighbor.back()->pos << '\n';
            if (nearest_node_target_add_neighbor){
                nearest_node_target->neighbor.pop_back();
            }else{
                // in some rare cases, target_node_ and cur_node have the same position, but pointing to different addresses
                target_node_ = cur_node;
            }
            // cout << "Last node of nearest node target: " << nearest_node_target->neighbor.back()->pos << '\n';

            // cout << "Expand number: " << expand_num << '\n';
            return true;
        }
        
        closed_list[cur_node] = true;
        for(auto node:cur_node->neighbor){
            if (closed_list.find(node)!=closed_list.end()) continue;
            if (node->closed) continue;

            float passage_passing_other = GetPassagePenalty(cur_node, node);
            
            float passed_passage_width = GetMinPassageWidthPassed(extended_visibility_passage_pts_, cur_node->pos, node->pos, resolution_);

            float narrow_threshold = 0.05/resolution_;
            if(passed_passage_width < narrow_threshold && passed_passage_width >= 0.0){
                // narrow passage, do not consider
                continue;
            }
            
            double g_tmp;
            double length_cost = norm(cur_node->pos - node->pos);
            double time_cost = length_cost / Vmax_;
            if (passed_passage_width < 0){
                // do not pass passage
                g_tmp = cur_node->g_cost + length_cost + time_cost +  passage_passing_weight_other_ * passage_passing_other;
            }else{
                // pass passage
                if (passed_passage_width >= cur_node->min_passage_width){
                    g_tmp = cur_node->g_cost + length_cost + time_cost + passage_passing_weight_other_ * passage_passing_other;
                }else{
                    g_tmp = cur_node->g_cost + length_cost + time_cost + passage_width_weight_ * 
                        (cur_node->min_passage_width - passed_passage_width) + passage_passing_weight_other_ * passage_passing_other;
                }
            }

            
            if(g_tmp < node->g_cost || node->parent == nullptr){
                node->parent = cur_node;
                node->g_cost = g_tmp;
                node->f_cost = node->g_cost + Heuristic(node, target_node_);
                node->dist = cur_node->dist + length_cost;
                node->time = cur_node->time + time_cost;
                node->vel = Vmax_;
                node->passage_start_time = cur_node->passage_start_time;
                
                if (passed_passage_width>0){
                    if (passed_passage_width < cur_node->min_passage_width){
                        node->min_passage_width = passed_passage_width;
                    }else{
                        node->min_passage_width = cur_node->min_passage_width;
                    }
                }else{
                    node->min_passage_width = cur_node->min_passage_width;
                }
                open_set.push(node);
            }
        }
    }
    
    cout << "Failed to find a path.\n";
    // cout << "Last node of nearest node target: " << nearest_node_target->neighbor.back()->pos << '\n';
    if (nearest_node_target_add_neighbor){
        nearest_node_target->neighbor.pop_back();
    }
    // cout << "Last node of nearest node target: " << nearest_node_target->neighbor.back()->pos << '\n';

    cout << "Expand number: " << expand_num << '\n';

    if(VISUALIZE_ASTAR_SEARCH){
        imshow("A* Visualization", back_img_);
        waitKey(0);
    }

    return false;
}

bool AStarPlanner::OneShotToTarget(PosNode* node){
    // Check whether node can be directly connected to the target node
    return NoCollision(node, target_node_, obs_vec_, obstacle_graph_);
}

tuple<vector<PosNode*>, vector<float>, vector<float>> AStarPlanner::GetPath(){
    // If find a path to target, then call GetPath() to get the path
    vector<PosNode*> path;
    vector<float> path_vel;
    vector<float> path_time;
    PosNode* cur_node = target_node_;
    // cout << "Total cost: " << cur_node->g_cost << '\n';
    while (cur_node != nullptr){
        path.push_back(cur_node);
        path_vel.push_back(cur_node->vel);
        path_time.push_back(cur_node->time);
        cur_node = cur_node->parent;
    }
    reverse(path.begin(), path.end());
    reverse(path_vel.begin(), path_vel.end());
    reverse(path_time.begin(), path_time.end());
    return make_tuple(path, path_vel, path_time);
}

vector<Point2f> ExtendSegment(Point2f p1, Point2f p2, float ext){
    Point2f p1_ext = p1 + ext * (p1 - p2) / cv::norm(p1 - p2);
    Point2f p2_ext = p2 + ext * (p2 - p1) / cv::norm(p2 - p1);
    return {p1_ext, p2_ext};
}

float AStarPlanner::GetPassagePenalty(PosNode* node1, PosNode* node2){
    // node1 to node2
    float passage_passing_penalty_other = 0.0;

    if (test_mode_ == 4 || test_mode_ == 2 || test_mode_ == 3){
        // complete passages detection

        for(int i = 0; i < extended_visibility_passage_pts_.size(); i++){

            vector<Point2f> passage_1 = extended_visibility_check_whole_passage_[2*i];
            vector<Point2f> passage_2 = extended_visibility_check_whole_passage_[2*i+1];
            Point2f mid_pt_1 = (passage_1[0] + passage_1[1]) / 2;
            Point2f mid_pt_2 = (passage_2[0] + passage_2[1]) / 2;
            vector<double> passage_time_duration;
            int passage_idx = i;

            float ext = 0.1/resolution_;
            vector<Point2f> passage_1_ext = ExtendSegment(passage_1[0], passage_1[1], ext);
            vector<Point2f> passage_2_ext = ExtendSegment(passage_2[0], passage_2[1], ext);
            bool passage_1_intersect = false, passage_2_intersect = false;

            if (SegmentIntersection(node1->pos, node2->pos, passage_1_ext[0], passage_1_ext[1]) == true){
                passage_1_intersect = true;
            }
            if (SegmentIntersection(node1->pos, node2->pos, passage_2_ext[0], passage_2_ext[1]) == true){
                passage_2_intersect = true;
            }

            int debug_passage_idx = 70;
            bool debug_flag = false;
            if(passage_1_intersect && passage_2_intersect){
                if(passage_idx == debug_passage_idx && debug_flag) cout << "Passage 1 and Passage 2 intersect with node1-node2.\n";
                // node1-node2 intersects with both passages
                float dist_1 = node1->dist + norm(node1->pos - mid_pt_1);
                float time_1 = dist_1 / Vmax_;
                float dist_2 = node1->dist + norm(node1->pos - mid_pt_2);
                float time_2 = dist_2 / Vmax_;

                passage_time_duration.push_back(min(time_1, time_2));
                passage_time_duration.push_back(max(time_1, time_2));
            }else if(passage_1_intersect || passage_2_intersect){
                // node1-node2 intersects with one of the passages

                if (node1->passage_start_time.find(passage_idx)==node1->passage_start_time.end()){
                    if(passage_idx == debug_passage_idx && debug_flag) cout << "node1-node2 intersects with enter of the passage.\n";
                    // enter a new passage
                    Point2f mid_pt_enter = passage_1_intersect ? mid_pt_1 : mid_pt_2;
                    float dist_tmp = node1->dist + norm(node1->pos - mid_pt_enter);
                    float time_tmp = dist_tmp / Vmax_;
                    node1->passage_start_time[passage_idx] = time_tmp;

                    if (i==start_in_passage_idx){
                        for (int path_idx = 0; path_idx < other_passage_time_.size(); path_idx++){
                            vector<double> other_agent_passage_time = other_passage_time_[path_idx];

                            double other_passage_time_1 = other_agent_passage_time[2*passage_idx];
                            double other_passage_time_2 = other_agent_passage_time[2*passage_idx+1];

                            float heavy_penalty = 5000.0;
                            if(other_passage_time_1>0 && other_passage_time_2>0){
                                vector<double> other_passage_time_duration = {min(other_passage_time_1, other_passage_time_2), max(other_passage_time_1, other_passage_time_2)};
                                
                                if(time_tmp > other_passage_time_duration[0] && time_tmp < other_passage_time_duration[1]){
                                    passage_passing_penalty_other += heavy_penalty;
                                }
                            }else if(other_passage_time_1>0){
                                if (passage_2_intersect){
                                    passage_passing_penalty_other += heavy_penalty;
                                }
                            }else if(other_passage_time_2>0){
                                if (passage_1_intersect){
                                    passage_passing_penalty_other += heavy_penalty;
                                }
                            }
                        }
                    }
                }else{
                    // exit this passage
                    Point2f mid_pt_exit = passage_1_intersect ? mid_pt_1 : mid_pt_2;
                    float dist_tmp = node1->dist + norm(node1->pos - mid_pt_exit);
                    float time_tmp = dist_tmp / Vmax_;

                    passage_time_duration.push_back(node1->passage_start_time[passage_idx]);
                    passage_time_duration.push_back(time_tmp);

                    node1->passage_start_time.erase(passage_idx);

                    if(passage_idx == debug_passage_idx && debug_flag) cout << "node1-node2 intersects with exit of the passage. its duration is " << passage_time_duration[0] << " " << passage_time_duration[1] << endl;
                }
            }

            if (passage_time_duration.size() == 2){

                // consider others' result
                for (int path_idx = 0; path_idx < other_passage_time_.size(); path_idx++){
                    vector<double> other_agent_passage_time = other_passage_time_[path_idx];

                    double other_passage_time_1 = other_agent_passage_time[2*passage_idx];
                    double other_passage_time_2 = other_agent_passage_time[2*passage_idx+1];

                    if(other_passage_time_1>0 && other_passage_time_2>0){
                        vector<double> other_passage_time_duration = {min(other_passage_time_1, other_passage_time_2), max(other_passage_time_1, other_passage_time_2)};
                        double time_lower = min(passage_time_duration[0], other_passage_time_duration[0]);
                        double time_upper = max(passage_time_duration[1], other_passage_time_duration[1]);
                        double time_overlap_lower = max(passage_time_duration[0], other_passage_time_duration[0]);
                        double time_overlap_upper = min(passage_time_duration[1], other_passage_time_duration[1]);

                        if (time_overlap_upper - time_overlap_lower > 0){
                            passage_passing_penalty_other += 1.0;

                            if(passage_idx == debug_passage_idx && debug_flag) {
                                cout << ">>>>>>>>>>>>>>>>>>>>>>>>" << endl;
                                cout << "Penalty add: " << 1.0 << endl;
                            }
                            
                        }else{
                            passage_passing_penalty_other += exp(alpha_ * fabs(time_overlap_upper - time_overlap_lower));
                        }

                        // double penalty_add = (time_overlap_upper - time_overlap_lower) / (time_upper - time_lower) + 1.0;
                        // passage_passing_penalty_other += penalty_add;

                        // cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
                        // cout << "Penalty add: " << penalty_add << endl;
                        
                    }
                    else if ((other_passage_time_1 > 0 || other_passage_time_2 >0) && replan_){
                        double other_passage_time = other_passage_time_1 > 0 ? other_passage_time_1 : other_passage_time_2;

                        vector<double> other_passage_time_duration = {0.0, other_passage_time};

                        double time_lower = min(passage_time_duration[0], other_passage_time_duration[0]);
                        double time_upper = max(passage_time_duration[1], other_passage_time_duration[1]);
                        double time_overlap_lower = max(passage_time_duration[0], other_passage_time_duration[0]);
                        double time_overlap_upper = min(passage_time_duration[1], other_passage_time_duration[1]);

                        if (time_overlap_upper - time_overlap_lower > 0){
                            passage_passing_penalty_other += 1.0;
                            
                        }else{
                            passage_passing_penalty_other += exp(alpha_ * fabs(time_overlap_upper - time_overlap_lower));
                        }
                    }
                        
                }
            }
        }
    }else{
        // passage segment detection
        for(int i = 0; i < extended_visibility_passage_pts_.size(); i++){

            float ext = 0.1/resolution_;
            Point2f p1 = extended_visibility_passage_pts_[i][0];
            Point2f p2 = extended_visibility_passage_pts_[i][1];
            Point2f p1_ext = p1 + ext * (p1 - p2) / cv::norm(p1 - p2);
            Point2f p2_ext = p2 + ext * (p2 - p1) / cv::norm(p2 - p1);

            if (SegmentIntersection(node1->pos, node2->pos, p1_ext, p2_ext) == true){

                // consider others' result
                Point2f mid_pt = (extended_visibility_passage_pts_[i][0] + extended_visibility_passage_pts_[i][1]) / 2;
                for (int path_idx = 0; path_idx < other_passage_time_.size(); path_idx++){
                    double other_passage_time = other_passage_time_[path_idx][i];
                    if (other_passage_time > 0){
                        float dist_tmp = node1->dist + norm(node1->pos - mid_pt);
                        float self_time = dist_tmp / Vmax_;

                        passage_passing_penalty_other += exp(alpha_ * fabs(self_time - other_passage_time));
                    }
                        
                }
            }
        }
    }

    return passage_passing_penalty_other;
}


vector<unordered_map<int,float>> ConstructTimeMap(vector<vector<float>> other_passage_passing_time){

    vector<unordered_map<int,float>> result;

    for (int agent = 0; agent < other_passage_passing_time.size(); agent++){
        unordered_map<int, float> tmp;
        vector<float> agent_passsage_passing_time = other_passage_passing_time[agent];
        for (int i = 0; i < agent_passsage_passing_time.size(); i++){
            float time = agent_passsage_passing_time[i];
            if (time > 0){
                tmp[i] = time;
            }else if (fabs(time) < 1e-4){
                tmp[i] = 0.0;
            }
        }
        result.push_back(tmp);
    }
    return result;
}