/**
 * Modified by: Hauser Dong
 * From Peking university
 * Last update: 2024.10.23
 * Brief Intro: This file is partly modified from https://github.com/HuangJingGitHub/HPSP
*/

#ifndef KD_TREE_INCLUDED
#define KD_TREE_INCLUDED

#include <list>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "obstacles.hpp"

using namespace cv;
using namespace std;

size_t max_size_t = numeric_limits<size_t>::max();
// size_t max_size_t = 100;

float SquaredNorm(const Point2f& pt) {
    return pt.x * pt.x + pt.y * pt.y;
}


struct PosNode
{
    Point2f pos;
    float cost = 0;
    float min_passage_width = 10000;
    float cur_passage_width = -1;  // The passage width passed by the edge parent node---current node
    size_t idx;     // if PosNode is used for obstacles, it is the index of the obstacle in obs_vec_; if PosNode is used for passage, it is the index of the passage in passage_pairs
    

    //  For kd-tree
    PosNode* left;
    PosNode* right;

    // For nearest passage finder
    PosNode* passage_parent;

    // For A* search
    list<PosNode*> neighbor;
    PosNode* parent;
    bool closed = false;
    double g_cost;
    double f_cost;
    double dist = 0.0;
    double time = 0.0;
    double vel = 0.0;
    unordered_map<int, double> passage_start_time;

    PosNode(): pos(Point2f(0, 0)), cost(0), parent(nullptr), left(nullptr), right(nullptr), g_cost(0),f_cost(0), idx(max_size_t), passage_parent(nullptr) {
        parent = nullptr;
        g_cost = 0.0;
        f_cost = 0.0;
        dist = 0.0;
        time = 0.0;
        vel = 0.0;
    }
    PosNode(Point2f initPos): pos(initPos), cost(0), parent(nullptr), left(nullptr), right(nullptr), g_cost(0),f_cost(0), idx(max_size_t), passage_parent(nullptr) {
        parent = nullptr;
        g_cost = 0.0;
        f_cost = 0.0;
        dist = 0.0;
        time = 0.0;
        vel = 0.0;
    }
    PosNode(pair<int, Point2f> obs_pair): pos(obs_pair.second), idx(obs_pair.first), cost(0), parent(nullptr), left(nullptr), right(nullptr), passage_parent(nullptr) {
        parent = nullptr;
        g_cost = 0.0;
        f_cost = 0.0;
        dist = 0.0;
        time = 0.0;
        vel = 0.0;
    }
    PosNode(PosNode* node_ptr){
        pos = node_ptr->pos;
        idx = node_ptr->idx;

        left = nullptr;
        right = nullptr;

        passage_parent = nullptr;
        
        parent = nullptr;
        g_cost = 0.0;
        f_cost = 0.0;
        dist = 0.0;
        time = 0.0;
        vel = 0.0;
    }
};


class kdTree{
private:
    const int kDimension_k_ = 2;
public:
    PosNode* kd_tree_root_;

    kdTree(): kd_tree_root_(nullptr) {}
    kdTree(PosNode* root_node): kd_tree_root_(root_node) {};
    ~kdTree();
    kdTree(const kdTree&);
    kdTree& operator=(const kdTree&);

    PosNode* CopyTree(PosNode* root){
        if (root == nullptr) return nullptr;
        PosNode* new_root = new PosNode(root);

        new_root->left = CopyTree(root->left);
        new_root->right = CopyTree(root->right);

        return new_root;
    }

    kdTree deepCopy(){
        kdTree new_tree;
        new_tree.kd_tree_root_ = CopyTree(kd_tree_root_);
        return new_tree;
    }

    

    void AddWithRoot(PosNode* root, PosNode* new_node, int depth) {
        if (depth % kDimension_k_ == 0) {
            if (new_node->pos.x <= root->pos.x) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else 
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
        else {
            if (new_node->pos.y <= root->pos.y) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
    }

    void Add(PosNode* new_node) {
        if (new_node == nullptr)
            return;

        if (kd_tree_root_ == nullptr)
            kd_tree_root_ = new_node;
        else
            AddWithRoot(kd_tree_root_, new_node, 0);
    }

    PosNode* GetCloestInTwo(PosNode* target, PosNode* candidate_1, PosNode* candidate_2) {
        if (candidate_1 == nullptr)
            return candidate_2;
        if (candidate_2 == nullptr)
            return candidate_1;

        if (SquaredNorm(target->pos - candidate_1->pos) <= SquaredNorm(target->pos - candidate_2->pos))
            return candidate_1;
        return candidate_2;
    }

    PosNode* FindNearestNodeWithRoot(PosNode* root, PosNode* target, int depth) {
        if (root == nullptr)
            return nullptr;
        
        PosNode *next_subtree = nullptr, *other_subtree = nullptr;
        if (depth % kDimension_k_ == 0) {
            if (target->pos.x <= root->pos.x) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }
        }
        else {
            if (target->pos.y <= root->pos.y) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }  
        }
        PosNode *temp_res = FindNearestNodeWithRoot(next_subtree, target, depth + 1),
                *cur_best = GetCloestInTwo(target, temp_res, root);
        float cur_dist_square = SquaredNorm(target->pos - cur_best->pos), dist_to_boundary_square, dist_to_boundary;
        if (depth % kDimension_k_ == 0) 
            dist_to_boundary = target->pos.x - root->pos.x;
        else
            dist_to_boundary = target->pos.y - root->pos.y;
        dist_to_boundary_square = dist_to_boundary * dist_to_boundary;
        
        if (cur_dist_square >= dist_to_boundary_square) {
            temp_res = FindNearestNodeWithRoot(other_subtree, target, depth + 1);
            cur_best = GetCloestInTwo(target, temp_res, cur_best);
        }
        return cur_best;
    }

    PosNode* FindNearestNode(PosNode* target) {
        return FindNearestNodeWithRoot(kd_tree_root_, target, 0);
    } 
   
    PosNode* FindNearestNode(const Point2f& target_pos) {
        PosNode* target_node = new PosNode(target_pos);
        PosNode* res = FindNearestNodeWithRoot(kd_tree_root_, target_node, 0);
        delete target_node;
        return res;
    }

    void RangeSearchWithRoot(PosNode* root, PosNode* parent, vector<PosNode*>& res_pt_vec, 
                            const float& x_min, const float& x_max, 
                            const float& y_min, const float& y_max, int depth) {
        if (root == nullptr)
            return;
        
        if (depth % kDimension_k_ == 0 && parent != nullptr) {
            if (root->pos.y <= parent->pos.y && parent->pos.y < y_min)
                return;
            if (root->pos.y > parent->pos.y && parent->pos.y > y_max)
                return;
        }
        else if (parent != nullptr) {
            if (root->pos.x <= parent->pos.x && parent->pos.x < x_min)
                return;
            if (root->pos.x > parent->pos.x && parent->pos.x > x_max)
                return;        
        }

        if (root->pos.x >= x_min && root->pos.x <= x_max && root->pos.y >= y_min && root->pos.y <= y_max)
            res_pt_vec.push_back(root);
        RangeSearchWithRoot(root->left, root, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
        RangeSearchWithRoot(root->right, root, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);           
    }

    vector<PosNode*> RangeSearch(const float& x_min, const float& x_max, const float& y_min, const float& y_max) {
        vector<PosNode*> res;
        if (x_min > x_max || y_min > y_max) {
            cout << "Invalid range for range search.\n";
            return res;
        }
        RangeSearchWithRoot(kd_tree_root_, nullptr, res, x_min, x_max, y_min, y_max, 0);
        return res;
    }

    PosNode* Search(PosNode* root, const Point2f& target_pos, int depth = 0) {
        if (root == nullptr)
            return nullptr;
        if (root->pos == target_pos)
            return root;
        
        if (depth % kDimension_k_ == 0) {
            if (target_pos.x <= root->pos.x)
                return Search(root->left, target_pos, depth + 1);
            else
                return Search(root->right, target_pos, depth + 1);
        }
        else {
            if (target_pos.y <= root->pos.y)
                return Search(root->left, target_pos, depth + 1);
            else
                return Search(root->right, target_pos, depth + 1);
        }
    }

    PosNode* Search(const Point2f& target_pos) {
        return Search(kd_tree_root_, target_pos);
    }

    void kNearestNeighborsNoCollision(PosNode* root, const PosNode* target, int k, const vector<PolygonObstacle>& obs_vec_, kdTree* obstacle_graph_, priority_queue<pair<double, PosNode*>>& pq, int depth = 0, PolygonObstacle* tmp_obs=nullptr);

    vector<PosNode*> kNearestNeighborsNoCollision(const PosNode* target, int k, const vector<PolygonObstacle>& obs_vec_, kdTree* obstacle_graph_, PolygonObstacle* tmp_obs=nullptr){
        priority_queue<pair<double, PosNode*>> pq;  // pair<distance, node>
        kNearestNeighborsNoCollision(kd_tree_root_, target, k, obs_vec_, obstacle_graph_, pq, 0, tmp_obs);

        vector<PosNode*> result;
        while(pq.empty() == false){
            result.push_back(pq.top().second);
            pq.pop();
        }
        reverse(result.begin(), result.end());
        return result;
    }

    void restartConnectionGraph(){
        restartConnectionGraph(kd_tree_root_);
    }

    void restartConnectionGraph(PosNode* root) {
        if (root == nullptr)
            return;
        
        root->parent = nullptr;
        root->f_cost = 0;
        root->g_cost = 0;
        root->min_passage_width = 10000;
        root->cur_passage_width = -1;
        root->dist = 0.0;
        
        if (root->left != nullptr) {
            restartConnectionGraph(root->left);
        }
        if (root->right != nullptr) {
            restartConnectionGraph(root->right);
        }
        
    }

    void deleteTree(PosNode* root) {
        if (root == nullptr)
            return;
        
        deleteTree(root->left);
        deleteTree(root->right);
        delete root;
    }

};

bool NoCollision(PosNode* node1, PosNode* node2, const vector<PolygonObstacle>& obs_vec_, kdTree* obstacle_graph_, PolygonObstacle* tmp_obs=nullptr);

void kdTree::kNearestNeighborsNoCollision(PosNode* root, const PosNode* target, int k, const vector<PolygonObstacle>& obs_vec_, kdTree* obstacle_graph_, priority_queue<pair<double, PosNode*>>& pq, int depth, PolygonObstacle* tmp_obs)
{
    if(root == nullptr)
        return;
    
    PosNode target_(*target);
    double dist = norm(root->pos - target->pos);
    if (NoCollision(root, &target_, obs_vec_, obstacle_graph_, tmp_obs)){
        if (pq.size() < k){
            pq.push(make_pair(dist, root));
        }else{
            if (dist < pq.top().first){
                pq.pop();
                pq.push(make_pair(dist, root));
            }
        }
    }

    int axis = depth % kDimension_k_;
    float diff;
    PosNode* next_subtree = nullptr, *other_subtree = nullptr;
    if (axis == 0){
        diff = target->pos.x - root->pos.x;
        if (target->pos.x <= root->pos.x){
            next_subtree = root->left;
            other_subtree = root->right;
        }else{
            next_subtree = root->right;
            other_subtree = root->left;
        }
    }else{
        diff = target->pos.y - root->pos.y;
        if (target->pos.y <= root->pos.y){
            next_subtree = root->left;
            other_subtree = root->right;
        }else{
            next_subtree = root->right;
            other_subtree = root->left;
        }
    }

    kNearestNeighborsNoCollision(next_subtree, target, k, obs_vec_, obstacle_graph_, pq, depth + 1, tmp_obs);

    if(pq.size() < k || fabs(diff) < pq.top().first){
        kNearestNeighborsNoCollision(other_subtree, target, k, obs_vec_, obstacle_graph_, pq, depth + 1, tmp_obs);
    }
    
}

kdTree::~kdTree() {
    kdTree::deleteTree(kd_tree_root_);
} 

kdTree::kdTree(const kdTree& copied_tree) {
    kd_tree_root_ = copied_tree.kd_tree_root_;
}

kdTree& kdTree::operator=(const kdTree& rhs) {
    kd_tree_root_ = rhs.kd_tree_root_;
    return *this;
}

void buildkdTree(kdTree& kd_tree,vector<Point2f>& pts, int depth = 0) {
    if (pts.empty())
        return;
    int k = 2;
    int axis = depth % k;

    sort(pts.begin(), pts.end(), [axis](const Point2f& a, const Point2f& b) {
        return axis == 0 ? a.x < b.x : a.y < b.y;
    });

    int median_idx = pts.size()/2;
    PosNode* node = new PosNode(pts[median_idx]);
    kd_tree.Add(node);

    vector<Point2f> left_pts(pts.begin(), pts.begin() + median_idx);
    vector<Point2f> right_pts(pts.begin() + median_idx + 1, pts.end());

    buildkdTree(kd_tree, left_pts, depth + 1);
    buildkdTree(kd_tree, right_pts, depth + 1);

    // printf("hahahahahahah\n");
}

void buildkdTree(kdTree& kd_tree, vector<pair<int, Point2f>>& pts, int depth = 0) {
    if (pts.empty())
        return;
    int k = 2;
    int axis = depth % k;

    sort(pts.begin(), pts.end(), [axis](const pair<int, Point2f>& a, const pair<int, Point2f>& b) {
        return axis == 0 ? a.second.x < b.second.x : a.second.y < b.second.y;
    });

    int median_idx = pts.size()/2;
    PosNode* node = new PosNode(pts[median_idx]);
    kd_tree.Add(node);

    vector<pair<int, Point2f>> left_pts(pts.begin(), pts.begin() + median_idx);
    vector<pair<int, Point2f>> right_pts(pts.begin() + median_idx + 1, pts.end());

    buildkdTree(kd_tree, left_pts, depth + 1);
    buildkdTree(kd_tree, right_pts, depth + 1);

}

bool NoCollision(PosNode* node1, PosNode* node2, const vector<PolygonObstacle>& obs_vec_, kdTree* obstacle_graph_, PolygonObstacle* tmp_obs) {
    // Check whether node1 and node2 can be directly connected without collision

    if (tmp_obs != nullptr) {
        if (ObstacleFree(*tmp_obs, node1->pos, node2->pos) == false){
            return false;
        }
    }

    double detect_radius = max(norm(node1->pos-node2->pos), 50.0);
    
    vector<PosNode*> near_obs = obstacle_graph_->RangeSearch(node1->pos.x - detect_radius, node1->pos.x + detect_radius, node1->pos.y - detect_radius, node1->pos.y + detect_radius);
    vector<PosNode*> near_obs_to_node = obstacle_graph_->RangeSearch(node2->pos.x - detect_radius, node2->pos.x + detect_radius, node2->pos.y - detect_radius, node2->pos.y + detect_radius);
    near_obs.insert(near_obs.end(), near_obs_to_node.begin(), near_obs_to_node.end());

    for (auto ob:near_obs){
        if (ObstacleFree(obs_vec_[ob->idx], node1->pos, node2->pos) == false)
            return false;
    }
    return true;
}

#endif