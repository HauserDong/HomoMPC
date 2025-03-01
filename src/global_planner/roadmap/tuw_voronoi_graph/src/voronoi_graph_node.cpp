#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <roadmap_builder/GenVoronoi.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_graph/voronoi_graph_node.h>
#include <tuw_voronoi_graph/voronoi_graph_generator.h>
#include <memory>
#include <tuw_multi_robot_msgs/Graph.h>
#include <string>


void publishTf(std::string mapName);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "voronoi_graph_node"); /// initializes the ros node with default name
    ros::NodeHandle n;

    tuw_graph::VoronoiGeneratorNode mapNode(n);

    return 0;
}

namespace tuw_graph
{
    VoronoiGeneratorNode::VoronoiGeneratorNode(ros::NodeHandle &n):
    voronoi_map::VoronoiPathGenerator(), VoronoiGraphGenerator(), Serializer(), n_(n), n_param_("~")
    {   
        //从 ROS 参数服务器获取参数值，如果参数不存在，则使用默认值
        double loop_rate;
        n_param_.param<double>("loop_rate", loop_rate, 0.1);
        n_param_.param<bool>("publish_voronoi_map_image", publishVoronoiMapImage_, false);
        n_param_.param<double>("map_inflation", inflation_, 0.1); /// [meters]
        n_param_.param<float>("segment_length", segment_length_, 1.0); /// [meters]
        ROS_INFO_STREAM("map_inflation: " << inflation_);

        //设置交叉点优化和末端段优化的默认值，并从参数服务器获取可能的自定义值
        crossingOptimization_ = 0.2;
        n_param_.param<float>("opt_crossings", crossingOptimization_, 0.2);
        n_param_.param<float>("opt_end_segments", endSegmentOptimization_, 0.2);
        //endSegmentOptimization_ = std::min<float>(endSegmentOptimization_, 0.7 * path_length_);

        pubSegments_ = n.advertise<tuw_multi_robot_msgs::Graph>("segments", 1);

        server_ = n.advertiseService("/gen_voronoi", &VoronoiGeneratorNode::gen_voronoi_callback, this);

        ros::spin();
    }

    bool VoronoiGeneratorNode::gen_voronoi_callback(roadmap_builder::GenVoronoi::Request &req, roadmap_builder::GenVoronoi::Response &res)
    {
        //将地图消息中的数据成员 data 复制到一个 std::vector<signed char> 类型的向量 map 中
        std::vector<signed char> map = req.occupancy_map.data;
        //定义一个 std::vector<double> 类型的向量 parameters，用来存储生成 Voronoi 图所需的参数
        std::vector<double> parameters;
        //向 parameters 向量中添加地图的原点坐标、分辨率以及其他几个参数，这些参数可能影响 Voronoi 图的生成
        parameters.push_back(req.occupancy_map.info.origin.position.x);
        parameters.push_back(req.occupancy_map.info.origin.position.y);
        parameters.push_back(req.occupancy_map.info.resolution);
        parameters.push_back(inflation_);
        parameters.push_back(segment_length_);
        parameters.push_back(endSegmentOptimization_);
        parameters.push_back(crossingOptimization_);
        
        boost::shared_ptr<const nav_msgs::OccupancyGrid> ptr = boost::make_shared<const nav_msgs::OccupancyGrid>(req.occupancy_map);
        createGraph(ptr);

        // 生成Voronoi的ROS消息
        tuw_multi_robot_msgs::Graph graph;
        //设置 graph 消息的头信息，包括坐标帧 ID、序列号和时间戳
        graph.header.frame_id = "map";
        graph.header.seq = 0;
        graph.header.stamp = ros::Time::now();

        //设置图的原点坐标
        graph.origin.position.x = origin_[0]; //TODO test
        graph.origin.position.y = origin_[1]; //TODO test

        for (auto it = segments_.begin(); it != segments_.end(); ++it)
        {   
            //创建 tuw_multi_robot_msgs::Graph 消息类型的实例 graph，该类型用于发布图数据
            tuw_multi_robot_msgs::Vertex seg;

            seg.id = (*it).getId();
            seg.weight = (*it).getLength();
            seg.width = (*it).getMinPathSpace() * resolution_;
            seg.valid = true;
            std::vector<Eigen::Vector2d> path = (*it).getPath();

            for (uint32_t i = 0; i < path.size(); i++)
            {
                //将每个路径点从地图坐标转换为 ROS 坐标（乘以分辨率），并将它们添加到段的路径中
                geometry_msgs::Point pos;
                pos.x = path[i][0] * resolution_;
                pos.y = path[i][1] * resolution_;
                pos.z = 0;
                //将转换后的路径点添加到 seg 的路径列表中
                seg.path.push_back(pos);
            }

            //ROS_INFO("distORIG: %i/%i", (*it)->GetPredecessors().size(), (*it)->GetSuccessors().size());
            //获取段的前驱节点列表
            std::vector<uint32_t> predecessors = (*it).getPredecessors();

            //将前驱节点 ID 添加到 seg 的前驱列表中
            for (uint32_t i = 0; i < predecessors.size(); i++)
            {
                seg.predecessors.push_back(predecessors[i]);
            }

            //获取段的后继节点列表
            std::vector<uint32_t> successors = (*it).getSuccessors();

            //将后继节点 ID 添加到 seg 的后继列表中
            for (uint32_t i = 0; i < successors.size(); i++)
            {
                seg.successors.push_back(successors[i]);
            }
            //将 seg 添加到 graph 的顶点列表中
            graph.vertices.push_back(seg);
        }

        // 服务器返回函数
        res.voronoi_map = graph;

        pubSegments_.publish(graph);

        return true;
    }

    //生成一个基于给定地图的 Voronoi 图
    void VoronoiGeneratorNode::createGraph(const nav_msgs::OccupancyGrid::ConstPtr &_map)
    //定义 createGraph 函数，它接收两个参数：一个指向 nav_msgs::OccupancyGrid 消息的智能指针和一个地图哈希值
    {
        //将地图消息的数据成员 data 复制到 std::vector<signed char> 类型的向量 map 中
        std::vector<signed char> map = _map->data;
        //清除之前存储的段（segments_）并重置段的 ID
        segments_.clear();
        Segment::resetId();

        ROS_INFO("Graph generator: Computing distance field ...");
        //从地图信息中提取原点坐标和分辨率，并存储在相应的成员变量中
        origin_[0] = _map->info.origin.position.x;
        origin_[1] = _map->info.origin.position.y;
        resolution_ = _map->info.resolution;

        //创建一个 cv::Mat 对象 m，它是 OpenCV 库中用于图像处理的矩阵类型。这里，它被初始化为与地图大小相同的矩阵，并使用地图数据
        cv::Mat m(_map->info.height, _map->info.width, CV_8SC1, map.data());
        //调用 prepareMap 函数准备地图数据，可能包括地图膨胀等操作
        prepareMap(m, map_, inflation_ / _map->info.resolution);
        //计算距离场，这是从地图中每个障碍物点到最近开放空间的距离
        computeDistanceField(map_, distField_);
        // std::cerr << "distField: " << distField_ << std::endl;

        ROS_INFO("Graph generator: Computing voronoi graph ...");
        computeVoronoiMap(distField_, voronoiMap_);

        ROS_INFO("Graph generator: Generating graph ...");
        //为势能图分配内存
        potential.reset(new float[m.cols * m.rows]);
        //计算每个像素的路径长度
        float pixel_path_length = segment_length_ / resolution_;
        //调用 calcSegments 函数计算段，这可能涉及根据 Voronoi 图和势能图生成导航路径
        segments_ = calcSegments(m, distField_, voronoiMap_, potential.get(), pixel_path_length,
                        crossingOptimization_ / resolution_, endSegmentOptimization_ / resolution_);

        //Check Directroy
        ROS_INFO("Graph generator: Created new Graph");
    }

    bool VoronoiGeneratorNode::loadGraph(std::size_t _hash)
    {
        segments_.clear();
        Segment::resetId();
        return load(graphCachePath_ + std::to_string(_hash) + "/", segments_, origin_, resolution_, map_);
    }

    bool VoronoiGeneratorNode::loadCustomGraph(std::string _path)
    {
        segments_.clear();
        Segment::resetId();
        return load(_path, segments_, origin_, resolution_);
    }

    //将计算得到的 Voronoi 图段发布到 ROS 主题中
    void VoronoiGeneratorNode::publishSegments()
    {
        tuw_multi_robot_msgs::Graph graph;
        //设置 graph 消息的头信息，包括坐标帧 ID、序列号和时间戳
        graph.header.frame_id = "map";
        graph.header.seq = 0;
        graph.header.stamp = ros::Time::now();

        //设置图的原点坐标
        graph.origin.position.x = origin_[0]; //TODO test
        graph.origin.position.y = origin_[1]; //TODO test

        for (auto it = segments_.begin(); it != segments_.end(); ++it)
        {   
            //创建 tuw_multi_robot_msgs::Graph 消息类型的实例 graph，该类型用于发布图数据
            tuw_multi_robot_msgs::Vertex seg;

            seg.id = (*it).getId();
            seg.weight = (*it).getLength();
            seg.width = (*it).getMinPathSpace() * resolution_;
            seg.valid = true;
            std::vector<Eigen::Vector2d> path = (*it).getPath();

            for (uint32_t i = 0; i < path.size(); i++)
            {
                //将每个路径点从地图坐标转换为 ROS 坐标（乘以分辨率），并将它们添加到段的路径中
                geometry_msgs::Point pos;
                pos.x = path[i][0] * resolution_;
                pos.y = path[i][1] * resolution_;
                pos.z = 0;
                //将转换后的路径点添加到 seg 的路径列表中
                seg.path.push_back(pos);
            }

            //ROS_INFO("distORIG: %i/%i", (*it)->GetPredecessors().size(), (*it)->GetSuccessors().size());
            //获取段的前驱节点列表
            std::vector<uint32_t> predecessors = (*it).getPredecessors();

            //将前驱节点 ID 添加到 seg 的前驱列表中
            for (uint32_t i = 0; i < predecessors.size(); i++)
            {
                seg.predecessors.push_back(predecessors[i]);
            }

            //获取段的后继节点列表
            std::vector<uint32_t> successors = (*it).getSuccessors();

            //将后继节点 ID 添加到 seg 的后继列表中
            for (uint32_t i = 0; i < successors.size(); i++)
            {
                seg.successors.push_back(successors[i]);
            }

            graph.vertices.push_back(seg);
        }
        //将 seg 添加到 graph 的顶点列表中
        pubSegments_.publish(graph);
    }

} // namespace tuw_graph
