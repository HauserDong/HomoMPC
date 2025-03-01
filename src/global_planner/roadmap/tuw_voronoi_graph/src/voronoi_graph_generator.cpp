/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tuw_voronoi_graph/voronoi_graph_generator.h>


namespace tuw_graph
{   
    //定义了 VoronoiGraphGenerator
    VoronoiGraphGenerator::VoronoiGraphGenerator()
    {

    }

    std::vector<Segment> VoronoiGraphGenerator::calcSegments(
        cv::Mat &_map, cv::Mat &_distField, cv::Mat &_voronoiPath,
        float* potential, float _path_length, float _optimizeCrossingPixels,
        float _optimizeEndSegmentsPixel)
    //定义了 VoronoiGraphGenerator 类的 calcSegments 成员函数，它接收多个参数：
    //_map：原始地图的 OpenCV cv::Mat 对象引用。
    //_distField：距离场的 OpenCV cv::Mat 对象引用。
    //_voronoiPath：Voronoi 图的路径矩阵。
    //potential：势能数组的指针。
    //_path_length：路径长度参数。
    //_optimizeCrossingPixels 和 _optimizeEndSegmentsPixel：优化交叉点和末端段的参数。
    {
        Segment_Expander exp;
        exp.Initialize(_map, _distField, _voronoiPath);
        std::vector<std::vector<Eigen::Vector2d>> points = exp.calcEndpoints(potential);
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> segments;

        int nx = _map.cols;
        int ny = _map.rows;

        exp.Initialize(_map, _distField, _voronoiPath);
        std::fill(potential, potential + nx * ny, -1);

        //调用 Segment_Expander 的 getGraph 方法来生成图的段，并存储在 segs 向量中
        std::vector<Segment> segs = exp.getGraph(points, potential, _path_length,
                            _optimizeCrossingPixels, _optimizeEndSegmentsPixel);

        for(uint32_t i = 0; i < segs.size(); i++)
        {
            std::vector<uint32_t> predecessors = segs[i].getPredecessors();
            std::vector<uint32_t> successors = segs[i].getSuccessors();
        }

        return std::vector<Segment>(segs);
    }
}
