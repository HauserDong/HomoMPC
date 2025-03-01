#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_map/thinning.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
#include <string>

using namespace cv;

namespace voronoi_map
{

    VoronoiPathGenerator::VoronoiPathGenerator()
    {

    }

    void VoronoiPathGenerator::prepareMap(const Mat& _map, Mat& _des, int erodeSize)
    {
        //定义一个静态局部变量 srcMap，用于存储转换后的地图数据。将 _map 转换为单通道、无符号字符类型的矩阵 srcMap
        static Mat srcMap;
        _map.convertTo(srcMap, CV_8UC1);

        //遍历 srcMap 中的每个元素，如果原始地图 _map 中的相应元素小于 0，则将其设置为 100
        for(int i = 0; i < srcMap.cols * srcMap.rows; i++)
        {
            if((signed char)_map.data[i] < 0)  srcMap.data[i] = 100;
        }

        //将 srcMap 的内容复制到输出参数 _des 中
        _des = srcMap;

        //对 srcMap 执行按位取反操作，这通常用于将地图中的障碍物（通常是 0 或非零值）转换为背景（通常是 255）
        cv::bitwise_not(srcMap, srcMap);
        //使用 Otsu 的全局阈值方法对 srcMap 进行二值化，阈值设置为 10，最大值为 255。Otsu 方法是一种自动阈值选择方法，用于将图像分割成两个类别
        cv::threshold(srcMap, _des, 10, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        
        //如果 erodeSize 不大于 0，则将其设置为 1。这是为了确保后续的腐蚀操作有一个有效的结构元素大小
        if(erodeSize <= 0){
            erodeSize = 1;
        }
        if(erodeSize > 0){
            //如果 erodeSize 大于 0，则创建一个椭圆形的结构元素，用于后续的腐蚀操作
            cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                        cv::Size( 2*erodeSize + 1, 2*erodeSize+1 ),
                                        cv::Point( erodeSize, erodeSize ) );
            //使用上面创建的结构元素对 _des 执行腐蚀操作。腐蚀操作会减少地图中的白色区域（通常是自由空间），增加黑色区域（通常是障碍物）
            cv::erode(_des, _des, element);
        }
    }


    void VoronoiPathGenerator::computeDistanceField(const cv::Mat& _map, cv::Mat& _distField)
    {
        cv::distanceTransform(_map, _distField, cv::DIST_L2, 3);
    }

    void VoronoiPathGenerator::computeVoronoiMap(const cv::Mat& _distField, cv::Mat& _voronoiMap)
    {
        Mat srcMap = _distField;
        srcMap.convertTo(_voronoiMap, CV_8UC1, 0.0);

        voronoi_map::greyscale_thinning(srcMap, _voronoiMap);
        cv::threshold(_voronoiMap, _voronoiMap, 1, 255, cv::THRESH_BINARY);
        voronoi_map::sceletonize(_voronoiMap, _voronoiMap);
    }
}
