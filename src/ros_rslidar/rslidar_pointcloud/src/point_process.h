#ifndef POINT_PROCESS
#define POINT_PROCESS

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件

using namespace std;


class Point_process{
private:
        pcl::PointCloud<pcl::PointXYZI>::Ptr points;


public:
        Point_process():points(NULL){}
        Point_process(pcl::PointCloud<pcl::PointXYZI>::Ptr &points){
                this->points=points;
        }


        void segmentGround();
        void segmentPoints(); //RANSAC算法
        void filter();
        void cleanDownPoints();
        void removeOutliers();
        void radiusOutlierRemoval();
        void displayPoints();
        void voxelFilter(); //体素滤波 下采样


};



#endif