#include "point_process.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZI PointT;

//DoN(Difference of Normail)
void Point_process::filter()
{
        
        // Create a search tree, use KDTreee for non-organized data.
        pcl::search::Search<PointXYZI>::Ptr tree;
        if (points->isOrganized ())
        {
                tree.reset (new pcl::search::OrganizedNeighbor<PointXYZI> ());
        }
        else
        {
                tree.reset (new pcl::search::KdTree<PointXYZI> (false));
        }
        // Set the input pointcloud for the search tree
        tree->setInputCloud (points);

        //生成法线估计器（OMP是并行计算，忽略）
        pcl::NormalEstimationOMP<PointXYZI, PointNormal> ne;
        ne.setInputCloud (points);
        ne.setSearchMethod (tree);
        //设定法线方向（要做差，同向很重要）
        ne.setViewPoint (std::numeric_limits<float>::max (),    std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

        double scale1=1;
        //计算小尺度法线
        pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);
        ne.setRadiusSearch (scale1);
        ne.compute (*normals_small_scale);
        
        double scale2=3;
        //计算大尺度法线
        pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);
        ne.setRadiusSearch (scale2);
        ne.compute (*normals_large_scale);
        

        //生成DoN分割器
        pcl::DifferenceOfNormalsEstimation<PointXYZI, PointNormal, PointNormal> don;
        don.setInputCloud (points);
        don.setNormalScaleLarge (normals_large_scale);
        don.setNormalScaleSmall (normals_small_scale);

        //计算法线差
        PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
        copyPointCloud<PointXYZI, PointNormal>(*points, *doncloud);
        don.computeFeature (*doncloud);

        double threshold=0.5;
        //生成滤波条件：把法线差和阈值比
        pcl::ConditionOr<PointNormal>::Ptr range_cond (
        new pcl::ConditionOr<PointNormal> ()
        );
        range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                                new pcl::FieldComparison<PointNormal> ("curvature", \
                                pcl::ComparisonOps::GT, threshold)));
        //生成条件滤波器,输入滤波条件和点云
        pcl::ConditionalRemoval<PointNormal> condrem(range_cond);
        condrem.setInputCloud (doncloud);

        //导出滤波结果
        pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);
        condrem.filter (*doncloud_filtered);
}

void Point_process::segmentGround(){
        //plane segmentation
        pcl::SACSegmentation<pcl::PointXYZI> plane_seg;
        pcl::PointIndices::Ptr plane_inliers ( new pcl::PointIndices );
        pcl::ModelCoefficients::Ptr plane_coefficients ( new pcl::ModelCoefficients );
        plane_seg.setOptimizeCoefficients (true);
        plane_seg.setModelType( pcl::SACMODEL_PLANE );
        plane_seg.setMethodType( pcl::SAC_RANSAC );
        plane_seg.setDistanceThreshold ( 30 );
        plane_seg.setInputCloud ( points );
        plane_seg.segment( *plane_inliers, *plane_coefficients );

}

void Point_process::segmentPoints(){
        // cloud are input cloud, maxIterations and
        // distanceThreshold are hyperparameters
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(1);

        seg.setInputCloud(points);
        // the indices in inliers are the points in line
        seg.segment(*inliers, *coefficients);

}

void Point_process::cleanDownPoints(){
        pcl::PointCloud<pcl::PointXYZI>::iterator it, temp;
        pcl::PointCloud<pcl::PointXYZI> newPoints;
        for(it = points->begin(); it!=points->end();)
        {
                //it->z=0;
                if( !(it->y>0.2 && it->y<15 && fabs(2*it->x) < it->y) )
                {
                                it->x=it->y=it->z=0;
                                //newPoints.push_back(*it);
                                temp=it;
                                ++it;
                                //points->erase(temp);
                }
                else
                        ++it;
        }               
        //pcl::copyPointCloud(newPoints, *points);
        
}

void Point_process::removeOutliers()
{
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;   //创建滤波器对象
        sor.setInputCloud (points);             //设置待滤波的点云
        sor.setMeanK (30);                            //设置在进行统计时考虑的临近点个数
        sor.setStddevMulThresh (1.0);       //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
        sor.filter (*points);             //滤波结果存储到cloud_filtered

}

void Point_process::radiusOutlierRemoval()
{
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> pcFilter;  //创建滤波器对象
        pcFilter.setInputCloud(points);             //设置待滤波的点云
        pcFilter.setRadiusSearch(0.8);               // 设置搜索半径
        pcFilter.setMinNeighborsInRadius(50);      // 设置一个内点最少的邻居数目
        pcFilter.filter(*points);        //滤波结果存储到cloud_filtered

}

void Point_process::displayPoints()
{
        /*
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0.5, 0.5, 0.5);
        viewer->addPointCloud<pcl::PointXYZI>(points, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem(1.0);
        while (!viewer->wasStopped())
        {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        } */       
}

void Point_process::voxelFilter()
{
        typename pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(points);
        cout<<"Before "<<points->size()<<endl;
        voxel_filter.setLeafSize(0.3, 0.3, 0.3);   // 0.3*0.3*0.3 单位m
        //typename pcl::PointCloud<PointT>::Ptr ds_cloud(new pcl::PointCloud<PointT>);
        voxel_filter.filter(*points);
        cout<<" After "<<points->size();
}
