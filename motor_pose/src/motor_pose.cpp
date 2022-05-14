#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include "pcl_visualizer_modify.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/common/shapes.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>

using namespace std;

typedef pcl::PointXYZ PointT;

int main()
{
    pcl::PointCloud<PointT>::Ptr motor(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr motor_normal(new pcl::PointCloud<pcl::Normal>);

    // load motor
    pcl::io::loadPCDFile("/home/upup/Downloads/MOST2022/src/motor_pose/motor_new.pcd", *motor);
    cout << "Motor point size: " << motor->size() << endl;

    //normal estimation
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(motor);
    ne.setKSearch(50);
    ne.compute(*motor_normal);

    // ransac cylinder to motor
    pcl::ModelCoefficients::Ptr coeff_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud(motor);
    seg.setInputNormals(motor_normal);
    seg.segment(*inliers_cylinder, *coeff_cylinder);
    //point_on_axis.x,y,z; axis_direction.x,y,z; radius
    std::cerr << "Cylinder coeff" << *coeff_cylinder << endl; 
    coeff_cylinder->values[6] = 0.0696/2.0;

    //centroid
    PointT min_point, max_point, center_point;    
    pcl::getMinMax3D(*motor, min_point, max_point);
    center_point.x = (min_point.x + max_point.x)/2;
    center_point.y = (min_point.y + max_point.y)/2;
    center_point.z = (min_point.z + max_point.z)/2;

    cout <<"Min (x, y, z) = " << min_point.x <<", "<< min_point.y <<", "<< min_point.z << endl
         <<"Max (x, y, z) = " << max_point.x <<", "<< max_point.y <<", "<< max_point.z << endl;
    cout <<"Center Point: " << center_point <<endl;
    coeff_cylinder->values[0] = center_point.x;
    coeff_cylinder->values[1] = center_point.y;
    coeff_cylinder->values[2] = center_point.z;//point_on_axis??????????? CHECK
    


    
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract.setInputCloud(motor);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cylinder(new pcl::PointCloud<PointT>);
    extract.filter(*cylinder);


    pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer("motor viewer"));
    view->removeAllPointClouds();
    view->setBackgroundColor(0, 0, 0);
    view->addCoordinateSystem(0.2f);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cylinder_color(cylinder, 0, 255, 0); // green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> motor_color(motor, 255, 0, 0); // green
    view->addPointCloud<PointT>(cylinder, cylinder_color, "cylinder",0);
    view->addPointCloud<PointT>(motor, motor_color, "motor", 0);
 
    view->addCylinder(*coeff_cylinder, "inliers");
    int numsides = 15;
    // view->addCylinderNew(*coeff_cylinder, numsides, "inliers");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder");
    view->spin();

    return(0);
}

//https://blog.csdn.net/l_h2010/article/details/41117053
//https://github.com/PointCloudLibrary/pcl/edit/master/visualization/include/pcl/visualization/pcl_visualizer.h
//https://github.com/PointCloudLibrary/pcl/edit/master/visualization/src/pcl_visualizer.cpp