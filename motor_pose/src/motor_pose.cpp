#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //"pcl::fromROSMsg"

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

#include <obj_detect/bbox.h>
#include <obj_detect/bboxes.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTRGB;
typedef boost::shared_ptr<pcl::PointCloud<PointTRGB>> PointCloudTRGBPtr;

struct Center2D
{
    int x;
    int y;
};

struct Center3D
{
    float x;
    float y;
    float z;
};

struct Box2D
{
    int xmin;
    int ymin;
    int xmax;
    int ymax;
};

struct Motor
{
    std::string obj_class;
    float score;            //probability
    Box2D box_pixel;
    Center2D center_pixel;
    Center3D center_point;
    PointCloudTRGBPtr depth_cloud;
    PointCloudTRGBPtr rgb_cloud;
};

std::vector<Motor> motor_all{};

bool save_organized_cloud = true;
std::string file_path_cloud_organized = "organized_cloud_tmp.pcd";
pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);

ros::Publisher top3_cloud_pub, motor_cloud_pub;
sensor_msgs::PointCloud2 top3_clouds_msg;
sensor_msgs::PointCloud2 top3_clouds_stock_msg;

void estimate_motor_pose(pcl::PointCloud<PointTRGB>::Ptr motor, pcl::PointCloud<PointTRGB>::Ptr motor_no_plane)
{
    // load motor
    // pcl::io::loadPCDFile("/home/upup/Downloads/MOST2022/src/motor_pose/motor_new.pcd", *motor);
    cout << "Motor point size: " << motor->size() << endl;
    
    if(motor->size()>0)
    {   
        pcl::ModelCoefficients::Ptr coeff_plane(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inlier_plane(new pcl::PointIndices);
        pcl::SACSegmentation<PointTRGB> seg_plane;
        seg_plane.setOptimizeCoefficients(true);
        seg_plane.setModelType(pcl::SACMODEL_PLANE);
        seg_plane.setMethodType(pcl::SAC_RANSAC);
        seg_plane.setDistanceThreshold(0.003);
        seg_plane.setInputCloud(organized_cloud_ori);
        seg_plane.segment(*inlier_plane, *coeff_plane);

        // pcl::PointCloud<PointTRGB>::Ptr motor_no_plane(new pcl::PointCloud<PointTRGB>);
        pcl::PointCloud<PointTRGB>::Ptr removed(new pcl::PointCloud<PointTRGB>);
        for(int n = 0; n<motor->points.size(); n++)
        {
            PointTRGB pt = motor->points[n];
            if(abs(coeff_plane->values[0]*pt.x + coeff_plane->values[1]*pt.y +coeff_plane->values[2]*pt.z +coeff_plane->values[3])>0.015 )
                motor_no_plane->push_back(pt);
            else
                removed->push_back(pt);
        }
        cout << "motor_no_plane point size: " << motor_no_plane->size() << endl;

        //normal estimation
        pcl::PointCloud<pcl::Normal>::Ptr motor_normal(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointTRGB>::Ptr tree(new pcl::search::KdTree<PointTRGB>());
        pcl::NormalEstimation<PointTRGB, pcl::Normal> ne;
        ne.setSearchMethod(tree);
        ne.setInputCloud(motor_no_plane);
        ne.setKSearch(50);
        ne.compute(*motor_normal);

        // ransac cylinder to motor
        pcl::ModelCoefficients::Ptr coeff_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
        pcl::SACSegmentationFromNormals<PointTRGB, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.05);
        seg.setRadiusLimits(0, 0.1);
        seg.setInputCloud(motor_no_plane);
        seg.setInputNormals(motor_normal);
        seg.segment(*inliers_cylinder, *coeff_cylinder);
        //point_on_axis.x,y,z; axis_direction.x,y,z; radius
        std::cerr << "Cylinder coeff" << *coeff_cylinder << endl; 
        coeff_cylinder->values[6] = 0.0696/2.0;

        //centroid
        PointTRGB min_point, max_point, center_point;    
        pcl::getMinMax3D(*motor_no_plane, min_point, max_point);
        center_point.x = (min_point.x + max_point.x)/2;
        center_point.y = (min_point.y + max_point.y)/2;
        center_point.z = (min_point.z + max_point.z)/2;

        cout <<"Min (x, y, z) = " << min_point.x <<", "<< min_point.y <<", "<< min_point.z << endl
            <<"Max (x, y, z) = " << max_point.x <<", "<< max_point.y <<", "<< max_point.z << endl;
        cout <<"Center Point: " << center_point <<endl;
        coeff_cylinder->values[0] = center_point.x;
        coeff_cylinder->values[1] = center_point.y;
        coeff_cylinder->values[2] = center_point.z;//point_on_axis??????????? CHECK
            
        // pcl::ExtractIndices<PointTRGB> extract;
        // pcl::ExtractIndices<pcl::Normal> extract_normals;
        // extract.setInputCloud(motor_no_plane);
        // extract.setIndices(inliers_cylinder);
        // extract.setNegative(false);
        // pcl::PointCloud<PointTRGB>::Ptr cylinder(new pcl::PointCloud<PointTRGB>);
        // extract.filter(*cylinder);

        // pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer("motor viewer"));
        // view->removeAllPointClouds();
        // view->setBackgroundColor(0, 0, 0);
        // view->addCoordinateSystem(0.2f);

        // pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> cylinder_color(motor_no_plane, 0, 255, 0); // green
        // pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> motor_color(removed, 255, 0, 0); // green
        // view->addPointCloud<PointTRGB>(motor_no_plane, cylinder_color, "cylinder",0);
        // view->addPointCloud<PointTRGB>(removed, motor_color, "motor", 0);
    
        // view->addCylinder(*coeff_cylinder, "inliers");
        // int numsides = 15;
        // // view->addCylinderNew(*coeff_cylinder, numsides, "inliers");
        // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder");
        // view->spin();
    }
}

void motor_cb(const obj_detect::bboxes::ConstPtr& boxes_msg)
{
    //==================================================//
    // Subscribe "/obj_detect/bboxes" topic
    //==================================================//
    //cout << "\nyolo_callback\n";
    //cout << "Bouding Boxes (header):" << boxes_msg->header << endl;
    //cout << "Bouding Boxes (image_header):" << boxes_msg->image_header << endl;

    int obj_num = boxes_msg->bboxes.size();

    motor_all.resize(obj_num);

    for(int k = 0; k < obj_num; ++k)
    {
        std::string obj_class = boxes_msg->bboxes[k].object_name;
        float score = boxes_msg->bboxes[k].score;
        int xmin = boxes_msg->bboxes[k].xmin;
        int xmax = boxes_msg->bboxes[k].xmax;
        int ymin = boxes_msg->bboxes[k].ymin;
        int ymax = boxes_msg->bboxes[k].ymax;
        int center_x = int((xmin +xmax)/2.0);
        int center_y = int((ymin +ymax)/2.0);

        motor_all[k].obj_class = obj_class;
        motor_all[k].score = score;
        motor_all[k].box_pixel.xmin = xmin;
        motor_all[k].box_pixel.xmax = xmax;
        motor_all[k].box_pixel.ymin = ymin;
        motor_all[k].box_pixel.ymax = ymax;
        motor_all[k].center_pixel.x = center_x;
        motor_all[k].center_pixel.y = center_y;
    }

    if(boxes_msg->bboxes.empty())
        obj_num = 0;

    cout << "Total Motor clusters = " << obj_num << endl;   //ERROR: display 1 even if no obj detected
}

void motor_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
{
    int height = organized_cloud_msg->height;
    int width = organized_cloud_msg->width;
    int points = height * width;

    cout<<"(height, width) = "<<height<<", "<<width<<endl;
    if((points==0))// && (save_organized_cloud ==true))
    {
        cout<<"PointCloud No points!!!!!!\n";
        //break? pass?
    }
    else
    {
        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        organized_cloud_ori->clear();
        pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);
        //========================================//
        // Go over all EVERY Yolov4 detected sauces
        // save 2D, 3D sauce information
        //========================================//
        for(int n = 0; n < motor_all.size(); ++n)
        {
            cout << "Sauce #" << n << endl;

            //=========================================//
            // Extract Sauce's Depth Cloud(Orgainized)
            // 2D pixel mapping to 3D points
            //=========================================//
            motor_all[n].depth_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();
            int xmin = motor_all[n].box_pixel.xmin;
            int xmax = motor_all[n].box_pixel.xmax;
            int ymin = motor_all[n].box_pixel.ymin;
            int ymax = motor_all[n].box_pixel.ymax;

            // //Ensure the 2D pixels are inside image's max width, height
            // if(xmin < 0) xmin = 114;//186;//0;
            // if(ymin < 0) ymin = 40;//74;//0;
            // if(xmax > img_width-1) xmax = 723;//1085;//img_width-1;
            // if(ymax > img_height-1) ymax = 424;//648;//img_height-1;
            // cout<<"\timgwidth, imgHeight = "<< img_width <<",  "<< img_height<<endl;
            cout<< "\tPixel (xmin, xmax, ymin, ymax) = "<< xmin << ", " << xmax <<", " << ymin << ", " << ymax << endl;

            //Map 2D pixel to 3D points
            int x_shift = 370;  //!!!!Change according to select_workspace
            int y_shift = 150;  //!!!!Change according to select_workspace
            for(int i = xmin; i <= xmax; i++)
            {
                for(int j = ymin; j<= ymax; j++)
                {
                    PointTRGB depth_pt = organized_cloud_ori->at(x_shift + i, y_shift + j);
                    if(pcl_isfinite(depth_pt.x) && pcl_isfinite(depth_pt.y) && pcl_isfinite(depth_pt.z))
                    {
                        motor_all[n].depth_cloud->push_back(depth_pt);
                    }
                }
            }
            cout << "\tExtract [depth_cloud] = " << motor_all[n].depth_cloud->size() << endl;
            pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_organized, *motor_all[n].depth_cloud); //savePCDFileASCII
        }

        //========================================//
        // Visualize highest sauce on rgb_cloud
        //========================================//

        //Display Top 3 Sauces via Rviz
        pcl::PointCloud<PointTRGB>::Ptr top3_clouds_stock(new pcl::PointCloud<PointTRGB>);

        for(int idx = 0; idx < motor_all.size(); ++idx)
        {
            *top3_clouds_stock = *top3_clouds_stock + *(motor_all[idx].depth_cloud);
        }


        pcl::PointCloud<PointTRGB>::Ptr motor_clear(new pcl::PointCloud<PointTRGB>);
        estimate_motor_pose(top3_clouds_stock, motor_clear);

        // Eigen::Vector3d vv1 = Eigen::Vector3d(vect_x[0], vect_x[1], vect_x[2]);
        // Eigen::Vector3d vv2 = Eigen::Vector3d(line_c1_c2_x, line_c1_c2_y, line_c1_c2_z);
        // Eigen::Quaterniond out = Eigen::Quaterniond::FromTwoVectors(vv1, vv2);

        // //=========rviz marker=========
        // Eigen::Quaterniond AQ;
        // visualization_msgs::Marker popcorn_arrow;
        // popcorn_arrow.header.frame_id = "camera_color_optical_frame";
        // popcorn_arrow.header.stamp = ros::Time();
        // popcorn_arrow.ns = "my_namespace";
        // popcorn_arrow.id = 0;
        // popcorn_arrow.type = visualization_msgs::Marker::ARROW;
        // popcorn_arrow.action = visualization_msgs::Marker::ADD;
        // popcorn_arrow.pose.position.x = pt_c1.x;
        // popcorn_arrow.pose.position.y = pt_c1.y;
        // popcorn_arrow.pose.position.z = pt_c1.z;
        // popcorn_arrow.pose.orientation.x = out.x();//pose_msg.orientation.x;
        // popcorn_arrow.pose.orientation.y = out.y();//pose_msg.orientation.y;
        // popcorn_arrow.pose.orientation.z = out.z();//pose_msg.orientation.z;
        // popcorn_arrow.pose.orientation.w = out.w();//pose_msg.orientation.w;
        // popcorn_arrow.scale.x = sqrt(pow(line_c1_c2_x,2)+pow(line_c1_c2_y,2)+pow(line_c1_c2_z,2)); //length
        // popcorn_arrow.scale.y = 0.003;  //width
        // popcorn_arrow.scale.z = 0.003;  //height
        // popcorn_arrow.color.a = 1.0;    // Don't forget to set the alpha!
        // popcorn_arrow.color.r = 0.0;
        // popcorn_arrow.color.g = 0.0;
        // popcorn_arrow.color.b = 1.0;

        // cout<< "popcorn_pose_topic rviz marker" <<endl;

        // popcorn_pose_pub.publish(popcorn_arrow);
        // //=========rviz marker=========

        //Publish pcl::PointCloud to ROS sensor::PointCloud2, and to topic
        pcl::toROSMsg(*motor_clear, top3_clouds_stock_msg);
        top3_clouds_stock_msg.header.frame_id = "camera_depth_optical_frame";
        motor_cloud_pub.publish(top3_clouds_stock_msg);
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_pose");
    cout << "motor_pose code\n";

    ros::NodeHandle nh;
    ros::Subscriber sub_yolo = nh.subscribe("/yolov4_motors_bboxes", 1, motor_cb);
    ros::Subscriber sub_motor_cloud = nh.subscribe("/camera/depth_registered/points", 1, motor_cloud_cb);

    motor_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/motor_cloud_pub", 1);
    ros::spin();

    return 0;
}


// int main()
// {
//     pcl::PointCloud<PointT>::Ptr motor(new pcl::PointCloud<PointT>());
//     pcl::PointCloud<pcl::Normal>::Ptr motor_normal(new pcl::PointCloud<pcl::Normal>);

//     // load motor
//     pcl::io::loadPCDFile("/home/upup/Downloads/MOST2022/src/motor_pose/motor_new.pcd", *motor);
//     cout << "Motor point size: " << motor->size() << endl;

//     //normal estimation
//     pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//     pcl::NormalEstimation<PointT, pcl::Normal> ne;
//     ne.setSearchMethod(tree);
//     ne.setInputCloud(motor);
//     ne.setKSearch(50);
//     ne.compute(*motor_normal);

//     // ransac cylinder to motor
//     pcl::ModelCoefficients::Ptr coeff_cylinder (new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
//     pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_CYLINDER);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setNormalDistanceWeight(0.1);
//     seg.setMaxIterations(10000);
//     seg.setDistanceThreshold(0.05);
//     seg.setRadiusLimits(0, 0.1);
//     seg.setInputCloud(motor);
//     seg.setInputNormals(motor_normal);
//     seg.segment(*inliers_cylinder, *coeff_cylinder);
//     //point_on_axis.x,y,z; axis_direction.x,y,z; radius
//     std::cerr << "Cylinder coeff" << *coeff_cylinder << endl;
//     coeff_cylinder->values[6] = 0.0696/2.0;

//     //centroid
//     PointT min_point, max_point, center_point;
//     pcl::getMinMax3D(*motor, min_point, max_point);
//     center_point.x = (min_point.x + max_point.x)/2;
//     center_point.y = (min_point.y + max_point.y)/2;
//     center_point.z = (min_point.z + max_point.z)/2;

//     cout <<"Min (x, y, z) = " << min_point.x <<", "<< min_point.y <<", "<< min_point.z << endl
//          <<"Max (x, y, z) = " << max_point.x <<", "<< max_point.y <<", "<< max_point.z << endl;
//     cout <<"Center Point: " << center_point <<endl;
//     coeff_cylinder->values[0] = center_point.x;
//     coeff_cylinder->values[1] = center_point.y;
//     coeff_cylinder->values[2] = center_point.z;//point_on_axis??????????? CHECK




//     pcl::ExtractIndices<PointT> extract;
//     pcl::ExtractIndices<pcl::Normal> extract_normals;
//     extract.setInputCloud(motor);
//     extract.setIndices(inliers_cylinder);
//     extract.setNegative(false);
//     pcl::PointCloud<PointT>::Ptr cylinder(new pcl::PointCloud<PointT>);
//     extract.filter(*cylinder);


//     pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer("motor viewer"));
//     view->removeAllPointClouds();
//     view->setBackgroundColor(0, 0, 0);
//     view->addCoordinateSystem(0.2f);

//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cylinder_color(cylinder, 0, 255, 0); // green
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> motor_color(motor, 255, 0, 0); // green
//     view->addPointCloud<PointT>(cylinder, cylinder_color, "cylinder",0);
//     view->addPointCloud<PointT>(motor, motor_color, "motor", 0);

//     view->addCylinder(*coeff_cylinder, "inliers");
//     int numsides = 15;
//     // view->addCylinderNew(*coeff_cylinder, numsides, "inliers");
//     view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder");
//     view->spin();

//     return(0);
// }

// //https://blog.csdn.net/l_h2010/article/details/41117053
// //https://github.com/PointCloudLibrary/pcl/edit/master/visualization/include/pcl/visualization/pcl_visualizer.h
// //https://github.com/PointCloudLibrary/pcl/edit/master/visualization/src/pcl_visualizer.cpp