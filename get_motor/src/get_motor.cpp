#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
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

#include "assembly_srv/GraspPose.h"


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

typedef pcl::PointXYZRGB PointTRGB;
typedef boost::shared_ptr<pcl::PointCloud<PointTRGB>> PointCloudTRGBPtr;

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
    Box2D box_pixel;
    Center3D center_point;
    PointCloudTRGBPtr motor_bbox_cloud;
    PointCloudTRGBPtr motor_cloud;
};

std::vector<Motor> motor_all{};

std::string file_path_cloud_organized = "organized_cloud_tmp.pcd";
pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);

ros::Publisher motor_cloud_pub, motor_pose_pub, motor_grasp_pose_pub;//, motor_grasp_pose_pub;
sensor_msgs::PointCloud2 motor_cloud_msg;
std_msgs::Float32MultiArray grasp_msg; 
void estimate_motor_pose(pcl::PointCloud<PointTRGB>::Ptr motor_bbox_cloud, pcl::ModelCoefficients::Ptr coeff_scene_plane, pcl::PointCloud<PointTRGB>::Ptr motor_cloud, pcl::ModelCoefficients::Ptr coeff_cylinder)
{
    if(motor_bbox_cloud->size()>0)
    {
        float plane_distance_thr = 0.02;
        pcl::PointCloud<PointTRGB>::Ptr removed(new pcl::PointCloud<PointTRGB>);

        for(int n = 0; n < motor_bbox_cloud->points.size(); n++)
        {
            PointTRGB pt = motor_bbox_cloud->points[n];

            if(abs(coeff_scene_plane->values[0]*pt.x + coeff_scene_plane->values[1]*pt.y + coeff_scene_plane->values[2]*pt.z + coeff_scene_plane->values[3]) > plane_distance_thr)
                motor_cloud->push_back(pt);
            else
                removed->push_back(pt);
        }

        //normal estimation
        pcl::PointCloud<pcl::Normal>::Ptr motor_normal(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointTRGB>::Ptr tree(new pcl::search::KdTree<PointTRGB>());
        pcl::NormalEstimation<PointTRGB, pcl::Normal> ne;
        ne.setSearchMethod(tree);
        ne.setInputCloud(motor_cloud);
        ne.setKSearch(50);
        ne.compute(*motor_normal);

        // ransac cylinder to motor
        // pcl::ModelCoefficients::Ptr coeff_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
        pcl::SACSegmentationFromNormals<PointTRGB, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.05);
        seg.setRadiusLimits(0, 0.1);
        seg.setInputCloud(motor_cloud);
        seg.setInputNormals(motor_normal);
        seg.segment(*inliers_cylinder, *coeff_cylinder);
        std::cerr << "Cylinder coeff: " //<< *coeff_cylinder << endl;
                    << "\n\tPoint_on_axis.x = " << coeff_cylinder->values[0]
                    << "\n\tPoint_on_axis.y = " << coeff_cylinder->values[1]
                    << "\n\tPoint_on_axis.z = " << coeff_cylinder->values[2]
                    << "\n\tAxis_direction.x = " << coeff_cylinder->values[3]
                    << "\n\tAxis_direction.y = " << coeff_cylinder->values[4]
                    << "\n\tAxis_direction.z = " << coeff_cylinder->values[5]
                    << "\n\tCylinder Radius = " << coeff_cylinder->values[6]
                    << endl;

        //centroid
        PointTRGB min_point, max_point, center_point;    
        pcl::getMinMax3D(*motor_cloud, min_point, max_point);
        center_point.x = (min_point.x + max_point.x)/2;
        center_point.y = (min_point.y + max_point.y)/2;
        center_point.z = (min_point.z + max_point.z)/2;

        cout <<"Min (x, y, z) = " << min_point.x <<", "<< min_point.y <<", "<< min_point.z << endl
            <<"Max (x, y, z) = " << max_point.x <<", "<< max_point.y <<", "<< max_point.z << endl;
        cout <<"Center Point: " << center_point <<endl;
        coeff_cylinder->values[0] = center_point.x;
        coeff_cylinder->values[1] = center_point.y;
        coeff_cylinder->values[2] = center_point.z;//point_on_axis??????????? CHECK
            
        // // === Use PCL_VISUALIZER to check Segmentation Result ===//
        // pcl::PointCloud<PointTRGB>::Ptr motor_cylinder(new pcl::PointCloud<PointTRGB>);
        // pcl::ExtractIndices<PointTRGB> extract;
        // extract.setInputCloud(motor_cloud);
        // extract.setIndices(inliers_cylinder);
        // extract.setNegative(false);
        // extract.filter(*motor_cylinder);

        // pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer("motor viewer"));
        // view->removeAllPointClouds();
        // view->setBackgroundColor(0, 0, 0);
        // view->addCoordinateSystem(0.02f);

        // pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> motor_color(motor_cloud, 255, 0, 0);
        // pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> removed_color(removed, 0, 255, 0);
        // view->addPointCloud<PointTRGB>(motor_cloud, motor_color, "motor_cloud",0);
        // view->addPointCloud<PointTRGB>(removed, removed_color, "removed", 0);

        // view->addCylinder(*coeff_cylinder, "inliers");
        // int numsides = 15;
        // // view->addCylinderNew(*coeff_cylinder, numsides, "inliers");
        // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "motor_cloud");
        // view->spin();
        // // === Use PCL_VISUALIZER to check Segmentation Result ===//
    }
}

void estimate_scene_plane_coeff(pcl::PointCloud<PointTRGB>::Ptr scene, pcl::ModelCoefficients::Ptr coeff_plane)
{
    pcl::PointCloud<PointTRGB>::Ptr plane(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr remains(new pcl::PointCloud<PointTRGB>);
    pcl::PointIndices::Ptr inlier_plane(new pcl::PointIndices);

    pcl::SACSegmentation<PointTRGB> seg_plane;
    seg_plane.setOptimizeCoefficients(true);
    seg_plane.setModelType(pcl::SACMODEL_PLANE);
    seg_plane.setMethodType(pcl::SAC_RANSAC);
    seg_plane.setDistanceThreshold(0.003);
    seg_plane.setInputCloud(scene);
    seg_plane.segment(*inlier_plane, *coeff_plane);

    // // === Use PCL_VISUALIZER to check Segmentation Result ===//
    // pcl::ExtractIndices<PointTRGB> extract;
    // extract.setInputCloud(scene);
    // extract.setIndices(inlier_plane);
    // extract.setNegative(false);
    // extract.filter(*plane);
    // extract.setNegative(true);
    // extract.filter(*remains);

    // pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer("scene plane viewer"));
    // view->removeAllPointClouds();
    // view->setBackgroundColor(0, 0, 0);
    // view->addCoordinateSystem(0.02f);

    // pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> plane_color(plane, 0, 255, 0); // green
    // pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> remains_color(remains, 255, 0, 0); // green
    // view->addPointCloud<PointTRGB>(plane, plane_color, "plane",0);
    // view->addPointCloud<PointTRGB>(remains, remains_color, "remains", 0);
    // view->addPlane(*coeff_plane, "inliers");
    // view->spin();
    // // === Use PCL_VISUALIZER to check Segmentation Result ===//
}

void motor_cb(const obj_detect::bboxes::ConstPtr& boxes_msg)
{
    //==================================================//
    // Subscribe "/yolov4_motors_bboxes" topic
    //==================================================//
    int total_motor_num = boxes_msg->bboxes.size();

    if(!boxes_msg->bboxes.empty())
    {
        motor_all.resize(total_motor_num);

        for(int k = 0; k < total_motor_num; ++k)
        {
            motor_all[k].box_pixel.xmin = boxes_msg->bboxes[k].xmin;
            motor_all[k].box_pixel.xmax = boxes_msg->bboxes[k].xmax;
            motor_all[k].box_pixel.ymin = boxes_msg->bboxes[k].ymin;
            motor_all[k].box_pixel.ymax = boxes_msg->bboxes[k].ymax;
        }
    }

    cout << "Total Motor clusters = " << total_motor_num << endl;
}

void motor_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
{
    int height = organized_cloud_msg->height;
    int width = organized_cloud_msg->width;
    int total_points = height * width;

    cout << "ori scene cloud (height, width) = " << height << ", " << width << endl;
    if(total_points == 0)
    {
        cout << "ori scene cloud No POINTS!!!!!!\n";
    }
    else
    {
        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        organized_cloud_ori->clear();
        pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);

        //========================================//
        // Estimate Scene Cloud Plane Coeff
        //========================================//
        pcl::ModelCoefficients::Ptr coeff_plane(new pcl::ModelCoefficients);
        estimate_scene_plane_coeff(organized_cloud_ori, coeff_plane);

        //========================================//
        // Go over all EVERY Yolov4 detected motors
        // save 3D motor information
        //========================================//
        for(int n = 0; n < motor_all.size(); ++n)
        {
            cout << "Motor #" << n << endl;

            //=========================================//
            // Extract Motors' Cloud from Scene Orgainized Cloud
            // 2D pixel mapping to 3D points
            //=========================================//
            motor_all[n].motor_bbox_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();

            int xmin = motor_all[n].box_pixel.xmin;
            int xmax = motor_all[n].box_pixel.xmax;
            int ymin = motor_all[n].box_pixel.ymin;
            int ymax = motor_all[n].box_pixel.ymax;

            cout<< "\tBBox Pixel (xmin, xmax, ymin, ymax) = " << xmin << ", " << xmax <<", " << ymin << ", " << ymax << endl;

            //Map 2D pixel to 3D points
            int x_shift = 370;  //!!!!Change according to select_workspace
            int y_shift = 150;  //!!!!Change according to select_workspace
            for(int i = xmin; i <= xmax; i++)
            {
                for(int j = ymin; j<= ymax; j++)
                {
                    PointTRGB rgb_pt = organized_cloud_ori->at(x_shift + i, y_shift + j);
                    if(pcl_isfinite(rgb_pt.x) && pcl_isfinite(rgb_pt.y) && pcl_isfinite(rgb_pt.z))
                    {
                        motor_all[n].motor_bbox_cloud->push_back(rgb_pt);
                    }
                }
            }
            cout << "\tExtract [motor_bbox_cloud] = " << motor_all[n].motor_bbox_cloud->size() << endl << endl;
            // pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_organized, *motor_all[n].motor_bbox_cloud);

            //===========================================//
            // Remove Scene Plane & Estimate Motor Pose
            //===========================================//
            motor_all[n].motor_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();
            pcl::ModelCoefficients::Ptr coeff_cylinder (new pcl::ModelCoefficients);
            estimate_motor_pose(motor_all[n].motor_bbox_cloud, coeff_plane, motor_all[n].motor_cloud, coeff_cylinder);
            ////point_on_axis.x,y,z; axis_direction.x,y,z; radius
            // std::cerr << "Cylinder coeff" << *coeff_cylinder << endl;

            Eigen::Vector3d vv1 = Eigen::Vector3d(1.0, 0.0, 0.0);
            Eigen::Vector3d vv2 = Eigen::Vector3d(coeff_cylinder->values[3], coeff_cylinder->values[4], coeff_cylinder->values[5]);
            vv2.normalize();
            Eigen::Quaterniond out = Eigen::Quaterniond::FromTwoVectors(vv1, vv2);
            Eigen::Vector3d euler = out.toRotationMatrix().eulerAngles(2, 1, 0);
            float yaw = euler[0];
            float pitch = euler[1];
            float roll = euler[2];

            Eigen::Vector3d vv3 = Eigen::Vector3d(0.0, 0.0, 1.0);
            Eigen::Quaterniond out1 = Eigen::Quaterniond::FromTwoVectors(vv3, vv2);
            Eigen::Vector3d euler1 = out1.toRotationMatrix().eulerAngles(2, 1, 0);
            
            //https://danceswithcode.net/engineeringnotes/rotations_in_3d/demo3D/rotations_in_3d_tool.html

            //=========rviz marker=========
            visualization_msgs::Marker motor_arrow;
            motor_arrow.header.frame_id = "camera_depth_optical_frame";
            motor_arrow.header.stamp = ros::Time();
            motor_arrow.ns = "my_namespace";
            motor_arrow.id = 0;
            motor_arrow.type = visualization_msgs::Marker::ARROW;
            motor_arrow.action = visualization_msgs::Marker::ADD;
            motor_arrow.pose.position.x = coeff_cylinder->values[0];
            motor_arrow.pose.position.y = coeff_cylinder->values[1];
            motor_arrow.pose.position.z = coeff_cylinder->values[2];
            motor_arrow.pose.orientation.x = out.x();//pose_msg.orientation.x;
            motor_arrow.pose.orientation.y = out.y();//pose_msg.orientation.y;
            motor_arrow.pose.orientation.z = out.z();//pose_msg.orientation.z;
            motor_arrow.pose.orientation.w = out.w();//pose_msg.orientation.w;
            motor_arrow.scale.x = 0.150;//sqrt(pow(coeff_cylinder->values[3],2)+pow(coeff_cylinder->values[4],2)+pow(coeff_cylinder->values[5],2)); //length
            motor_arrow.scale.y = 0.008;  //width
            motor_arrow.scale.z = 0.008;  //height
            motor_arrow.color.a = 1.0;    // Don't forget to set the alpha!
            motor_arrow.color.r = 1.0;
            motor_arrow.color.g = 0.0;
            motor_arrow.color.b = 0.0;

            cout<< "motor_pose_topic rviz marker" <<endl;

            motor_pose_pub.publish(motor_arrow);
            //=========rviz marker=========

            //=========rviz marker=========
            visualization_msgs::Marker grasp_arrow;
            grasp_arrow.header.frame_id = "camera_depth_optical_frame";
            grasp_arrow.header.stamp = ros::Time();
            grasp_arrow.ns = "my_namespace";
            grasp_arrow.id = 0;
            grasp_arrow.type = visualization_msgs::Marker::ARROW;
            grasp_arrow.action = visualization_msgs::Marker::ADD;
            grasp_arrow.pose.position.x = coeff_cylinder->values[0];
            grasp_arrow.pose.position.y = coeff_cylinder->values[1];
            grasp_arrow.pose.position.z = coeff_cylinder->values[2];
            grasp_arrow.pose.orientation.x = out1.x();//pose_msg.orientation.x;
            grasp_arrow.pose.orientation.y = out1.y();//pose_msg.orientation.y;
            grasp_arrow.pose.orientation.z = out1.z();//pose_msg.orientation.z;
            grasp_arrow.pose.orientation.w = out1.w();//pose_msg.orientation.w;
            grasp_arrow.scale.x = 0.150;//sqrt(pow(coeff_cylinder->values[3],2)+pow(coeff_cylinder->values[4],2)+pow(coeff_cylinder->values[5],2)); //length
            grasp_arrow.scale.y = 0.008;  //width
            grasp_arrow.scale.z = 0.008;  //height
            grasp_arrow.color.a = 1.0;    // Don't forget to set the alpha!
            grasp_arrow.color.r = 0.0;
            grasp_arrow.color.g = 0.0;
            grasp_arrow.color.b = 1.0;

            cout<< "grasp_pose_topic rviz marker" <<endl;

            motor_grasp_pose_pub.publish(grasp_arrow);
            //=========rviz marker=========

            // //pos, euler, phi
            // //(x, y, z), (), phi
            // // std_msgs::Float32MultiArray grasp_msg; 
            // grasp_msg.data.push_back(coeff_cylinder->values[0]); // x
            // grasp_msg.data.push_back(coeff_cylinder->values[1]); // y
            // grasp_msg.data.push_back(coeff_cylinder->values[2]); // z
            // grasp_msg.data.push_back(yaw); //?
            // grasp_msg.data.push_back(pitch); //?  
            // grasp_msg.data.push_back(roll); //?  
            // grasp_msg.data.push_back(0.0); // phi
            // // motor_grasp_pose_pub.publish(grasp_msg);

            float x = coeff_cylinder->values[0];
            float y = coeff_cylinder->values[1];
            float z = coeff_cylinder->values[2];

            //change to 4*4 matrix: cam_H_motor
            cv::Mat rvec= cv::Mat::zeros(1,3,CV_32F); //(yaw, pitch, roll);
            cv::Mat rot = cv::Mat::zeros(3, 3, CV_32F);
            rvec.at<float>(0,0)=yaw;
            rvec.at<float>(0,1)=pitch;
            rvec.at<float>(0,2)=roll;
            
            cv::Rodrigues(rvec, rot);
            cout << "rvec:\n" << rvec
                 << "rot:\n"  << rot << endl;
            grasp_msg.data.clear();
            grasp_msg.data.push_back(rot.at<float>(0, 0));
            grasp_msg.data.push_back(rot.at<float>(0, 1));
            grasp_msg.data.push_back(rot.at<float>(0, 2));
            grasp_msg.data.push_back(x);
            grasp_msg.data.push_back(rot.at<float>(1, 0));
            grasp_msg.data.push_back(rot.at<float>(1, 1));
            grasp_msg.data.push_back(rot.at<float>(1, 2));
            grasp_msg.data.push_back(y);
            grasp_msg.data.push_back(rot.at<float>(2, 0));
            grasp_msg.data.push_back(rot.at<float>(2, 1));
            grasp_msg.data.push_back(rot.at<float>(2, 2));
            grasp_msg.data.push_back(z);
            grasp_msg.data.push_back(0);
            grasp_msg.data.push_back(0);
            grasp_msg.data.push_back(0);
            grasp_msg.data.push_back(1);

            // cv::Mat came_H_motor = []
            // came_H_motor = np.mat([ [rot[0, 0], rot[0, 1], rot[0, 2], x],
            //                         [rot[1, 0], rot[1, 1], rot[1, 2], y],
            //                         [rot[2, 0], rot[2, 1], rot[2, 2], z],
            //                         [   0,         0,         0,        1   ] ])

            pcl::toROSMsg(*motor_all[n].motor_cloud, motor_cloud_msg);
            motor_cloud_msg.header.frame_id = "camera_depth_optical_frame";
            motor_cloud_pub.publish(motor_cloud_msg);
        }

        pcl::PointCloud<PointTRGB>::Ptr motor_clear(new pcl::PointCloud<PointTRGB>);
        PointTRGB motor_center;

        // Eigen::Vector3d vv1 = Eigen::Vector3d(vect_x[0], vect_x[1], vect_x[2]);
        // Eigen::Vector3d vv2 = Eigen::Vector3d(line_c1_c2_x, line_c1_c2_y, line_c1_c2_z);
        // Eigen::Quaterniond out = Eigen::Quaterniond::FromTwoVectors(vv1, vv2);

        // //=========rviz marker=========
        // Eigen::Quaterniond AQ;
        // visualization_msgs::Marker motor_arrow;
        // motor_arrow.header.frame_id = "camera_color_optical_frame";
        // motor_arrow.header.stamp = ros::Time();
        // motor_arrow.ns = "my_namespace";
        // motor_arrow.id = 0;
        // motor_arrow.type = visualization_msgs::Marker::ARROW;
        // motor_arrow.action = visualization_msgs::Marker::ADD;
        // motor_arrow.pose.position.x = pt_c1.x;
        // motor_arrow.pose.position.y = pt_c1.y;
        // motor_arrow.pose.position.z = pt_c1.z;
        // motor_arrow.pose.orientation.x = out.x();//pose_msg.orientation.x;
        // motor_arrow.pose.orientation.y = out.y();//pose_msg.orientation.y;
        // motor_arrow.pose.orientation.z = out.z();//pose_msg.orientation.z;
        // motor_arrow.pose.orientation.w = out.w();//pose_msg.orientation.w;
        // motor_arrow.scale.x = sqrt(pow(line_c1_c2_x,2)+pow(line_c1_c2_y,2)+pow(line_c1_c2_z,2)); //length
        // motor_arrow.scale.y = 0.003;  //width
        // motor_arrow.scale.z = 0.003;  //height
        // motor_arrow.color.a = 1.0;    // Don't forget to set the alpha!
        // motor_arrow.color.r = 0.0;
        // motor_arrow.color.g = 0.0;
        // motor_arrow.color.b = 1.0;

        // cout<< "motor_pose_topic rviz marker" <<endl;

        // motor_pose_pub.publish(motor_arrow);
        // //=========rviz marker=========


        // //========================================//
        // // Visualize All Detected Motors
        // //========================================//
        // //Display Top 3 Sauces via Rviz
        // pcl::PointCloud<PointTRGB>::Ptr top3_clouds_stock(new pcl::PointCloud<PointTRGB>);

        // for(int idx = 0; idx < motor_all.size(); ++idx)
        // {
        //     *top3_clouds_stock = *top3_clouds_stock + *(motor_all[idx].motor_bbox_cloud);
        // }

        // //Publish pcl::PointCloud to ROS sensor::PointCloud2, and to topic
        // pcl::toROSMsg(*motor_clear, motor_cloud_msg);
        // motor_cloud_msg.header.frame_id = "camera_depth_optical_frame";
        // motor_cloud_pub.publish(motor_cloud_msg);
    }
}

bool get_pose(assembly_srv::GraspPose::Request& req, assembly_srv::GraspPose::Response& res)
{
    // res.grasp_pose = grasp_msg.data;
    // cout << "res.grasp_pose" << res.grasp_pose.data << endl;
    if(req.picture_pose_reached == true)
    {
        res.grasp_pose = grasp_msg.data; //float 4*4 matrix
        cout <<"grasp_msg.data response sent" << endl;
    }
    else
        cout << "[motor] waiting to reach take pic pose\n";

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_motor");
    cout << "get_motor code\n";

    ros::NodeHandle nh;
    ros::Subscriber sub_yolo = nh.subscribe("/yolov4_motors_bboxes", 1, motor_cb);
    ros::Subscriber sub_scene_cloud = nh.subscribe("/camera/depth_registered/points", 1, motor_cloud_cb);

    ros::ServiceServer service = nh.advertiseService("/motor_grasp_pose", get_pose);
    motor_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/motor_cloud", 1);
    motor_pose_pub = nh.advertise<visualization_msgs::Marker>("/motor_pose", 1);
    motor_grasp_pose_pub = nh.advertise<visualization_msgs::Marker>("/motor_grasp_pose", 1);
    // motor_grasp_pose_pub = nh.advertise<std_msgs::Float32MultiArray>("/motor_grasp_pose", 1);

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