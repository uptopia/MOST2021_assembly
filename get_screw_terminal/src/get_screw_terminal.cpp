#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <part_afford_seg/seg_out.h>

#include "assembly_srv/GraspPose.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
typedef pcl::PointXYZRGB PointTRGB;
typedef boost::shared_ptr<pcl::PointCloud<PointTRGB>> PointCloudTRGBPtr;

pcl::PointCloud<PointTRGB>::Ptr screw_head(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr screw_body(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr screw(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr terminal_pos(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr terminal_neg(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);

ros::Publisher screw_pub, screw_pose_pub, termA_pub, termA_pose_pub, screw_grasp_pose_pub, termA_grasp_pose_pub;
sensor_msgs::PointCloud2 screw_msg, terminal_msg;
std_msgs::Float32MultiArray screw_grasp_msg, terminal_grasp_msg; 
void screw_cb(const part_afford_seg::seg_out::ConstPtr& seg_msg)
{   
    screw->clear();
    cv_bridge::CvImagePtr cv_ptr;
    cout << "seg_msg->centers.size() = " << seg_msg->centers.size() << endl;

    for(int i = 0; i < seg_msg->centers.size(); i++)
    {
        int x_shift = 370 + seg_msg->bbox_xmin[i];  //!!!!Change according to select_workspace & bbox position
        int y_shift = 150 + seg_msg->bbox_ymin[i];  //!!!!Change according to select_workspace & bbox position
        // cout << "object # " << i << endl;
        // cout << seg_msg->centers[i].c1_x + x_shift<< " "
        //     << seg_msg->centers[i].c1_y + y_shift << " "
        //     << seg_msg->centers[i].c2_x + x_shift << " "
        //     << seg_msg->centers[i].c2_y + y_shift << " "
        //     << seg_msg->centers[i].angle << " "
        //     << seg_msg->bbox_xmin[i] << " "
        //     << seg_msg->bbox_ymin[i] << " "
        //     << seg_msg->object_name[i] << endl;


        cv_ptr = cv_bridge::toCvCopy(seg_msg->roi[i], sensor_msgs::image_encodings::TYPE_8UC3);
        cv::Mat screw_color = cv::Mat(cv_ptr->image);
        int nRows = screw_color.rows;
        int nCols = screw_color.cols;

        std::vector<cv::Mat> rgbChannels(3);
        cv::split(screw_color, rgbChannels);

        // screw_head->clear();
        // screw_body->clear();
        // screw->clear();
        // terminal_pos->clear();
        // terminal_neg->clear();

        for(int y = 0; y < nRows; y++)
        {
            for(int x = 0; x < nCols; x++)
            {
                int idx_x = x + x_shift;
                int idx_y = y + y_shift;
                
                PointTRGB pt = organized_cloud_ori->at(idx_x, idx_y);
                if(pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z))
                {   
                    if(seg_msg->object_name[i] == "screw")
                    {
                        if(((int)rgbChannels[2].at<uchar>(y, x) > 0) || ((int)rgbChannels[1].at<uchar>(y, x) > 0))
                            screw->push_back(pt);
                    }
                    else if(seg_msg->object_name[i] == "terminal+")
                    {
                        if(((int)rgbChannels[1].at<uchar>(y, x) > 0) || ((int)rgbChannels[0].at<uchar>(y, x) > 0))
                            terminal_pos->push_back(pt);
                    }
                    else if(seg_msg->object_name[i] == "terminal-")
                    {
                        if(((int)rgbChannels[1].at<uchar>(y, x) > 0) || ((int)rgbChannels[0].at<uchar>(y, x) > 0))
                            terminal_neg->push_back(pt);
                    }
                }
            }
        }

        PointTRGB arrow_head, arrow_body, term_headA, term_bodyA, term_headB, term_bodyB;
        if(seg_msg->object_name[i] == "screw")
        {
            arrow_head = organized_cloud_ori->at(x_shift + seg_msg->centers[i].c1_x, y_shift + seg_msg->centers[i].c1_y);
            arrow_body = organized_cloud_ori->at(x_shift + seg_msg->centers[i].c2_x, y_shift + seg_msg->centers[i].c2_y);
            cout<< "arrow_head: " << arrow_head.x << ", " << arrow_head.y << ", " << arrow_head.z << endl;
            cout<< "arrow_body: " << arrow_body.x << ", " << arrow_body.y << ", " << arrow_body.z << endl;

            //=========rviz marker=========
            Eigen::Vector3d vv1 = Eigen::Vector3d(1.0, 0.0, 0.0);//arrow_head.x, arrow_head.y, arrow_head.z);
            Eigen::Vector3d vv2 = Eigen::Vector3d(arrow_body.x - arrow_head.x, arrow_body.y - arrow_head.y, arrow_body.z - arrow_head.z);
            cout << "vv1 = " <<vv1<<endl;
            cout << "vv2 = " <<vv2<<endl;
            vv2.normalize();
            Eigen::Quaterniond out = Eigen::Quaterniond::FromTwoVectors(vv1, vv2);
            Eigen::Vector3d euler = out.toRotationMatrix().eulerAngles(2, 1, 0);
            float yaw = euler[0];
            float pitch = euler[1];
            float roll = euler[2];

            Eigen::Vector3d vv3 = Eigen::Vector3d(0.0, 0.0, 1.0);
            Eigen::Quaterniond out1 = Eigen::Quaterniond::FromTwoVectors(vv3, vv2);
            Eigen::Vector3d euler1 = out1.toRotationMatrix().eulerAngles(2, 1, 0);

            visualization_msgs::Marker screw_arrow;
            screw_arrow.header.frame_id = "camera_depth_optical_frame";
            screw_arrow.header.stamp = ros::Time();
            screw_arrow.ns = "my_namespace";
            screw_arrow.id = 0;
            screw_arrow.type = visualization_msgs::Marker::ARROW;
            screw_arrow.action = visualization_msgs::Marker::ADD;
            screw_arrow.pose.position.x = arrow_head.x;
            screw_arrow.pose.position.y = arrow_head.y;
            screw_arrow.pose.position.z = arrow_head.z;
            screw_arrow.pose.orientation.x = out.x();
            screw_arrow.pose.orientation.y = out.y();
            screw_arrow.pose.orientation.z = out.z();
            screw_arrow.pose.orientation.w = out.w();
            screw_arrow.scale.x = 0.050;
            screw_arrow.scale.y = 0.008;  //width
            screw_arrow.scale.z = 0.008;  //height
            screw_arrow.color.a = 1.0;    // Don't forget to set the alpha!
            screw_arrow.color.r = 1.0;
            screw_arrow.color.g = 0.0;
            screw_arrow.color.b = 0.0;

            cout<< "screw_pose rviz marker" <<endl;

            screw_pose_pub.publish(screw_arrow);
            //=========rviz marker=========

            //=========rviz marker=========
            visualization_msgs::Marker grasp_arrow;
            grasp_arrow.header.frame_id = "camera_depth_optical_frame";
            grasp_arrow.header.stamp = ros::Time();
            grasp_arrow.ns = "my_namespace";
            grasp_arrow.id = 0;
            grasp_arrow.type = visualization_msgs::Marker::ARROW;
            grasp_arrow.action = visualization_msgs::Marker::ADD;
            grasp_arrow.pose.position.x = arrow_head.x;
            grasp_arrow.pose.position.y = arrow_head.y;
            grasp_arrow.pose.position.z = arrow_head.z;
            grasp_arrow.pose.orientation.x = out1.x();//pose_msg.orientation.x;
            grasp_arrow.pose.orientation.y = out1.y();//pose_msg.orientation.y;
            grasp_arrow.pose.orientation.z = out1.z();//pose_msg.orientation.z;
            grasp_arrow.pose.orientation.w = out1.w();//pose_msg.orientation.w;
            grasp_arrow.scale.x = 0.050;//sqrt(pow(coeff_cylinder->values[3],2)+pow(coeff_cylinder->values[4],2)+pow(coeff_cylinder->values[5],2)); //length
            grasp_arrow.scale.y = 0.008;  //width
            grasp_arrow.scale.z = 0.008;  //height
            grasp_arrow.color.a = 1.0;    // Don't forget to set the alpha!
            grasp_arrow.color.r = 0.0;
            grasp_arrow.color.g = 0.0;
            grasp_arrow.color.b = 1.0;

            cout<< "grasp_pose_topic rviz marker" <<endl;

            screw_grasp_pose_pub.publish(grasp_arrow);
            //=========rviz marker=========
        }
        else if(seg_msg->object_name[i] == "terminal-")
        {
            term_headA = organized_cloud_ori->at(x_shift + seg_msg->centers[i].c1_x, y_shift + seg_msg->centers[i].c1_y);
            term_bodyA = organized_cloud_ori->at(x_shift + seg_msg->centers[i].c2_x, y_shift + seg_msg->centers[i].c2_y);
            cout<< "term_headA: " << term_headA.x << ", " << term_headA.y << ", " << term_headA.z << endl;
            cout<< "term_bodyA: " << term_bodyA.x << ", " << term_bodyA.y << ", " << term_bodyA.z << endl;

            //=========rviz marker=========
            Eigen::Vector3d vv1 = Eigen::Vector3d(1.0, 0.0, 0.0);//term_headA.x, term_headA.y, term_headA.z);
            Eigen::Vector3d vv2 = Eigen::Vector3d(term_bodyA.x - term_headA.x, term_bodyA.y - term_headA.y, term_bodyA.z - term_headA.z);
            cout << "vv1 = " <<vv1<<endl;
            cout << "vv2 = " <<vv2<<endl;
            vv2.normalize();
            Eigen::Quaterniond out = Eigen::Quaterniond::FromTwoVectors(vv1, vv2);
            Eigen::Vector3d euler = out.toRotationMatrix().eulerAngles(2, 1, 0);
            float yaw = euler[0];
            float pitch = euler[1];
            float roll = euler[2];

            Eigen::Vector3d vv3 = Eigen::Vector3d(0.0, 0.0, 1.0);
            Eigen::Quaterniond out1 = Eigen::Quaterniond::FromTwoVectors(vv3, vv2);
            Eigen::Vector3d euler1 = out1.toRotationMatrix().eulerAngles(2, 1, 0);

            visualization_msgs::Marker term_arrowA;
            term_arrowA.header.frame_id = "camera_depth_optical_frame";
            term_arrowA.header.stamp = ros::Time();
            term_arrowA.ns = "my_namespace";
            term_arrowA.id = 0;
            term_arrowA.type = visualization_msgs::Marker::ARROW;
            term_arrowA.action = visualization_msgs::Marker::ADD;
            term_arrowA.pose.position.x = term_headA.x;
            term_arrowA.pose.position.y = term_headA.y;
            term_arrowA.pose.position.z = term_headA.z;
            term_arrowA.pose.orientation.x = out.x();
            term_arrowA.pose.orientation.y = out.y();
            term_arrowA.pose.orientation.z = out.z();
            term_arrowA.pose.orientation.w = out.w();
            term_arrowA.scale.x = 0.050;
            term_arrowA.scale.y = 0.008;  //width
            term_arrowA.scale.z = 0.008;  //height
            term_arrowA.color.a = 1.0;    // Don't forget to set the alpha!
            term_arrowA.color.r = 1.0;
            term_arrowA.color.g = 0.0;
            term_arrowA.color.b = 0.0;

            cout<< "termA_pose rviz marker" <<endl;

            termA_pose_pub.publish(term_arrowA);
            //=========rviz marker=========

            //=========rviz marker=========
            visualization_msgs::Marker grasp_arrow;
            grasp_arrow.header.frame_id = "camera_depth_optical_frame";
            grasp_arrow.header.stamp = ros::Time();
            grasp_arrow.ns = "my_namespace";
            grasp_arrow.id = 0;
            grasp_arrow.type = visualization_msgs::Marker::ARROW;
            grasp_arrow.action = visualization_msgs::Marker::ADD;
            grasp_arrow.pose.position.x = term_headA.x;
            grasp_arrow.pose.position.y = term_headA.y;
            grasp_arrow.pose.position.z = term_headA.z;
            grasp_arrow.pose.orientation.x = out1.x();//pose_msg.orientation.x;
            grasp_arrow.pose.orientation.y = out1.y();//pose_msg.orientation.y;
            grasp_arrow.pose.orientation.z = out1.z();//pose_msg.orientation.z;
            grasp_arrow.pose.orientation.w = out1.w();//pose_msg.orientation.w;
            grasp_arrow.scale.x = 0.050;//sqrt(pow(coeff_cylinder->values[3],2)+pow(coeff_cylinder->values[4],2)+pow(coeff_cylinder->values[5],2)); //length
            grasp_arrow.scale.y = 0.008;  //width
            grasp_arrow.scale.z = 0.008;  //height
            grasp_arrow.color.a = 1.0;    // Don't forget to set the alpha!
            grasp_arrow.color.r = 0.0;
            grasp_arrow.color.g = 0.0;
            grasp_arrow.color.b = 1.0;

            cout<< "termA_pose_topic rviz marker" <<endl;

            termA_grasp_pose_pub.publish(grasp_arrow);
            //=========rviz marker=========
        }

        

        // //pos, euler, phi
        // //(x, y, z), (), phi
        // std_msgs::Float32MultiArray grasp_msg; 
        // grasp_msg.data.push_back(arrow_head.x); // x
        // grasp_msg.data.push_back(arrow_head.y); // y
        // grasp_msg.data.push_back(arrow_head.z); // z
        // grasp_msg.data.push_back(yaw); //?
        // grasp_msg.data.push_back(pitch); //?  
        // grasp_msg.data.push_back(roll); //?  
        // grasp_msg.data.push_back(0.0); // phi
        // screw_grasp_pose_pub.publish(grasp_msg);
    }
    cout << "screw points " << screw->points.size() <<endl;
    pcl::toROSMsg(*screw, screw_msg);
    screw_msg.header.frame_id = "camera_depth_optical_frame";
    screw_pub.publish(screw_msg);
}

void orgainized_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
{
    int height = organized_cloud_msg->height;
    int width = organized_cloud_msg->width;
    int points = height * width;
    cout << "(height, width) = " << height << ", " << width << endl;

    organized_cloud_ori->clear();
    pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);
}

bool get_screw_pose(assembly_srv::GraspPose::Request& req, assembly_srv::GraspPose::Response& res)
{
    if(req.picture_pose_reached == true)
    {
        res.grasp_pose = screw_grasp_msg.data;
        cout << "screw_grasp_msg.data response sent" << endl;
    }
    else
        cout << "[screw] waiting to reach take pic pose\n";

    return true;
}

bool get_terminal_pose(assembly_srv::GraspPose::Request& req, assembly_srv::GraspPose::Response& res)
{
    if(req.picture_pose_reached == true)
    {
        res.grasp_pose = terminal_grasp_msg.data;
        cout << "terminal_grasp_msg.data response sent" << endl;
    }
    else
        cout << "[terminal] waiting to reach take pic pose\n";

    return true;
}


int main(int argc, char** argv)
{   
    ros::init(argc, argv, "get_screw_terminal");
    cout << "screw cloud node" << endl;

    ros::NodeHandle nh;
    ros::Subscriber sub_seg = nh.subscribe("/seg_out", 1, screw_cb);
    ros::Subscriber sub_organized_cloud = nh.subscribe("/camera/depth_registered/points", 1, orgainized_cloud_cb);
    
    //screw
    screw_pub = nh.advertise<sensor_msgs::PointCloud2>("screw_cloud", 1);
    screw_pose_pub = nh.advertise<visualization_msgs::Marker>("screw_pose", 1);
    screw_grasp_pose_pub = nh.advertise<visualization_msgs::Marker>("screw_grasp_pose", 1);
    ros::ServiceServer screw_service = nh.advertiseService("/screw_grasp_pose", get_screw_pose);

    //terminal+
    termA_pub = nh.advertise<sensor_msgs::PointCloud2>("termA_cloud", 1);
    termA_pose_pub = nh.advertise<visualization_msgs::Marker>("termA_pose", 1);
    termA_grasp_pose_pub = nh.advertise<visualization_msgs::Marker>("termA_grasp_pose", 1);
    ros::ServiceServer termA_service = nh.advertiseService("/termA_grasp_pose", get_terminal_pose);

    // //terminal-
    // termA_pub = nh.advertise<sensor_msgs::PointCloud2>("termA_cloud", 1);
    // termA_pose_pub = nh.advertise<visualization_msgs::Marker>("termA_pose", 1);
    // termA_grasp_pose_pub = nh.advertise<visualization_msgs::Marker>("termA_grasp_pose", 1);
    // ros::ServiceServer termA_service = nh.advertiseService("/termA_grasp_pose", get_terminal_pose);
    ros::spin();
    
    return 0;
}