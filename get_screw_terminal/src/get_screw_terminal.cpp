#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <part_afford_seg/seg_out.h>

using namespace std;
typedef pcl::PointXYZRGB PointTRGB;
typedef boost::shared_ptr<pcl::PointCloud<PointTRGB>> PointCloudTRGBPtr;

pcl::PointCloud<PointTRGB>::Ptr screw_head(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr screw_body(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr screw(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr terminal_pos(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr terminal_neg(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);

ros::Publisher screw_pub, screw_pose_pub, terminal_pub, terminal_pose_pub;
sensor_msgs::PointCloud2 screw_msg, terminal_msg;

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
                            // cout << (int)rgbChannels[2].at<uchar>(idx_y, idx_x) << "; " << (int)rgbChannels[1].at<uchar>(idx_y, idx_x) << endl;
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
                    // else
                    // {

                    // }
                }
            }
        }

        PointTRGB arrow_head = organized_cloud_ori->at(x_shift + seg_msg->centers[i].c1_x, y_shift + seg_msg->centers[i].c1_y);
        PointTRGB arrow_body = organized_cloud_ori->at(x_shift + seg_msg->centers[i].c2_x, y_shift + seg_msg->centers[i].c2_y);

        //=========rviz marker=========
        Eigen::Vector3d vv1 = Eigen::Vector3d(0.0, 0.0, 0.0);//arrow_head.x, arrow_head.y, arrow_head.z);
        Eigen::Vector3d vv2 = Eigen::Vector3d(arrow_body.x, arrow_body.y, arrow_body.z);
        vv2.normalize();
        Eigen::Quaterniond out = Eigen::Quaterniond::FromTwoVectors(vv1, vv2);

        Eigen::Quaterniond AQ;
        visualization_msgs::Marker screw_arrow;
        screw_arrow.header.frame_id = "camera_color_optical_frame";
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
        screw_arrow.scale.x = 0.030;
        screw_arrow.scale.y = 0.008;  //width
        screw_arrow.scale.z = 0.008;  //height
        screw_arrow.color.a = 1.0;    // Don't forget to set the alpha!
        screw_arrow.color.r = 0.0;
        screw_arrow.color.g = 0.0;
        screw_arrow.color.b = 1.0;

        cout<< "screw_pose rviz marker" <<endl;

        screw_pose_pub.publish(screw_arrow);
        //=========rviz marker=========
    }
    cout << "screw points " << screw->points.size() <<endl;
    pcl::toROSMsg(*screw, screw_msg);
    screw_msg.header.frame_id = "camera_color_optical_frame";
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

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "get_screw_terminal");
    cout << "screw cloud node" << endl;

    ros::NodeHandle nh;
    ros::Subscriber sub_seg = nh.subscribe("/seg_out", 1, screw_cb);
    ros::Subscriber sub_organized_cloud = nh.subscribe("/camera/depth_registered/points", 1, orgainized_cloud_cb);
    
    screw_pub = nh.advertise<sensor_msgs::PointCloud2>("screw_cloud", 1);
    screw_pose_pub = nh.advertise<visualization_msgs::Marker>("screw_pose", 1);
    terminal_pub = nh.advertise<sensor_msgs::PointCloud2>("terminal_cloud", 1);
    terminal_pose_pub = nh.advertise<visualization_msgs::Marker>("terminal_pose", 1);
    ros::spin();
    
    return 0;
}