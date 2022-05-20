#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

ros::Publisher screw_pub;
sensor_msgs::PointCloud2 screw_msg;

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
    }
    cout << "screw points " << screw->points.size() <<endl;
    pcl::toROSMsg(*screw, screw_msg);
    screw_msg.header.frame_id = "camera_color_optical_frame";
    screw_pub.publish(screw_msg);

    // cout << seg_msg->centers.size() << endl;
    // for(int i = 0; i < seg_msg->centers.size(); i++)
    // {
    //     cout << "object # " << i << endl;
    //     cout << seg_msg->centers[i].c1_x << " "
    //         << seg_msg->centers[i].c1_y << " "
    //         << seg_msg->centers[i].c2_x << " "
    //         << seg_msg->centers[i].c2_y << " "
    //         << seg_msg->centers[i].angle << " "
    //         << seg_msg->bbox_xmin << " "
    //         << seg_msg->bbox_ymin << " "<< endl;
    //     // cout << seg_msg->roi[i] << endl;
    // }
}

void screw_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
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
    ros::init(argc, argv, "screw_cloud");
    cout << "screw cloud node" << endl;

    ros::NodeHandle nh;
    ros::Subscriber sub_seg = nh.subscribe("/seg_out", 1, screw_cb);
    ros::Subscriber sub_screw_cloud = nh.subscribe("/camera/depth_registered/points", 1, screw_cloud_cb);
    
    screw_pub = nh.advertise<sensor_msgs::PointCloud2>("screw_topic", 1);
    ros::spin();
    
    return 0;
}