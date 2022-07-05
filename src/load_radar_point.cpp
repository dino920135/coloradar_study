#include <stdio.h>
#include <stdlib.h>
#include <iomanip>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "TsRadar.h"

#define postprocess

using namespace std;
using namespace TsRadar;

TsRadarLib *TsRadarCore = new TsRadarLib;

// Realtime
#ifdef Realtime
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_fusion");
	ros::NodeHandle n("~");
    ros::Subscriber sub_RScan = n.subscribe("/mmWaveDataHdl/RScan", 100, RScan_callback);
    ros::spin();

    return 0;
}
#endif
#ifdef postprocess
int main(int argc, char** argv)
{
    rosbag::Bag bag;
    cout << "Opening bagfile ......";
    bag.open("/home/point001/TS/Coloradar/outdoors_run1.bag", rosbag::bagmode::Read);
    cout << " Done." << endl;

    std::vector<std::string> topics;
    topics.push_back(std::string("/mmWaveDataHdl/RScan"));
    topics.push_back(std::string("/lidar_ground_truth"));
    
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator iter;
    iter = view.begin();

    // Visualizing Viewer initializing
    TsRadarCore->pclViewerInit();
    
    int countFrame = 0;
    while(iter != view.end())
    {
        auto m = *iter;

        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != NULL)
        {
            // Wait for last pcl viewer rendering
            TsRadarCore->pclThreadVis_wait();

            // Clear previous cloud for new scan
            TsRadarCore->RscanMem.cloud.clear();
            TsRadarCore->RScan_callback(pc_msg);
            
            // Tranform to DG pointcloud by Ground Truth pose
            pcl::transformPointCloud (TsRadarCore->RscanMem.cloud, TsRadarCore->RscanMem.cloud, TsRadarCore->RscanMem.trans_nb);
            *TsRadarCore->pclViewerMem.cloudDg += TsRadarCore->RscanMem.cloud;

            // string pcd_path = "../../PCD/" + to_string(msg->header.stamp.toNSec()) + ".pcd";
            // SavePCD(cloud, pcd_path);
            countFrame ++;
        }

        nav_msgs::Odometry::ConstPtr gt_msg = m.instantiate<nav_msgs::Odometry>();
        if (gt_msg != NULL )
        {
            // Wait for last pcl viewer rendering
            TsRadarCore->pclThreadVis_wait();

            TsRadarCore->Ground_truth_callback(gt_msg);
            pcl::PointXYZRGB pose;
            pose.x = TsRadarCore->RscanMem.trans_nb(0, 3);
            pose.y = TsRadarCore->RscanMem.trans_nb(1, 3);
            pose.z = TsRadarCore->RscanMem.trans_nb(2, 3);
            pose.r = 255;

            TsRadarCore->pclViewerMem.cloudPose->push_back(pose);
        }
        iter++;

        // Visualizing with new thread every 10 frames
        if (countFrame > 10)
        {
            TsRadarCore->pclThreadVis();
            countFrame = 0;
        }
    }
    bag.close();

    TsRadarCore->pclThreadVis_wait();
    // hold viewer
    while (1)
        TsRadarCore->pclViewerMem.viewer->spin();
    
    return 0;
}
#endif