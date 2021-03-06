#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Eigen>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <thread>

namespace TsRadar
{
    class TsRadarLib
    {
        typedef struct RscanMemory
        {
            Eigen::Quaterniond q_nb; // Rotation in Quoternion
            Eigen::Matrix3d C_nb; // Rotation in DCM
            Eigen::Matrix4f trans_nb; // Rotation & Translation in one Homogenious form
            Eigen::Vector3f position;

            pcl::PointCloud<pcl::PointXYZI> cloud; // Current PointCloud from 

        } RscanMemoryStruct, *RscanMemoryStructPtr;

        typedef struct pclViwerMemory
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCur;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudDg;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPose;

            pcl::visualization::PCLVisualizer::Ptr viewer;

            bool pc_update = 0;
                
        } pclViwerMemoryStruct, *pclViwerMemoryStructPtr;

        public:
            RscanMemory RscanMem;

            void RScan_callback(const sensor_msgs::PointCloud2::ConstPtr &RScan_msg);
            void Ground_truth_callback(const nav_msgs::Odometry::ConstPtr &GT_msg);

            pclViwerMemory pclViewerMem;
            std::thread pclViewerThread;
            void pclViewerInit();
            void pclViewerRender();
            void pclThreadVis();
            void pclThreadVis_wait();

            void SavePCD(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::string& path);

        private:

    };
}