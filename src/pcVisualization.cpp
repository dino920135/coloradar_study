#include "TsRadar.h"

namespace TsRadar {
    void TsRadarLib::pclViewerInit()
    {
        pclViewerMem.viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D viewer"));

        pclViewerMem.cloudCur = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        pclViewerMem.cloudDg = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pclViewerMem.cloudPose = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

        // Set Color Rule for clouds
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> cloudCurColor(pclViewerMem.cloudCur, "r");
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> cloudColor(pclViewerMem.cloudDg, "intensity");
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> cloudPoseColor(pclViewerMem.cloudPose, "r");
    }

    void TsRadarLib::pclViewerRender()
    {
        // Point cloud After Direct Georeferencing (Mapping)
        pclViewerMem.viewer->removePointCloud("cloudCur", 0);
        pclViewerMem.viewer->addPointCloud<pcl::PointXYZRGB> (pclViewerMem.cloudCur, "cloudCur");

        // Point cloud After Direct Georeferencing (Mapping)
        pclViewerMem.viewer->removePointCloud("cloudDg", 0);
        pclViewerMem.viewer->addPointCloud<pcl::PointXYZI> (pclViewerMem.cloudDg, "cloudDg");

        // Point cloud with Position Coordinate
        pclViewerMem.viewer->removePointCloud("cloudPose", 0);
        pclViewerMem.viewer->addPointCloud<pcl::PointXYZRGB> (pclViewerMem.cloudPose, "cloudPose");

        // Set point Size for cloudPose
        pclViewerMem.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudCur");
        pclViewerMem.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudPose");
        pclViewerMem.viewer->spinOnce();
    }

    void TsRadarLib::pclThreadVis()
    {
            pclViewerThread = std::thread( &TsRadar::TsRadarLib::pclViewerRender, this);
    }

    void TsRadarLib::pclThreadVis_wait()
    {
        if (pclViewerThread.joinable())
            pclViewerThread.join();
    }

    void TsRadarLib::SavePCD(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::string& path)
    {
        cout << "pcd_path: " << path << endl;
        pcl::io::savePCDFileASCII(path, cloud);
    }
}
