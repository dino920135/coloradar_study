#include "TsRadar.h"

namespace TsRadar {
    void TsRadarLib::pclViewerInit()
    {
        pclViewerMem.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pclViewerMem.cloudDg = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pclViewerMem.cloudPose = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        pclViewerMem.viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D viewer"));
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(pclViewerMem.cloud, "intensity");
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> fildColor2(pclViewerMem.cloudPose, "r");
    }

    void TsRadarLib::pclViewerRender()
    {
        // while(true)
        // {
        pclViewerMem.viewer->removePointCloud("cloud", 0);
        pclViewerMem.viewer->addPointCloud<pcl::PointXYZI> (pclViewerMem.cloudDg, "cloud");
        pclViewerMem.viewer->removePointCloud("cloudPose", 0);
        pclViewerMem.viewer->addPointCloud<pcl::PointXYZRGB> (pclViewerMem.cloudPose, "cloudPose");
        pclViewerMem.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudPose");
        pclViewerMem.viewer->spinOnce();
        // }
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
