#include "TsRadar.h"

using namespace std;

namespace TsRadar
{
    void TsRadarLib::RScan_callback(const sensor_msgs::PointCloud2::ConstPtr &RScan_msg)
    {
        double t = RScan_msg->header.stamp.toSec();
        cout << setprecision(14);
        cout << t << endl;
        int point_bytes = RScan_msg->point_step;
        int offset_x, offset_y, offset_z, offset_intensity;

        const auto& fields = RScan_msg->fields;
        for (int f = 0; f < fields.size(); ++f)
        {
            if (fields[f].name == "x") offset_x = fields[f].offset;
            if (fields[f].name == "y") offset_y = fields[f].offset;
            if (fields[f].name == "z") offset_z = fields[f].offset;
            if (fields[f].name == "intensity") offset_intensity = fields[f].offset;
        }

        // Tranformation
        for (int i = 0; i < RScan_msg->width; ++i)
        {
            pcl::PointXYZI point;
            point.x = *(float*)(RScan_msg->data.data() + point_bytes*i + offset_x);
            point.y = *(float*)(RScan_msg->data.data() + point_bytes*i + offset_y);
            point.z = *(float*)(RScan_msg->data.data() + point_bytes*i + offset_z);

            point.intensity = *(unsigned char*)(RScan_msg->data.data() + point_bytes*i + offset_intensity);

            if (sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)) < 2)
                continue;
            // else if (point.intensity <= 50)
            //     continue;


            auto tmp = point.x;
            // point.x = point.z;
            point.x = -point.y;
            point.y = tmp;

            // cout << "intensity " << point.intensity << " " << (point.intensity < 50) << endl;

            RscanMem.cloud.push_back(point);
        }
        RscanMem.cloud.width = RScan_msg->width;
        RscanMem.cloud.height = RScan_msg->height;
    }

    void TsRadarLib::Ground_truth_callback(const nav_msgs::Odometry::ConstPtr &GT_msg)
    {
        RscanMem.trans_nb = Eigen::Matrix4f::Identity();
        /* Reminder: how transformation matrices work :

        |-------> This column is the translation
        | 1 0 0 x |  \
        | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
        | 0 0 1 z |  /
        | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

        METHOD #1: Using a Matrix4f
        This is the "manual" method, perfect to understand but error prone !
        */
        // Translation
        RscanMem.trans_nb(0, 3) = GT_msg->pose.pose.position.x;
        RscanMem.trans_nb(1, 3) = GT_msg->pose.pose.position.y;
        RscanMem.trans_nb(2, 3) = GT_msg->pose.pose.position.z;

        // Rotation
        // Quaternion
        RscanMem.q_nb.w() = GT_msg->pose.pose.orientation.w;
        RscanMem.q_nb.x() = GT_msg->pose.pose.orientation.x;
        RscanMem.q_nb.y() = GT_msg->pose.pose.orientation.y;
        RscanMem.q_nb.z() = GT_msg->pose.pose.orientation.z;
        // Rotation Matrix
        RscanMem.C_nb = RscanMem.q_nb.normalized().toRotationMatrix();
        // Transformation Matrix (in homogenious form)
        RscanMem.trans_nb(0, 0) = RscanMem.C_nb(0, 0);
        RscanMem.trans_nb(0, 1) = RscanMem.C_nb(0, 1);
        RscanMem.trans_nb(0, 2) = RscanMem.C_nb(0, 2);
        RscanMem.trans_nb(1, 0) = RscanMem.C_nb(1, 0);
        RscanMem.trans_nb(1, 1) = RscanMem.C_nb(1, 1);
        RscanMem.trans_nb(1, 2) = RscanMem.C_nb(1, 2);
        RscanMem.trans_nb(2, 0) = RscanMem.C_nb(2, 0);
        RscanMem.trans_nb(2, 1) = RscanMem.C_nb(2, 1);
        RscanMem.trans_nb(2, 2) = RscanMem.C_nb(2, 2);

        // cout << RscanMem->trans_nb << endl;
    }
}