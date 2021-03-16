#ifndef _UTILITY_ROTATING
#define _UTILITY_ROTATING

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tfMessage.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/SVD>

#include <deque>
#include <memory>
#include <limits>
#include <mutex>

struct PointXYZTIR
{
    PCL_ADD_POINT4D;
    double timestamp;
    PCL_ADD_INTENSITY;
    uint8_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZTIR,
    (float, x, x) (float, y, y) (float, z, z) (double, timestamp, timestamp)
    (float, intensity, intensity) (uint8_t, ring, ring)
)


Eigen::Matrix4d read_transform(std::string file)
{
    Eigen::Matrix4d transform;
    std::ifstream src;

    src.open(file);
    for (int i = 0; i < 4; i++)
    {
        src >> transform(i,0);
        src >> transform(i,1);
        src >> transform(i,2);
        src >> transform(i,3);
    }
    src.close();

    return transform;
}

class utility
{
public:
    ros::NodeHandle nh;
    int N_SCAN;
    int Horizon_SCAN;

public:
    utility()
    {
        N_SCAN = 16;
        Horizon_SCAN = 1250;
    }
    double getBeginTime(pcl::PointCloud<PointXYZTIR>::Ptr laserCloud)
    {
        double mintime = std::numeric_limits<float>::max();
        for (size_t i = 0; i < laserCloud->points.size(); i++)
        {
            mintime = std::min(mintime, laserCloud->points[i].timestamp);
        }

        return mintime;
    }
    double getEndTime(pcl::PointCloud<PointXYZTIR>::Ptr laserCloud)
    {
        double maxtime = 0;
        for (size_t i = 0; i < laserCloud->points.size(); i++)
        {
            maxtime = std::max(maxtime, laserCloud->points[i].timestamp);
        }

        return maxtime;
    }

    bool getAndPopEncoderTf(ros::Time time, std::deque<geometry_msgs::TransformStamped> &EncoderQueue, geometry_msgs::TransformStamped& Tf)
    {
        if ((EncoderQueue.front().header.stamp - time).toSec() > 0.15)
        {
            ROS_WARN("Unsyncronized Lidar and Encoder!");
            return false;
        }

        while(time > EncoderQueue.front().header.stamp)
        {
            EncoderQueue.pop_front();
            if (EncoderQueue.empty())
            {
                ROS_WARN("Empty encoder queue!");
                return false;
            }
        }

        Tf = std::move(EncoderQueue.front());
        return true;
    }
    

    bool checkIfFail(geometry_msgs::TransformStamped test)
    {
        if (test.header.frame_id == "fail")
            return true;
        return false;
    }
    // geometry_msgs::TransformStamped get
};


#endif