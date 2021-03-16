#include "utility.h"
//  Point Struct



class SweepRegistration: public utility
{
private:

    ros::Subscriber subLaserCloud;
    ros::Subscriber subImu;
    ros::Subscriber subTf;

    ros::Publisher pubDeskewCloud;

    pcl::PointCloud<PointXYZTIR>::Ptr laserCloud;
    pcl::PointCloud<PointXYZTIR>::Ptr sweepCloud;

    sensor_msgs::PointCloud2 curCloudMsg;

    geometry_msgs::TransformStamped curEncoderEnd;
    geometry_msgs::TransformStamped lastEncoderEnd;
    
    // std_msgs::Header cloudHeader;
    // std_msgs::Header lastCloudHeader;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    std::deque<sensor_msgs::Imu> imuQueue;
    std::deque<geometry_msgs::TransformStamped> encoderQueue;

    bool init;
    double lastEndtime;

    Eigen::Quaterniond q_fix;

public:
    SweepRegistration(): init(false), lastEndtime(0)
    {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 20, &SweepRegistration::laserCloudHandler, this, ros::TransportHints().tcpNoDelay());
        subImu = nh.subscribe<sensor_msgs::Imu>("/imu/data",1000, &SweepRegistration::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subTf = nh.subscribe<tf::tfMessage>("tf", 1000, &SweepRegistration::encoderHandler, this,  ros::TransportHints().tcpNoDelay());
        
        q_fix = Eigen::Quaterniond(0,0.957591,0.288116,-0.00282002).normalized();

        allocateMemory();
    }

    ~SweepRegistration(){};

    void allocateMemory()
    {
        laserCloud.reset(new pcl::PointCloud<PointXYZTIR>());
        sweepCloud.reset(new pcl::PointCloud<PointXYZTIR>());
    }
    
    bool preserveCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 5)
            return false;
        
        return true;
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // store cloud
        if (!preserveCloud(laserCloudMsg) || encoderQueue.empty())
            return;

        
        if (!init)
        {
            if (getAndPopEncoderTf(cloudQueue.front().header.stamp, encoderQueue, lastEncoderEnd))
            {
                cloudQueue.pop_front();
                init = true;
                return;
            }
        }


        curCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (!getAndPopEncoderTf(curCloudMsg.header.stamp, encoderQueue, curEncoderEnd))
            return;

        if ((curEncoderEnd.header.stamp - lastEncoderEnd.header.stamp).toSec() > 0.15)
        {
            lastEncoderEnd = curEncoderEnd;
            ROS_WARN("Encoder gap too long");
            return;
        }

        // transform msg to cloud
        pcl::fromROSMsg(curCloudMsg, *laserCloud);

        double beginTime = getBeginTime(laserCloud);

        ROS_INFO("Laser_begin:%d %.9d", curCloudMsg.header.stamp.sec, curCloudMsg.header.stamp.nsec);
        ROS_INFO("Last Encoder Time:%d %.9d", lastEncoderEnd.header.stamp.sec, lastEncoderEnd.header.stamp.nsec);
        ROS_INFO("Cur Encoder Time:%d %.9d", curEncoderEnd.header.stamp.sec, curEncoderEnd.header.stamp.nsec);
        ROS_INFO("Last Encoder value:x:%f,y:%f, z:%f, w:%f", lastEncoderEnd.transform.rotation.x, lastEncoderEnd.transform.rotation.y, lastEncoderEnd.transform.rotation.z, lastEncoderEnd.transform.rotation.w);
        ROS_INFO("Cur Encoder value:x:%f,y:%f, z:%f, w:%f", curEncoderEnd.transform.rotation.x, curEncoderEnd.transform.rotation.y, curEncoderEnd.transform.rotation.z, curEncoderEnd.transform.rotation.w);
        
        ROS_INFO("----------------------------------");
        
        Eigen::Quaterniond q_encoderBegin(lastEncoderEnd.transform.rotation.w, lastEncoderEnd.transform.rotation.x, lastEncoderEnd.transform.rotation.y, lastEncoderEnd.transform.rotation.z);
        Eigen::Quaterniond q_encoderEnd(curEncoderEnd.transform.rotation.w, curEncoderEnd.transform.rotation.x, curEncoderEnd.transform.rotation.y, curEncoderEnd.transform.rotation.z);
        double duration = (curEncoderEnd.header.stamp - lastEncoderEnd.header.stamp).toSec();

        // std::cout << q_encoderBegin.matrix() << std::endl;
        // std::cout << q_encoderEnd.matrix() << std::endl;
        // double duration = static_cast<double>(T_encoderEnd.header.stamp.sec) - static_cast<double>(T_encoderBegin.header.stamp.sec) + 
        //                     1e-9f * (static_cast<double>(T_encoderEnd.header.stamp.nsec) - static_cast<double>(T_encoderBegin.header.stamp.nsec));

        // ROS_INFO("Time_begin:%d %d", T_encoderBegin.header.stamp.sec, T_encoderBegin.header.stamp.nsec);
        // ROS_INFO("Time_end:%d %d", curCloudMsg.header.stamp.sec,curCloudMsg.header.stamp.nsec);
        // ROS_INFO("DURATION:%lf", duration);
        // ROS_INFO("x:%f,y:%f, z:%f, w:%f", T_encoderBegin.transform.rotation.x, T_encoderBegin.transform.rotation.y, T_encoderBegin.transform.rotation.z, T_encoderBegin.transform.rotation.w);

        pcl::io::savePLYFileASCII("/home/stereye/code/RotatingLidar/ori/" + std::to_string(curCloudMsg.header.stamp.sec) + "." + std::to_string(curCloudMsg.header.stamp.nsec) + ".ply", *laserCloud);
        pcl::PointCloud<PointXYZTIR>::Ptr originCloud;
        originCloud.reset(new pcl::PointCloud<PointXYZTIR>());
        originCloud->points.resize(laserCloud->points.size());
        for (size_t i = 0; i < laserCloud->points.size(); i++)
        {
            auto &point = laserCloud->points[i];
            // Eigen::Quaterniond curEncoderRot =  q_encoderBegin.slerp(rate, q_encoderEnd);
            Eigen::Vector3d v_point(point.x, point.y, point.z);
            v_point = q_encoderBegin *q_fix* v_point;
            originCloud->points[i].x = v_point.x();
            originCloud->points[i].y = v_point.y();
            originCloud->points[i].z = v_point.z();
            originCloud->points[i].intensity = laserCloud->points[i].intensity;
        }

        pcl::io::savePLYFileASCII("/home/stereye/code/RotatingLidar/before/" + std::to_string(curCloudMsg.header.stamp.sec) + "." + std::to_string(curCloudMsg.header.stamp.nsec)  + ".ply", *originCloud);

        for (size_t i = 0; i < laserCloud->points.size(); i++)
        {
            auto &point = laserCloud->points[i];
            double rate = (point.timestamp - beginTime) / duration;
            // std::cout << point.timestamp << " " << beginTime << " " << rate << std::endl;
            Eigen::Quaterniond curEncoderRot = q_encoderBegin.slerp(rate, q_encoderEnd);
            Eigen::Vector3d v_point(point.x, point.y, point.z);
            v_point = curEncoderRot * q_fix * v_point;
            point.x = v_point.x();
            point.y = v_point.y();
            point.z = v_point.z();
        }

        pcl::io::savePLYFileASCII("/home/stereye/code/RotatingLidar/after/" + std::to_string(curCloudMsg.header.stamp.sec) + "." + std::to_string(curCloudMsg.header.stamp.nsec) + ".ply", *laserCloud);


        lastEncoderEnd = curEncoderEnd;
        laserCloud->clear();
    }

    void imuHandler(const sensor_msgs::ImuConstPtr& imuMsg)
    {
        // imuQueue.push_back(*imuMsg);
    }

    void encoderHandler(const tf::tfMessageConstPtr& encoderMsg)
    {
        encoderQueue.push_back(encoderMsg->transforms[0]);
    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "SweepRegi");

    SweepRegistration SR;

    ROS_INFO("\033[1;32m----> Sweep Registeration Started.\033[0m");

    ros::spin();

    return 0;
}