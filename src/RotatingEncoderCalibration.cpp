#include "utility.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl-1.10/pcl/sample_consensus/ransac.h>
#include <pcl-1.10/pcl/sample_consensus/sac_model_plane.h>
#include <pcl-1.10/pcl/common/transforms.h>
#include <pcl-1.10/pcl/filters/passthrough.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/autodiff_cost_function.h>


typedef pcl::PointCloud<PointXYZTIR> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<tf::tfMessage,sensor_msgs::PointCloud2> MyPolicy; 


struct PlanePointFactor
{
    PlanePointFactor(Eigen::Vector4d point): m_point(point)
    {}

    template<typename T>
    bool operator() (const T* planeCoef, T* residual) const
    {
        Eigen::Matrix<T, 4, 1> point_T;
        point_T << T(m_point[0]), T(m_point[1]), T(m_point[2]), T(m_point[3]);

        residual[0] = (planeCoef[0] * point_T[0] + planeCoef[1] * point_T[1] + planeCoef[2] * point_T[2] + planeCoef[3] * point_T[3])
                        / ceres::sqrt(ceres::pow(planeCoef[0], 2) + ceres::pow(planeCoef[1], 2) + ceres::pow(planeCoef[2], 2));

        return true;
    }

    Eigen::Vector4d m_point;
};



struct PlanePlaneFactor
{
    
    PlanePlaneFactor(Eigen::Vector4d firstFramePlane, Eigen::Vector4d secondFramePlane, Eigen::Matrix4d firstEncoder, Eigen::Matrix4d secondEncoder)
        : m_firstFramePlane(firstFramePlane), m_secondFramePlane(secondFramePlane), m_firstEncoder(firstEncoder), m_secondEncoder(secondEncoder)
    {}

    template <typename T>
    bool operator() (const T *R_t, T *residual) const
    {
        // parameter is (theta_y, theta_z, y, z)
        T theta_y = R_t[0], theta_z = R_t[1];
        T y = R_t[2], z = R_t[3];

        // transform double vector to template
        Eigen::Matrix<T, 4, 1> firstFramePlane_T;
        Eigen::Matrix<T, 4, 1> secondFramePlane_T;
        firstFramePlane_T << T(m_firstFramePlane[0]), T(m_firstFramePlane[1]), T(m_firstFramePlane[2]), T(m_firstFramePlane[3]);
        secondFramePlane_T << T(m_secondFramePlane[0]), T(m_secondFramePlane[1]), T(m_secondFramePlane[2]), T(m_secondFramePlane[3]);

        // transform double to template
        Eigen::Matrix<T, 4, 4> T_lidar_to_axis;
        T_lidar_to_axis << ceres::cos(R_t[0])*ceres::cos(R_t[1]), -ceres::sin(R_t[1]), ceres::cos(R_t[1])*ceres::sin(R_t[0]), T(0),
                           ceres::sin(R_t[1])*ceres::cos(R_t[0]), ceres::cos(R_t[1]), ceres::sin(R_t[0])*ceres::sin(R_t[1]), R_t[2],
                           -ceres::sin(R_t[0]), T(0), ceres::cos(R_t[0]), R_t[3],
                           T(0), T(0), T(0), T(1);

        Eigen::Matrix<T, 4, 4> T_axis_rotate_first;
        Eigen::Matrix<T, 4, 4> T_axis_rotate_second;
        
        T_axis_rotate_first << T(m_firstEncoder(0,0)), T(m_firstEncoder(0,1)), T(m_firstEncoder(0,2)), T(m_firstEncoder(0,3)),
                               T(m_firstEncoder(1,0)), T(m_firstEncoder(1,1)), T(m_firstEncoder(1,2)), T(m_firstEncoder(1,3)),
                               T(m_firstEncoder(2,0)), T(m_firstEncoder(2,1)), T(m_firstEncoder(2,2)), T(m_firstEncoder(2,3)),
                               T(m_firstEncoder(3,0)), T(m_firstEncoder(3,1)), T(m_firstEncoder(3,2)), T(m_firstEncoder(3,3));

        T_axis_rotate_second << T(m_secondEncoder(0,0)), T(m_secondEncoder(0,1)), T(m_secondEncoder(0,2)), T(m_secondEncoder(0,3)),
                                T(m_secondEncoder(1,0)), T(m_secondEncoder(1,1)), T(m_secondEncoder(1,2)), T(m_secondEncoder(1,3)),
                                T(m_secondEncoder(2,0)), T(m_secondEncoder(2,1)), T(m_secondEncoder(2,2)), T(m_secondEncoder(2,3)),
                                T(m_secondEncoder(3,0)), T(m_secondEncoder(3,1)), T(m_secondEncoder(3,2)), T(m_secondEncoder(3,3));
        
        // std::cout << T_axis_rotate.inverse() <<std::endl;
        // std::cout << "############\n";
        // std::cout << "residual[0]:\n";
        // std::cout << residual[0] << std::endl;
        // std::cout << "_curFramePlane:\n";
        // std::cout << _curFramePlane.transpose()[0] << ", " <<  _curFramePlane.transpose()[1] << ", " << _curFramePlane.transpose()[2] << ", " << _curFramePlane.transpose()[3]<< std::endl;
        // std::cout << "------------\n";
        // std::cout << "curFramePlane_T:\n";
        // std::cout << curFramePlane_T.transpose()[0] << ", " <<  curFramePlane_T.transpose()[1] << ", " << curFramePlane_T.transpose()[2] << ", " << curFramePlane_T.transpose()[3]<< std::endl;
        // std::cout << "------------\n";
        // std::cout << "curNorm:\n";
        // std::cout << curNorm.transpose()[0] << ", " <<  curNorm.transpose()[1] << ", " << curNorm.transpose()[2] << std::endl;
        // std::cout << "------------\n";
        // std::cout << "fixFramePlane_T:\n";
        // std::cout << fixFramePlane_T.transpose()[0] << ", " <<  fixFramePlane_T.transpose()[1] << ", " << fixFramePlane_T.transpose()[2] << ", " << fixFramePlane_T.transpose()[3]<< std::endl;
        // std::cout << "------------\n";
        // std::cout << "fixNorm:\n";
        // std::cout << fixNorm.transpose()[0] << ", " <<  fixNorm.transpose()[1] << ", " << fixNorm.transpose()[2] << std::endl;
        // std::cout << "------------\n";

        // Transform plane1 to plane0 coordinate
        Eigen::Matrix<T, 4, 1> firstPlaneInAxis = (T_lidar_to_axis.inverse() * T_axis_rotate_first.inverse()).transpose() * firstFramePlane_T;
        Eigen::Matrix<T, 4, 1> secondPlaneInAxis = (T_lidar_to_axis.inverse() * T_axis_rotate_second.inverse()).transpose() * secondFramePlane_T;

        // Residual
        Eigen::Matrix<T, 3, 1> curNorm(firstPlaneInAxis(0), firstPlaneInAxis(1), firstPlaneInAxis(2));
        Eigen::Matrix<T, 3, 1> fixNorm(secondPlaneInAxis(0), secondPlaneInAxis(1), secondPlaneInAxis(2));
        residual[0] = T(1) - ceres::abs(curNorm.normalized().dot(fixNorm.normalized()));
        residual[1] = firstPlaneInAxis[3] - secondPlaneInAxis[3];
        
        return true;
    }

    Eigen::Vector4d m_firstFramePlane, m_secondFramePlane;
    Eigen::Matrix4d m_firstEncoder, m_secondEncoder;
};


class EncoderCalib: utility
{
private:
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher m_pubPlaneCloud;
    ros::Publisher m_compensatedCloud;

    // Subscriber
    ros::Subscriber m_subEncoder;
    ros::Subscriber m_subLaserCloud;

    // Vectors
    std::vector<geometry_msgs::TransformStamped> m_encoderStartVec;
    std::vector<geometry_msgs::TransformStamped> m_encoderEndVec;
    std::vector<PointCloud::Ptr> m_cloudVec;
    std::vector<Eigen::Vector4d> m_planeVec;

    // Queues
    std::deque<geometry_msgs::TransformStamped> m_encoderCacheQueue;
    std::deque<sensor_msgs::PointCloud2> m_cloudCacheQueue;

    // Messages
    geometry_msgs::TransformStamped m_lastEncoderMsg;
    geometry_msgs::TransformStamped m_curEncoderMsg;
    
    //
    bool init;
    bool calibReady;
    bool compensateReady;

    double m_x_min;
    double m_y_min;
    double m_x_max;
    double m_y_max;
    double m_ransac_threshold;

    Eigen::Vector4d m_lidarToAxis;
    Eigen::Vector4d m_lastlidarToAxis;


private:
    Eigen::Matrix4d lidarToAxisMatrix()
    {
        Eigen::Matrix4d toAxis;
        toAxis << cos(m_lidarToAxis[0])*cos(m_lidarToAxis[1]), -sin(m_lidarToAxis[1]), cos(m_lidarToAxis[1])*sin(m_lidarToAxis[0]), 0,
                  sin(m_lidarToAxis[1])*cos(m_lidarToAxis[0]), cos(m_lidarToAxis[1]), sin(m_lidarToAxis[0])*sin(m_lidarToAxis[1]), m_lidarToAxis[2],
                  -sin(m_lidarToAxis[0]), 0, cos(m_lidarToAxis[0]), m_lidarToAxis[3],
                  0, 0, 0, 1;
        return toAxis;
    }

public:
    EncoderCalib(): calibReady(false), init(false), compensateReady(false)
    {
        // Subscribe
        m_subEncoder = nh.subscribe<tf::tfMessage>("tf", 1000, &EncoderCalib::encoderHandler, this,  ros::TransportHints().tcpNoDelay());
        m_subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 20, &EncoderCalib::laserCloudHandler, this, ros::TransportHints().tcpNoDelay());
        
        // Publisher
        m_pubPlaneCloud = nh.advertise<sensor_msgs::PointCloud2>("planeCloud", 1);
        m_compensatedCloud = nh.advertise<sensor_msgs::PointCloud2>("compensatedCloud", 1);

        // Init the range of the plane
        nh.param<double>("x_min", m_x_min, -0.5);
        nh.param<double>("y_min", m_y_min, -0.5);
        nh.param<double>("x_max", m_x_max, 1.7);
        nh.param<double>("x_max", m_y_max, 2.3);
        nh.param<double>("ransac_threshold", m_ransac_threshold, 0.01);

        // Initial value
        m_lidarToAxis << 0.0, 0.0, 0.0, 0.0;
        m_lastlidarToAxis << 0.0, 0.0, 0.0, 0.0;
    }

    ~EncoderCalib(){};


    void compensate(PointCloud::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud, geometry_msgs::TransformStamped startPose, geometry_msgs::TransformStamped endPose)
    {
        Eigen::Quaterniond start_r(startPose.transform.rotation.w, startPose.transform.rotation.x, startPose.transform.rotation.y, startPose.transform.rotation.z);
        Eigen::Quaterniond end_r(endPose.transform.rotation.w, endPose.transform.rotation.x, endPose.transform.rotation.y, endPose.transform.rotation.z);
        double duration = (endPose.header.stamp - startPose.header.stamp).toSec();
        double beginTime = getBeginTime(cloud);

        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            auto point = cloud->points[i];
            double rate = (point.timestamp - beginTime) / duration;
            // std::cout << point.timestamp << " " << beginTime << " " << rate << std::endl;
            Eigen::Quaterniond cur_r = start_r.slerp(rate, end_r);
            Eigen::Matrix4d cur_r_4d = Eigen::Matrix4d::Identity();
            cur_r_4d.block<3,3>(0,0) = cur_r.matrix();

            Eigen::Vector4d v_point(point.x, point.y, point.z, 1);
            v_point = cur_r_4d * lidarToAxisMatrix() * v_point;

            out_cloud->points[i].x = v_point(0);
            out_cloud->points[i].y = v_point(1);
            out_cloud->points[i].z = v_point(2);
        }

    }
        
    bool preserveCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        m_cloudCacheQueue.push_back(*laserCloudMsg);
        if (m_cloudCacheQueue.size() <= 5)
            return false;
        
        return true;
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // Test if calibReady
        if (calibReady)
            return;

        // store cloud
        if (!preserveCloud(laserCloudMsg) || m_encoderCacheQueue.empty())
            return;

        if (!init)
        {
            if (getAndPopEncoderTf(m_cloudCacheQueue.front().header.stamp, m_encoderCacheQueue, m_lastEncoderMsg))
            {
                m_cloudCacheQueue.pop_front();
                init = true;
                return;
            }
        }
        
        // Get the corresponding cloud and encoder msg
        sensor_msgs::PointCloud2 curCloudMsg = std::move(m_cloudCacheQueue.front());
        m_cloudCacheQueue.pop_front();

        if (!getAndPopEncoderTf(curCloudMsg.header.stamp, m_encoderCacheQueue, m_curEncoderMsg))
            return;

        if ((m_curEncoderMsg.header.stamp - m_lastEncoderMsg.header.stamp).toSec() > 0.15)
        {
            m_lastEncoderMsg = m_curEncoderMsg;
            ROS_WARN("Encoder gap too long");
            return;
        }
        
        ROS_INFO("Laser_begin:%d %.9d", curCloudMsg.header.stamp.sec, curCloudMsg.header.stamp.nsec);
        ROS_INFO("Last Encoder Time:%d %.9d", m_lastEncoderMsg.header.stamp.sec, m_lastEncoderMsg.header.stamp.nsec);
        ROS_INFO("Cur Encoder Time:%d %.9d", m_curEncoderMsg.header.stamp.sec, m_curEncoderMsg.header.stamp.nsec);
        ROS_INFO("Last Encoder value:x:%f,y:%f, z:%f, w:%f", m_lastEncoderMsg.transform.rotation.x, m_lastEncoderMsg.transform.rotation.y, m_lastEncoderMsg.transform.rotation.z, m_lastEncoderMsg.transform.rotation.w);
        ROS_INFO("Cur Encoder value:x:%f,y:%f, z:%f, w:%f", m_curEncoderMsg.transform.rotation.x, m_curEncoderMsg.transform.rotation.y, m_curEncoderMsg.transform.rotation.z, m_curEncoderMsg.transform.rotation.w);
        
        ROS_INFO("ADD");
        // Transform to PointCloud type
        PointCloud::Ptr laserCloud(new PointCloud);
        pcl::fromROSMsg(curCloudMsg, *laserCloud);

        // Store the corresponding cloud and encoder msg
        m_encoderStartVec.push_back(m_lastEncoderMsg);
        m_encoderEndVec.push_back(m_curEncoderMsg);
        m_cloudVec.push_back(laserCloud);
        ROS_ASSERT (m_cloudVec.size() == m_encoderEndVec.size());
        ROS_ASSERT (m_cloudVec.size() == m_encoderStartVec.size());

        // Enough data, calib ready
        if (m_encoderStartVec.size() > 40)
        {
            calibReady = true;
        }
        
    
    }

    bool ready()
    {
        return calibReady;
    }

    void encoderHandler(const tf::tfMessageConstPtr& encoderMsg)
    {
        m_encoderCacheQueue.push_back(encoderMsg->transforms[0]);
    }

    void optimizePlane()
    {
        m_planeVec.clear();
        ros::Rate r(10);

        for (size_t i = 0; i < m_cloudVec.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);
            cloudxyz->points.resize(m_cloudVec[i]->points.size());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_compensated(new pcl::PointCloud<pcl::PointXYZ>);
            cloud_compensated->points.resize(m_cloudVec[i]->points.size());

            if (compensateReady)
            {
                Eigen::Matrix4d transform_back;
                transform_back = lidarToAxisMatrix().inverse() *fromMsgToMatrix(m_encoderEndVec[i]).inverse() ;
                compensate(m_cloudVec[i], cloud_compensated, m_encoderStartVec[i], m_encoderEndVec[i]);
                pcl::transformPointCloud(*cloud_compensated, *cloudxyz, transform_back);

                sensor_msgs::PointCloud2 compensated_msg;
                pcl::toROSMsg(*cloud_compensated, compensated_msg);
                compensated_msg.header.frame_id = "encoder_fixed";
                compensated_msg.header.stamp = ros::Time(0);
                m_compensatedCloud.publish(compensated_msg);

            }else
            {
                pcl::copyPointCloud(*m_cloudVec[i], *cloudxyz);
            }


            // Filter by x
            pcl::PassThrough<pcl::PointXYZ> pass_x;
            pass_x.setInputCloud(cloudxyz);
            pass_x.setFilterFieldName("x");
            pass_x.setFilterLimits(m_x_min, m_x_max);
            pass_x.filter(*cloudxyz);

            // Filter by y            z
            pcl::PassThrough<pcl::PointXYZ> pass_y;
            pass_x.setInputCloud(cloudxyz);
            pass_x.setFilterFieldName("y");
            pass_x.setFilterLimits(m_y_min, m_y_max);
            pass_x.filter(*cloudxyz);

            // Generate plane
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
                new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloudxyz));
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
            ransac.setDistanceThreshold(m_ransac_threshold);

            if (ransac.computeModel())
            {
                // Get plane coefficient
                Eigen::VectorXf plane_coef;
                ransac.getModelCoefficients(plane_coef);
                if (plane_coef[3] < 0)
                {
                    plane_coef = -plane_coef;
                } 

                // Publish plane cloud
                std::vector<int> inlier_indicies;
                PointCloud::Ptr plane_cloud(new PointCloud);
                sensor_msgs::PointCloud2 out_cloud;

                ransac.getInliers(inlier_indicies);
                pcl::copyPointCloud<pcl::PointXYZ>(*cloudxyz, inlier_indicies, *plane_cloud);
                pcl::toROSMsg(*plane_cloud, out_cloud);

                out_cloud.header.frame_id = "encoder_fixed";
                out_cloud.header.stamp = ros::Time(0);
                m_pubPlaneCloud.publish(out_cloud);
                r.sleep();
                // Store plane coefficient
                m_planeVec.push_back(plane_coef.cast<double>());
            }
        }
        
    }

    void optimizeExtrinsic()
    {
        ceres::Problem problem;
        ceres::Solver::Options option;
        ceres::Solver::Summary summary;
        option.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        option.linear_solver_type = ceres::DENSE_QR;
        option.minimizer_progress_to_stdout = true;
    
        for (size_t i = 0; i < m_planeVec.size() - 1; i++)
        {
            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PlanePlaneFactor, 2, 4>(
                new PlanePlaneFactor(m_planeVec[i], m_planeVec[i+1], fromMsgToMatrix(m_encoderEndVec[i]), fromMsgToMatrix(m_encoderEndVec[i+1])));
            problem.AddResidualBlock(cost_function, nullptr, m_lidarToAxis.data());
        }

        ceres::Solve(option, &problem, &summary);

        std::cout << summary.BriefReport() <<"\n";
        std::cout << m_lidarToAxis <<"\n";

        compensateReady = true;
    }
};



int main(int argc, char** argv)
{

    ros::init(argc, argv, "encoderCalib");
    google::InitGoogleLogging(argv[0]);
    EncoderCalib calib;

    ros::Rate r(0.01);

    while(ros::ok())
    {
        if (calib.ready())
        {
            calib.optimizePlane();

            calib.optimizeExtrinsic();
            // break;
            r.sleep();
        }
        
        ros::spinOnce();
    }


    return 0;
}