#include "utility.h"

#include <pcl-1.10/pcl/sample_consensus/ransac.h>
#include <pcl-1.10/pcl/sample_consensus/sac_model_plane.h>
#include <pcl-1.10/pcl/common/transforms.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Geometry>
#include <ceres/autodiff_cost_function.h>
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;


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
        


        // Transform plane1 to plane0 coordinate
        Eigen::Matrix<T, 4, 1> firstPlaneInAxis = (T_lidar_to_axis.inverse() * T_axis_rotate_first.inverse()).transpose() * firstFramePlane_T;
        Eigen::Matrix<T, 4, 1> secondPlaneInAxis = (T_lidar_to_axis.inverse() * T_axis_rotate_second.inverse()).transpose() * secondFramePlane_T;

        // Residual
        Eigen::Matrix<T, 3, 1> curNorm(firstPlaneInAxis(0), firstPlaneInAxis(1), firstPlaneInAxis(2));
        Eigen::Matrix<T, 3, 1> fixNorm(secondPlaneInAxis(0), secondPlaneInAxis(1), secondPlaneInAxis(2));
        // residual[0] = (firstPlaneInAxis[0] - secondPlaneInAxis[0]);
        // residual[1] = (firstPlaneInAxis[1] - secondPlaneInAxis[1]);
        // residual[2] = (firstPlaneInAxis[2] - secondPlaneInAxis[2]);
        // residual[3] = (firstPlaneInAxis[3] - secondPlaneInAxis[3]);
        residual[0] = T(1) - ceres::abs(curNorm.normalized().dot(fixNorm.normalized())) + ceres::abs(firstPlaneInAxis[3] - secondPlaneInAxis[3]) ;
        // residual[1] = ;

        // std::cout << "############\n";
        // std::cout << "residual[0]:\n";
        // std::cout << residual[0] << std::endl;
        // std::cout << "curNorm:\n";
        // std::cout << curNorm.transpose()[0] << ", " <<  curNorm.transpose()[1] << ", " << curNorm.transpose()[2] << std::endl;
        // std::cout << "------------\n";
        // std::cout << "fixNorm:\n";
        // std::cout << fixNorm.transpose()[0] << ", " <<  fixNorm.transpose()[1] << ", " << fixNorm.transpose()[2] << std::endl;
        // std::cout << "------------\n";
        // std::cout << "firstPlaneInAxis[3]:\n";
        // std::cout << firstPlaneInAxis[3] << std::endl;
        // std::cout << "------------\n";
        // std::cout << "secondPlaneInAxis[3]\n";
        // std::cout << secondPlaneInAxis[3] << std::endl;
        // std::cout << "------------\n";
        return true;
    }

    Eigen::Vector4d m_firstFramePlane, m_secondFramePlane;
    Eigen::Matrix4d m_firstEncoder, m_secondEncoder;
};

int main(int argc, char** argv)
{
    double cloud_num = 7;
    // Initial rt(theta_y, theta_z, y, z)
    Eigen::Vector4d lidarToAxis(0, -0 , -0.15, 0.01); 

    google::InitGoogleLogging(argv[0]);

    // store cloud
    std::vector<PointCloud::Ptr> ground_data;
    std::vector<Eigen::Matrix4d> ground_encoders;
    std::vector<Eigen::Vector4d> ground_planes;
    std::vector<PointCloud::Ptr> whole;

    std::vector<PointCloud::Ptr> wall_data;
    std::vector<Eigen::Matrix4d> wall_encoders;
    std::vector<Eigen::Vector4d> wall_planes;

    

    // ground
    for (int i = 2560; i < 6145; i+=256) 
    {   
        std::string name = std::to_string(i);
        if (i < 1000)
        {
            name = std::to_string(0) + name;
        }
        PointCloud::Ptr cloud(new PointCloud);
        pcl::io::loadPCDFile("/home/stereye/data/livox/ground_plane/" + name +  ".pcd", *cloud);
        ground_data.push_back(cloud);

        PointCloud::Ptr cloud_whole(new PointCloud);
        pcl::io::loadPCDFile("/home/stereye/data/livox/whole/" + name +  ".pcd", *cloud_whole);
        whole.push_back(cloud_whole);

        double angle = static_cast<double>(i) / 8192.0 * 2.0 * M_PI;
        std::cout << angle << std::endl;
        Eigen::Matrix4d encoderMat;
        encoderMat << 1, 0, 0, 0,
                0, std::cos(angle), -std::sin(angle), 0,
                0, std::sin(angle), std::cos(angle), 0,
                0, 0, 0, 1;
        ground_encoders.push_back(encoderMat);
        // std::cout << encoderMat <<std::endl;
    }


    for (size_t i = 0; i < ground_data.size(); i++)
    {

        // Generate plane
        pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(ground_data[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
        ransac.setDistanceThreshold(0.01);

        if (ransac.computeModel())
        {
            // Get plane coefficient
            Eigen::VectorXf plane_coef;
            ransac.getModelCoefficients(plane_coef);
            if (plane_coef[3] < 0)
            {
                plane_coef = -plane_coef;
            } 

            ground_planes.push_back(plane_coef.cast<double>());
            std::cout << plane_coef.transpose() << std::endl;
        }
    }

    //wall
    for (int i = 768; i < 6656; i+=256) 
    {   
        std::string name = std::to_string(i);
        if (i < 1000)
        {
            name = std::to_string(0) + name;
        }
        PointCloud::Ptr cloud(new PointCloud);
        pcl::io::loadPCDFile("/home/stereye/data/livox/calib_decimal/" + name +  ".pcd", *cloud);
        wall_data.push_back(cloud);

        double angle = static_cast<double>(i) / 8192.0 * 2.0 * M_PI;
        std::cout << angle << std::endl;
        Eigen::Matrix4d encoderMat;
        encoderMat << 1, 0, 0, 0,
                0, std::cos(angle), -std::sin(angle), 0,
                0, std::sin(angle), std::cos(angle), 0,
                0, 0, 0, 1;
        wall_encoders.push_back(encoderMat);
        // std::cout << encoderMat <<std::endl;
    }


    for (size_t i = 0; i < wall_data.size(); i++)
    {

        // Generate plane
        pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(wall_data[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
        ransac.setDistanceThreshold(0.01);

        if (ransac.computeModel())
        {
            // Get plane coefficient
            Eigen::VectorXf plane_coef;
            ransac.getModelCoefficients(plane_coef);
            if (plane_coef[3] < 0)
            {
                plane_coef = -plane_coef;
            } 

            wall_planes.push_back(plane_coef.cast<double>());
            std::cout << plane_coef.transpose() << std::endl;
        }
    }   


    /* Solve */
    ceres::Problem problem;
    ceres::Solver::Options option;
    ceres::Solver::Summary summary;
    option.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    option.linear_solver_type = ceres::DENSE_QR;
    option.minimizer_progress_to_stdout = true;

    for (size_t i = 0; i < ground_encoders.size() - 1 ; i++)
    {
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PlanePlaneFactor, 1, 4>(
            new PlanePlaneFactor(ground_planes[i], ground_planes[i+1], ground_encoders[i], ground_encoders[i+1]));
        problem.AddResidualBlock(cost_function, nullptr, lidarToAxis.data());
    }

    for (size_t i = 0; i < wall_encoders.size() - 1 ; i++)
    {
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PlanePlaneFactor, 1, 4>(
            new PlanePlaneFactor(wall_planes[i], wall_planes[i+1], wall_encoders[i], wall_encoders[i+1]));
        problem.AddResidualBlock(cost_function, nullptr, lidarToAxis.data());
    }

    ceres::Solve(option, &problem, &summary);

    std::cout << summary.BriefReport() <<"\n";
    std::cout << lidarToAxis <<"\n";
    /* Solve */

    // lidarToAxis[0] = 0;
    // lidarToAxis[1] = 0;
    // lidarToAxis[2] = 0;
    // lidarToAxis[3] = 0;

    // Test result
    for (int i = 0; i < ground_encoders.size(); i++)
    {
        PointCloud::Ptr out_cloud(new PointCloud);
        Eigen::Matrix4d T_lidar_to_axis;

        T_lidar_to_axis << cos(lidarToAxis[0])*cos(lidarToAxis[1]), -sin(lidarToAxis[1]), cos(lidarToAxis[1])*sin(lidarToAxis[0]), 0,
                           sin(lidarToAxis[1])*cos(lidarToAxis[0]), cos(lidarToAxis[1]), sin(lidarToAxis[0])*sin(lidarToAxis[1]), lidarToAxis[2],
                           -sin(lidarToAxis[0]), 0, cos(lidarToAxis[0]), lidarToAxis[3],
                           0, 0, 0, 1;

        // std::cout << T_axis_rotate << std::endl;
        // Eigen::Matrix4d transform = T_axis_rotate*T_lidar_to_axis;
        Eigen::Matrix4d transform =  ground_encoders[i] * T_lidar_to_axis;

        pcl::transformPointCloud(*ground_data[i], *out_cloud, transform);

        pcl::io::savePLYFile("/home/stereye/data/livox/result/" + std::to_string(i) + ".ply", *out_cloud);

        // pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
        //     new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(out_cloud));
        // pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        // ransac.setDistanceThreshold(0.01);

        // if (ransac.computeModel())
        // {
        //     // Get plane coefficient
        //     Eigen::VectorXf plane_coef;
        //     ransac.getModelCoefficients(plane_coef);
        //     if (plane_coef[3] < 0)
        //     {
        //         plane_coef = -plane_coef;
        //     } 

        //     planes.push_back(plane_coef.cast<double>());
        //     std::cout << plane_coef.transpose() << std::endl;
        // }
    }


    return 0;
}