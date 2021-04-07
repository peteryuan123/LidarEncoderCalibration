#include "utility.h"

#include <pcl-1.10/pcl/sample_consensus/ransac.h>
#include <pcl-1.10/pcl/sample_consensus/sac_model_plane.h>
#include <pcl-1.10/pcl/common/transforms.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Geometry>
#include <ceres/autodiff_cost_function.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

struct PlaneErrorFactor
{
    
    PlaneErrorFactor(Eigen::Vector4d fixFramePlane, Eigen::Vector4d curFramePlane, double rotationAngle)
        : _fixFramePlane(fixFramePlane), _curFramePlane(curFramePlane), _rotationAngle(rotationAngle)
    {}

    template <typename T>
    bool operator() (const T *R_t, T *residual) const
    {
        T theta_y = R_t[0], theta_z = R_t[1];
        T y = R_t[2], z = R_t[3];

        Eigen::Matrix<T, 4, 4> T_lidar_to_axis;
        Eigen::Matrix<T, 4, 4> T_axis_rotate;
        Eigen::Matrix<T, 4, 1> fixFramePlane_T;
        Eigen::Matrix<T, 4, 1> curFramePlane_T;
        fixFramePlane_T << T(_fixFramePlane[0]), T(_fixFramePlane[1]), T(_fixFramePlane[2]), T(_fixFramePlane[3]);
        curFramePlane_T << T(_curFramePlane[0]), T(_curFramePlane[1]), T(_curFramePlane[2]), T(_curFramePlane[3]);
        // Eigen::Matrix<T, 4, 1> fixFramePlane_T = Eigen::Map<Eigen::Matrix<T, 4, 1>>((T*)(_fixFramePlane.data()));
        // Eigen::Matrix<T, 4, 1> curFramePlane_T = Eigen::Map<Eigen::Matrix<T, 4, 1>>((T*)(_curFramePlane.data()));

        // T_lidar_to_axis << ceres::cos(theta_z), -ceres::sin(theta_z), T(0) , T(0),
        //                    ceres::cos(theta_y)*ceres::sin(theta_z), ceres::cos(theta_y)*ceres::cos(theta_z), -ceres::sin(theta_y), y,
        //                    ceres::sin(theta_y)*ceres::sin(theta_z), ceres::cos(theta_z)*ceres::sin(theta_y), ceres::cos(theta_y), z,
        //                    T(0), T(0), T(0), T(1);

        T_lidar_to_axis << ceres::cos(R_t[0])*ceres::cos(R_t[1]), -ceres::sin(R_t[1]), ceres::cos(R_t[1])*ceres::sin(R_t[0]), T(0),
                           ceres::sin(R_t[1])*ceres::cos(R_t[0]), ceres::cos(R_t[1]), ceres::sin(R_t[0])*ceres::sin(R_t[1]), R_t[2],
                           -ceres::sin(R_t[0]), T(0), ceres::cos(R_t[0]), R_t[3],
                           T(0), T(0), T(0), T(1);
        
        T_axis_rotate << T(1), T(0), T(0), T(0),
                         T(0), T(cos(_rotationAngle)), -T(sin(_rotationAngle)), T(0),
                         T(0), T(sin(_rotationAngle)), T(cos(_rotationAngle)), T(0),
                         T(0), T(0), T(0), T(1);
        

        
        // Eigen::Matrix<T, 4, 1> curPlaneInAxis = (T_lidar_to_axis.inverse() *  T_axis_rotate.inverse()).transpose() * curFramePlane_T;
        // Eigen::Matrix<T, 4, 1> fixPlaneInAxis = (T_lidar_to_axis.inverse() *  T_axis_rotate.inverse()).transpose() * fixFramePlane_T;
        // residual[0] = (curPlaneInAxis - fixPlaneInAxis).squaredNorm();

        Eigen::Matrix<T, 4, 1> curPlaneInFixFrame = (T_lidar_to_axis.inverse() * T_axis_rotate.inverse() * T_lidar_to_axis).transpose() * curFramePlane_T;
        Eigen::Matrix<T, 3, 1> curNorm(curPlaneInFixFrame(0), curPlaneInFixFrame(1), curPlaneInFixFrame(2));
        Eigen::Matrix<T, 3, 1> fixNorm(fixFramePlane_T(0), fixFramePlane_T(1), fixFramePlane_T(2));

        residual[0] = T(1) - ceres::abs(curNorm.normalized().dot(fixNorm.normalized()));
        residual[1] = curPlaneInFixFrame[3] - fixFramePlane_T[3];
        
        // residual[0] = T(1) - ceres::abs(curNorm.normalized().dot(fixNorm.normalized())) ;
        // residual[1] =  + curPlaneInFixFrame(3) - fixFramePlane_T(3);
        // residual[0] = (curPlaneInFixFrame - fixFramePlane_T).squaredNorm();

        // std::cout << _rotationAngle << std::endl;
        // std::cout << R_t[0] << " " <<  R_t[1] << " " <<  R_t[2] << " "<< R_t[3] << std::endl;
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
        return true;
    }

    Eigen::Vector4d _fixFramePlane, _curFramePlane;
    double _rotationAngle;
};

int main(int argc, char** argv)
{
    double cloud_num = 7;
    // Initial rt(theta_y, theta_z, y, z)
    Eigen::Vector4d initial_rt(0, 0, 0, 0.); 

    google::InitGoogleLogging(argv[0]);

    // store cloud
    std::vector<PointCloud::Ptr> data;
    for (int i = 0; i < cloud_num*10; i+=10) 
    {
        PointCloud::Ptr cloud(new PointCloud);
        pcl::io::loadPLYFile("/home/stereye/code/RotatingLidar/generated_plane/angle_" + std::to_string(i) + ".ply", *cloud);
        data.push_back(cloud);
    }

    // Get the first plane;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p_0(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(data[0]));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_0(model_p_0);
    Eigen::VectorXf plane_0;
    ransac_0.setDistanceThreshold(0.01);
    if (ransac_0.computeModel(1))
    {
        ransac_0.getModelCoefficients(plane_0);
    }
    std::cout << plane_0.transpose() << std::endl;


    // Add residual block
    ceres::Problem problem;
    ceres::Solver::Options option;

    for (double i = 1.0; i < cloud_num; i++)
    {
        // Get plane each rotation
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(data[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        Eigen::VectorXf plane;
    
        ransac.setDistanceThreshold(0.01);
        if (ransac.computeModel(1))
        {
            ransac.getModelCoefficients(plane);
        }

        double angle = 10.0 * i / 180.0 * M_PI;

        // Add residual block
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PlaneErrorFactor, 2, 4>(new PlaneErrorFactor(plane_0.cast<double>(), plane.cast<double>(), -angle));
        problem.AddResidualBlock(cost_function, nullptr, initial_rt.data());

        // test
        Eigen::Matrix4d T_axis_rotate;
        Eigen::Matrix4d T_lidar_to_axis;
        T_lidar_to_axis <<  0.877583 , 0.479426 ,0, 0.3,
                -0.479426,  0.877583, 0.0000000, 0.02, 
                0.0000000,  0.0000000,  1.0000000, 0.02,
                0, 0, 0, 1;
        double angle_i = -i*10.0 / 180.0 * M_PI;
        T_axis_rotate << 1, 0, 0, 0,
                        0, cos(angle_i), -sin(angle_i), 0,
                        0, sin(angle_i), cos(angle_i), 0,
                        0, 0, 0, 1; 

        std::cout << plane.transpose() << std::endl;
        std::cout << ((T_lidar_to_axis.inverse() * T_axis_rotate.inverse() * T_lidar_to_axis).transpose() * plane.cast<double>()).transpose() << std::endl;
        std::cout << "-----------\n";
    }
    std::cout << "plane test done\n";


    // Optimize
    // option.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
    option.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    option.linear_solver_type = ceres::DENSE_QR;
    option.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(option, &problem, &summary);

    // Test
    std::cout << summary.FullReport() <<"\n";
    std::cout << initial_rt.transpose() <<  std::endl;

    // Test result
    for (int i = 0; i < cloud_num; i++)
    {
        PointCloud::Ptr out_cloud(new PointCloud);
        Eigen::Matrix4d T_lidar_to_axis;
        Eigen::Matrix4d T_axis_rotate;

        T_lidar_to_axis << cos(initial_rt[0])*cos(initial_rt[1]), -sin(initial_rt[1]), cos(initial_rt[1])*sin(initial_rt[0]), 0,
                           sin(initial_rt[1])*cos(initial_rt[0]), cos(initial_rt[1]), sin(initial_rt[0])*sin(initial_rt[1]), initial_rt[2],
                           -sin(initial_rt[0]), 0, cos(initial_rt[0]), initial_rt[3],
                           0, 0, 0, 1;
        // T_lidar_to_axis <<  0.877583 , 0.479426 ,0, 0.3,
        //         -0.479426,  0.877583, 0.0000000, 0.02, 
        //         0.0000000,  0.0000000,  1.0000000, 0.02,
        //         0, 0, 0, 1;
        double angle = -i*10.0 / 180.0 * M_PI;
        std::cout << angle << std::endl;
        T_axis_rotate << 1, 0, 0, 0,
                        0, cos(angle), -sin(angle), 0,
                        0, sin(angle), cos(angle), 0,
                        0, 0, 0, 1; 
        // std::cout << T_axis_rotate << std::endl;
        // Eigen::Matrix4d transform = T_axis_rotate*T_lidar_to_axis;
        Eigen::Matrix4d transform =  T_axis_rotate * T_lidar_to_axis;

        pcl::transformPointCloud(*data[i], *out_cloud, transform);

        pcl::io::savePLYFile("/home/stereye/code/RotatingLidar/generated_plane/" + std::to_string(i) + ".ply", *out_cloud);
    }

    return 0;
}