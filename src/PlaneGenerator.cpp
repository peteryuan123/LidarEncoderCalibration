#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>


#include <string.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

Eigen::Vector4d read_plane(std::string file)
{
    std::ifstream src;
    Eigen::Vector4d plane;

    src.open(file);
    src >> plane(0) >> plane(1) >> plane(2) >> plane(3);
    src.close();

    return plane;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "planeGenerator");
    ros::NodeHandle n;
    // Indicate the plane equation
    Eigen::Vector4d plane = read_plane("/home/stereye/code/RotatingLidar/RotatinLidar_ws/src/config/plane/plane.txt");

    /* Indicate the extrinsic between lidar frame and encoder frame */
    Eigen::AngleAxisd yaw(0.01, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch(0.01, Eigen::Vector3d::UnitY());
    std::cout << yaw.matrix() << std::endl;

    Eigen::Matrix4d lidar_in_encoder;
    lidar_in_encoder <<  0.8337121,  0.5521994,  0.0000000, 0.3,
                        -0.5521994,  0.8337121,  0.0000000, 0.005, 
                        0.0000000,  0.0000000,  1.0000000, 0.005,
                        0, 0, 0, 1;
    lidar_in_encoder.block<3,3>(0,0) = yaw.matrix() * pitch.matrix() ;
    std::cout << lidar_in_encoder << std::endl;

    // Rotation matrix
    double angle = std::atof(argv[1]);
    angle = angle / 180 * M_PI;
    Eigen::AngleAxisd rotation_v(angle, Eigen::Vector3d::UnitX());
    Eigen::Matrix4d rotation4d = Eigen::Matrix4d::Identity();
    rotation4d.block<3,3>(0,0) = rotation_v.matrix();

    // Generate point plane
    PointCloud::Ptr plane_cloud(new PointCloud);
    plane_cloud->points.resize(40000);
    std::cout <<plane<<"\n";
    size_t count = 0;
    for (double x = 0; x < 5; x += 0.025)
    {
        for (double y = 0; y < 5; y += 0.025)
        {
            double z = (plane(3) - plane(0) * x - plane(1) * y) / plane(2);
            plane_cloud->points[count].x = x;
            plane_cloud->points[count].y = y;
            plane_cloud->points[count].z = z;
            count++;
        }
    }

    // lidar_0 -> axis -> axis_rotate -> lidar_1
    Eigen::Matrix4d transform =  lidar_in_encoder.inverse() * rotation4d * lidar_in_encoder;
    
    
    std::cout << transform << std::endl;
    std::cout << rotation4d << std::endl;
    std::cout << rotation4d.inverse() <<std::endl;
    // std::cout << plane_cloud->points[0].x << std::endl;

    // pcl::io::savePLYFile("/home/stereye/code/RotatingLidar/generated_plane/angle1_" + std::string(argv[1]) + ".ply", *plane_cloud);

    // transform and save cloud
    PointCloud::Ptr out(new PointCloud);
    out->points.resize(40000);
    pcl::transformPointCloud(*plane_cloud, *out, transform, true);
    pcl::io::savePLYFile("/home/stereye/code/RotatingLidar/generated_plane/angle_" + std::string(argv[1]) + ".ply", *out);
}