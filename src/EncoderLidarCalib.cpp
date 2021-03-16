// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>

// #include <pcl-1.8/pcl/io/pcd_io.h>
// #include <pcl-1.8/pcl/registration/icp.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <boost/thread/thread.hpp>
// #include <pcl/console/parse.h>

#include "utility.h"

typedef pcl::PointXYZ PointType;

// bool next_iteration = false;
// //设置键盘交互函数
// void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing)
// {
//         if(event.getKeySym() == "space" && event.keyDown())
//                 next_iteration = true;
// }

Eigen::Matrix4d LeftQuaternion(Eigen::Quaterniond q)
{
    Eigen::Matrix4d left_quaternion_matrix;

    left_quaternion_matrix << q.w(), -q.x(), -q.y(), -q.z(),
                              q.x(), q.w(), -q.z(), q.y(),
                              q.y(), q.z(), q.w(), -q.x(),
                              q.z(), -q.y(), q.x(), q.w();

    return left_quaternion_matrix;

}

Eigen::Matrix4d RightQuaternion(Eigen::Quaterniond q)
{
    Eigen::Matrix4d right_quaternion_matrix;

    right_quaternion_matrix <<  q.w(), -q.x(), -q.y(), -q.z(),
                                q.x(), q.w(), q.z(), -q.y(),
                                q.y(), -q.z(), q.w(), q.x(),
                                q.z(), q.y(), -q.x(), q.w();

    return right_quaternion_matrix;
}


int main(int argc, char **argv)
{
    Eigen::Matrix4d _0_to_1 = read_transform("/home/stereye/data/geoslam/3-5/0-1.txt");
    Eigen::Matrix4d _2_to_3 = read_transform("/home/stereye/data/geoslam/3-5/2-3.txt");
    Eigen::Matrix4d _4_to_5 = read_transform("/home/stereye/data/geoslam/3-5/4-5.txt");

    Eigen::Quaterniond e0 = Eigen::Quaterniond(0.018984189295, 0.99981978404, 0, 0);
    Eigen::Quaterniond e1 = Eigen::Quaterniond(-0.999973447759, 0.00728723380709, 0, 0);
    Eigen::Quaterniond e2 = Eigen::Quaterniond(-0.999899308675, 0.0141905782858, 0, 0);
    Eigen::Quaterniond e3 = Eigen::Quaterniond(-0.00441071888328, 0.999990272732, 0, 0);
    Eigen::Quaterniond e4 = Eigen::Quaterniond(0.299664133508, 0.954044761575, 0, 0);
    Eigen::Quaterniond e5 = Eigen::Quaterniond(-0.888318288005, 459228286581, 0, 0);

    Eigen::Quaterniond e0_to_1 = e1*e0.inverse();
    Eigen::Quaterniond e2_to_3 = e2*e3.inverse();
    Eigen::Quaterniond e4_to_5 = e4*e5.inverse();

    Eigen::Quaterniond lidar0_to_1_q = Eigen::Quaterniond(_0_to_1.block<3,3>(0,0));
    Eigen::Quaterniond lidar2_to_3_q = Eigen::Quaterniond(_2_to_3.block<3,3>(0,0));
    Eigen::Quaterniond lidar4_to_5_q = Eigen::Quaterniond(_4_to_5.block<3,3>(0,0));

    Eigen::Quaterniond q_fix(0,0.957591,0.288116,-0.00282002);

    std::cout << q_fix.matrix() << "\n-------------\n";

    std::cout << e0_to_1.matrix() << std::endl;



    Eigen::Matrix<double,8,4> Q;
    Q.block<4,4>(0,0) = LeftQuaternion(lidar0_to_1_q) - RightQuaternion(e0_to_1);
    // Q.block<4,4>(4,0) = LeftQuaternion(lidar2_to_3_q) - RightQuaternion(e2_to_3);
    Q.block<4,4>(4,0) = LeftQuaternion(lidar4_to_5_q) - RightQuaternion(e4_to_5);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    std::cout << svd.singularValues() << std::endl;
    std::cout << "----------------\n";
    std::cout << svd.matrixV() <<std::endl;
    std::cout << "----------------\n";


    Eigen::Quaterniond r(svd.matrixV()(0,3), svd.matrixV()(1,3), svd.matrixV()(2,3), svd.matrixV()(3,3));

    Eigen::Matrix<double, 3, 3> R;
    R.block<3,3>(0,0) = _0_to_1.block<3,3>(0,0);
    // R.block<3,3>(3,0) = _2_to_3.block<3,3>(0,0);
    // R.block<3,3>(6,0) = _4_to_5.block<3,3>(0,0);
    Eigen::Matrix<double, 3, 1> t;
    t.block<3,1>(0,0) = _0_to_1.block<3,1>(0,3);
    // t.block<3,1>(3,0) = _2_to_3.block<3,1>(0,3);
    // t.block<3,1>(6,0) = _4_to_5.block<3,1>(0,3);

    Eigen::Matrix<double, 3, 3> i;
    i.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    // i.block<3,3>(3,0) = Eigen::Matrix3d::Identity();
    // i.block<3,3>(6,0) = Eigen::Matrix3d::Identity();
    std::cout << R << std::endl;
    std::cout << "--------------" << std::endl;
    std::cout << i << std::endl;

    R = R - i;
    std::cout << R << std::endl;
    std::cout << "--------------" << std::endl;

    std::cout << t << std::endl;

    std::cout << "--------------" << std::endl;

    std::cout << R.transpose() * R;


}

// int main(int argc, char**argv)
// {
//     pcl::PointCloud<PointType>::Ptr CloudRef (new pcl::PointCloud<PointType>);
//     pcl::PointCloud<PointType>::Ptr CloudAlign (new pcl::PointCloud<PointType>);
//     pcl::PointCloud<PointType>::Ptr final (new pcl::PointCloud<PointType>);

//     pcl::IterativeClosestPoint<PointType,PointType> icp;

//     pcl::io::loadPCDFile("/home/stereye/data/geoslam/3-5/00/1614950474.506738000.pcd", *CloudRef);
//     pcl::io::loadPCDFile("/home/stereye/data/geoslam/3-5/01/1614950516.968498000.pcd", *CloudAlign);
    
//     icp.setInputSource(CloudAlign);
//     icp.setInputTarget(CloudRef);
//     icp.setMaxCorrespondenceDistance(100);
//     icp.setTransformationEpsilon(1e-10);
//     icp.setEuclideanFitnessEpsilon(0.001);
//     icp.setMaximumIterations(500);
//     icp.align(*final);

//     boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp test"));  //定义窗口共享指针
//     int v1 ; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
//     int v2 ;
//     view->createViewPort(0.0,0.0,6.0,6.0,v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
//     view->createViewPort(0.0,0.0,6.0,6.0,v2);

//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(CloudRef,250,0,0); //设置源点云的颜色为红色
//     view->addPointCloud(CloudRef,sources_cloud_color,"sources_cloud_v1",v1);
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color (CloudAlign,0,250,0);  //目标点云为绿色
//     view->addPointCloud(CloudAlign,target_cloud_color,"target_cloud_v1",v1); //将点云添加到v1窗口

//     view->setBackgroundColor(0.0,0.05,0.05,v1); //设着两个窗口的背景色
//     view->setBackgroundColor(0.05,0.05,0.05,v2);

//     view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sources_cloud_v1");  //设置显示点的大小
//     view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v1");

//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(final,255,255,255);  //设置配准结果为白色
//     view->addPointCloud(final,aligend_cloud_color,"aligend_cloud_v2",v2);
//     view->addPointCloud(CloudAlign,target_cloud_color,"target_cloud_v2",v2);

//     view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"aligend_cloud_v2");
//     view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v2");

//     view->registerKeyboardCallback(&keyboardEvent,(void*)NULL);  //设置键盘回调函数
//     int iterations = 0; //迭代次数
//     while(!view->wasStopped())
//     {
//             view->spinOnce();  //运行视图
//             if (next_iteration)
//             {
//                     icp.align(*final);  //icp计算
//                     cout <<"has conveged:"<<icp.hasConverged()<<"score:"<<icp.getFitnessScore()<<endl;
//                     cout<<"matrix:\n"<<icp.getFinalTransformation()<<endl;
//                     cout<<"iteration = "<<++iterations;
//                     /*... 如果icp.hasConverged=1,则说明本次配准成功，icp.getFinalTransformation()可输出变换矩阵   ...*/
//                     if (iterations == 1000)  //设置最大迭代次数
//                             return 0;
//                     view->updatePointCloud(final,aligend_cloud_color,"aligend_cloud_v2");
                    
//             }
//             next_iteration = false;  //本次迭代结束，等待触发

//     }
// }