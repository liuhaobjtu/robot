/**
 * @Author: liuhao
 * @CreateTime: 2020-10-29
 * @Description:
 */
#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"       // 迭代最近点算法icp配准的头文件
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/filters/voxel_grid.h"

#include <sys/time.h>    // gettimeofday settimeofday
#include <time.h>    // localtime gmtime asctime time

int main() {    //int argc, char **argv
    int argc = 4;
    char *temp[] = {"_", "../../../data/cloud_002.pcd", "../../../data/cloud_003.pcd", "0.3"};
    char **argv = temp;
    // ./01_iterative_closest_point2 ../../../data/cloud_002.pcd ../../../data/cloud_003.pcd 0.3

    // 定义输入和输出点云
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

    // 将pcd中的数据加载到cloud中
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_source) == -1) {
        PCL_ERROR ("Couldn't read file cloud_002.pcd \n");      // PCL_ERROR是PCL的宏定义
        return (-1);
    }

    // 将pcd中的数据加载到cloud中
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_target) == -1) {
        PCL_ERROR ("Couldn't read file cloud_003.pcd \n");      // PCL_ERROR是PCL的宏定义
        return (-1);
    }

    //===================================================================================================
    // ... and downsampling the point cloud
    // 降采样点云, 减少计算量
    // 定义体素大小
    const float voxel_grid_size = atof(argv[3]);
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;

    vox_grid.setInputCloud(cloud_source);
    // 设置叶子节点的大小lx, ly, lz
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud);
    cloud_source = tempCloud;


    vox_grid.setInputCloud(cloud_target);
    // 设置叶子节点的大小lx, ly, lz
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud2);
    cloud_target = tempCloud2;

    //===================================================================================================
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setWindowName("3D Viewer2");

    // 设置背景色为灰色（非必须）
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_source, 0, 0, 255);   // 设置点云颜色
    viewer->addPointCloud(cloud_source, single_color, "cloud_002");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_002");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud_target, 255, 0, 0);
    viewer->addPointCloud(cloud_target, rgb, "cloud_003");  // 可以去掉rgb
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_003");

    // 添加一个0.5倍缩放的坐标系（非必须）
    viewer->addCoordinateSystem(2.0);

    while (!viewer->wasStopped()) {
        // 每次循环调用内部的重绘函数
        viewer->spinOnce();
    }

    //===================================================================================================
    // 创建IterativeClosestPoint的实例
    // setInputSource将cloud_in作为输入点云
    // setInputTarget将平移后的cloud_out作为目标点云
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setMaximumIterations(100);//设置迭代次数
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);

    // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
    // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
    // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
    // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
    pcl::PointCloud<pcl::PointXYZ> Final;       // TODO Final存储配准变换后的源点云,跟cloud_out很接近

    struct timeval tv_start{};
    gettimeofday(&tv_start, nullptr);    // TODO (从1970年开始计算的)
    double start = tv_start.tv_sec * 1000 + tv_start.tv_usec * 0.001;
//    printf("timeval获取到的时间：%f\n", start);

    icp.align(Final);                           // align是重载函数,icp和模板匹配是同一个align函数
//    icp.setMaximumIterations(1);  //  当再次调用.align ()函数时，我们设置该变量为1。

    struct timeval tv_end{};
    gettimeofday(&tv_end, nullptr);    // TODO (从1970年开始计算的)
    double end = tv_end.tv_sec * 1000 + tv_end.tv_usec * 0.001;
//    printf("timeval获取到的时间：%f\n", end);

    printf("耗时：%f毫秒\n", end - start);

//    Eigen::Matrix4f guess;      // TODO 自己加的猜测矩阵，用来测试
//    guess << 1, 0, 0, 0.7,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
//    guess.resize(4,4);
//    icp.align(Final, guess);

    //TODO icp.getFitnessScore()越小越好
    std::cout << "has converged:" << icp.hasConverged() << " 　　误差: " << icp.getFitnessScore() << std::endl;

//    // 从原点云cloud_in变换到目标点云cloud_out
//    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();

    // TODO Matrix4可以简单些吗
    Eigen::Matrix4f matrix = icp.getFinalTransformation();      // TODO 自己想到的办法
    std::cout << "输出变换矩阵:" << std::endl << matrix << std::endl;

//    // 保存滤波&降采样后的点云图
    pcl::io::savePCDFileBinary("Final.pcd", Final);
    std::cout << "icp.pcd saved" << std::endl;

    //===================================================================================================

    return 0;
}

//耗时：124.723145毫秒
//has converged:1 　　误差: 0.0674022
//输出变换矩阵:
//0.99977 -0.0162021 -0.0140964 -0.0631661
//0.0143029   0.991961  -0.125739   0.131599
//0.0160204   0.125508   0.991964  0.0590191
//0          0          0          1
//icp.pcd saved
