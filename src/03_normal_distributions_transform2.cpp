/**
 * @Author: liuhao
 * @CreateTime: 2020-10-29
 * @Description:
 */
#include <iostream>
#include <thread>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/registration/ndt.h"                   // 正态分布变换ndt配准的头文件
#include "pcl/filters/approximate_voxel_grid.h"     // 降采样的头文件

#include "pcl/visualization/pcl_visualizer.h"

//using namespace std::chrono_literals;   // C++14才有的

int main() {    // int argc, char **argv
    int argc = 3;
    char *temp[] = {"_", "../../../data/cloud_002.pcd", "../../../data/cloud_003.pcd"};
    char **argv = temp;
    // ./03_normal_distributions_transform2 ../../../data/cloud_002.pcd ../../../data/cloud_003.pcd
//===================================================================================================
    // Loading first scan of room.
    // 加载首次的房间扫描数据作为目标点云 target_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *target_cloud) == -1) {
        PCL_ERROR ("Couldn't read file cloud_003.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from cloud_003.pcd" << std::endl;

    // Loading second scan of room from new perspective.
    // 加载从新的视角得到的房间第二次扫描数据作为输入源点云 source_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *source_cloud) == -1) {
        PCL_ERROR ("Couldn't read file cloud_002.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << source_cloud->size() << " data points from cloud_002.pcd" << std::endl;
//===================================================================================================
    // 配准操作是完成 【源点云source_cloud】到【目标点云target_cloud】坐标系变换矩阵的估算
    // 即求出source_cloud变换到target_cloud的变换矩阵

    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    // 过滤输入点云到约10%的原始大小，以提高配准速度。
    // 这里用任何其他均匀过滤器都可以，目标点云target_cloud不需要进行滤波处理，
    // 因为NDT算法在目标点云对应的体素Voxel网格数据结构计算时，不使用单个点，而是使用体素的点。即已做了降采样处理。
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(2, 2, 2);
    approximate_voxel_filter.setInputCloud(source_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
              << " data points from cloud_002.pcd" << std::endl;
//===================================================================================================
    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(source_cloud, source_color, "source cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "source cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped()) {
        viewer_final->spinOnce();
    }


//===================================================================================================
    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(0.01); // 为终止条件设置最小转换差异
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.1);   // 为More-Thuente线搜索设置最大步长
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(1.0); // 设置NDT网格结构的分辨率 VoxelGridCovariance

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(35); //设置匹配迭代的最大次数

    // Setting point cloud to be aligned.
    ndt.setInputSource(filtered_cloud); // 设置过滤后的输入源点云（第二次扫描数据）    // 用的采样之后的filtered_cloud
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(target_cloud);   // 设置目标点云（第一次扫描数据）

//    // Set initial alignment estimate found using robot odometry.
//    // 在这部分的代码块中我们创建了一个点云配准变换矩阵的初始估计，虽然算法运行并不需要这样的一个初始变换矩阵，
//    // 但是有了它易于得到更好的结果，尤其是当参考坐标系之间有较大差异时（本例即是），
//    // 在机器人应用程序（例如用于生成此数据集的应用程序）中，通常使用里程表数据生成初始转换。
//    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());  // 0.6931弧度约等于40角度,是初始估计的
//    Eigen::Translation3f init_translation(1.79387, 0, 0);               // 1.79387弧度约等于102角度,是初始估计的
//    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    struct timeval tv_start{};
    gettimeofday(&tv_start, nullptr);    // TODO (从1970年开始计算的)
    double start = tv_start.tv_sec * 1000 + tv_start.tv_usec * 0.001;
//    printf("timeval获取到的时间：%f\n", start);

//    ndt.align(*output_cloud, init_guess);
    ndt.align(*output_cloud);   // TODO align重载函数,output_cloud是存储配准变换后的源点云,init_guess是初始估计的变换矩阵

    struct timeval tv_end{};
    gettimeofday(&tv_end, nullptr);    // TODO (从1970年开始计算的)
    double end = tv_end.tv_sec * 1000 + tv_end.tv_usec * 0.001;
//    printf("timeval获取到的时间：%f\n", end);

    printf("耗时：%f毫秒\n", end - start);


    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud(*source_cloud, *output_cloud, ndt.getFinalTransformation());// 用的采样之前的source_cloud

    // Saving transformed input cloud.
    pcl::io::savePCDFileASCII("ndt.pcd", *output_cloud);
    std::cout << "ndt.pcd saved" << std::endl;
//===================================================================================================

//===================================================================================================
    return (0);
}


//Loaded 61022 data points from cloud_003.pcd
//        Loaded 64621 data points from cloud_002.pcd
//        Filtered cloud contains 62 data points from cloud_002.pcd
//        耗时：90.061035毫秒
//Normal Distributions Transform has converged:1 score: 0.684244
//ndt.pcd saved

