cmake_minimum_required(VERSION 3.16)

project(PCL_Lesson)

set(CMAKE_CXX_STANDARD 11)


#=====================================================================================
#=====================================================================================

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
# 若只是sleep_for时间休眠,CMakeLists不需要加这个设置
# 若是std::thread开启线程，CMakeLists就需要加这个设置
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#=====================================================================================
#=====================================================================================

# 使CMAKE_BUILD_TYPE(看clion的配置是Debug)变成小写debug,并赋值给cmake_build_type
string(TOLOWER ${CMAKE_BUILD_TYPE} cmake_build_type)    # cmake_build_type是我自己定义的
# 设置输出根目录为build/debug
set(OUTPUT_DIRECTORY_ROOT ${PROJECT_SOURCE_DIR}/build/${cmake_build_type})

#=====================================================================================
#=====================================================================================

#使用绝对路径找到3rdparty文件夹
set(3rdparty_Path "/home/liuhao/CLionProjects/Robot_modules_3rdparty/3rdparty")  # cmake文件夹里面的文件需要用到3rdparty_Path路径

# 使用绝对路径找到cmake文件夹
# 模块模式,在CMAKE_MODULE_PATH查找Find<PackageName>.cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/liuhao/CLionProjects/Robot_modules_3rdparty/cmake")

#=====================================================================================
#=====================================================================================

##find_package(Eigen3_3.3.7_14.04 MODULE REQUIRED)
#find_package(Eigen3_3.3.7_16.04 MODULE REQUIRED)

##find_package(PCL_1.9.0_14.04 MODULE REQUIRED)     # ubuntu14.04编译1.9.0成功了
##find_package(PCL_1.9.0_16.04 MODULE REQUIRED)     # ubuntu16.04编译1.9.0成功了
find_package(PCL_1.10.0_16.04 MODULE REQUIRED)     # ubuntu16.04编译1.10.0成功了

#=====================================================================================
#=====================================================================================

add_subdirectory(src)

#=====================================================================================
#=====================================================================================
