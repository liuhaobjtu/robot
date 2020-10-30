
# 设置可执行程序输出到build/debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin/" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib/" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib/" CACHE PATH "Archive directory" FORCE)


add_executable(01_iterative_closest_point2 01_iterative_closest_point2.cpp)
target_link_libraries(01_iterative_closest_point2 ${PCL_LIBRARIES})

add_executable(03_normal_distributions_transform2 03_normal_distributions_transform2.cpp)
target_link_libraries(03_normal_distributions_transform2 ${PCL_LIBRARIES})


add_executable(test test.cpp)
target_link_libraries(test ${PCL_LIBRARIES})


