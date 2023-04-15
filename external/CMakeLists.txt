add_library(external config.cpp config.h)

add_subdirectory(sl_lidar_2)
add_subdirectory(kinect)
add_subdirectory(zed)
add_subdirectory(com)
add_subdirectory(yolo-v7-cpp)



target_link_libraries(external PUBLIC slamtec)
target_link_libraries(external PUBLIC kinect)
target_link_libraries(external PUBLIC zed)
target_link_libraries(external PUBLIC com)
target_link_libraries(external PUBLIC yolo)
