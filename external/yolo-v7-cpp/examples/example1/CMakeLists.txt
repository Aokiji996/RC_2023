add_executable(test opencv_test.cpp)

add_yaml(test)
add_thread(test)
add_opencv(test)
add_pcl(test)
add_zed(test)
add_k4a(test)

target_link_libraries(test PUBLIC yolo)