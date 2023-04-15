add_executable(zed_test test.cpp)

add_yaml(zed_test)
add_thread(zed_test)
add_opencv(zed_test)
add_pcl(zed_test)
add_zed(zed_test)
add_k4a(zed_test)

target_link_libraries(zed_test PUBLIC zed)
