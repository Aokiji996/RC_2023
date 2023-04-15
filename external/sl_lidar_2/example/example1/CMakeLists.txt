add_executable(sl_test test.cpp)

add_thread(sl_test)
add_opencv(sl_test)
add_pcl(sl_test)
add_zed(sl_test)
add_k4a(sl_test)


target_link_libraries(sl_test PUBLIC slamtec)
