file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_executable(ER_POINTS_RUN ${srcs})


add_thread(ER_POINTS_RUN)
add_opencv(ER_POINTS_RUN)
add_pcl(ER_POINTS_RUN)
add_zed(ER_POINTS_RUN)
add_k4a(ER_POINTS_RUN)
add_cuda(ER_POINTS_RUN)

target_include_directories(ER_POINTS_RUN PUBLIC include)
target_link_libraries(ER_POINTS_RUN PUBLIC external)
