file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_executable(ER_YOLO_RUN ${srcs})


add_thread(ER_YOLO_RUN)
add_opencv(ER_YOLO_RUN)
add_pcl(ER_YOLO_RUN)
add_zed(ER_YOLO_RUN)
add_k4a(ER_YOLO_RUN)

target_include_directories(ER_YOLO_RUN PUBLIC include)
target_link_libraries(ER_YOLO_RUN PUBLIC external)
