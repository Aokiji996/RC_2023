file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(kinect ${srcs})
target_include_directories(kinect PUBLIC include)
message_info("add library hello: version ${HELLO_VERSION}")

# add the dependencies of the library
add_opencv(kinect)
add_pcl(kinect)
add_k4a(kinect)
