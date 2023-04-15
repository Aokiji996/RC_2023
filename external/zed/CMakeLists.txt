
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(zed ${srcs})
target_include_directories(zed PUBLIC include)
message_info("add library hello: version ${HELLO_VERSION}")

# add the dependencies of the library
add_pcl(zed)
add_opencv(zed)
add_zed(zed)
add_cuda(zed)
