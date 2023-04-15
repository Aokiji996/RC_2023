file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(com STATIC ${srcs})
target_include_directories(com PUBLIC include)

add_yaml(com)
add_thread(com)

message_info("add library slamtic: version ${SLAMTIC_VERSION}")

# add the dependencies of the library


