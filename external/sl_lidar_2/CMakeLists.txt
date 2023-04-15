
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(slamtec STATIC ${srcs})
target_include_directories(slamtec PUBLIC include)
#target_include_directories(slamtec PUBLIC sdk/include)
add_subdirectory(sdk)
target_link_libraries(slamtec PUBLIC sl_sdk)
add_yaml(slamtec)
add_thread(slamtec)	

message_info("add library slamtic: version ${SLAMTIC_VERSION}")

# add the dependencies of the library


