file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp src/*.h include/*.h)
add_library(sl_sdk STATIC ${srcs})
target_include_directories(sl_sdk PUBLIC include src)