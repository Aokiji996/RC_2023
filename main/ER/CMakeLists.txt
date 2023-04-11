file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_executable(ER_RUN ${srcs})

target_include_directories(ER_RUN PUBLIC include)
target_link_libraries(ER_RUN PUBLIC external)