add_library(external config.cpp config.h)

add_subdirectory(hello)

target_link_libraries(external PUBLIC hello)
