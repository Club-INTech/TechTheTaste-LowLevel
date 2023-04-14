add_library(function function.c)
add_subdirectory(header)
target_link_libraries(function com asserv pico_stdlib action )
