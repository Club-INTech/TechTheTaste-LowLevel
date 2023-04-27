add_library(com com.c)
add_subdirectory(header)
target_link_libraries(com asserv action pico_stdlib hardware_uart)
