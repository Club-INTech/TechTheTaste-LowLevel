add_library(com com.c)
add_subdirectory(header)
target_link_libraries(com Nyx pico_stdlib hardware_uart)
