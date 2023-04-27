add_subdirectory(asserv)
add_subdirectory(action)
add_subdirectory(com)
add_executable(Asserv main.c)
target_link_libraries(Asserv com asserv action pico_stdlib)
pico_enable_stdio_usb(Asserv 1)
pico_enable_stdio_uart(Asserv 0)
pico_add_extra_outputs(Asserv)

