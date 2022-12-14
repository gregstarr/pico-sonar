# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(sonar C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

add_subdirectory(fatfs)

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.c
)

pico_generate_pio_header(sonar ${CMAKE_CURRENT_LIST_DIR}/gpio_driver.pio)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_pio
    fatfs
    pico_multicore
)
