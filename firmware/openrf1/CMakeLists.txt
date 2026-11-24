cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake" CACHE FILEPATH "ARM toolchain")
project(OpenRF1_UART2 LANGUAGES C ASM)
set(LINK_SCRIPT "stm32f103rc.ld" CACHE STRING "Linker script")
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(openrf1_platform
    startup/startup_stm32f103rc.S
    vendor/CMSIS/system_stm32f10x.c
    vendor/FWlib/src/stm32f10x_rcc.c
    vendor/FWlib/src/stm32f10x_gpio.c
    vendor/FWlib/src/stm32f10x_usart.c)
add_executable(openrf1_wheel_jog src/main.c ${openrf1_platform})
add_executable(openrf1_collector src/collector_main.c src/collector_control.c ${openrf1_platform})

foreach(program IN ITEMS openrf1_wheel_jog openrf1_collector)
    set_target_properties(${program} PROPERTIES SUFFIX ".elf")
    target_include_directories(${program} PRIVATE src vendor/CMSIS vendor/FWlib/inc)
    target_compile_definitions(${program} PRIVATE STM32F10X_HD USE_STDPERIPH_DRIVER HSE_VALUE=8000000)
    target_compile_options(${program} PRIVATE -mcpu=cortex-m3 -mthumb -Og -g3 -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra)
    target_link_options(${program} PRIVATE -mcpu=cortex-m3 -mthumb -nostdlib
        "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/${LINK_SCRIPT}"
        -Wl,--gc-sections "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${program}.map")
    target_link_libraries(${program} PRIVATE gcc)
    set_property(TARGET ${program} APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/linker/${LINK_SCRIPT}")
    add_custom_command(TARGET ${program} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:${program}>" "${CMAKE_CURRENT_BINARY_DIR}/${program}.hex"
        COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:${program}>" "${CMAKE_CURRENT_BINARY_DIR}/${program}.bin"
        COMMAND "${ARM_SIZE}" "$<TARGET_FILE:${program}>"
        VERBATIM)
endforeach()
