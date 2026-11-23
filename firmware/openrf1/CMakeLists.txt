cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake" CACHE FILEPATH "ARM toolchain")
project(OpenRF1_UART2 LANGUAGES C ASM)
set(LINK_SCRIPT "stm32f103rc.ld" CACHE STRING "Linker script")
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_executable(openrf1_wheel_jog
    src/main.c
    startup/startup_stm32f103rc.S
    vendor/CMSIS/system_stm32f10x.c
    vendor/FWlib/src/stm32f10x_rcc.c
    vendor/FWlib/src/stm32f10x_gpio.c
    vendor/FWlib/src/stm32f10x_usart.c)
set_target_properties(openrf1_wheel_jog PROPERTIES SUFFIX ".elf")
target_include_directories(openrf1_wheel_jog PRIVATE src vendor/CMSIS vendor/FWlib/inc)
target_compile_definitions(openrf1_wheel_jog PRIVATE STM32F10X_HD USE_STDPERIPH_DRIVER HSE_VALUE=8000000)
target_compile_options(openrf1_wheel_jog PRIVATE -mcpu=cortex-m3 -mthumb -Og -g3 -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra)
target_link_options(openrf1_wheel_jog PRIVATE -mcpu=cortex-m3 -mthumb -nostdlib
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker/${LINK_SCRIPT}"
    -Wl,--gc-sections "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/openrf1_wheel_jog.map")
target_link_libraries(openrf1_wheel_jog PRIVATE gcc)
set_property(TARGET openrf1_wheel_jog APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/linker/${LINK_SCRIPT}")
add_custom_command(TARGET openrf1_wheel_jog POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:openrf1_wheel_jog>" "${CMAKE_CURRENT_BINARY_DIR}/openrf1_wheel_jog.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:openrf1_wheel_jog>" "${CMAKE_CURRENT_BINARY_DIR}/openrf1_wheel_jog.bin"
    COMMAND "${ARM_SIZE}" "$<TARGET_FILE:openrf1_wheel_jog>"
    VERBATIM)
