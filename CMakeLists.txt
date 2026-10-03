cmake_minimum_required(VERSION 3.22)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake"
    CACHE FILEPATH "ARM GCC toolchain")
project(stm32f407_bt_oled LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver")
set(DEVICE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx")
set(CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/CMSIS/Core/Include")
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

add_executable(${PROJECT_NAME}
    Core/Src/main.c
    Core/Src/servo.c
    Core/Src/arm_kinematics.c
    Core/Src/arm_collision.c
    Core/Src/arm_trim.c
    Core/Src/arm_trim_project.c
    Core/Src/arm_trim_service.c
    Core/Src/arm_trim_input.c
    Core/Src/board_inputs.c
    Core/Src/board_app.c
    Core/Src/start_button.c
    Core/Src/vision_data.c
    Core/Src/maxicam.c
    Core/Src/laser.c
    Core/Src/turn_right.c
    Core/Src/action_fsm.c
    Core/Src/mission_fsm.c
    Core/Src/route_fsm.c
    Core/Src/uart_driver.c
    Core/Src/uart_tx_queue.c
    Core/Src/serial_io.c
    Core/Src/bluetooth_driver.c
    Core/Src/motor_driver.c
    Core/Src/emm42_driver.c
    Core/Src/mecanum.c
    Core/Src/mecanum_test.c
    Core/Src/car_control.c
    Core/Src/remote_heading.c
    Core/Src/uart_bridge.c
    Core/Src/usart2_dma.c
    Core/Src/jy61.c
    Core/Src/heading_control.c
    Core/Src/pid_tuner.c
    Core/Src/ssd1306.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    cmake/newlib_init.c
    ${DEVICE_DIR}/Source/Templates/system_stm32f4xx.c
    ${DEVICE_DIR}/Source/Templates/gcc/startup_stm32f407xx.s
    ${HAL_DIR}/Src/stm32f4xx_hal.c
    ${HAL_DIR}/Src/stm32f4xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dma.c
    ${HAL_DIR}/Src/stm32f4xx_hal_i2c.c
    ${HAL_DIR}/Src/stm32f4xx_hal_uart.c)
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_HAL_DRIVER STM32F407xx)
option(ARM_TRIM_ENABLE "Enable the independent arm trim service and phone page" ON)
if(ARM_TRIM_ENABLE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ARM_TRIM_ENABLE=1)
endif()
set(USART2_RX_PERIOD_MS "5" CACHE STRING "JY61 DMA poll period in milliseconds (5 or 2)")
if(NOT USART2_RX_PERIOD_MS STREQUAL "5" AND NOT USART2_RX_PERIOD_MS STREQUAL "2")
    message(FATAL_ERROR "USART2_RX_PERIOD_MS must be 5 or 2")
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE USART2_RX_PERIOD_MS=${USART2_RX_PERIOD_MS}U)
option(CAR_TEST_INPUTS_ENABLE "Compile bench-only physical inputs" OFF)
if(CAR_TEST_INPUTS_ENABLE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE CAR_TEST_INPUTS_ENABLE=1)
endif()
option(CAR_MECANUM_TEST_MODE "Enable gated 30 RPM, 1 second mecanum direction tests" OFF)
option(CAR_HEADING_TEST_MODE "Enable low-speed heading test mailbox" OFF)
if((CAR_HEADING_TEST_MODE OR CAR_MECANUM_TEST_MODE) AND NOT CAR_TEST_INPUTS_ENABLE)
    message(FATAL_ERROR "Car test modes require CAR_TEST_INPUTS_ENABLE=ON")
endif()
if(CAR_HEADING_TEST_MODE AND CAR_MECANUM_TEST_MODE)
    message(FATAL_ERROR "Select either heading or mecanum direction tests")
endif()
if(CAR_HEADING_TEST_MODE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE CAR_HEADING_TEST_MODE=1)
endif()
if(CAR_MECANUM_TEST_MODE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE CAR_MECANUM_TEST_MODE=1)
endif()
target_include_directories(${PROJECT_NAME} PRIVATE Core/Inc)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
    ${HAL_DIR}/Inc ${DEVICE_DIR}/Include ${CORE_DIR})
target_compile_options(${PROJECT_NAME} PRIVATE ${MCU_FLAGS}
    -Os -g3 -ffunction-sections -fdata-sections
    $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Werror>)
target_link_options(${PROJECT_NAME} PRIVATE ${MCU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407_FLASH.ld
    -nostartfiles --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
target_link_libraries(${PROJECT_NAME} PRIVATE m)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/STM32F407_FLASH.ld")
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}>
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}>
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    VERBATIM)
