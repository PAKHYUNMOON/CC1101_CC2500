# =============================================================================
# CC1101 + CC2500 MICS Implant Communication System
#
# Build with target selection:
#
#   Build Master (Programmer / external device):
#     cmake -B build/master -DBUILD_TARGET=master
#     cmake --build build/master
#
#   Build Slave (Implant / implantable device):
#     cmake -B build/slave -DBUILD_TARGET=slave
#     cmake --build build/slave
#
# Each target produces ONE firmware binary; only the corresponding main()
# is compiled in, eliminating multiple-definition errors.
#
# Common library code (cc1101, cc2500, common/) is shared across both
# targets and compiled with the corresponding application source.
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# ----- Build target selection -----
set(BUILD_TARGET "master" CACHE STRING "Build target: master | slave")
set_property(CACHE BUILD_TARGET PROPERTY STRINGS master slave)

if(NOT BUILD_TARGET MATCHES "^(master|slave)$")
    message(FATAL_ERROR
        "BUILD_TARGET must be either 'master' or 'slave' (got: ${BUILD_TARGET})")
endif()

message(STATUS "Building target: ${BUILD_TARGET}")

# ----- Toolchain (cross-compile for STM32U575) -----
# Override with: cmake -DCMAKE_TOOLCHAIN_FILE=path/to/arm-none-eabi.cmake
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_C_COMPILER  arm-none-eabi-gcc)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_OBJCOPY     arm-none-eabi-objcopy)
    set(CMAKE_SIZE        arm-none-eabi-size)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
endif()

project(cc_mics_${BUILD_TARGET} C ASM)

# ----- MCU flags (STM32U575 = Cortex-M33 with FPU) -----
set(MCU_FLAGS
    -mcpu=cortex-m33
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -DSTM32U575xx
    -DUSE_HAL_DRIVER
)

set(COMMON_C_FLAGS
    ${MCU_FLAGS}
    -Wall -Wextra
    -ffunction-sections
    -fdata-sections
    -fno-common
    -fstack-usage
    -Os                           # size-optimized for low power
)

add_compile_options(${COMMON_C_FLAGS})
add_link_options(
    ${MCU_FLAGS}
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nano.specs
    --specs=nosys.specs
)

# ----- Common include paths -----
set(COMMON_INC
    ${CMAKE_SOURCE_DIR}/cc1101/header
    ${CMAKE_SOURCE_DIR}/cc2500/header
    ${CMAKE_SOURCE_DIR}/common
    # User must supply STM32U5 HAL include paths via -DSTM32_HAL_INC=...
    ${STM32_HAL_INC}
)

# ----- Common source files (radios + protocol + low-power) -----
set(COMMON_SRC
    cc1101/main/cc1101.c
    cc2500/main/cc2500.c
    common/stm32u575_lowpower.c
)

# ----- Application source (selected by BUILD_TARGET) -----
if(BUILD_TARGET STREQUAL "master")
    set(APP_SRC      app/master/main.c)
    set(APP_DEFINES  BUILD_MASTER=1 ROLE_NAME="master")
elseif(BUILD_TARGET STREQUAL "slave")
    set(APP_SRC      app/slave/main.c)
    set(APP_DEFINES  BUILD_SLAVE=1 ROLE_NAME="slave")
endif()

# ----- Firmware target -----
add_executable(firmware
    ${APP_SRC}
    ${COMMON_SRC}
)

target_include_directories(firmware PRIVATE ${COMMON_INC})
target_compile_definitions(firmware PRIVATE ${APP_DEFINES})

# ----- Post-build: hex/bin and size report -----
set_target_properties(firmware PROPERTIES
    OUTPUT_NAME "cc_mics_${BUILD_TARGET}"
    SUFFIX ".elf"
)

add_custom_command(TARGET firmware POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:firmware> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:firmware>
    COMMENT "Generating .hex / .bin and size report"
)
