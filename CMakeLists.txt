cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME threenibian)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User/inverter_svpwm/inverter_svpwm.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    # PFC and inverter ADC, watchdog, control, PWM, and application modules
    User/pfc_adc/pfc_adc.c
    User/pfc_adc/pfc_adc.h
    User/pfc_adc_watchdog/pfc_adc_watchdog.c
    User/pfc_adc_watchdog/pfc_adc_watchdog.h
    User/pfc_control/pfc_control.c
    User/pfc_control/pfc_control.h
    User/pfc_pwm/pfc_pwm.c
    User/pfc_pwm/pfc_pwm.h
    User/pfc_app/pfc_app.c
    User/pfc_app/pfc_app.h
    User/inverter_adc/inverter_adc.c
    User/inverter_adc/inverter_adc.h
    User/inverter_adc_watchdog/inverter_adc_watchdog.c
    User/inverter_adc_watchdog/inverter_adc_watchdog.h
    User/inverter_control/inverter_control.c
    User/inverter_control/inverter_control.h
    User/inverter_svpwm/inverter_svpwm.h
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

    ${CMAKE_CURRENT_SOURCE_DIR}/User/pfc_adc
    ${CMAKE_CURRENT_SOURCE_DIR}/User/pfc_adc_watchdog
    ${CMAKE_CURRENT_SOURCE_DIR}/User/pfc_control
    ${CMAKE_CURRENT_SOURCE_DIR}/User/pfc_pwm
    ${CMAKE_CURRENT_SOURCE_DIR}/User/pfc_app
    ${CMAKE_CURRENT_SOURCE_DIR}/User/inverter_adc
    ${CMAKE_CURRENT_SOURCE_DIR}/User/inverter_adc_watchdog
    ${CMAKE_CURRENT_SOURCE_DIR}/User/inverter_control
    ${CMAKE_CURRENT_SOURCE_DIR}/User/inverter_svpwm
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
