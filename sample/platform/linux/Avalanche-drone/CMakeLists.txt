cmake_minimum_required(VERSION 2.8)
project(Avalanche-drone_noise)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)
include_directories(${MODULES_HEADER_DIR})
include_directories(${FLIGHT_MODULES_HEADER_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)
include_directories(/usr/local/fftw/include)
link_directories(/usr/local/fftw/lib)


FILE(GLOB SOURCE_FILES *.hpp *.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
    )

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
    main.cpp
    SignalSearch.cpp
    ADC.cpp
    IIRFilter.cpp
    HField.cpp
    V3D.cpp
    Simulation.cpp
    FlightController.cpp
    )

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_library(BCM2835_LIBRARIES NAMES bcm2835)
target_link_libraries(${PROJECT_NAME} ${WIRINGPI_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${BCM2835_LIBRARIES})
target_link_libraries(${PROJECT_NAME} fftw3)
target_link_libraries(${PROJECT_NAME} djiosdk-core)