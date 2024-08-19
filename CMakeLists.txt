cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(projeto-final-embarcados)

include_directories(${CMAKE_SOURCE_DIR}/lib)

set(SOURCES
    main/main.c
    lib/imu_tools.c
    lib/sensor_imu.c
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE idf::freertos idf::esp_common)