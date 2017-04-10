cmake_minimum_required(VERSION 3.1)

project(aero-optical-flow)

find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)

set(MAVLINK_DIALECT common)

add_custom_command(
    OUTPUT mavlink-c-stamp
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/modules/mavlink/pymavlink/tools/mavgen.py --lang=C --wire-protocol 2.0
            -o include/mavlink ${CMAKE_SOURCE_DIR}/modules/mavlink/message_definitions/v1.0/${MAVLINK_DIALECT}.xml
    COMMAND touch mavlink-c-stamp
)
add_custom_target(mavlink-c ALL DEPENDS mavlink-c-stamp)

add_subdirectory(modules/OpticalFlow)

add_executable(aero-optical-flow src/main.cpp src/camera.cpp mavlink-c-stamp)
set_property(TARGET aero-optical-flow PROPERTY CXX_STANDARD 11)
target_include_directories(aero-optical-flow PUBLIC ${CMAKE_BINARY_DIR}/include/mavlink/${MAVLINK_DIALECT})
target_link_libraries(aero-optical-flow ${OpenCV_LIBS})
target_link_libraries(aero-optical-flow OpticalFlow)
install(TARGETS aero-optical-flow DESTINATION bin)