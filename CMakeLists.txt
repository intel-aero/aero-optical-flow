cmake_minimum_required(VERSION 2.8)

project(aero-optical-flow)

find_package(OpenCV REQUIRED)

add_executable(aero-optical-flow src/main.cpp src/camera.cpp)
target_link_libraries(aero-optical-flow ${OpenCV_LIBS})
install(TARGETS aero-optical-flow DESTINATION bin)