cmake_minimum_required(VERSION 3.20)
project(clion)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

include_directories(include)
aux_source_directory(src SRC)
add_executable(${PROJECT_NAME} main.cpp ${SRC} src/utils.cpp include/utils.h)
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})