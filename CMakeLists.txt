cmake_minimum_required(VERSION 3.14)
project(ETCapture)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("." , "Pangolin/include" , "Pangolin/build/src/include")

file( GLOB SRCS *.cpp *.h)
add_executable(ETCapture ${SRCS})

target_link_libraries(ETCapture GL)
target_link_libraries(ETCapture GLEW)
target_link_libraries(ETCapture /home/rob/ETCapture/Pangolin/build/src/libpangolin.so)
target_link_libraries(ETCapture ${OpenCV_LIBS})
