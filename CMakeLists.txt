cmake_minimum_required(VERSION 3.16)
project(ShapeScanner)
set(CMAKE_CXX_STANDARD 14)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLDUE_DIRS}
                    src/3rd_party/Argumenr_helper
                    src/basic
                    src/utilis)
add_executable(ShapeScanner src/main.cpp)
target_link_libraries(ShapeScanner ${OpenCV_LIBS})