cmake_minimum_required(VERSION 3.13)
project("2D Game")

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/tako/cmake)

add_subdirectory("dependencies/tako")
include(tako)
SET(EXECUTABLE game)
add_executable(${EXECUTABLE} WIN32
        "src/Main.cpp" "src/Octree.cpp")

tako_setup(${EXECUTABLE})
target_link_libraries(${EXECUTABLE} PRIVATE tako)

tako_assets_dir("${CMAKE_CURRENT_SOURCE_DIR}/Assets/")
