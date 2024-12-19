cmake_minimum_required(VERSION 3.30)
project("2D Game")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/tako/cmake)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(EMSCRIPTEN)
	set(EMSCRIPTEN_PTHREADS_COMPILER_FLAGS "-pthread")
	set(EMSCRIPTEN_PTHREADS_LINKER_FLAGS "${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")

	string(APPEND CMAKE_C_FLAGS " ${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")
	string(APPEND CMAKE_CXX_FLAGS " ${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")
	string(APPEND CMAKE_EXE_LINKER_FLAGS " ${EMSCRIPTEN_PTHREADS_LINKER_FLAGS}")
endif()

add_subdirectory("dependencies/tako")
include(tako)
SET(EXECUTABLE flocking)
add_executable(${EXECUTABLE} "src/Main.cpp")
target_sources(${EXECUTABLE}
	PUBLIC FILE_SET CXX_MODULES FILES
	"src/Rect.cppm"
	"src/Boid.cppm"
	"src/Octree.cppm"
	"src/ExpandingPoolAllocator.cppm"
)

tako_setup(${EXECUTABLE})
target_link_libraries(${EXECUTABLE} PRIVATE tako)

tako_assets_dir(${EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/Assets/")
