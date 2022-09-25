cmake_minimum_required(VERSION 3.6)
project(sgmpython)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-O3 -Wall")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(CUDA REQUIRED)
find_package(PythonInterp 3.8 REQUIRED)
find_package(PythonLibs 3.8 REQUIRED)
find_package(LibSGM)
find_package(Eigen3)
find_package(Boost 1.47.0 REQUIRED  COMPONENTS system filesystem) 
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_LESS 3.0)
	message(FATAL_ERROR "Error: OpenCV version requires at least 3.0")
endif()


include_directories(
    ${PYTHON_INCLUDE_DIRS}
    ./pybind11/include
    )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include )
message(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(
    /usr/local/cuda/lib64
    )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${LIBSGM_INCLUDE_DIRS}) 


cuda_add_library(gpu_library SHARED
  gpu_library.cu)

target_link_libraries(gpu_library
  ${PYTHON_LIBRARIES}
  cudart  sgm ${CUDA_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${LIBSGM_LIBRARY} Eigen3::Eigen)
  
  
cuda_add_executable(stereo_test stereosgm_image.cpp)
target_link_libraries(stereo_test
  ${PYTHON_LIBRARIES}
  cudart  sgm ${CUDA_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES}  ${LIBSGM_LIBRARY} Eigen3::Eigen)


set_target_properties(gpu_library PROPERTIES PREFIX "")
set_target_properties(stereo_test PROPERTIES PREFIX "")
