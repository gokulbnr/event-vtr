cmake_minimum_required(VERSION 3.11)

project(cam_pub LANGUAGES C CXX)
add_compile_options(-std=c++17 -O3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -march=native")

# set(OPENGL_opengl_LIBRARY "/usr/lib/x86_64-linux-gnu/libGL.so")
# set(OPENGL_glx_LIBRARY "/usr/lib/x86_64-linux-gnu/libGLX.so")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  roslib
  message_generation
  std_msgs
  geometry_msgs
)

find_package(Torch REQUIRED)

find_package(nlohmann_json REQUIRED)

find_package(SDL2 REQUIRED)

find_package(OpenGL REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)

# Metavision SDK for EVK4
find_package(MetavisionSDK COMPONENTS core stream base ui REQUIRED)

## OpenCV
find_package(OpenCV REQUIRED)

find_package(CURL REQUIRED)

find_package(HDF5 REQUIRED COMPONENTS C CXX)


## libcaer (via pkg-config)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(LIBCAER REQUIRED libcaer)

add_message_files(
	FILES
	Goal.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Catkin packages + system includes
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge roslib message_runtime
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${LIBCAER_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

link_directories(/home/gokulbnr/robostack/.pixi/envs/noetic/lib)

# link_directories(
#   ${LIBCAER_LIBRARY_DIRS}
# )

# ## Executable
# add_executable(dvs_frame_publisher src/davis.cpp)

# target_link_libraries(dvs_frame_publisher
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${LIBCAER_LIBRARIES}
# )

# add_executable(realsense_publisher src/realsense.cpp)

# target_link_libraries(realsense_publisher
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   realsense2
# )

add_executable(evk_publisher src/evk4.cpp)
target_link_libraries(evk_publisher
  ${catkin_LIBRARIES}
  MetavisionSDK::stream
  MetavisionSDK::core
  ${OpenCV_LIBS}
  Boost::program_options
  ${SDL2_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
)

message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")

add_executable(fftconvolve src/torch_corr.cpp)
target_link_libraries(fftconvolve "${TORCH_LIBRARIES}" ${OpenCV_LIBS})
set_property(TARGET fftconvolve PROPERTY CXX_STANDARD 14)

add_executable(xcorr src/xcorr.cpp)
target_link_libraries(xcorr 
${catkin_LIBRARIES}
${TORCH_LIBRARIES} 
nlohmann_json::nlohmann_json 
MetavisionSDK::stream
MetavisionSDK::core
${SDL2_LIBRARIES})
set_property(TARGET xcorr PROPERTY CXX_STANDARD 14)
