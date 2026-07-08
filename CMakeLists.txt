cmake_minimum_required(VERSION 3.16)
project(wbr_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif()

set(MUJOCO_DIR "/opt/mujoco-3.3.6" CACHE PATH "MuJoCo installation directory")

include_directories(
  ${MUJOCO_DIR}/include
  ${CMAKE_SOURCE_DIR}/mujoco/simulate
  ${CMAKE_SOURCE_DIR}/wbr_mujoco/src/lodepng
)

find_package(Threads REQUIRED)
find_package(glfw3 REQUIRED)

file(GLOB SIM_SRC
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  mujoco/simulate/simulate.cc
  wbr_mujoco/src/lodepng/lodepng.cpp
)

add_executable(wbr_mujoco ${SIM_SRC} wbr_mujoco/src/main.cc)

target_link_libraries(wbr_mujoco
  ${MUJOCO_DIR}/lib/libmujoco.so
  glfw
  Threads::Threads
)

set_target_properties(wbr_mujoco PROPERTIES
  BUILD_RPATH "${MUJOCO_DIR}/lib"
  INSTALL_RPATH "${MUJOCO_DIR}/lib"
)
