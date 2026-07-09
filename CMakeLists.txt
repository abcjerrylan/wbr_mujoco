cmake_minimum_required(VERSION 3.16)
project(wbr_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif()

set(MUJOCO_DIR "${CMAKE_SOURCE_DIR}/mujoco_interface/mujoco" CACHE PATH "MuJoCo root")

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- common ---
add_library(msg common/src/msg.cpp)
target_include_directories(msg PUBLIC common/include)

add_library(robot_msgs INTERFACE)
target_include_directories(robot_msgs INTERFACE common/include)

add_library(robot_ipc INTERFACE)
target_include_directories(robot_ipc INTERFACE common/include)
target_link_libraries(robot_ipc INTERFACE robot_msgs rt pthread)

add_library(control_core INTERFACE)
target_include_directories(control_core INTERFACE controller/include)

# --- mujoco_interface (submodule / nested repo) ---
add_subdirectory(mujoco_interface)

# --- bridge ---
add_library(bridge
  bridge/src/shm_bridge.cpp
  bridge/src/sim_config.cpp
  bridge/src/app_control.cpp
)
target_include_directories(bridge PUBLIC bridge/include)
target_link_libraries(bridge PUBLIC
  mujoco_interface
  robot_ipc
  robot_msgs
  yaml-cpp
)

add_executable(sim sim/main.cc)
target_link_libraries(sim PRIVATE bridge sim_rt)
set_target_properties(sim PROPERTIES
  BUILD_RPATH "${MUJOCO_DIR}/lib"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

# --- controller ---
add_executable(ctrl
  controller/src/main.cpp
  controller/src/app.cpp
  controller/src/config.cpp
)
target_include_directories(ctrl PRIVATE controller/include)
target_link_libraries(ctrl PRIVATE
  control_core
  msg
  robot_ipc
  robot_msgs
  yaml-cpp
  Threads::Threads
)
set_target_properties(ctrl PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

add_custom_target(all_sim DEPENDS sim)

add_executable(test_import tests/test_import.cpp)
target_link_libraries(test_import PRIVATE mujoco_interface)
set_target_properties(test_import PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_executable(test_adapter tests/test_adapter.cpp)
target_link_libraries(test_adapter PRIVATE bridge mujoco_interface)
set_target_properties(test_adapter PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_executable(test_shm tests/test_shm.cpp)
target_link_libraries(test_shm PRIVATE robot_ipc robot_msgs rt pthread)
set_target_properties(test_shm PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
