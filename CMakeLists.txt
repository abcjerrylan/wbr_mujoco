cmake_minimum_required(VERSION 3.16)
project(wbr_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif()

# Sibling checkout: ../mujoco_interface (override with -DMUJOCO_INTERFACE_DIR=...)
if(NOT MUJOCO_INTERFACE_DIR)
  set(MUJOCO_INTERFACE_DIR "${CMAKE_SOURCE_DIR}/../mujoco_interface" CACHE PATH
      "Path to mujoco_interface repository")
endif()
if(NOT EXISTS "${MUJOCO_INTERFACE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "mujoco_interface not found at ${MUJOCO_INTERFACE_DIR}.\n"
    "Clone it next to wbr_mujoco or pass -DMUJOCO_INTERFACE_DIR=/path/to/mujoco_interface")
endif()

set(MUJOCO_DIR "${MUJOCO_INTERFACE_DIR}/mujoco" CACHE PATH "MuJoCo root")
set(ECAL_ROOT "${MUJOCO_INTERFACE_DIR}/third_party/ecal/usr" CACHE PATH "Local eCAL prefix")
set(ECAL_LIB_DIR "${ECAL_ROOT}/lib/x86_64-linux-gnu")

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- common (in-proc msg bus only) ---
add_library(msg common/src/msg.cpp)
target_include_directories(msg PUBLIC common/include)

add_library(control_core INTERFACE)
target_include_directories(control_core INTERFACE controller/include)

# --- mujoco_interface (static lib; server exe built in its own repo or via all_sim) ---
enable_testing()
add_subdirectory(${MUJOCO_INTERFACE_DIR} ${CMAKE_BINARY_DIR}/mujoco_interface)

# --- controller ---
add_executable(ctrl
  controller/src/main.cpp
  controller/src/app.cpp
  controller/src/config.cpp
  controller/src/ecal_io.cpp
)
target_include_directories(ctrl PRIVATE controller/include)
target_link_libraries(ctrl PRIVATE
  control_core
  msg
  mujoco_interface_core
  yaml-cpp
  Threads::Threads
)
set_target_properties(ctrl PROPERTIES
  BUILD_RPATH "${ECAL_LIB_DIR};${MUJOCO_DIR}/lib"
  INSTALL_RPATH "${ECAL_LIB_DIR};${MUJOCO_DIR}/lib"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

add_custom_target(all_sim DEPENDS mujoco_interface)

add_executable(test_import tests/test_import.cpp)
target_link_libraries(test_import PRIVATE mujoco_interface_core)
set_target_properties(test_import PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
