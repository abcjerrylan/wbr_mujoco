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
set(MUJOCO_INTERFACE_BUILD_DIR "${MUJOCO_INTERFACE_DIR}/build" CACHE PATH
    "Path to standalone mujoco_interface build directory")

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)

if(TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

option(WBR_EMBED_MUJOCO_INTERFACE
  "Build mujoco_interface as a subdirectory of wbr_mujoco" OFF)

# --- common (in-proc msg bus only) ---
add_library(msg common/src/msg.cpp)
target_include_directories(msg PUBLIC common/include)

add_library(control_core INTERFACE)
target_include_directories(control_core INTERFACE controller/include)

# --- mujoco_interface ---
enable_testing()
if(WBR_EMBED_MUJOCO_INTERFACE)
  add_subdirectory(${MUJOCO_INTERFACE_DIR} ${CMAKE_BINARY_DIR}/mujoco_interface)
  add_custom_target(all_sim DEPENDS mujoco_interface)
else()
  set(MUJOCO_INTERFACE_CORE_LIB
      "${MUJOCO_INTERFACE_BUILD_DIR}/libmujoco_interface_core.a"
      CACHE FILEPATH "Path to standalone libmujoco_interface_core.a")
  if(NOT EXISTS "${MUJOCO_INTERFACE_CORE_LIB}")
    message(FATAL_ERROR
      "Standalone mujoco_interface_core library not found:\n"
      "  ${MUJOCO_INTERFACE_CORE_LIB}\n"
      "Build mujoco_interface first:\n"
      "  cmake -S ${MUJOCO_INTERFACE_DIR} -B ${MUJOCO_INTERFACE_BUILD_DIR} -DCMAKE_BUILD_TYPE=Release\n"
      "  cmake --build ${MUJOCO_INTERFACE_BUILD_DIR} -j\n"
      "Or configure with -DWBR_EMBED_MUJOCO_INTERFACE=ON for the old integrated build.")
  endif()
  if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
    message(FATAL_ERROR "MuJoCo headers not found at ${MUJOCO_DIR}/include")
  endif()
  if(NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so")
    message(FATAL_ERROR "MuJoCo library not found at ${MUJOCO_DIR}/lib/libmujoco.so")
  endif()
  if(NOT EXISTS "${ECAL_LIB_DIR}/libecal_core.so")
    message(FATAL_ERROR "eCAL library not found at ${ECAL_LIB_DIR}/libecal_core.so")
  endif()

  add_library(ecal_core SHARED IMPORTED GLOBAL)
  set_target_properties(ecal_core PROPERTIES
    IMPORTED_LOCATION "${ECAL_LIB_DIR}/libecal_core.so"
    INTERFACE_INCLUDE_DIRECTORIES "${ECAL_ROOT}/include"
  )

  add_library(mujoco_interface_core STATIC IMPORTED GLOBAL)
  set_target_properties(mujoco_interface_core PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_INTERFACE_CORE_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES
      "${MUJOCO_INTERFACE_DIR}/include;${MUJOCO_DIR}/include;${ECAL_ROOT}/include"
    INTERFACE_LINK_LIBRARIES
      "yaml-cpp::yaml-cpp;Threads::Threads;glfw;ecal_core;${MUJOCO_DIR}/lib/libmujoco.so"
  )
endif()

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

add_executable(test_import tests/test_import.cpp)
target_link_libraries(test_import PRIVATE mujoco_interface_core)
set_target_properties(test_import PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
