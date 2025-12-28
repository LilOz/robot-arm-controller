cmake_minimum_required(VERSION 3.15)
project(RobotArmProject LANGUAGES CXX)

# --------------------------
# C++ standard
# --------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --------------------------
# Dependencies
# --------------------------
include(FetchContent)

# ---- nlohmann::json ----
if(NOT TARGET nlohmann_json::nlohmann_json)
  find_package(nlohmann_json CONFIG QUIET)
  if(NOT TARGET nlohmann_json::nlohmann_json)
    FetchContent_Declare(
      json
      GIT_REPOSITORY https://github.com/nlohmann/json.git
      GIT_TAG v3.11.3
    )
    FetchContent_MakeAvailable(json)
  endif()
endif()

# ---- Eigen ----
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 3.4 CONFIG QUIET)
  if(NOT TARGET Eigen3::Eigen)
    FetchContent_Declare(
      eigen
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG 3.4.0
    )
    FetchContent_MakeAvailable(eigen)
  endif()
endif()

# --------------------------
# Core library
# --------------------------
file(GLOB_RECURSE ROBOT_CORE_SOURCES
  ${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_library(robot_core STATIC ${ROBOT_CORE_SOURCES})

target_include_directories(robot_core
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(robot_core
  PUBLIC
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

# --------------------------
# Examples
# --------------------------
add_subdirectory(examples)
