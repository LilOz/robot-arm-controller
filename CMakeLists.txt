cmake_minimum_required(VERSION 3.15)
project(RobotArmProject LANGUAGES CXX)

# Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Collect all source files automatically
file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Create the executable
add_executable(robot_arm ${SOURCES})

# Includes
target_include_directories(robot_arm
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# --------------------------
# Dependencies: prefer existing, else FetchContent
# --------------------------
include(FetchContent)

# ---- nlohmann::json ----
# If a parent project already provided the target, use it.
if(NOT TARGET nlohmann_json::nlohmann_json)
  # Try an installed package first.
  find_package(nlohmann_json CONFIG QUIET)

  # If still not found, fetch it.
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
# If a parent project already provided the target, use it.
if(NOT TARGET Eigen3::Eigen)
  # Try an installed package first.
  find_package(Eigen3 3.4 CONFIG QUIET)

  # If still not found, fetch it.
  if(NOT TARGET Eigen3::Eigen)
    FetchContent_Declare(
      eigen
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG 3.4.0
    )
    FetchContent_MakeAvailable(eigen)
  endif()
endif()

# Link libraries
target_link_libraries(robot_arm
  PRIVATE
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

