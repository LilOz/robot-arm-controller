cmake_minimum_required(VERSION 3.15)
project(RobotArmProject LANGUAGES CXX)

# Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Collect all source files automatically
file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Create the executable
add_executable(robot_arm ${SOURCES})

# Copy config files to build dir for easy access
file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})

# --------------------------
# Fetch nlohmann::json
# --------------------------
include(FetchContent)
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# --------------------------
# Fetch Eigen (header-only)
# --------------------------
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

# Link libraries
target_link_libraries(robot_arm
    PRIVATE
        nlohmann_json::nlohmann_json
        Eigen3::Eigen
)

