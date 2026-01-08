cmake_minimum_required(VERSION 3.5)
project(unitree_lidar_sdk)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG") 

include_directories(include)

# This command tells CMake where to find the library file automatically
link_directories(lib/${CMAKE_SYSTEM_PROCESSOR})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

# --- Existing Examples ---
add_executable(example_lidar_udp
  examples/example_lidar_udp.cpp
)
target_link_libraries(example_lidar_udp libunitree_lidar_sdk.a pthread)

add_executable(example_lidar_serial
   examples/example_lidar_serial.cpp
)
target_link_libraries(example_lidar_serial libunitree_lidar_sdk.a pthread)

# --- YOUR NEW CEILING MEASUREMENT TOOL ---
add_executable(example_ceiling_measurement
   examples/example_ceiling_measurement.cpp
)

# FIX: Link simply by name. The 'link_directories' above handles the path.
target_link_libraries(example_ceiling_measurement
   libunitree_lidar_sdk.a
   pthread
)