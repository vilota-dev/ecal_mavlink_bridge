cmake_minimum_required(VERSION 3.15)

# Specify your project's name
project(ecal_mavlink_bridge)

# Specify at least C++11, better C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable strict handling of warnings
add_definitions("-Wall -Wextra -Werror")

# Finds MAVSDK for lacal install
set(cmake_config_DIR "/home/$ENV{USER}/git/MAVSDK/build/install/lib/cmake")
set(MAVSDK_DIR ${cmake_config_DIR}/MAVSDK)
find_package(MAVSDK CONFIG REQUIRED)

find_package(spdlog REQUIRED)

# Specify your app's executable name, and list of source files used to create it.
add_executable(test_timesync
    src/test_timesync.cpp
    # ... any other source files
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(test_timesync
    MAVSDK::mavsdk             #All apps link against mavsdk library
    spdlog::spdlog
    # ... Any other linked libraries
)

# Specify your app's executable name, and list of source files used to create it.
add_executable(send_vision_position_estimate
    src/send_vision_position_estimate.cpp
    # ... any other source files
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(send_vision_position_estimate
    MAVSDK::mavsdk             #All apps link against mavsdk library
    spdlog::spdlog
    # ... Any other linked libraries
)


find_package(eCAL REQUIRED)

set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
find_package(CapnProto REQUIRED)

# create the prefix, so the actual capnp file is in the path like capnp/*.capnp
set(CAPNPC_SRC_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS 
    thirdparty/ecal-common/src/capnp/header.capnp
    thirdparty/ecal-common/src/capnp/se3.capnp
    thirdparty/ecal-common/src/capnp/so3.capnp
    thirdparty/ecal-common/src/capnp/vector3d.capnp
    thirdparty/ecal-common/src/capnp/twist3.capnp
    thirdparty/ecal-common/src/capnp/odometry3d.capnp
)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}/ecal-common/src/ # generated headers from capnp
)

# Specify your app's executable name, and list of source files used to create it.
add_executable(ecal_odometry_mavlink_bridge
    src/ecal_odometry_mavlink_bridge.cpp
    ${CAPNP_SRCS}
    ${CAPNP_HDRS}
    # ... any other source files
)

# Specify your app's executable name and a list of linked libraries
target_link_libraries(ecal_odometry_mavlink_bridge
    MAVSDK::mavsdk             #All apps link against mavsdk library
    spdlog::spdlog
    CapnProto::capnp
    eCAL::core
    # ... Any other linked libraries
)