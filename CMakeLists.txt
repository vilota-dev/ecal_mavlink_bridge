cmake_minimum_required(VERSION 3.15)

# Specify your project's name
project(ecal_mavlink_bridge VERSION 0.1.0)

# Specify at least C++11, better C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# configure build types and build c-flags
if( NOT CMAKE_BUILD_TYPE )
  # Release, RelWithAssert, RelWithEigenNan
  set( CMAKE_BUILD_TYPE RelWithAssert) # Choice of Release (ignore assert) and RelWithDebInfo (includes assert)
endif()

message(WARNING "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                      # cmake default: "-O3 -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELWITHASSERT "-O3")
set(CMAKE_CXX_FLAGS_RELWITHEIGENNAN  "-O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")   # enable nan initialisation for eigen

# flags for error
set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-error=class-memaccess")

# setup compiler flags
set(CMAKE_CXX_FLAGS "${BASALT_CXX_FLAGS} ${BASALT_MARCH_FLAGS} ${BASALT_PASSED_CXX_FLAGS}")

message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

# Enable strict handling of warnings
add_definitions("-Wall -Wextra -Werror")

# Set default installation directory
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory" FORCE)
endif()

# turn off dependency of fmt in Sophus
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
add_subdirectory(thirdparty/spdlog)

# Finds MAVSDK for lacal install
set(cmake_config_DIR "/home/$ENV{USER}/git/MAVSDK/build/install/lib/cmake")
set(MAVSDK_DIR ${cmake_config_DIR}/MAVSDK)
find_package(MAVSDK CONFIG REQUIRED)


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


find_package(eCAL 5.11 REQUIRED)

set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
find_package(CapnProto REQUIRED)

# create the prefix, so the actual capnp file is in the path like capnp/*.capnp
set(CAPNPC_SRC_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS 
    thirdparty/ecal-common/src/capnp/header.capnp
    thirdparty/ecal-common/src/capnp/se3.capnp
    thirdparty/ecal-common/src/capnp/so3.capnp
    thirdparty/ecal-common/src/capnp/vector3.capnp
    thirdparty/ecal-common/src/capnp/twist3.capnp
    thirdparty/ecal-common/src/capnp/odometry3d.capnp
    thirdparty/ecal-common/src/capnp/mavstate.capnp
)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}/ecal-common/src/ # generated headers from capnp
    thirdparty/basalt-headers/thirdparty/eigen
    thirdparty/basalt-headers/thirdparty/Sophus
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

install(TARGETS ecal_odometry_mavlink_bridge test_timesync send_vision_position_estimate
EXPORT EcalOdometryMavlinkBridgeTargets
# RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
# LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
# ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
set_target_properties(ecal_odometry_mavlink_bridge PROPERTIES INSTALL_RPATH "$ORIGIN/../lib" )
set_target_properties(test_timesync PROPERTIES INSTALL_RPATH "$ORIGIN/../lib" )
set_target_properties(send_vision_position_estimate PROPERTIES INSTALL_RPATH "$ORIGIN/../lib" )

# hardcode is easier to manage, than automation script
file(GLOB SDK_TO_INSTALL "${cmake_config_DIR}/../libmavsdk.so*")
install(
FILES 
  ${SDK_TO_INSTALL}
TYPE LIB)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
include(CPackConfigDeb)