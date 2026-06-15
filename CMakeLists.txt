cmake_minimum_required(VERSION 3.20)
project(trossen_adamo
    VERSION 0.1.0
    DESCRIPTION "Adamo integration for Trossen leader/follower teleop"
    HOMEPAGE_URL "https://github.com/lukeschmitt-tr/trossen_adamo"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(ADAMO_TROSSEN_BUILD_LEADER   "Build the trossen_leader binary"   ON)
option(ADAMO_TROSSEN_BUILD_FOLLOWER "Build the trossen_follower binary" ON)
option(ADAMO_TROSSEN_BUILD_VR_HEADSET "Build the vr_headset binary" ON)
option(ADAMO_TROSSEN_BUILD_VR_FOLLOWER "Build the vr_follower binary" ON)

# ---------------------------------------------------------------------------
# Adamo C SDK — resolved via find_package(Adamo CONFIG).
#
# Download the SDK tarball from <https://install.adamohq.com/sdk/v<X.Y.Z>/>,
# extract it, and either:
#   - install it to a system prefix (CMake finds it automatically), or
#   - pass -DCMAKE_PREFIX_PATH=/path/to/extracted-sdk.
#
# The follower's RealSense streamer needs the video bindings, so request
# them before find_package — AdamoConfig.cmake gates ADAMO_HAS_VIDEO behind
# this option.
# ---------------------------------------------------------------------------
set(Adamo_VIDEO ON CACHE BOOL "Enable the Adamo video bindings (required by trossen_follower)")

find_package(Adamo CONFIG REQUIRED)
find_package(Threads REQUIRED)

# VR dependencies
if(ADAMO_TROSSEN_BUILD_VR_HEADSET OR ADAMO_TROSSEN_BUILD_VR_FOLLOWER OR ADAMO_TROSSEN_BUILD_VR_BIMANUAL)
    find_package(nlohmann_json REQUIRED)
    find_package(trossen_vr CONFIG REQUIRED)
endif()

if(NOT TARGET Adamo::adamo_cpp)
    message(FATAL_ERROR
        "Adamo::adamo_cpp target is required. Install the 'video' SDK build or rebuild with ADAMO_BUILD_CPP=ON.")
endif()

# ---------------------------------------------------------------------------
# libtrossen_arm — system install, local checkout, or upstream fetch.
# ---------------------------------------------------------------------------
include(FindTrossenArm)

# ---------------------------------------------------------------------------
# Common (header-only) library.
# ---------------------------------------------------------------------------
add_library(trossen_adamo_common INTERFACE)
target_include_directories(trossen_adamo_common INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/common/include"
)
target_compile_features(trossen_adamo_common INTERFACE cxx_std_17)
target_link_libraries(trossen_adamo_common INTERFACE
    Adamo::adamo_cpp
    TrossenArm::TrossenArm
)
if(UNIX)
    target_link_libraries(trossen_adamo_common INTERFACE pthread)
endif()

# ---------------------------------------------------------------------------
# Binaries
# ---------------------------------------------------------------------------
if(ADAMO_TROSSEN_BUILD_LEADER)
    add_subdirectory(leader)
endif()
if(ADAMO_TROSSEN_BUILD_FOLLOWER)
    add_subdirectory(follower)
endif()
if(ADAMO_TROSSEN_BUILD_VR_HEADSET)
    add_subdirectory(vr_headset)
endif()
if(ADAMO_TROSSEN_BUILD_VR_FOLLOWER)
    add_subdirectory(vr_follower)
endif()
option(ADAMO_TROSSEN_BUILD_VR_BIMANUAL "Build VR bimanual binary" OFF)
if(ADAMO_TROSSEN_BUILD_VR_BIMANUAL)
    add_subdirectory(vr_bimanual)
endif()

# libadamo ships with install_name @rpath/libadamo.dylib (macOS) and
# DT_SONAME libadamo.so (Linux). Both need the loader to find the file via
# RPATH (macOS) or the standard search path (Linux) at runtime. Add the
# imported lib's directory to BUILD_RPATH so binaries run from the build
# tree without LD_LIBRARY_PATH / DYLD_LIBRARY_PATH gymnastics. Installed
# binaries are expected to ship next to a system-installed SDK or bring
# their own RPATH.
get_target_property(_adamo_loc Adamo::adamo IMPORTED_LOCATION)
if(_adamo_loc)
    get_filename_component(_adamo_dir "${_adamo_loc}" DIRECTORY)
    foreach(_target trossen_leader trossen_follower vr_headset vr_follower vr_bimanual)
        if(TARGET ${_target})
            set_property(TARGET ${_target} APPEND PROPERTY BUILD_RPATH "${_adamo_dir}")
        endif()
    endforeach()
endif()
