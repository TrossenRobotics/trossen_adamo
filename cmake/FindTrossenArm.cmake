# Resolves an imported target `TrossenArm::TrossenArm`.
#
# Resolution order:
#   1. find_package(libtrossen_arm CONFIG QUIET) — system or earlier install.
#   2. TROSSEN_ARM_DIR cache var: a checkout/install with
#        ${TROSSEN_ARM_DIR}/include/libtrossen_arm/trossen_arm.hpp
#        ${TROSSEN_ARM_DIR}/lib/<os>/<arch>/libtrossen_arm.a
#   3. FetchContent from upstream (https://github.com/TrossenRobotics/trossen_arm).
#
# Override the upstream source by setting TROSSEN_ARM_GIT_REPOSITORY and
# TROSSEN_ARM_GIT_TAG. Disable the network fetch with -DTROSSEN_ARM_FETCH=OFF.
#
# Upstream support matrix (libtrossen_arm, Trossen Robotics):
#   x86_64:        Ubuntu 20.04, 22.04, 24.04
#   arm64/aarch64: Ubuntu 20.04, 22.04, 24.04, macOS 14, macOS 15
# (macOS x86_64 is NOT supported upstream.)

if(TARGET TrossenArm::TrossenArm)
    return()
endif()

# ---------------------------------------------------------------------------
# OS / arch normalisation matched to the upstream lib/<os>/<arch>/ layout.
# ---------------------------------------------------------------------------
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(_trossen_os "macos")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(_trossen_os "linux")
else()
    message(FATAL_ERROR
        "TrossenArm: unsupported OS '${CMAKE_SYSTEM_NAME}'. "
        "libtrossen_arm supports Ubuntu (20.04/22.04/24.04) and macOS (14/15).")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(_trossen_arch "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_trossen_arch "aarch64")
else()
    message(FATAL_ERROR
        "TrossenArm: unsupported CPU arch '${CMAKE_SYSTEM_PROCESSOR}'. "
        "libtrossen_arm supports x86_64 and arm64/aarch64.")
endif()

if(_trossen_os STREQUAL "macos" AND _trossen_arch STREQUAL "x86_64")
    message(FATAL_ERROR
        "TrossenArm: macOS x86_64 is not supported upstream. "
        "libtrossen_arm ships only macOS arm64 binaries.")
endif()

message(STATUS "TrossenArm: target host ${_trossen_os}/${_trossen_arch}")

# ---------------------------------------------------------------------------
# 1) Already-installed package.
# ---------------------------------------------------------------------------
find_package(libtrossen_arm CONFIG QUIET)
if(TARGET libtrossen_arm)
    add_library(TrossenArm::TrossenArm ALIAS libtrossen_arm)
    message(STATUS "TrossenArm: using find_package(libtrossen_arm)")
    return()
endif()

# ---------------------------------------------------------------------------
# 2) Local checkout via TROSSEN_ARM_DIR.
# ---------------------------------------------------------------------------
set(TROSSEN_ARM_DIR "" CACHE PATH "Path to a libtrossen_arm checkout/install")
if(TROSSEN_ARM_DIR AND EXISTS "${TROSSEN_ARM_DIR}/include/libtrossen_arm/trossen_arm.hpp")
    set(_trossen_lib "${TROSSEN_ARM_DIR}/lib/${_trossen_os}/${_trossen_arch}/libtrossen_arm.a")
    if(EXISTS "${_trossen_lib}")
        add_library(_trossen_arm_imported STATIC IMPORTED)
        set_target_properties(_trossen_arm_imported PROPERTIES
            IMPORTED_LOCATION "${_trossen_lib}"
            INTERFACE_INCLUDE_DIRECTORIES "${TROSSEN_ARM_DIR}/include"
        )
        if(UNIX)
            target_link_libraries(_trossen_arm_imported INTERFACE pthread)
        endif()
        add_library(TrossenArm::TrossenArm ALIAS _trossen_arm_imported)
        message(STATUS "TrossenArm: using checkout at ${TROSSEN_ARM_DIR}")
        return()
    else()
        message(WARNING
            "TrossenArm: TROSSEN_ARM_DIR set but ${_trossen_lib} is missing; "
            "falling back to FetchContent.")
    endif()
endif()

# ---------------------------------------------------------------------------
# 3) FetchContent fallback.
# ---------------------------------------------------------------------------
option(TROSSEN_ARM_FETCH "Fetch libtrossen_arm from upstream when not found locally" ON)
if(NOT TROSSEN_ARM_FETCH)
    message(FATAL_ERROR
        "libtrossen_arm not found and TROSSEN_ARM_FETCH=OFF. "
        "Install it system-wide, set TROSSEN_ARM_DIR, or enable the fetch.")
endif()

set(TROSSEN_ARM_GIT_REPOSITORY "https://github.com/TrossenRobotics/trossen_arm.git"
    CACHE STRING "Upstream libtrossen_arm git repository")
set(TROSSEN_ARM_GIT_TAG "v1.10.0"
    CACHE STRING "Upstream libtrossen_arm git tag/branch/commit")

include(FetchContent)
FetchContent_Declare(libtrossen_arm
    GIT_REPOSITORY "${TROSSEN_ARM_GIT_REPOSITORY}"
    GIT_TAG        "${TROSSEN_ARM_GIT_TAG}"
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  _disabled  # download only; do not add_subdirectory() upstream
)
# We can't reuse upstream's `libtrossen_arm` target because it is declared
# `STATIC IMPORTED` (not GLOBAL) inside their CMakeLists, so it is invisible
# outside their subdirectory scope. Pointing SOURCE_SUBDIR at a path that
# does not exist tells FetchContent to populate the source but skip
# add_subdirectory; we then build our own imported target against the
# pre-built static library that ships in the source tree.
FetchContent_MakeAvailable(libtrossen_arm)

set(_trossen_lib "${libtrossen_arm_SOURCE_DIR}/lib/${_trossen_os}/${_trossen_arch}/libtrossen_arm.a")
if(NOT EXISTS "${_trossen_lib}")
    message(FATAL_ERROR
        "TrossenArm: expected static library at ${_trossen_lib} after fetch — "
        "upstream may not ship a binary for ${_trossen_os}/${_trossen_arch}.")
endif()

add_library(_trossen_arm_imported STATIC IMPORTED GLOBAL)
set_target_properties(_trossen_arm_imported PROPERTIES
    IMPORTED_LOCATION "${_trossen_lib}"
    INTERFACE_INCLUDE_DIRECTORIES "${libtrossen_arm_SOURCE_DIR}/include"
)
if(UNIX)
    target_link_libraries(_trossen_arm_imported INTERFACE pthread)
endif()
add_library(TrossenArm::TrossenArm ALIAS _trossen_arm_imported)
message(STATUS "TrossenArm: fetched ${TROSSEN_ARM_GIT_REPOSITORY}@${TROSSEN_ARM_GIT_TAG} → ${_trossen_lib}")
