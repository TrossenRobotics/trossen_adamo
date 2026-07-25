find_package(realsense2 REQUIRED)
find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)

add_executable(trossen_follower
    follower.cpp
)

target_include_directories(trossen_follower PRIVATE
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)

target_link_directories(trossen_follower PRIVATE
    ${ZED_LIBRARY_DIR}
    ${CUDA_LIBRARY_DIRS}
)

target_link_libraries(trossen_follower
    PRIVATE
        trossen_adamo_common
        realsense2::realsense2
        ${ZED_LIBRARIES}
        ${CUDA_LIBRARIES}
)

# When realsense2 is not installed system-wide, librealsense often lives next
# to its CMake package — propagate its lib dir to our RPATH so the binary
# finds it at runtime without LD_LIBRARY_PATH gymnastics.
if(realsense2_DIR)
    get_filename_component(_rs_prefix "${realsense2_DIR}/../../.." ABSOLUTE)
    set_target_properties(trossen_follower PROPERTIES
        BUILD_RPATH "${_rs_prefix}/lib"
    )
endif()

include(GNUInstallDirs)
install(TARGETS trossen_follower RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
