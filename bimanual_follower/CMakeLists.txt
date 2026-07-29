if(ADAMO_TROSSEN_ENABLE_REALSENSE)
    find_package(realsense2 REQUIRED)
endif()
if(ADAMO_TROSSEN_ENABLE_ZED)
    find_package(ZED 4 REQUIRED)
    find_package(CUDA REQUIRED)
endif()
if(NOT ADAMO_TROSSEN_ENABLE_REALSENSE AND NOT ADAMO_TROSSEN_ENABLE_ZED)
    message(WARNING
        "bimanual_follower: built with no camera backend enabled "
        "(ADAMO_TROSSEN_ENABLE_REALSENSE and ADAMO_TROSSEN_ENABLE_ZED are both OFF); "
        "only --no-camera will work at runtime.")
endif()

add_executable(bimanual_follower
    bimanual_follower.cpp
)

target_link_libraries(bimanual_follower
    PRIVATE
        trossen_adamo_common
)

if(ADAMO_TROSSEN_ENABLE_REALSENSE)
    target_compile_definitions(bimanual_follower PRIVATE ADAMO_TROSSEN_HAS_REALSENSE)
    target_link_libraries(bimanual_follower PRIVATE realsense2::realsense2)

    # Propagate the realsense lib dir to RPATH so the binary finds it at runtime.
    if(realsense2_DIR)
        get_filename_component(_rs_prefix "${realsense2_DIR}/../../.." ABSOLUTE)
        set_target_properties(bimanual_follower PROPERTIES
            BUILD_RPATH "${_rs_prefix}/lib"
        )
    endif()
endif()

if(ADAMO_TROSSEN_ENABLE_ZED)
    target_compile_definitions(bimanual_follower PRIVATE ADAMO_TROSSEN_HAS_ZED)
    target_include_directories(bimanual_follower PRIVATE
        ${ZED_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
    )
    target_link_directories(bimanual_follower PRIVATE
        ${ZED_LIBRARY_DIR}
        ${CUDA_LIBRARY_DIRS}
    )
    target_link_libraries(bimanual_follower PRIVATE
        ${ZED_LIBRARIES}
        ${CUDA_LIBRARIES}
    )
endif()

include(GNUInstallDirs)
install(TARGETS bimanual_follower RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
