find_package(realsense2 REQUIRED)

add_executable(vr_follower vr_follower.cpp)

# Reuses the follower's RealSense streamer.
target_include_directories(vr_follower PRIVATE
    "${PROJECT_SOURCE_DIR}/follower"
)

target_link_libraries(vr_follower PRIVATE
    trossen_adamo_common
    trossen_vr::trossen_vr_lib
    realsense2::realsense2
)

include(GNUInstallDirs)
install(TARGETS vr_follower RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
