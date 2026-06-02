find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(trossen_vr REQUIRED)

add_executable(vr_follower vr_follower.cpp)

target_link_libraries(vr_follower PRIVATE
    trossen_adamo_common
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    trossen_vr::trossen_vr_lib
)

include(GNUInstallDirs)
install(TARGETS vr_follower RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
