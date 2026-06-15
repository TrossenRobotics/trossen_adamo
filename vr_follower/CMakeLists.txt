add_executable(vr_follower vr_follower.cpp)
target_link_libraries(vr_follower PRIVATE trossen_adamo_common)

include(GNUInstallDirs)
install(TARGETS vr_follower RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
