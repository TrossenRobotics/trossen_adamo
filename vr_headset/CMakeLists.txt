add_executable(vr_headset vr_headset.cpp)
target_link_libraries(vr_headset PRIVATE trossen_adamo_common)

include(GNUInstallDirs)
install(TARGETS vr_headset RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
