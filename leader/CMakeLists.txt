add_executable(trossen_leader leader.cpp)
target_link_libraries(trossen_leader PRIVATE trossen_adamo_common)

include(GNUInstallDirs)
install(TARGETS trossen_leader RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
