add_executable(bimanual_leader bimanual_leader.cpp)
target_link_libraries(bimanual_leader PRIVATE trossen_adamo_common)

include(GNUInstallDirs)
install(TARGETS bimanual_leader RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
