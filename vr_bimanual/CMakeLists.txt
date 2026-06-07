add_executable(vr_bimanual
    vr_bimanual.cpp
)

target_link_libraries(vr_bimanual
    PRIVATE
        trossen_adamo_common
)

include(GNUInstallDirs)
install(TARGETS vr_bimanual RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
