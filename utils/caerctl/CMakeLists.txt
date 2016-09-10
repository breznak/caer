# Compile cAERCtl
ADD_EXECUTABLE(caer-ctl ../../ext/sshs/sshs_helper.c ../ext/linenoise/linenoise.c caerctl.c)
TARGET_LINK_LIBRARIES(caer-ctl ${LIBUV_LIBRARIES})
INSTALL(TARGETS caer-ctl DESTINATION ${CMAKE_INSTALL_BINDIR})
