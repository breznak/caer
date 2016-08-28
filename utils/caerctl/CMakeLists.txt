# Currently not supported on Windows due to missing shell library.
# Just edit the XML config file and restart for now!
IF (NOT OS_WINDOWS)
	# Compile cAERCtl
	ADD_EXECUTABLE(caer-ctl ../../ext/sshs/sshs_helper.c ../ext/linenoise/linenoise.c caerctl.c)
	INSTALL(TARGETS caer-ctl DESTINATION ${CMAKE_INSTALL_BINDIR})
ENDIF()
