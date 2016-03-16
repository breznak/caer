IF (NOT ENABLE_BAFILTER)
	SET(ENABLE_BAFILTER 0 CACHE BOOL "Enable the background activity filtering module")
ENDIF()

IF (ENABLE_BAFILTER)
	ADD_DEFINITIONS(-DENABLE_BAFILTER=1)

	SET(CAER_BA_FILES modules/backgroundactivityfilter/backgroundactivityfilter.c)

	SET(CAER_C_SRC_FILES ${CAER_C_SRC_FILES} ${CAER_BA_FILES} PARENT_SCOPE)
ENDIF()
