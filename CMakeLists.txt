# Init CMake (require at least version 2.8)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# General build settings
IF (NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Possible build types: None Debug Release RelWithDebInfo MinSizeRel")
ENDIF()

# Project name and version
PROJECT(cAER C)
SET(PROJECT_VERSION_MAJOR 0)
SET(PROJECT_VERSION_MINOR 9)
SET(PROJECT_VERSION_PATCH 4)
SET(PROJECT_VERSION_NOREV ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
IF (NOT PROJECT_REVISION)
	EXECUTE_PROCESS(COMMAND svnversion OUTPUT_VARIABLE PROJECT_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE)
	# EXECUTE_PROCESS(COMMAND git rev-parse HEAD OUTPUT_VARIABLE PROJECT_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE)
ENDIF()
SET(PROJECT_VERSION ${PROJECT_VERSION_NOREV}-r${PROJECT_REVISION})
MESSAGE(STATUS "Project version is: ${PROJECT_VERSION}")

# Set compiler info
SET(CC_CLANG FALSE)
SET(CC_GCC FALSE)
SET(CC_ICC FALSE)
SET(CC_MSVC FALSE)

IF ("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
	SET(CC_CLANG TRUE)
ELSEIF ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
	SET(CC_GCC TRUE)
ELSEIF ("${CMAKE_C_COMPILER_ID}" STREQUAL "Intel")
	SET(CC_ICC TRUE)
ELSEIF ("${CMAKE_C_COMPILER_ID}" STREQUAL "MSVC")
	SET(CC_MSVC TRUE)
ENDIF()

# Test if we are on a big-endian architecture
INCLUDE(TestBigEndian)
TEST_BIG_ENDIAN(SYSTEM_BIGENDIAN)

# C99 standard needed
IF (CC_GCC OR CC_CLANG)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
ENDIF()

# Check size of various types
INCLUDE(CheckTypeSize)
CHECK_TYPE_SIZE("size_t"        SIZEOF_SIZE_T)
CHECK_TYPE_SIZE("void *"        SIZEOF_VOID_PTR)

IF (NOT "${SIZEOF_VOID_PTR}" STREQUAL "${SIZEOF_SIZE_T}")
	MESSAGE(SEND_ERROR "Size of void * and size_t must be the same!")
ENDIF()

# Check for existence of functions
INCLUDE(CheckFunctionExists)
CHECK_FUNCTION_EXISTS(posix_memalign HAVE_POSIX_MEMALIGN)

# Check threads support
FIND_PACKAGE(Threads)
SET(HAVE_PTHREADS ${CMAKE_USE_PTHREADS_INIT})
SET(HAVE_WIN32_THREADS ${CMAKE_USE_WIN32_THREADS_INIT})

IF (HAVE_PTHREADS)
	# Check specific include files
	CHECK_INCLUDE_FILES(unistd.h HAVE_UNISTD_H)

	# pthread_rwlock_t support
	ADD_DEFINITIONS(-D_XOPEN_SOURCE=600)

	# endian conversions
	ADD_DEFINITIONS(-D_BSD_SOURCE=1)
ENDIF()

# Enable all warnings for GCC
IF (CC_GCC OR CC_CLANG)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic -Wall -Wextra")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wunused-parameter -Wformat=2 -Winit-self -Wundef")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wstrict-prototypes -Wmissing-prototypes -Wredundant-decls")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wshadow -Wbad-function-cast -Wfloat-equal")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wconversion -Wunused -Wstrict-overflow=5")
ENDIF()

# Search for external libraries with pkg-config
INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(LIBUSB1 REQUIRED libusb-1.0)
PKG_CHECK_MODULES(MXML REQUIRED mxml)
INCLUDE_DIRECTORIES(${LIBUSB1_INCLUDE_DIRS} ${MXML_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBUSB1_LIBRARY_DIRS} ${MXML_LIBRARY_DIRS})

# Add local directory to include paths
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/)

# Compile cAER
file(GLOB_RECURSE CAER_EXT_FILES ext/*.c)
file(GLOB_RECURSE CAER_C_FILES base/*.c modules/*.c)

ADD_EXECUTABLE(caer ${CAER_EXT_FILES} ${CAER_C_FILES} main.c)
TARGET_LINK_LIBRARIES(caer ${CMAKE_THREAD_LIBS_INIT} rt m ${LIBUSB1_LIBRARIES} ${MXML_LIBRARIES})
INSTALL(TARGETS caer DESTINATION bin)

# Compile cAERCtl
ADD_EXECUTABLE(caerctl ext/sshs/sshs_helper.c utils/ext/linenoise/linenoise.c utils/caerctl/caerctl.c)
INSTALL(TARGETS caerctl DESTINATION bin)

# Compile UDP stream statistics program
ADD_EXECUTABLE(udpststat utils/udpststat/udpststat.c)
INSTALL(TARGETS udpststat DESTINATION bin)

# Compile TCP stream statistics program
ADD_EXECUTABLE(tcpststat utils/tcpststat/tcpststat.c)
INSTALL(TARGETS tcpststat DESTINATION bin)

# Compile Unix local stream statistics program
ADD_EXECUTABLE(unixststat utils/unixststat/unixststat.c)
INSTALL(TARGETS unixststat DESTINATION bin)
