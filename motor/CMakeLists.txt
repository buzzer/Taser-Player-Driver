cmake_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (playerplugin_taser)

# Include this CMake module to get most of the settings needed to build
#SET (CMAKE_MODULE_PATH "/informatik/isr/tams/software/player64/share/cmake/Modules")
#SET (CMAKE_MODULE_PATH "/usr/share/cmake/Modules" "/informatik/home/rockel/local/share/cmake/Modules")
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/informatik/home/rockel/local/share/cmake/Modules")
# TODO add CMakeLists.txt to directory
#ADD_SUBDIRECTORY( ../../../tams/taser/ben/src/devices ../../../tams/taser/ben/src/devices/build)

SET(CMAKE_BUILD_TYPE "Debug")
# add debug definitions
IF( CMAKE_BUILD_TYPE STREQUAL "Debug" )
  ADD_DEFINITIONS( -ggdb -pg -g3 -Wall )
ENDIF()


# TODO remove QT dependency
find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
include_directories (
  ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
  ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
  ${QT_QTNETWORK_INCLUDE_DIR}
  .)

SET(SRC_FILES
  taserdriver
  #packet
  #logger
  )

INCLUDE (UsePlayerPlugin)

PLAYER_ADD_PLUGIN_DRIVER ( ${SRC_FILES} SOURCES ${SRC_FILES} )

#ADD_EXECUTABLE(${SRC_FILES})
#TARGET_LINK_LIBRARIES(taserdriver udpclient ${QT_LIBRARIES})
TARGET_LINK_LIBRARIES(taserdriver ${QT_LIBRARIES})
