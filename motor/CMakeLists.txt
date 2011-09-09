CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (playerplugin_tasers)

# Include this CMake module to get most of the settings needed to build
SET (CMAKE_MODULE_PATH "/informatik/isr/tams/software/player64/share/cmake/Modules")
SET (CMAKE_INCLUDE_PATH "/informatik/home/rockel/tams/taser/ben/src/")
ADD_SUBDIRECTORY("${CMAKE_INCLUDE_PATH}" "${CMAKE_CURRENT_BINARY_DIR}/bensrc")
ADD_SUBDIRECTORY("${CMAKE_INCLUDE_PATH}/devices" "${CMAKE_INCLUDE_PATH}/bensrc")
# TODO remove QT dependency
find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
include_directories (
  ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
  ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
  ${QT_QTNETWORK_INCLUDE_DIR}
  .)

INCLUDE (UsePlayerPlugin)

PLAYER_ADD_PLUGIN_DRIVER (taserdriver SOURCES taserdriver.cc INCLUDEDIRS /Users/sebastian/tams/taser/ben/src)
