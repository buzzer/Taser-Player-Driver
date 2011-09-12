CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (playerplugin_taser)

# Include this CMake module to get most of the settings needed to build
SET (CMAKE_MODULE_PATH "/informatik/isr/tams/software/player64/share/cmake/Modules")
# TODO add CMakeLists.txt to directory
ADD_SUBDIRECTORY( ../../../tams/taser/ben/src/devices ../../../tams/taser/ben/src/devices/build)


# TODO remove QT dependency
find_package ( Qt4 REQUIRED )
include ( ${QT_USE_FILE} )
include_directories (
  ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
  ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
  ${QT_QTNETWORK_INCLUDE_DIR}
  .)

INCLUDE (UsePlayerPlugin)

PLAYER_ADD_PLUGIN_DRIVER (
  taserdriver
  SOURCES
  taserdriver
  INCLUDEDIRS
  ../../../tams/taser/ben/src
  ../../../tams/taser/ben/src/devices
  ../../../tams/taser/ben/src/canserver
  ../../../tams/taser/ben/src/canbus
  ../../../robotdev/peak-linux-driver-7.3/driver
  )
