CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (playerplugin_examples)

# Include this CMake module to get most of the settings needed to build
#SET (CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules")
SET (CMAKE_MODULE_PATH "/informatik/isr/tams/software/player64/share/cmake/Modules")
INCLUDE (UsePlayerPlugin)

PLAYER_ADD_PLUGIN_DRIVER (exampledriver SOURCES exampledriver.cc INCLUDEDIRS /Users/sebastian/tams/taser/ben/src)
