CMAKE_MINIMUM_REQUIRED (VERSION 2.4 FATAL_ERROR)
PROJECT (taserplayerplugin)

# Include this CMake module to get most of the settings needed to build
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/informatik/home/rockel/local/share/cmake/Modules")

SET(CMAKE_BUILD_TYPE "Debug")
# Add debug definitions
IF( CMAKE_BUILD_TYPE STREQUAL "Debug" )
  ADD_DEFINITIONS( -ggdb -pg -g3 -Wall )
  ADD_DEFINITIONS( -DQT_DEBUG ) # qDebug() traces
ENDIF()

# to link shared libraries
ADD_DEFINITIONS( -fPIC )
#ADD_DEFINITIONS( ${QT_DEFINITIONS} )
#ADD_DEFINITIONS( -DQT )

# TODO remove QT dependency
FIND_PACKAGE ( Qt4  COMPONENTS QtCore QtNetwork QtGui REQUIRED )
FIND_PACKAGE ( Boost 1.40.0 COMPONENTS system REQUIRED )
SET(QT_USE_QTNETWORK 1)
INCLUDE ( ${QT_USE_FILE} )
INCLUDE_DIRECTORIES (
  ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
  ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
  ${QT_QTNETWORK_INCLUDE_DIR}
  .
  )
SET(BOOST_LIBS ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

# Define names
SET(HDR_EXT .h)
SET(LIB_EXT .a)
SET(LIB_NAME taserdriver)
#SET(LIB_PACKET packet)
#SET(LIB_LOGGER logger)
SET(LIB_PREFIX lib)
SET(SRC_FILES ${LIB_NAME})
SET(INC_FILES ${LIB_NAME}${HDR_EXT})

#SET(EXE_SERVER testServer)

# tell cmake to create .moc files for all files in the variable qtproject_SRCS that require such a file.
# note: this assumes that you use #include "header.moc" in your files
#qt4_automoc(${SRC_FILES})

QT4_WRAP_CPP(MOC_SRCS ${INC_FILES})
#QT4_WRAP_CPP(MOC_TEST_SRCS ${EXE_SERVER}${HDR_EXT})

INCLUDE (UsePlayerPlugin)

PLAYER_ADD_PLUGIN_DRIVER ( ${LIB_NAME} SOURCES ${SRC_FILES} )

# Include additional libraries
#ADD_LIBRARY (${LIB_PACKET} ${LIB_PACKET}) 
#ADD_LIBRARY (${LIB_LOGGER} ${LIB_LOGGER}) 

# Link the shared library
TARGET_LINK_LIBRARIES(
  ${LIB_NAME}
  #${PROJECT_BINARY_DIR}/${LIB_PREFIX}${LIB_PACKET}${LIB_EXT}
  #${PROJECT_BINARY_DIR}/${LIB_PREFIX}${LIB_LOGGER}${LIB_EXT}
  #${QT_LIBRARIES}
  ${BOOST_LIBS}
  )

# Tests
#ADD_EXECUTABLE (${EXE_SERVER} ${EXE_SERVER})
#TARGET_LINK_LIBRARIES(${EXE_SERVER} ${BOOST_LIBS})
