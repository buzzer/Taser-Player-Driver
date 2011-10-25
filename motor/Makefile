# Makefile for the Taser Projekt
# 2011-10-25 Sebastian Rockel
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
CC      = g++
LIBDIRS = -L /usr/lib64
LIBS    = -l boost_system -l boost_thread
CTAGS   = ctags
TAGFILE = tags
TARGET  = wallfollow
SRCS    = ${TARGET:=.cpp}
OBJS    = ${SRCS:.cpp=.obj}
INC     = include
DEP     = ${SRCS} ${INC}/${SRCS:.cpp=.h} Makefile
TAGSRCS = `pkg-config --cflags playerc++ | sed -e 's/-I//g' | sed -e 's/ .*//g'`
CFLAGSSTD=-pg    \
          -g3    \
          -ggdb    \
          -funit-at-a-time \
          -Wall  \
          -Wcast-align\
          -Waggregate-return \
          -Wcast-qual\
          -Wcomment\
          -Wno-deprecated-declarations\
          -Wdisabled-optimization\
          -Wreturn-type -Wfatal-errors\
          -Wunused
DEP_STD=Makefile

testClientBoost: testClientBoost.cc ${DEP_STD}
	${CC} -o testClientBoost ${CFLAGSSTD} testClientBoost.cc ${LIBDIRS} ${LIBS}

testServerBoost: testServerBoost.cc simplepacket.o ${DEP_STD}
	${CC} -o testServerBoost ${CFLAGSSTD} testServerBoost.cc simplepacket.o ${LIBDIRS} ${LIBS}

-cpp.:
	${CC} -c $<

clean:
	rm -f testClientBoost
	rm -f testServerBoost
