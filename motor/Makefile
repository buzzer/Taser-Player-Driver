# Desc: Example plugin makefile
# CVS: $Id: Makefile.example 4226 2007-10-24 19:01:14Z thjc $

INCPATH = -I/usr/share/qt4/mkspecs/default -I. -I/usr/include/QtCore -I/usr/include/QtNetwork -I/usr/include/QtGui -I/usr/include -I. -I.. -I../devices -I../canbus -I.

all: libtaserdriver.so

%.o: %.cc
	$(CXX) -Wall -fpic -g3 `pkg-config --cflags playercore` -c $< $(INCPATH)

libtaserdriver.so: taserdriver.o
	$(CXX) -shared -nostartfiles -o $@ $^

clean:
	rm -f *.o *.so
