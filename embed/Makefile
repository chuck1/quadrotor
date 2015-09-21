
libs = -ldrone

cfiles = main.cpp sketch.cpp EmbeddedDrone.cpp AccelGyro.cpp

all:
	g++ $(cfiles) -o embed -I. -I../drone/include -L../drone/build/static $(libs) -std=c++0x
