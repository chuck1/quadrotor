
cpp_files = main.cpp 

cflags = -Wall -std=c++0x -g -rdynamic

all:
	g++ ${cpp_files} -g -c -o main.o -lquadrotor -lmath -lgalaxy $(cflags)
	g++ main.o -o quad -lquadrotor -lmath -lgalaxy $(cflags)




