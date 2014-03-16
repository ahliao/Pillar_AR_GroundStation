CC = g++
CFLAGS = -g -Wall
SRCS := $(wildcard *.cpp)
#SRCS = ../Sources/keyboard_controller.cpp ../Sources/drone_init.cpp
PROG = main
 
OPENCV = `pkg-config opencv --cflags --libs`
LIBS =  -lavformat -lavcodec -lavutil -lz -lm -lpthread -lswscale -lpthread
	 
$(PROG):$(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(PROG) $(LIBS) $(OPENCV)
