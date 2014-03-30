PROG_DIR = ../Build/Release
PROG = Pillar_AR_GroundStation

CPP = g++
CPPFLAGS = -g -Wall

OBJS = $(wildcard *.o)

OPENCV = `pkg-config opencv --cflags --libs`
LIBS =  -lavformat -lavcodec -lavutil -lz -lm -lpthread -lswscale -lboost_thread-mt -lncurses

all: $(PROG_DIR)
	$(CPP) -o $(PROG_DIR)/$(PROG) $(OBJS) $(LIBS) $(OPENCV)

$(PROG_DIR):
	mkdir $(PROG_DIR)

clean:
	rm -rf $(OBJS)
