AR_NAV_SDK_DIR = /home/ahliao/AR_SDK/ARDrone_SDK_2_0_1/ARDroneLib/Soft/Common/
AR_MATH_SDK_DIR = /home/ahliao/AR_SDK/ARDrone_SDK_2_0_1/ARDroneLib/Soft/Lib/
VP_SDK_DIR = /home/ahliao/AR_SDK/ARDrone_SDK_2_0_1/ARDroneLib/VP_SDK/
HOME_DIR = /home/ahliao/

CC = g++
CFLAGS = -g -Wall
SRCS_DIR = ../Sources
SRCS := $(wildcard ../Sources/*.cpp)
#SRCS = ../Sources/keyboard_controller.cpp ../Sources/drone_init.cpp
PROG_DIR = Release
PROG = keyboard_controller
 
#OPENCV = `pkg-config opencv --cflags --libs`
LIBS =  -lcurses -lavformat -lavcodec -lavutil -lz -lm -lpthread -lswscale
	 
$(PROG):$(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(PROG_DIR)/$(PROG) $(LIBS) -I $(AR_NAV_SDK_DIR) -I $(VP_SDK_DIR) -I $(AR_MATH_SDK_DIR)
