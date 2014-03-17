AR_NAV_SDK_DIR = /home/ahliao/AR_SDK/ARDrone_SDK_2_0_1/ARDroneLib/Soft/Common/
# EDIT to where you keep the AR Drone SDK
# This is used mainly for the navdata struct and 
# some other structs made by Parrot
ARDRONELIB_DIR = /home/ahliao/AR_SDK/ARDrone_SDK_2_0_1/ARDroneLib/


AR_COMMON_DIR = $(ARDRONELIB_DIR)Soft/Common/
AR_LIB_DIR = $(ARDRONELIB_DIR)Soft/Lib/
#AR_VP_DIR = $(ARDRONELIB_DIR)VP_SDK/

CC = g++
CFLAGS = -g -Wall
SRCS_DIR = ../Sources
SRCS_VIDEO_DIR = $(SRCS_DIR)/Video
SRCS := $(wildcard $(SRCS_DIR)/*.cpp)
SRCS += $(wildcard $(SRCS_VIDEO_DIR)/*.cpp)
PROG_DIR = Release
PROG = Pillar_AR_GroundStation
 
OPENCV = `pkg-config opencv --cflags --libs`
LIBS =  -lavformat -lavcodec -lavutil -lz -lm -lpthread -lswscale
	 
$(PROG):$(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(PROG_DIR)/$(PROG) $(LIBS) -I $(AR_LIB_DIR) $(OPENCV)
