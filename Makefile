# Recursively calls the Makefiles in each directory to compile the source
SRCS_DIR = Sources
SRCS_VID_DIR = $(SRCS_DIR)/Video
SRCS_APRIL_DIR = $(SRCS_VID_DIR)/apriltags
OBJ_DIR = obj

all:
	cd $(SRCS_DIR); $(MAKE)
	cd $(SRCS_VID_DIR); $(MAKE)
	cd $(SRCS_APRIL_DIR); $(MAKE)
	cd $(OBJ_DIR); $(MAKE)
	echo yay

clean:
	cd $(OBJ_DIR); $(MAKE) clean
	cd $(SRCS_DIR); $(MAKE) clean
	cd $(SRCS_VID_DIR); $(MAKE) clean
	cd $(SRCS_APRIL_DIR); $(MAKE) clean
	
