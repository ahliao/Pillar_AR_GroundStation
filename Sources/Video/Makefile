CPP = g++
CPPFLAGS = -g -Wall
OBJ_DIR = ../../obj/

VIDEOSRCS := $(wildcard *.cpp)
VIDEOOBJ = $(addprefix $(OBJ_DIR),$(VIDEOSRCS:.cpp=.o))

all: $(VIDEOOBJ)

$(VIDEOOBJ): $(OBJ_DIR)%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir $(OBJ_DIR)

clean: 
	echo "Nothing to clean"
