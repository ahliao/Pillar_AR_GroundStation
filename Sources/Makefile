CPP = g++
CPPFLAGS = -g -Wall
OBJ_DIR = ../obj/

MAINSRCS := $(wildcard *.cpp)
MAINOBJ = $(addprefix $(OBJ_DIR),$(MAINSRCS:.cpp=.o))

all: $(MAINOBJ)

$(MAINOBJ): $(OBJ_DIR)%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir $(OBJ_DIR)

clean: 
	echo "Nothing to clean"
