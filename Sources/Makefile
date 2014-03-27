CPP = g++
CPPFLAGS = -g -Wall -Werror -Wvla -pedantic -Wextra -Wconversion -O3
OBJ_DIR = ../obj/

MAINSRCS := $(wildcard *.cpp)
MAINOBJ = $(addprefix $(OBJ_DIR),$(MAINSRCS:.cpp=.o))

all: $(MAINOBJ)

$(MAINOBJ): $(OBJ_DIR)%.o: %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@ -fopenmp

$(OBJ_DIR):
	mkdir $(OBJ_DIR)

clean: 
	echo "Nothing to clean"
