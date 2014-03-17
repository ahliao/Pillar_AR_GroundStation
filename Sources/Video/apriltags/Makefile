CPP = gcc
CPPFLAGS = -g -Wall -std=gnu99
OBJ_DIR = ../../../obj/

APRILSRCS := $(wildcard *.c)
APRILOBJ = $(addprefix $(OBJ_DIR),$(APRILSRCS:.c=.o))

all: $(APRILOBJ)

$(APRILOBJ): $(OBJ_DIR)%.o: %.c
	$(CPP) $(CPPFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir $(OBJ_DIR)

clean: 
	echo "Nothing to clean"
