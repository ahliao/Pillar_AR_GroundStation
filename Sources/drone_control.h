// File: keyboard_controller.h
// Author: Pillar Technologies MDP Team
// Date Created: 3/9/2014
// Function: Header file for keyboard_controller.cpp

#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H


// REQUIRES: run in a seperate process (maybe try threads later?)
// EFFECTS:  handles the control of the drone through the keyboard
//			 Return 1 if the user quit the program
void drone_control();

#endif // DRONE_CONTROLLER_H
