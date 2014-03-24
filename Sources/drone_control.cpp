// File: keyboard_controller.cpp
// Author: Pillar Technologies MDP Team
// Date Created: 3/9/2014
// Function: Functions for controlling the drone with a keyboard

// Pillar headers
#include "drone_control.h"
#include "drone_init.h"

// Libraries for networking and communciation
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

// NCurses API for the keyboard input
#include <curses.h>
//int seq = 0;

void drone_control()
{
	/*char command[256];
	char input = 0;
	input = 0;
	if (seq < 2) 	// set the config for reduced data (twice for sureness)
		sprintf(command, "AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", seq);
	else if (seq < 4) // tell drone it is on flat land
		sprintf(command, "AT*FTRIM=%d\r", seq);
	else if (seq < 6)
		sprintf(command, "AT*CONFIG=%d,\"video:video_codec\",\"131\"\r", seq);
	else if (seq < 8)
		// Change the video
		sprintf(command, "AT*CONFIG=%d,\"video:video_channel\",\"1\"\r", seq);*/
	/*else if ((input = getch()) != 0) 
	{	// DO NOT HOLD DOWN KEYS
		if (input == '1') 
			sprintf(command, "AT*CONFIG=%d,\"leds:leds_anim\",\"3,1073741824,2\"\r", seq);
		else if (input == '2') 
			sprintf(command, "AT*CONFIG=%d,\"leds:leds_anim\",\"2,1073741824,2\"\r", seq);
		else if (input == ' ')
			sprintf(command, "AT*REF=%d,%d\r", seq, 290718208);
		else if (input == 'z')
			sprintf(command, "AT*REF=%d,%d\r", seq, 290717696);
		else if (input == 'x')	// hover
			sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq, 0,
				0, 0, 0, 0);
		else if (input == 'w')
			sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq, 1,
				0, -1097229926, 0, 0);
		else if (input == 's')
			sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq, 1,
				0, 1050253722, 0, 0); // 1061997773
		else if (input == 'a')
			sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq, 1,
				-1097229926, 0, 0, 0);	// -1085485875 = -0.8
		else if (input == 'd')
			sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq, 1,
				1050253722, 0, 0, 0);
		else if (input == 'm')
			sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq, 1,
				0, 0, 0, 1050253722);
		else if (input == 'n')
			sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq, 1,
				0, 0, 0, -1097229926);
		else if (input == 'q') {
			sprintf(command, "AT*REF=%d,%d\r", seq, 290717696);
			running = 0;
			return 1;
		}
	}*/
	/*else
		sprintf(command, "AT*COMWDG=%d\r",seq); // reset comm watchdog

	sendto(at_socket, command, strlen(command), 0, 
			(struct sockaddr*)&drone_at, sizeof(drone_at));
	seq++;*/
	//usleep(100000); // should be less than 0.5s to get all data
}
