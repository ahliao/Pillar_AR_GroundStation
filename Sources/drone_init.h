// File: drone_init.h
// Author: Pillar Technologies MDP Team
// Date Created: 3/9/2014
// Function: Header file for drone_init.cpp 

#ifndef DRONE_INIT_H
#define DRONE_INIT_H

#include <cstdio>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

// Headers from the Parrot API
#include "navdata_common.h"

// Macros for connecting to the drone
#define NAVDATA_PORT	5554
#define AT_PORT			5556
#define NAVDATA_BUFFER_SIZE		2048
#define WIFI_MYKONOS_IP			"192.168.1.1"

enum DroneConfigOptions {
	NAVDATA_DEMO,
	NAVDATA_OPTIONS,
	ATTITUDE_MAX,
	CONTROL_VZ_MAX,
	CONTROL_YAW,
	OUTDOOR,
	FLIGHT_WITHOUT_SHELL,
	VIDEO_CODEC_FPS,
	VIDEO_CODEC,
	LEDS_ANIM
};

enum NavdataOptions {
	NAVDATA_PHYS,
	NAVDATA_RAW,
	NAVDATA_ALL
};

// The sockets connecting to the drone
extern int at_socket,			// sendto
		navdata_socket;	// recvfrom

// structs to hold the config of the socket addresses
extern struct sockaddr_in pc_addr, drone_at, drone_nav, from;

// the command message to be sent
extern char command[256];

// the current command number being sent
extern uint32_t seq;

// REQUIRES: a is a valid float
// EFFECTS:  the 32-bit integer form of a
int IEEE754toInt(const float &a);

// REQUIRES: the global sockaddr_in structs, at_socket and navdata_socket
// MODIFIES: the required stuff
// EFFECTS:  Initializes the communication between the computer and drone
int init_ports();

// EFFECTS: closes at_socket and navdata_socket
void close_ports();

// REQURIES: navdata_socket is connected with the pc_addr
// MODIFIES: terminal (ncurses)
// EFFECTS:  receives the navdata from the drone and displays it
int get_navdata(navdata_t ** data);

// REQUIRES: The drone and computer are connected and on
// EFFECTS:	 sends AT commands to set the config values on the drone
void drone_config();

// if b is true, sets drone to send reduced navdata
void set_drone_navdata_demo(bool b);

// Config additional navdata options
void set_drone_navdata_options(NavdataOptions options);

// Set which gain/config values to use if outside or inside
void set_drone_outdoor(bool outside);

// Tell drone if OUTDOOR shell is attached
// set to false if using indoor shell
void set_drone_outdoor_shell(bool outsideShell);

///////////////////////////////////////////////////////////
// These functions are set to the inside or outside field///
// Depending on what 'outside' is set to //////////////////

// Sets the max attitude the drone can go (in millimeters)
void set_drone_attitude_max(uint32_t max);

// Sets the max vertical speed of the drone (200 to 2000 mm/sec)
void set_drone_vz_max(uint32_t max);

// Sets the max yaw (rotate) speed (0.7 to 6.11 rad/sec)
void set_drone_vyaw_max(uint32_t max);

/////////////////////////////////////////////////////////////

// Send an AT command to the drone
void send_drone_command(char command[]);
#endif // DRONE_INIT_H
