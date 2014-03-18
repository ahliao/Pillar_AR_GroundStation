// File: drone_init.cpp
// Author: Pillar Technologies MDP Team
// Date Created: 3/9/2014
// Function: Contains the functions for connecting the
//			 computer to the AR Drone 2 through UDP/TCP

#include "drone_init.h"

// The sockets connecting to the drone
int at_socket = -1,			// sendto
	navdata_socket = -1;	// recvfrom

// structs to hold the config of the socket addresses
struct sockaddr_in 	pc_addr, drone_at, drone_nav, from;
char msg[NAVDATA_BUFFER_SIZE];	// navdata message

// Converts the IEEE 754 floating-point format to the respective integer form
int IEEE754toInt(const float &a)
{
	union {		// bad practice, but it works
		float f;
		int i;
	} u;
	u.f = a;
	return u.i;
}

// Initialize the ports for the AT commands and the Navdata
int init_ports() 
{
	if ((at_socket = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {
		std::cerr << "Error creating at_socket\n";
		return 1;
	}

	if ((navdata_socket = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {
		std::cerr << "Error creating navdata_socket\n";
		return 1;
	}

	// for recvfrom
	pc_addr.sin_family = AF_INET;
	pc_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	pc_addr.sin_port = htons(0);

	// for sendto AT
	drone_at.sin_family = AF_INET;
	drone_at.sin_addr.s_addr = inet_addr(WIFI_MYKONOS_IP);
	drone_at.sin_port = htons(AT_PORT);

	// for sendto navdata init
	drone_nav.sin_family = AF_INET;
	drone_nav.sin_addr.s_addr = inet_addr(WIFI_MYKONOS_IP);
	drone_nav.sin_port = htons(NAVDATA_PORT);

	if (bind(navdata_socket, (struct sockaddr *) &pc_addr, sizeof(pc_addr)) < 0) {
		std::cout << "Error binding navdata_socket to pc_addr\n";
		return 0;
	}
	else std::cout << "bound navdaa_socket\n";

	return 0;	// return zero if nothing went wrong
}

int get_navdata(navdata_t **data)
{
	// read the navdata received
	//mvprintw(3,0,"Navdata Received %d", i);
	int l, size = 0;
	size = recvfrom(navdata_socket, &msg[0], NAVDATA_BUFFER_SIZE, 0x0, 
			(struct sockaddr *)&from, (socklen_t *) &l);
	if (size == 0) return 1;
	//mvprintw(4,0,"read %d", size); 
	*data = (navdata_t *) msg;
	/*mvprintw(5,0,"header %d", data->header);
	mvprintw(6,0,"Battery %d", 
			((navdata_demo_t*)((data->options)))->vbat_flying_percentage);
	mvprintw(7,0,"Alt %d", 
			((navdata_demo_t*)((data->options)))->altitude);
	mvprintw(8,0,"Vx %d",
			((navdata_demo_t*)((data->options)))->vx);
	mvprintw(9,0,"Theta %f",
			((navdata_demo_t*)((data->options)))->theta);*/
	return 0;
}

void close_ports() {
	// close the sockets
	close(at_socket);
	close(navdata_socket);
}
