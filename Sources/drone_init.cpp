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
char command[256];

uint32_t seq = 0;

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

void close_ports() {
	// close the sockets
	close(at_socket);
	close(navdata_socket);
}

int get_navdata(navdata_t **data)
{
	// read the navdata received
	int l, size = 0;
	size = recvfrom(navdata_socket, &msg[0], NAVDATA_BUFFER_SIZE, 0x0, 
			(struct sockaddr *)&from, (socklen_t *) &l);
	if (size == 0) return 1;
	*data = (navdata_t *) msg;
	return 0;
}

// REQUIRES: The drone and computer are connected and on
// EFFECTS:	 sends AT commands to set the config values on the drone
void drone_config()
{
	// init the drone configs
	// TODO: Decide what we want here
}

// if b is true, sets drone to send reduced navdata
void set_drone_navdata_demo(bool b)
{
	if (b)
		sprintf(command, "AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", seq);
	else
		sprintf(command, "AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", seq);
	send_drone_command(command);
}

// Config additional navdata options
void set_drone_navdata_options(NavdataOptions options)
{
	// TODO: choose more navdata options
	sprintf(command, "AT*CONFIG=%d,\"general:navdata_options\",\"13\"\r", seq);
	send_drone_command(command);
}

// Set which gain/config values to use if outside or inside
void set_drone_outdoor(bool outside)
{
	if (outside)
		sprintf(command, "AT*CONFIG=%d,\"control:outdoor\",\"TRUE\"\r", seq);
	else
		sprintf(command, "AT*CONFIG=%d,\"control:outdoor\",\"FALSE\"\r", seq);
	send_drone_command(command);
}

// Tell drone if OUTDOOR shell is attached
// set to false if using indoor shell
void set_drone_outdoor_shell(bool outsideShell)
{
	if (outsideShell)
		sprintf(command, "AT*CONFIG=%d,\"control:flight_without_shell\",\"TRUE\"\r", seq);
	else
		sprintf(command, "AT*CONFIG=%d,\"control:flight_without_shell\",\"FALSE\"\r", seq);
	send_drone_command(command);
}

///////////////////////////////////////////////////////////
// These functions are set to the inside or outside field///
// Depending on what 'outside' is set to //////////////////

// Sets the max attitude the drone can go (in millimeters)
void set_drone_attitude_max(uint32_t max)
{
	sprintf(command, "AT*CONFIG=%d,\"control:altitude_max\",\"%d\"\r", seq, max);
	send_drone_command(command);
}

// Sets the max vertical speed of the drone (200 to 2000 mm/sec)
void set_drone_vz_max(uint32_t max)
{
	sprintf(command, "AT*CONFIG=%d,\"control:control_vz_max\",\"%d\"\r", seq, max);
	send_drone_command(command);
}

// Sets the max yaw (rotate) speed (0.7 to 6.11 rad/sec)
void set_drone_vyaw_max(uint32_t max)
{
	sprintf(command, "AT*CONFIG=%d,\"control:control_yaw\",\"%d\"\r", seq, max);
	send_drone_command(command);
}

// Send the AT* command to the drone and increment the seq number
void send_drone_command(char command[])
{
	// TODO: make sure sockets are open
	sendto(at_socket, command, strlen(command), 0, 
			(struct sockaddr*)&drone_at, sizeof(drone_at));
	++seq;
}
