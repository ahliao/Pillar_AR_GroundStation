// File: drone_controller.h
// Author: Pillar Technologies MDP Team
// Date Created: 3/23/2014
// Function: Header file for the DroneController class
// C++ class implementation of the drone_init and drone_control files

#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <cstdio>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <vector>
#include <ctime>

// Headers for the navdata structs
#include "navdata_common.h"

#include "Video/tag_reader.h"

// Macros for connecting to the drone
#define NAVDATA_PORT	5554
#define AT_PORT			5556
#define NAVDATA_BUFFER_SIZE		2048
#define WIFI_MYKONOS_IP			"192.168.1.1"

enum NavdataOptions {
	NAVDATA_PHYS,
	NAVDATA_RAW,
	NAVDATA_ALL
};

enum ControlBasic {
	TAKEOFF,
	LAND, 
	EMERGENCY
};

enum VideoChannel {
	FRONT,
	BOTTOM
};

const static int GRID_COL = 1;
const static int GRID_ROW = 1;
const static int GRID_STEP = 10;

class DroneController
{
	public:
		// Constructor and destructor
		DroneController();
		~DroneController();

		// REQUIRES: the global sockaddr_in structs, at_socket and navdata_socket
		// MODIFIES: the required stuff
		// EFFECTS:  Initializes the communication between the computer and drone
		bool init_ports();

		// EFFECTS:  returns true if sockets are connected, else false
		bool is_connected();

		// EFFECTS: closes at_socket and navdata_socket
		void close_ports();

		// REQURIES: navdata_socket is connected with the pc_addr
		// MODIFIES: terminal (ncurses)
		// EFFECTS:  receives the navdata from the drone and displays it
		int get_navdata(navdata_t ** data);

		///////////////////////////
		//// Control Functions ////
		///////////////////////////

		// Control Loop
		void control_loop(const navdata_t *const data, const std::vector<TagData> &tagdata);

		// Basic takeoff, land, and emergency commands
		void control_basic(ControlBasic cmd);

		// Translate and rotate the drone
		void control_move(const bool enable, const float roll, 
				const float pitch, const float vx, const float rotspeed);

		// Translate and rotate the drone 
		// Uses magnetometer for rotation
		// For external magnetometers
		// NEEDS TO BE TESTED
		void control_move_mag(bool enable, float roll, float pitch, 
				float vx, float rotspeed, float magpsi, float magpsiaccur);

		// Tell the drone it is horizontal
		void control_ftrim();

		// Drone LED Animation
		void control_led(const uint8_t led, const float freq, const uint8_t dur);

		// Reset the communication watchdog
		void reset_comm_watchdog();

		// REQUIRES: The drone and computer are connected and on
		// EFFECTS:	 sends AT commands to set default configs 
		void default_config();

		// if b is true, sets drone to send reduced navdata
		void config_navdata_demo(bool b);

		// Config additional navdata options
		void config_options(NavdataOptions options);

		// Set which gain/config values to use if outside or inside
		void config_outdoor(bool outside);

		// Tell drone if OUTDOOR shell is attached
		// set to false if using indoor shell
		void config_outdoor_shell(bool outsideShell);

		///////////////////////////////////////////////////////////
		// These functions are set to the inside or outside field///
		// Depending on what 'outside' is set to //////////////////

		// Sets the max attitude the drone can go (in millimeters)
		void config_attitude_max(uint32_t max);

		// Sets the max vertical speed of the drone (200 to 2000 mm/sec)
		void config_vz_max(uint32_t max);

		// Sets the max yaw (rotate) speed (0.7 to 6.11 rad/sec)
		void config_vyaw_max(uint32_t max);

		/////////////////////////////////////////////////////////////

		// Video config
		void config_video_channel(VideoChannel channel);

	private:
		// REQUIRES: a is a valid float
		// EFFECTS:  the 32-bit integer form of a
		int IEEE754toInt(const float a);

		// Send an AT command to the drone
		void send_drone_command(const char command[]);

		//// Member variables

		// The sockets connecting to the drone
		int at_socket,			// sendto
			navdata_socket;	// recvfrom

		// structs to hold the config of the socket addresses
		sockaddr_in pc_addr, drone_at, drone_nav, from;
		
		// the command message to be sent
		char command[256];

		// the current command number being sent
		uint32_t seq;

		// Holds the navdata
		char msg[2048];	// navdata message

		// The flight time
		timeval takeoff_time, curr_time;
		bool flying;

		// The center of the camera frame
		int frame_mid_x, frame_mid_y;

		// bool if currently connected to the drone
		bool m_connected;
};

#endif // DRONE_CONTROLLER_H
