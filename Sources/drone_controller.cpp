// File: drone_controller.cpp
// Author: Pillar Technologies MDP Team
// Date Created: 2/23/2014
// Function: Implementation of the DroneController class
// C++ class implementation of the drone_init and drone_control files

#include "drone_controller.h"

// Constructor
DroneController::DroneController()
{
	m_connected = init_ports();
	// Config the drone
	default_config();
}

// Destructor
DroneController::~DroneController()
{
	close_ports();
}

// REQUIRES: the global sockaddr_in structs, at_socket and navdata_socket
// MODIFIES: the required stuff
// EFFECTS:  Initializes the communication between the computer and drone
bool DroneController::init_ports()
{
	if ((at_socket = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {
		std::cerr << "Error creating at_socket\n";
		return false;
	}

	if ((navdata_socket = socket (AF_INET, SOCK_DGRAM, 0)) < 0) {
		std::cerr << "Error creating navdata_socket\n";
		return false;
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
		std::cerr << "Error binding navdata_socket to pc_addr\n";
		return false;
	}
	else std::cout << "Successfully bound navdata_socket\n";

	int32_t one = 1;
	// set unicast mode on
	sendto(navdata_socket, &one, 4, 0, (sockaddr *) &drone_nav, sizeof(drone_nav));

	return true;	// return true if nothing went wrong
}

// EFFECTS:  returns true if sockets are connected, else false
bool DroneController::is_connected()
{
	return m_connected;
}

// EFFECTS: closes at_socket and navdata_socket
void DroneController::close_ports()
{
	// close the sockets
	if (at_socket != -1) close(at_socket);
	if (navdata_socket != -1) close(navdata_socket);
}

// REQURIES: navdata_socket is connected with the pc_addr
// MODIFIES: terminal (ncurses)
// EFFECTS:  receives the navdata from the drone and displays it
int DroneController::get_navdata(navdata_t ** data)
{
	// read the navdata received
	int l, size = 0;
	size = recvfrom(navdata_socket, &msg[0], NAVDATA_BUFFER_SIZE, 0x0, 
			(struct sockaddr *)&from, (socklen_t *) &l);
	if (size == 0) return 1;
	*data = (navdata_t *) msg;
	return 0;
}

///////////////////////////
//// Control Functions ////
///////////////////////////

//////////////////////////
// Control Loop //////////
//////////////////////////

int DroneController::control_loop(const navdata_t *const navdata, const std::vector<TagData> &tagdata)
{
	if (navdata == 0) {
		std::cerr << "ERROR: Navdata is not valid.\n";
		return 1;
	}

	navdata_demo_t* nav_demo = ((navdata_demo_t*)(navdata->options));

	// Take off if currently landed
	if (nav_demo->ctrl_state == Landed) {
		control_led(1, 2.0, 2);
		control_ftrim();
		control_basic(TAKEOFF);
		gettimeofday(&takeoff_time, NULL);
	}

	// Land if flight time is over 10 seconds
	gettimeofday(&curr_time, NULL);
	long delta = curr_time.tv_sec - takeoff_time.tv_sec;
	if (delta >= 10) {
		control_led(2, 2.0, 2);
		control_basic(LAND);
	}

	control_move(false, 0, 0, 0, 0);

	return 0;
}

///////////////////////////
///// End Control Loop ////
///////////////////////////

// Basic takeoff, land, and emergency commands
void DroneController::control_basic(ControlBasic cmd)
{
	uint32_t m = (1 << 18) | (1 << 20) | (1 << 22) | (1 << 24) | (1 << 28);
	if (cmd == TAKEOFF) {
		m |= (1 << 9);
		m &= ~(1 << 8);
	}
	else if (cmd == LAND) {
		m &= ~(1 << 9);
		m &= ~(1 << 8);
	}
	else if (cmd == EMERGENCY) m |= (1 << 8);
	sprintf(command, "AT*REF=%d,%d\r", seq, cmd);
	send_drone_command(command);
}

// Translate and rotate the drone
void DroneController::control_move(const bool enable, const float &roll, 
		const float &pitch, const float &vx, const float &rotspeed)
{
	sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq, enable,
			IEEE754toInt(roll), IEEE754toInt(pitch), IEEE754toInt(vx),
			IEEE754toInt(rotspeed));
	send_drone_command(command);
}

// Translate and rotate the drone 
// Uses magnetometer for rotation
// For external magnetometers
// NEEDS TO BE TESTED
void DroneController::control_move_mag(bool enable, float roll, float pitch, 
		float vx, float rotspeed, float magpsi, float magpsiaccur)
{
	sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d,%d,%d\r", seq, enable,
			IEEE754toInt(roll), IEEE754toInt(pitch), IEEE754toInt(vx),
			IEEE754toInt(rotspeed), IEEE754toInt(magpsi), 
			IEEE754toInt(magpsiaccur));
	send_drone_command(command);
}

// Tell the drone it is horizontal
void DroneController::control_ftrim()
{
	sprintf(command, "AT*FTRIM=%d\r", seq);
	send_drone_command(command);
}

// Drone LED Animation
void DroneController::control_led(const uint8_t led, const float &freq, const uint8_t dur)
{
	sprintf(command, "AT*CONFIG=%d,\"leds:leds_anim\",\"%d,%d,%d\"\r", 
			seq, led, IEEE754toInt(freq), dur);
	send_drone_command(command);
}

// REQUIRES: The drone and computer are connected and on
// EFFECTS:	 sends AT commands to set default configs 
void DroneController::default_config()
{
	// set to reduced navdata
	config_navdata_demo(true);
	config_options(NAVDATA_ALL);
	config_outdoor(false);
	config_outdoor_shell(false);
	config_attitude_max(5000); // 5 meters
	config_vz_max(750);		// 0.75 m/sec
	config_vyaw_max(3);		// 3 rad/sec
}

// if b is true, sets drone to send reduced navdata
void DroneController::config_navdata_demo(bool b)
{
	if (b)
		sprintf(command, "AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", seq);
	else
		sprintf(command, "AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", seq);
	send_drone_command(command);
}

// Config additional navdata options
void DroneController::config_options(NavdataOptions options)
{
	// TODO: choose more navdata options
	sprintf(command, "AT*CONFIG=%d,\"general:navdata_options\",\"13\"\r", seq);
	send_drone_command(command);
}

// Set which gain/config values to use if outside or inside
void DroneController::config_outdoor(bool outside)
{
	if (outside)
		sprintf(command, "AT*CONFIG=%d,\"control:outdoor\",\"TRUE\"\r", seq);
	else
		sprintf(command, "AT*CONFIG=%d,\"control:outdoor\",\"FALSE\"\r", seq);
	send_drone_command(command);
}

// Tell drone if OUTDOOR shell is attached
// set to false if using indoor shell
void DroneController::config_outdoor_shell(bool outsideShell)
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
void DroneController::config_attitude_max(uint32_t max)
{
	sprintf(command, "AT*CONFIG=%d,\"control:altitude_max\",\"%d\"\r", seq, max);
	send_drone_command(command);
}

// Sets the max vertical speed of the drone (200 to 2000 mm/sec)
void DroneController::config_vz_max(uint32_t max)
{
	sprintf(command, "AT*CONFIG=%d,\"control:control_vz_max\",\"%d\"\r", seq, max);
	send_drone_command(command);
}

// Sets the max yaw (rotate) speed (0.7 to 6.11 rad/sec)
void DroneController::config_vyaw_max(uint32_t max)
{
	sprintf(command, "AT*CONFIG=%d,\"control:control_yaw\",\"%d\"\r", seq, max);
	send_drone_command(command);
}

// REQUIRES: a is a valid float
// EFFECTS:  the 32-bit integer form of a
int DroneController::IEEE754toInt(const float &a)
{
	union {		// bad practice, but it works
		float f;
		int i;
	} u;
	u.f = a;
	return u.i;
}

// Send an AT command to the drone
void DroneController::send_drone_command(char command[])
{
	// TODO: make sure sockets are open
	if (at_socket < 0) {
		std::cerr << "AT Socket not open\n";
		return;
	}

	sendto(at_socket, command, strlen(command), 0, 
			(struct sockaddr*)&drone_at, sizeof(drone_at));
	++seq;
}
