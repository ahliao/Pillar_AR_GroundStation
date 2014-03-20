/**
 *	Support class to handle configuration commands
 *	to the AR Drone 2 using pre-connected sockets
 *	Language: C++
 *	Written by: Alex Liao
*/

#ifndef AR_DRONE_H
#define AR_DRONE_H

// Socket headers
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

class ARSupport
{
	public:
		// Constructor and Destructor
		ARSupport();
		~ARSupport();
	
	private:

};

#endif
