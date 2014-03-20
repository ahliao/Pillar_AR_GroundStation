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
		ARSupport(int ar_sock, int navdata_sock, sockaddr_in pc_addr,
				sockaddr_in drone_at, sockaddr_in drone_nav);
		~ARSupport();

		
	
	private:
		// The sockets
		int m_at_socket, m_navdata_socket;

		// Socket address config
		sockaddr_in m_pc_addr, m_drone_at, m_drone_nav, m_from;

};

#endif
