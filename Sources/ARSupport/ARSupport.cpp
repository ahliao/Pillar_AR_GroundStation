/**
 *	ARSupport.cpp: implementation for 
 *	support class to handle configuration
 *	commands to the AR Drone 2
 *	Language: C++
 *	Written by: Alex Liao
*/

#include "ARSupport.h"

ARSupport::ARSupport(int ar_sock, int navdata_sock, sockaddr_in pc_addr,
		sockaddr_in drone_at, sockaddr_in drone_nav)
	: m_ar_sock(ar_sock), m_navdata_sock(navdata_sock),
	  m_pc_addr(pc_addr), m_drone_at(drone_at), m_drone_nav(drone_nav)
{
}

~ARSupport()
{

}
