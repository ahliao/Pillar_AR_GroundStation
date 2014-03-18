// Controller for the AR Drone 2
// using UDP and AT commands
// for use by the Pillar Technologies MDP Team
// Created on: 3/2/2014

// Pillar headers
#include "keyboard_controller.h"
#include "drone_init.h"

// Video headers
#include "Video/ardrone_video.h"
#include "Video/ardrone_constants.h"
#include "Video/socket_p.h"
#include "Video/tag_reader.h"

// Navdata structs
#include "navdata_common.h"

// Libraries for networking and communciation
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <csignal>

// ffmpeg library for video decoding
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

// Include the OpenMP
#include "omp.h"

// TODO: Consider using SDL for the UI

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// NCurses API for the keyboard input
#include <curses.h>

// DEBUG libraries
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
	//signal(SIGPIPE, SIG_IGN); // Prevents termination if socket is down

	// Create the Video handler
	cout << "Creating ARDrone2Video Object\n";
	ARDrone2Video *m_video = new ARDrone2Video();
	Address video_address("192.168.1.1", 5555);	// Address to the drone's video
	// Connect the handler to the drone
	m_video->start(video_address);

	// Check if video is connected
	// If not, continue program without video
	if (m_video->isStarted()) cout << "Successful in connecting to drone\n";

	// Init any image processing 
	// TODO: handle the bottom camera
	Mat p = Mat(360, 640, CV_8UC3);	// Mat to store video frame

	// Init TagReader object
	TagReader m_tagreader;
	cout << "Created TagReader\n";
	// Init TagData
	TagData data;
	
	// Init the Navdata and command sockets
	init_ports();

	navdata_t* navdata = 0;

	char command[256];
	int seq = 0;

	int32_t one = 1;
	cout << navdata_socket << endl;
	// set unicast mode on
	sendto(navdata_socket, &one, 4, 0, (sockaddr *) &drone_nav, sizeof(drone_nav));
	// Handle image processing here

	for(;;) {
		m_video->fetch();			// Decode the frame
		m_video->latestImage(p);	// Store frame into the Mat
		// Do image processing
		m_tagreader.process_Mat(p, data);
		cout << "Tag ID: " << data.id << endl;
		
		// Handle control using video and navdata
		if (drone_control()) break;

		//sendto(at_socket, command, strlen(command), 0, 
		//		(struct sockaddr*)&drone_at, sizeof(drone_at));
		//seq++;
		//cout << seq << endl;

		// Get navdata
		get_navdata(&navdata);

		// Output navdata
		cout << "header " << navdata->header << endl
			 << "Battery " << ((navdata_demo_t*)(navdata->options))->vbat_flying_percentage << endl
			 << "Alt " << ((navdata_demo_t*)(navdata->options))->altitude << endl
			 << "Vx " << ((navdata_demo_t*)(navdata->options))->velocity._0 << endl
			 << "Vy " << ((navdata_demo_t*)(navdata->options))->velocity._1 << endl
			 << "Vz " << ((navdata_demo_t*)(navdata->options))->velocity._2 << endl
			 << "Pitch " << ((navdata_demo_t*)(navdata->options))->pitch << endl
			 << "Roll " << ((navdata_demo_t*)(navdata->options))->roll << endl
			 << "Yaw " << ((navdata_demo_t*)(navdata->options))->yaw << endl;

		// show the frame
		imshow("Camera", p);
		if (waitKey(1) == 27) break;
	}

	// close the sockets
	close_ports();

	// Halt the video handler
	m_video->end();
	delete m_video;

	return 0;
}
