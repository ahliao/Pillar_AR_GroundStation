// Controller for the AR Drone 2
// using UDP and AT commands
// for use by the Pillar Technologies MDP Team
// Created on: 3/2/2014

// Pillar headers
#include "drone_controller.h"

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

// STD headers
#include <vector>

// TODO: Consider using SDL for the UI

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// DEBUG libraries
#include <iostream>

// Timer Tests
#include <ctime>

// Multithreading with the C lib
#include <pthread.h>

using namespace std;
using namespace cv;
	
// Create the mutex to sync shared data
pthread_mutex_t mutex;

navdata_t* navdata = 0;
bool running = true;

DroneController m_controller;

// Thread to handle retrieving the navdata
void* get_navdata(void *)
{
	while (running) {
		pthread_mutex_lock(&mutex);
		m_controller.get_navdata(&navdata);
		pthread_mutex_unlock(&mutex);
	}
	pthread_exit((void*) 0);
}

int main()
{
	//signal(SIGPIPE, SIG_IGN); // Prevents termination if socket is down

	// Create the Drone Controller
	cout << "Creating DroneController Object...\n";
	m_controller.init_ports();

	// Create the Video handler
	cout << "Creating ARDrone2Video Object\n";
	ARDrone2Video m_video;
	Address video_address("192.168.1.1", 5555);	// Address to the drone's video
	// Connect the handler to the drone
	m_video.start(video_address);

	// Check if video is connected
	// If not, continue program without video
	if (m_video.isStarted()) cout << "Successful in connecting to drone\n";

	// Init any image processing 
	Mat p = Mat(360, 640, CV_8UC3);	// Mat to store video frame

	// Init TagReader object
	TagReader m_tagreader;
	cout << "Created TagReader\n";
	// Init TagData
	vector<TagData> tagdata;

	// Create the mutex to sync shared data
	pthread_mutex_init(&mutex, NULL);

	// Create the thread to retreive navdata
	pthread_t thread1;
	pthread_create(&thread1, NULL, &get_navdata, NULL);

	// TODO: make the control (or video) into another thread

	for(;;) {
		m_video.fetch();			// Decode the frame
		m_video.latestImage(p);	// Store frame into the Mat

		// Timing test
		timeval start, end;
		gettimeofday(&start, NULL);
		
		// Do image processing
		m_tagreader.process_Mat(p, tagdata);

		gettimeofday(&end, NULL);
		long delta = (end.tv_sec  - start.tv_sec) * 1000000u + 
				 end.tv_usec - start.tv_usec;
		cout << "Tag detect time: " << delta << " ms" << endl;

		for (vector<TagData>::iterator it = tagdata.begin(); it != tagdata.end(); ++it)
			cout << "Tag ID: " << it->id << endl;
		
		// Handle control using tagdata and navdata
		m_controller.control_loop(navdata, tagdata);

		// Output navdata
		if (navdata != 0) {
		cout << "header " << navdata->header << endl
			 << "Battery " << ((navdata_demo_t*)(navdata->options))->vbat_flying_percentage << endl
			 << "Alt " << ((navdata_demo_t*)(navdata->options))->altitude << endl
			 << "Vx " << ((navdata_demo_t*)(navdata->options))->velocity._0 << endl
			 << "Vy " << ((navdata_demo_t*)(navdata->options))->velocity._1 << endl
			 << "Vz " << ((navdata_demo_t*)(navdata->options))->velocity._2 << endl
			 << "Pitch " << ((navdata_demo_t*)(navdata->options))->pitch << endl
			 << "Roll " << ((navdata_demo_t*)(navdata->options))->roll << endl
			 << "Yaw " << ((navdata_demo_t*)(navdata->options))->yaw << endl;
		}

		// show the frame
		// TODO: Probably switch to SDL or other
		if(p.size().width > 0 && p.size().height > 0) imshow("Camera", p);
		else cerr << "ERROR: Mat is not valid\n";
		if (waitKey(1) == 27) break;
	}
	running = false;
	pthread_join(thread1, NULL);
	pthread_mutex_destroy(&mutex);

	// close the sockets
	m_controller.close_ports();

	// Halt the video handler
	m_video.end();

	return 0;
}
