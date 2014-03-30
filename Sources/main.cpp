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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// DEBUG libraries
#include <iostream>
#include <fstream>

// Timer Tests
#include <ctime>

// Multithreading with the C lib
#include <boost/thread.hpp>
#include <pthread.h>

using namespace std;
using namespace cv;
	
// Create the mutex to sync shared data
pthread_mutex_t mutex;

navdata_t* navdata = 0;
volatile bool running = true;

DroneController m_controller;
Mat p;	// Mat to store video frame  (Takes up a ton of memory)
ARDrone2Video m_video;
TagReader m_tagreader;
// Init TagData
vector<TagData> tagdata;

// Move variables
int roll_slider = 100;
int ROLL_MAX = 200;
int pitch_slider = 100;
int PITCH_MAX = 200;
int vz_slider = 100;
int VZ_MAX = 200;
int rot_slider = 100;
int ROT_MAX = 200;

// DEBUG
bool VIDEO_CAP = true;

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

void* get_video(void *)
{
	while (running) {
		pthread_mutex_lock(&mutex);
//		m_video.fetch();			// Decode the frame
//		m_video.latestImage(p);	// Store frame into the Mat
		m_tagreader.process_Mat(p, tagdata);	// Image processing
		pthread_mutex_unlock(&mutex);
	}
	pthread_exit((void*) 0);
}

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}

int main()
{
	//signal(SIGPIPE, SIG_IGN); // Prevents termination if socket is down
	
	// ofstream to write the txt file for tagdata
	ofstream tagfile;
	int count = 0;

	// Create the Drone Controller
	cout << "Creating DroneController Object...\n";
	m_controller.init_ports();
	if (!m_controller.is_connected()) return 1;

	// Create the Video handler
	cout << "Creating ARDrone2Video Object\n";
	Address video_address("192.168.1.1", 5555);	// Address to the drone's video
	// Connect the handler to the drone
	m_video.start(video_address);

	// Check if video is connected
	// If not, continue program without video
	if (m_video.isStarted()) cout << "Successful in connecting to drone\n";
	else { cerr << "ERROR: Could not connect to drone\n"; return 1; }

	// Init any image processing 
	p = Mat(360, 640, CV_8UC3, Scalar(0,200,0));	// Mat to store video frame

	// Init TagReader object
	cout << "Created TagReader\n";

	// Create the mutex to sync shared data
	pthread_mutex_init(&mutex, NULL);

	// Create the thread to retreive navdata
	pthread_t thread1;
	pthread_create(&thread1, NULL, &get_navdata, NULL);
	//pthread_t thread2;
	//pthread_create(&thread2, NULL, &get_video, NULL);

	// Taking a picture variables
	timeval picture_time, current_time;
	gettimeofday(&picture_time, NULL);

	// OpenCV GUI stuff
	namedWindow("GUI", WINDOW_AUTOSIZE);
	/*char TrackbarName[50];
	sprintf( TrackbarName, "Roll %f", (float)(roll_slider-100.0)/100.0);
	createTrackbar( TrackbarName, "GUI", &roll_slider, 
			ROLL_MAX, on_trackbar);
	sprintf( TrackbarName, "Pitch %f", (float)(pitch_slider-100.0)/100.0);
	createTrackbar( TrackbarName, "GUI", &pitch_slider, 
			PITCH_MAX, on_trackbar);
	sprintf( TrackbarName, "VZ %f", (float)(vz_slider-100.0)/100.0);
	createTrackbar( TrackbarName, "GUI", &vz_slider, 
			VZ_MAX, on_trackbar);
	sprintf( TrackbarName, "Rot %f", (float)(rot_slider-100.0)/100.0);
	createTrackbar( TrackbarName, "GUI", &rot_slider, 
			ROT_MAX, on_trackbar);*/

	// TODO: make the control (or video) into another thread
	// TODO: Found mem leak in zarray.c 23 in calloc
	for(;;) {
		m_video.fetch();			// Decode the frame
		m_video.latestImage(p);	// Store frame into the Mat

		// Timing test
		timeval start, end;
		gettimeofday(&start, NULL);
		
		// Do image processing
		m_tagreader.process_Mat(p, tagdata,p);	// Image processing

		gettimeofday(&end, NULL);
		long delta = (end.tv_sec  - start.tv_sec) * 1000000u + 
				 end.tv_usec - start.tv_usec;
		cout << "Tag detect time: " << delta << " ms" << endl;

		// Handle control using tagdata and navdata
		if (m_controller.control_loop(navdata, tagdata)) break;

		// Output navdata
		if (navdata != 0) {
		cout << "header " << navdata->header << endl
			 << "Battery " << ((navdata_demo_t*)(navdata->options))->vbat_flying_percentage << endl
			 << "State " << ((navdata_demo_t*)(navdata->options))->ctrl_state << endl
			 << "Alt " << ((navdata_demo_t*)(navdata->options))->altitude << endl
			 << "Vx " << ((navdata_demo_t*)(navdata->options))->velocity._0 << endl
			 << "Vy " << ((navdata_demo_t*)(navdata->options))->velocity._1 << endl
			 << "Vz " << ((navdata_demo_t*)(navdata->options))->velocity._2 << endl
			 << "Pitch " << ((navdata_demo_t*)(navdata->options))->pitch << endl
			 << "Roll " << ((navdata_demo_t*)(navdata->options))->roll << endl
			 << "Yaw " << ((navdata_demo_t*)(navdata->options))->yaw << endl;
		if (!navdata) cout << "STATE is Landed\n";
		else if (navdata->ardrone_state & (1U << 0))
			cout << "State is flying\n";
		else cout << "State is unknown\n";
		}

		gettimeofday(&current_time, NULL);
		delta = (current_time.tv_sec  - picture_time.tv_sec) * 1u;
		if (delta > 1 && VIDEO_CAP) {
			// take a picture
			char filename[64];
			char tagfilename[64];
			sprintf(filename, "image%d.tif", count);
			sprintf(tagfilename, "image%d.txt", count);
			cout << "Took a picture";
			imwrite(filename, p);
			tagfile.open(tagfilename);
			for (uint8_t i = 0; i < tagdata.size(); ++i)
				tagfile << tagdata[i].id << " " << 
					tagdata[i].img_x << " " << tagdata[i].img_y << endl;
			tagfile.close();
			count++;
			gettimeofday(&picture_time, NULL);
		}

		// show the frame
		// TODO: Probably switch to SDL or other
		if(p.size().width > 0 && p.size().height > 0) {
			imshow("GUI", p);
		} else {
			cerr << "ERROR: Mat is not valid\n";
			break;
		}
		char input = (char) waitKey(1);
		if (input == 27) {
			m_controller.control_basic(LAND);
			break;
		}
		else if (input == 'p') {
			char filename[64];
			char tagfilename[64];
			sprintf(filename, "image%d.tif", count);
			sprintf(tagfilename, "image%d.txt", count);
			//sprintf(filename, "image.jpg");
			cout << "Took a picture";
			imwrite(filename, p);
			tagfile.open(tagfilename);
			for (uint8_t i = 0; i < tagdata.size(); ++i)
				tagfile << tagdata[i].id << " " << 
					tagdata[i].img_x << " " << tagdata[i].img_y << endl;
			tagfile.close();
			count++;
		/*} else if (input == ' ') {
			m_controller.control_basic(TAKEOFF);
		} else if (input == 'q') {
			m_controller.control_basic(LAND);
		} else if (input == 'n') {
			m_controller.control_move(true, (float)roll_slider/100.0f - 1, 
					(float)pitch_slider/100.0f - 1,
					(float)vz_slider/100.0f - 1, 
					(float)rot_slider/100.0f - 1);*/
		}
	}

	cout << "Halting threads\n";
	running = false;
	pthread_join(thread1, NULL);
	//pthread_join(thread2, NULL);
	pthread_mutex_destroy(&mutex);

	// close the sockets
	cout << "Closing ports\n";
	m_controller.close_ports();

	// Halt the video handler
	m_video.end();

	return 0;
}
