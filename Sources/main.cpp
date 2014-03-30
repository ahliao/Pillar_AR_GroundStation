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

// Ncurses for Terminal GUI
#include <ncurses.h>

// DEBUG libraries
#include <iostream>
#include <fstream>

// Timer Tests
#include <ctime>

// Multithreading with the C lib
#include <boost/thread.hpp>

// GUI Variables
// Pointers to the NCurses windows
WINDOW *mainwin;
WINDOW *navwin, *tagwin, *mapwin, *headerwin, *inputwin, *feedbackwin;
int feedbackRow;

using namespace std;
using namespace cv;

void get_navdata(bool* running, DroneController *m_controller, navdata_t** navdata);
void get_video(bool *running, ARDrone2Video *m_video, Mat* p, 
		TagReader *m_tagreader, vector<TagData> *tagdata);

void initGUI();
void drawFeedback(const char str[]);

int main()
{
	DroneController m_controller;
	Mat p;	// Mat to store video frame  (Takes up a ton of memory)
	ARDrone2Video m_video;
	TagReader m_tagreader;
	vector<TagData> tagdata;
	navdata_t* navdata = 0;

	bool* running = new bool(true);

	// ofstream to write the txt file for tagdata
	ofstream tagfile;
	int count = 0;

	// GUI init
	initGUI();

	char str[80];	// Buffer for user input
	
	// Create the Drone Controller
	//cout << "Creating DroneController Object...\n";
	drawFeedback("Creating DroneController object...");
	m_controller.init_ports();
	if (!m_controller.is_connected()) return 1;

	// Create the Video handler
	//cout << "Creating ARDrone2Video Object\n";
	drawFeedback("Creating ARDrone2Video object...");
	Address video_address("192.168.1.1", 5555);	// Address to the drone's video
	// Connect the handler to the drone
	m_video.start(video_address);

	// Check if video is connected
	// If not, continue program without video
	if (m_video.isStarted()) drawFeedback("Successful in connecting to drone");
	else { drawFeedback("ERROR: Cout not connect to drone"); return 1; }
	//if (m_video.isStarted()) cout << "Successful in connecting to drone\n";
	//else { cerr << "ERROR: Could not connect to drone\n"; return 1; }

	// Init any image processing 
	p = Mat(360, 640, CV_8UC3, Scalar(0,200,0));	// Mat to store video frame

	// Init TagReader object
	//cout << "Created TagReader\n";

	// Create the thread to retreive navdata
	//boost::thread navThread(get_navdata, running, &m_controller, &navdata);
	//boost::thread vidThread(get_video, running, &m_video, &p, &m_tagreader, &tagdata);

	// TODO: make the control (or video) into another thread
	// TODO: Found mem leak in zarray.c 23 in calloc
	for(;;) {
		// Handle control using tagdata and navdata
		if (m_controller.control_loop(navdata, tagdata)) break;

		// Output navdata
		/*if (navdata != 0) {
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
		}*/

		// show the frame
		// TODO: Probably switch to SDL or other
		if(p.size().width > 0 && p.size().height > 0) imshow("Camera", p);
		else {
			drawFeedback("ERROR: Mat is not valid");
			break;
		}
		char input = (char) waitKey(1);
		if (input == 27) break;
		/*else if (input == 'p') {
			char filename[64];
			char tagfilename[64];
			sprintf(filename, "image%d.jpg", count);
			sprintf(tagfilename, "image%d.txt", count);
			cout << "Took a picture";
			imwrite(filename, p);
			tagfile.open(tagfilename);
			for (uint8_t i = 0; i < tagdata.size(); ++i)
				tagfile << tagdata[i].id << " " << 
					tagdata[i].img_x << " " << tagdata[i].img_y << endl;
			tagfile.close();
			count++;
		}*/
	}
	m_controller.control_basic(LAND);

	//cout << "Halting threads\n";
	drawFeedback("Halting threads");
	*running = false;
	//navThread.join();
	//vidThread.join();

	// close the sockets
	//cout << "Closing ports\n";
	drawFeedback("Closing ports");
	m_controller.close_ports();

	// Halt the video handler
	drawFeedback("Halting video processor");
	m_video.end();

	// Memory management
	drawFeedback("Cleaning up");
	delete running;

	return 0;
}
	
void initGUI()
{
	feedbackRow = 1;
	int row, col;
	system("resize -s 40 120");
	initscr();
	cbreak();
	curs_set(0);
	refresh();

	mainwin = newwin(LINES, COLS, 0, 0);
	navwin = newwin(22, 80, 0, 0); 
	headerwin = newwin(4, 35, 0, 80);
	tagwin = newwin(10, 35, 4, 80); 
	mapwin = newwin(16, 35, 14, 80); 
	feedbackwin = newwin(8, 80, 22, 0); 
	inputwin = newwin(3, 115, 30, 0); 
	wclear(mainwin);
	wclear(navwin);
	wclear(headerwin);
	wclear(tagwin);
	wclear(mapwin);
	wclear(feedbackwin);
	wclear(inputwin);
	box(navwin, 0, 0);
	box(headerwin, 0, 0);
	box(tagwin, 0, 0);
	box(mapwin, 0, 0);
	box(feedbackwin, 0, 0);
	box(inputwin, 0, 0);
	wrefresh(mainwin);

	mvwprintw(navwin, 1, 1, "Navigation Data");
	wrefresh(navwin);

	mvwprintw(headerwin, 1, 1, "Pillar Technologies MDP 2014");
	mvwprintw(headerwin, 2, 1, "AR Drone 2 Ground Station v0.6");
	mvwprintw(headerwin, 3, 1, "");
	getmaxyx(headerwin, row, col);
	wrefresh(headerwin);

	mvwprintw(tagwin, 1, 1, "Tag Data");
	wrefresh(tagwin);

	mvwprintw(mapwin, 1, 1, "Local Map");
	wrefresh(mapwin);

	wrefresh(feedbackwin);
	mvwprintw(inputwin, 1, 1, ">>");
	wrefresh(inputwin);
}

void drawFeedback(const char str[])
{
	// TODO: scroll upwards
	char feedback[80];
	sprintf(feedback, "$ %s", str);
	mvwprintw(feedbackwin, feedbackRow, 1, feedback);
	if (feedbackRow <= 5) feedbackRow += 1;
	else feedbackRow = 1;
	wrefresh(feedbackwin);
}

// Thread to handle retrieving the navdata
void get_navdata(bool* running, DroneController *m_controller, navdata_t** navdata)
{
	while (*running) {
		m_controller->get_navdata(navdata);
		/*cout << "header " << (*navdata)->header << endl
			 << "Pitch " << ((navdata_demo_t*)((*navdata)->options))->pitch << endl
			 << "Roll " << ((navdata_demo_t*)((*navdata)->options))->roll << endl
			 << "Yaw " << ((navdata_demo_t*)((*navdata)->options))->yaw << endl;*/
		//mvwprintw(navwin, 2, 1, "Header %d", (*navdata)->header);
	}
}

// Thread to handle getting the video processing
void get_video(bool *running, ARDrone2Video *m_video, Mat* p, 
		TagReader *m_tagreader, vector<TagData> *tagdata)
{
	while (*running) {
		m_video->fetch();			// Decode the frame
		m_video->latestImage(*p);	// Store frame into the Mat
		m_tagreader->process_Mat(*p, *tagdata);	// Image processing
	}
}
