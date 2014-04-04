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
#include <boost/date_time/posix_time/posix_time.hpp>

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
void handle_control(bool *running, DroneController *m_controller, 
		navdata_t* navdata, vector<TagData> *tagdata, 
		bool *cmdsent, timeval *currtime, float *cmdduration);

void redraw(bool* running);
void initGUI();
void drawFeedback(const char str[]);

int main()
{
	//freopen("/dev/null", "r", stderr); 

	DroneController m_controller;
	Mat p;	// Mat to store video frame  (Takes up a ton of memory)
	ARDrone2Video m_video;
	TagReader m_tagreader;
	vector<TagData> tagdata;
	navdata_t* navdata = 0;

	// Time that a command was sent
	timeval timecmdsent;
	// true if a command was send and still needs to be finished
	bool cmdsent = false;
	// duration the command sent should be sent
	float cmdduration = 0.0f;

	bool* running = new bool(true);

	// ofstream to write the txt file for tagdata
	ofstream tagfile;
	int count = 0;

	// GUI init
	initGUI();
	
	// Create the Drone Controller
	drawFeedback("Creating DroneController object...");
	m_controller.init_ports();
	if (!m_controller.is_connected()) return 1;

	// Create the Video handler
	drawFeedback("Creating ARDrone2Video object...");
	Address video_address("192.168.1.1", 5555);	// Address to the drone's video
	// Connect the handler to the drone
	m_video.start(video_address);

	// Check if video is connected
	// If not, continue program without video
	if (m_video.isStarted()) drawFeedback("Successful in connecting to drone");
	else { drawFeedback("ERROR: Cout not connect to drone"); return 1; }

	// Init any image processing 
	p = Mat(360, 640, CV_8UC3, Scalar(0,200,0));	// Mat to store video frame

	// Create the thread to retreive navdata
	drawFeedback("Starting threads...");
	boost::thread navThread(get_navdata, running, &m_controller, &navdata);
	boost::thread vidThread(get_video, running, &m_video, &p, &m_tagreader, &tagdata);
	boost::thread controlThread(handle_control, running, &m_controller, navdata,
		   &tagdata, &cmdsent, &timecmdsent, &cmdduration);
	boost::thread drawThread(redraw, running);

	// TODO: make the control (or video) into another thread
	// TODO: Found mem leak in zarray.c 23 in calloc
	char str[80];
	char command[10];
	char direction[10];
	float value = 0, duration = 0;
	int numargs = 0;
	timespec tim1, tim2;
	while(*running) {
		// Handle control using tagdata and navdata
		// TODO: Move to another thread
		wmove(inputwin, 1, 4);
		wgetstr(inputwin, str);
		numargs = sscanf(str, "%s %s %f %f", command, direction, &value, &duration);
		if (numargs == 1) {
			if (!strcmp(command, "exit")) *running = false;
			else if (!strcmp(command, "led")) m_controller.control_led(1, 2.0, 2);
			else if (!strcmp(command, "takeoff")) {
				m_controller.control_ftrim();
				m_controller.control_basic(TAKEOFF);
			} else if (!strcmp(command, "land")) {
				m_controller.control_basic(LAND);
			} else if (!strcmp(command, "help")) {
				m_controller.control_basic(EMERGENCY);
			} else if (!strcmp(command, "down")) {
				m_controller.control_move(true, 0, 0, -0.1f, 0);
			} else if (!strcmp(command, "hover")) {
				m_controller.control_move(false, 0, 0, 0, 0);
			} else if (!strcmp(command, "forward")) {
				m_controller.control_basic(FORWARD);
			} else if (!strcmp(command, "down")) {
				m_controller.control_basic(DOWN);
			} else if (!strcmp(command, "backward")) {
				m_controller.control_basic(BACKWARD);
			} else if (!strcmp(command, "up")) {
				m_controller.control_basic(UP);
			} else if (!strcmp(command, "left")) {
				m_controller.control_basic(LEFT);
			} else if (!strcmp(command, "right")) {
				m_controller.control_basic(RIGHT);
			}
		} else if (numargs == 4) {
			// TODO: add to a queue?
			if (!strcmp(command, "move") && !cmdsent) {
				drawFeedback("move...");
				if (!strcmp(direction, "roll")) {
					drawFeedback("move roll ...");
					m_controller.control_move(true, value, 0, 0, 0);
					cmdsent = true;
					gettimeofday(&timecmdsent, NULL);
					cmdduration = duration;
					//boost::this_thread::sleep(boost::posix_time::microseconds(duration * 1000000));
					//m_controller.control_move(false, 0, 0, 0, 0);
				} else if (!strcmp(direction, "pitch")) {
					m_controller.control_move(true, 0, value, 0, 0);
					cmdsent = true;
					gettimeofday(&timecmdsent, NULL);
					cmdduration = duration;
				} else if (!strcmp(direction, "alt")) {
					m_controller.control_move(true, 0, 0, value, 0);
					cmdsent = true;
					gettimeofday(&timecmdsent, NULL);
					cmdduration = duration;
				} else if (!strcmp(direction, "yaw")) {
					m_controller.control_move(true, 0, 0, 0, value);
					cmdsent = true;
					gettimeofday(&timecmdsent, NULL);
					cmdduration = duration;
				}
			}
		}
		
		werase(inputwin);
		box(inputwin, 0, 0);
		mvwprintw(inputwin, 1, 1, ">>");

		//if (input == 27) break;
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

	// Stop all the threads
	drawFeedback("Halting threads");
	*running = false;
	navThread.join();
	controlThread.join();
	drawThread.join();
	vidThread.join();

	// close the sockets
	drawFeedback("Closing ports");
	m_controller.close_ports();

	// Halt the video handler
	drawFeedback("Halting video processor");
	m_video.end();
	
	// Memory management
	drawFeedback("Cleaning up");
	delete running;

	// Clean up the GUI
	drawFeedback("Cleaning up GUI");
	delwin(mainwin);
	delwin(headerwin);
	delwin(navwin);
	delwin(tagwin);
	delwin(mapwin);
	delwin(feedbackwin);
	delwin(inputwin);
	endwin();

	return 0;
}
	
void initGUI()
{
	feedbackRow = 1;
	int row, col;
	int resized = system("resize -s 40 120");
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
}

// Thread to handle retrieving the navdata
void get_navdata(bool* running, DroneController *m_controller, navdata_t** navdata)
{
	while (*running) {
		m_controller->get_navdata(navdata);
		navdata_demo_t* nav_demo = (navdata_demo_t*)((*navdata)->options);
		werase(navwin);
		mvwprintw(navwin, 1, 1, "Navigation Data");
		mvwprintw(navwin, 2, 1, "Header %d", (*navdata)->header);
		mvwprintw(navwin, 3, 1, "Battery: %d", nav_demo->vbat_flying_percentage);
		mvwprintw(navwin, 4, 1, "State: %d", nav_demo->ctrl_state);
		mvwprintw(navwin, 5, 1, "Altitude: %d", nav_demo->altitude);
		mvwprintw(navwin, 6, 1, "Vx: %f", nav_demo->velocity._0);
		mvwprintw(navwin, 7, 1, "Vy: %f", nav_demo->velocity._1);
		mvwprintw(navwin, 8, 1, "Vz: %f", nav_demo->velocity._2);
		mvwprintw(navwin, 9, 1, "Pitch: %f", nav_demo->pitch);
		mvwprintw(navwin, 10, 1, "Roll:  %f", nav_demo->roll);
		mvwprintw(navwin, 11, 1, "Yaw:   %f", nav_demo->yaw);
	}
}

void redraw(bool* running)
{
	while (*running) {
		box(navwin, 0, 0);
		wrefresh(navwin);
		wrefresh(feedbackwin);
		wrefresh(tagwin);

		timespec tim1, tim2;
		tim1.tv_sec = 0;
		tim1.tv_nsec = 70000000;
		nanosleep(&tim1, &tim2);
	}
}

// Thread to handle getting the video processing
void get_video(bool *running, ARDrone2Video *m_video, Mat* p, 
		TagReader *m_tagreader, vector<TagData> *tagdata)
{
	while (*running) {
		m_video->fetch();			// Decode the frame
		m_video->latestImage(*p);	// Store frame into the Mat
		//m_tagreader->process_Mat(*p, tagdata, *p);	// Image processing

		// Display the tag data
		werase(tagwin);
		box(tagwin, 0, 0);
		mvwprintw(tagwin, 1, 1, "Tags found: %d", tagdata->size());
		for (uint16_t i = 0; i < tagdata->size(); ++i) {
			mvwprintw(tagwin, i+2, 1, "Tag: %d at (%d, %d)", tagdata->at(i).id,
					tagdata->at(i).img_x, tagdata->at(i).img_y);
		}

		if(p->size().width > 0 && p->size().height > 0) {
			imshow("Camera", *p);
			// TODO: save the images
			waitKey(1);
		}
		else {
			drawFeedback("ERROR: Mat is not valid");
		}
	}
}

// Thread to run the control loop
void handle_control(bool *running, DroneController *m_controller, 
		navdata_t* navdata, vector<TagData> *tagdata, 
		bool *cmdsent, timeval *timecmdsent, float *cmdduration)
{
	long int timediff = 0;
	timeval currtime;
	while (*running) {
		m_controller->control_loop(navdata, *tagdata);

		if (*cmdsent) {
			gettimeofday(&currtime, NULL);
			timediff = (currtime.tv_sec  - timecmdsent->tv_sec) * 1000000 + 
				(currtime.tv_usec - timecmdsent->tv_usec);
			if (timediff >= *cmdduration * 1000000) {
				//m_controller->control_move(false, 0, 0, 0, 0);
				m_controller->control_move(true, 0, 0, 0, 0);
				*cmdsent = false;
			}
		}
	}
}
