#include <iostream>

#include "ardrone_video.h"
#include "ardrone_constants.h"
#include "socket_p.h"

#include <csignal>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

//#include <pthread.h>

using namespace std;
using namespace cv;

int main() 
{
	signal(SIGPIPE, SIG_IGN);

	// Threads
	//pthread_t video_thread;

	//int iret1;

	// Init the AR Drone 2 Video Class
	cout << "Creating ARDrone2Video\n";
	ARDrone2Video *m_video = new ARDrone2Video();

	Address video_address("192.168.1.1", 5555);

	cout << "Call start()\n";
	m_video->start(video_address);

	if (m_video->isStarted()) cout << "Started...\n";

	cout << "Wakeup\n";
	//m_video->wakeup();

	Mat p = Mat(360, 640, CV_8UC3);

	//iret1 = pthread_create( &video_thread, NULL, &ARDrone2Video::fetch, m_video);

	// Try getting images
	cout << "Fetch\n";
	while(1) {
		m_video->fetch();
		m_video->latestImage(p);

		// TODO: multithreading for display and getting video
		imshow("Test", p);
		if(waitKey(1) == 27) break;
	}

	//pthread_join(video_thread, NULL);

	cout << "Call end()\n";
	m_video->end();

	delete m_video;

	return 0;
}
