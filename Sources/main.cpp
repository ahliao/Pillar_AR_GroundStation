// Controller for the AR Drone 2
// using UDP and AT commands
// for use by the Pillar Technologies MDP Team
// Created on: 3/2/2014

// Pillar headers
#include "keyboard_controller.h"
#include "drone_init.h"

// Headers from the Parrot API
#include "navdata_common.h"

// Libraries for networking and communciation
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

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

// TODO: Consider using SDL
// NCurses API for the keyboard input
#include <curses.h>

//char navmsg[NAVDATA_BUFFER_SIZE];

using namespace std;

int main()
{
	// Init Curses
	/*initscr();	// curses call to init window
	cbreak();			// config so no waiting for Enter
	noecho();			// config for no echo of input
	clear();

	int32_t one = 1, zero = 0;

	// NCurses variables
	int term_row, term_col;
	getmaxyx(stdscr, term_row, term_col);

	mvprintw(0,0,"AR Drone 2 Controller v0.02\n");*/

	// register the codecs and file formats
	av_register_all();
	avcodec_register_all();
	avformat_network_init();

	// Open video file
	//mvprintw(1,1,"WORKED");
	//refresh();
	const char		*drone_addr = "http://192.168.1.1:5555";
	AVFormatContext *pFormatCtx = NULL;
	while (avformat_open_input(&pFormatCtx, drone_addr, NULL, NULL) != 0) {
		//mvprintw(3,0,"Could not open the video file...Retrying...");
		cout << "Could not open video file\n";
		//refresh();
	}
	//mvprintw(2,1,"WORKED");
	//refresh();

	avformat_find_stream_info(pFormatCtx, NULL);	// Get stream info
	av_dump_format(pFormatCtx, 0, drone_addr, 0);	// dump to std output

	// Find decoder for the stream
	AVCodecContext	*pCodecCtx;
	AVCodec			*pCodec;
	pCodecCtx	= pFormatCtx->streams[0]->codec;
	pCodec		= avcodec_find_decoder(pCodecCtx->codec_id);

	avcodec_open2(pCodecCtx, pCodec, NULL);	// open codec 

	/*if(init_ports()) 
	{
		mvprintw(0,0, "Error initializing the ports");
		exit(1);
	}

	if (bind(navdata_socket, (struct sockaddr *) &pc_addr, sizeof(pc_addr)) < 0) {
		mvprintw(1,0,"Error binding navdata_socket to pc_addr");
		exit(1);
	}

	// set unicast mode on
	sendto(navdata_socket, &one, 4, 0, 
			(struct sockaddr *) &drone_nav, sizeof(drone_nav));*/
	
	// variables for getting video
	AVPacket packet;
	AVFrame *pFrame;
	pFrame = avcodec_alloc_frame();
	int attemptNum, frameDecoded;

	// Converting the frame
	AVFrame *pFrame_BGR24;
	uint8_t *buffer_BGR24;
	struct SwsContext *pConvertCtx_BGR24;

	// Allocate AVFrame 
	pFrame_BGR24 = avcodec_alloc_frame();
	if (pFrame_BGR24 == NULL) {
		mvprintw(12,0,"Could not allocate pFrame_BGR24");
		return 1;
	}
	// find image size and allocate buffer
	buffer_BGR24 = (uint8_t *) av_malloc(avpicture_get_size(PIX_FMT_BGR24,
				pCodecCtx->width, pCodecCtx->height));
	// Assign buffer to image planes
	avpicture_fill((AVPicture *) pFrame_BGR24, buffer_BGR24,
			PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);
	// format conversion context
	pConvertCtx_BGR24 = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt,
			pCodecCtx->width, pCodecCtx->height, PIX_FMT_BGR24,
			SWS_SPLINE, NULL, NULL, NULL);


	// loop for controlling the drone
	int i = 0;
	while(1) {
		//if (drone_control()) break;

		// read the navdata received
		//if (get_navdata(i++)) break;

		// Read and convert the video frames
		//mvprintw(12,0,"Frame Read: %d", attemptNum);
		// read frame
		if(av_read_frame(pFormatCtx, &packet) < 0){
			//mvprintw(13,0,"Could not read frame");
			continue;
		}

		// Decode the frame
		if (avcodec_decode_video2(pCodecCtx, pFrame, &frameDecoded, &packet) < 0) {
			//mvprintw(13,0,"Could no decode frame");
			continue;
		}

		// convert the frame to BGR
		if (frameDecoded) {
			sws_scale(pConvertCtx_BGR24, pFrame->data, pFrame->linesize, 0,
					pCodecCtx->height, pFrame_BGR24->data, pFrame_BGR24->linesize);
			attemptNum++;
		}
		//refresh();
	}

	// Release the frames
	av_free(pFrame);
	av_free(pFrame_BGR24);
	av_free(buffer_BGR24);
	sws_freeContext(pConvertCtx_BGR24);
	avcodec_close(pCodecCtx);
	avformat_close_input(&pFormatCtx);

	// TODO closing program stuff (move to separate function)
	//endwin();	// return the terminal to the system

	// close the sockets
	//close(at_socket);
	//close(navdata_socket);

	return 0;
}
