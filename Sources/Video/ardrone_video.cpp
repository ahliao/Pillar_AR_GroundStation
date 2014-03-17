// Based on ardrone_video_p.cpp
// From https://github.com/kipr/
// Shortened to include only support for AR Drone 2.0

#include "ardrone_video.h"
#include <iostream>

bool ARDrone2Video::s_init = false;

ARDrone2Video::~ARDrone2Video()
{
}


ARDrone2Video::ARDrone2Video() :
	m_codecCtx(0),
	m_frame(0),
	m_frameBgr(0),
	m_bufferBgr(0),
	lastFrame(0),
	m_state(WaitForIFrame)	// starting state is to wait for IFrame
{
	initAV();
}

// open the socket to the address and begin decoding
// TODO: multithreading?
bool ARDrone2Video::start(const Address &address)
{
	m_address = address;
	// Setup the socket
	if (!setupSocket(m_socket)) {
		std::cerr << "Failed to setup video stream socket." << std::endl;
		return false;
	}

	// connect the socket to the address
	if (!m_socket.connect(m_address)) {
	//if (!m_socket.bind(5555)) {
		std::cerr << "Failed to connect to video stream." << std::endl;
		perror("connect");
		m_socket.close();
		return false;
	}
	// make it so the socket doesn't block (doesn't freeze program when wait)
	m_socket.setBlocking(false);

	// const pointer to a codec
	AVCodec *const codec = avcodec_find_decoder(CODEC_ID_H264);
	if (!codec) {
		std::cerr << "avcodec_find_decoder() failed." << std::endl;
		return false;
	}

	// Find the video decoder
	m_codecCtx = avcodec_alloc_context3(codec);
	m_codecCtx->pix_fmt = PIX_FMT_YUV420P;
	m_codecCtx->skip_frame = AVDISCARD_DEFAULT;
	m_codecCtx->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
	m_codecCtx->err_recognition = FF_ER_CAREFUL;
	m_codecCtx->skip_loop_filter = AVDISCARD_DEFAULT;
	m_codecCtx->workaround_bugs = FF_BUG_AUTODETECT;
	m_codecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
	m_codecCtx->codec_id = CODEC_ID_H264;
	m_codecCtx->skip_idct = AVDISCARD_DEFAULT;
	m_codecCtx->width = 640;
	m_codecCtx->height = 360;

	// Open the video stream
	if (avcodec_open2(m_codecCtx, codec, NULL) < 0) {
		std::cerr << "avcodec_open2() failed." << std::endl;
		return false;
	}

	// Allocate video buffers
	m_frame = avcodec_alloc_frame();
	m_frameBgr = avcodec_alloc_frame();
	m_bufferBgr = (uint8_t *) av_mallocz(avpicture_get_size(PIX_FMT_BGR24,
				m_codecCtx->width, m_codecCtx->height) * sizeof(uint8_t));

	// Assign the buffers to image planes in m_frameBgr
	avpicture_fill((AVPicture *) m_frameBgr, m_bufferBgr, PIX_FMT_BGR24,
			m_codecCtx->width, m_codecCtx->height);
	m_convertCtx = sws_getContext(m_codecCtx->width, m_codecCtx->height, 
			m_codecCtx->pix_fmt, m_codecCtx->width, m_codecCtx->height,
			PIX_FMT_BGR24, SWS_SPLINE, 0, 0, 0);

	// Allocate the OpenCV Mat
	m_img = cv::Mat(((m_codecCtx->height == 368) ? 360 : m_codecCtx->height),
			m_codecCtx->width, CV_8UC3);

	return true;
}

bool ARDrone2Video::isStarted() const
{
	return m_socket.isOpen();
}

// send wakeup packet to start video stream 
bool ARDrone2Video::wakeup()
{
	// if socket is not open, return
	if (!m_socket.isOpen()) {
		return false;
	}

	const static char startCode[4] = { 0x01, 0x00, 0x00, 0x00 };
	if (m_socket.sendto(startCode, sizeof(startCode), m_address) 
			!= sizeof(startCode)) {
		perror("video wakeup");
		return false;
	}
	return true;
}

bool ARDrone2Video::fetch()
{
	if(!isStarted()) return false;

	unsigned char part[40000];

	ssize_t readLength = 0;
	if((readLength = m_socket.recv(part, 
					sizeof(part))) < 0 && errno != EAGAIN) {
		perror("fetchVideo");
		return false;
	}

	if(readLength < 0) return true;

	parrot_video_encapsulation_t header;
	memcpy(&header, part, sizeof(parrot_video_encapsulation_t));
	// std::cout << header->signature << std::endl;
	if (strncmp((const char *)header.signature, "PaVE", 4)) {
		return true;
	}

	// printPave(header, std::cout);

	if (m_state == Normal && header.frame_number != lastFrame + 1) {
		m_state = WaitForIFrame;
		//std::cout << "FRAME MISSED (got " << header.frame_number 
		//	<< ", expected" << (lastFrame + 1) <<  ")" << std::endl;
		return true;
	} else if (m_state == WaitForIFrame && 
		(header.frame_type == FRAME_TYPE_IDR_FRAME
		|| header.frame_type == FRAME_TYPE_I_FRAME)) {
		//std::cout << "Got our I Frame" << std::endl;
		m_state = Normal;
	} else if(m_state == WaitForIFrame) {
		return true;
	}
	lastFrame = header.frame_number;

	size_t read = 0;

	// Is it bad to assume things? Yes. Is is *computationally fast* to assume things? Yes.
	unsigned char payload[40000];

	// Copy over part of the payload that was sent with the header
	memcpy(payload, part + header.header_size, 
			readLength - header.header_size);
	read += readLength - header.header_size;

	double lastRead = seconds();
	while(read < header.payload_size && seconds() - lastRead < 0.1) {
		//std::cout << read << " of " << header.payload_size << std::endl;
		if((readLength = m_socket.recv(payload + read, 
				header.payload_size - read)) < 0 && errno != EAGAIN) {
			perror("fetchVideo");
			return false;
		}
		if(readLength < 0) {
			msleep(10);
			continue;
		}
		read += readLength;
		lastRead = seconds();
	}
	
	//std::cout << "Read " << read << " bytes from video stream" << std::endl;

	adjustDimensions(header);

	// Start decoding the data
	AVPacket packet;
	packet.data = payload;
	packet.size = header.payload_size;

	int done = 0;

	if(avcodec_decode_video2(m_codecCtx, m_frame, &done, &packet) < 0) {
		//std::cout << "Didn't decode frame" << std::endl;
		return true;
	}

	if(!done) return true;
	
	sws_scale(m_convertCtx, (const uint8_t *const *)m_frame->data, 
		m_frame->linesize,
		0, m_codecCtx->height, m_frameBgr->data, m_frameBgr->linesize);

	// These are so nothing tries to write to the image buffer
	// while the new frame is being put in
	// TODO: add in multithreading support
	//m_mutex.lock();
	
	memcpy(m_img.ptr(), m_frameBgr->data[0], 
		m_codecCtx->width * ((m_codecCtx->height == 368)
		? 360 : m_codecCtx->height) * sizeof(uint8_t) * 3);

	//m_mutex.unlock();

	return true;
}

bool ARDrone2Video::end()
{
	if(!isStarted()) return false;

	if(m_frame) {
		av_free(m_frame);
		m_frame = 0;
	}

	if(m_frameBgr) {
		av_free(m_frameBgr);
		m_frameBgr = 0;
	}

	if(m_bufferBgr) {
		av_free(m_bufferBgr);
		m_bufferBgr = 0;
	}

	if(m_codecCtx) {
		avcodec_close(m_codecCtx);
		m_codecCtx = 0;
	}

	m_socket.close();
	m_address = Address();

	return true;
}

void ARDrone2Video::latestImage(cv::Mat &image) const
{
	//m_mutex.lock();
	image = m_img.clone();
	//m_mutex.unlock();
}


bool ARDrone2Video::setupSocket(Socket &socket) const
{
	socket.close(); // make sure socket is closed
	socket = Socket::tcp();	// open it for TCP
	
	// See if the socket opened
	bool success = true;
	if (success) success &= socket.setReusable(true);
	if (!success) {
		perror("setupSocket");
		socket.close();
	}

	return success;
}

// adjust the dimensions for AR Drone 2
void ARDrone2Video::adjustDimensions(const parrot_video_encapsulation_t &pave)
{
	// If already adjusted, return
	if (m_codecCtx->width == pave.display_width
			&& m_codecCtx->height == pave.display_height) return;

	m_codecCtx->width = pave.display_width;
	m_codecCtx->height = pave.display_height;

	m_img = cv::Mat(((m_codecCtx->height == 368) ? 360 : m_codecCtx->height),
			m_codecCtx->width, CV_8UC3);
	m_bufferBgr = (uint8_t *) av_realloc(m_bufferBgr, 
			m_img.rows * m_img.cols * m_img.elemSize() * sizeof(uint8_t));

	avpicture_fill((AVPicture *) m_frameBgr, m_bufferBgr, PIX_FMT_BGR24, 
			m_codecCtx->width, m_codecCtx->height);

	m_convertCtx = sws_getCachedContext(m_convertCtx, pave.display_width, 
			pave.display_height, m_codecCtx->pix_fmt, pave.display_width,
			pave.display_height, m_codecCtx->pix_fmt, SWS_FAST_BILINEAR, 
			0, 0, 0);
}

// Print out the Pave header to the ostream o
void ARDrone2Video::printPave(const parrot_video_encapsulation_t &pave, std::ostream &o) {
	using namespace std;
	o << "PaVE Dump: " << endl;
	o << "\tVersion: " << (uint32_t)pave.version << endl;
	o << "\tVideo Codec: " << (uint32_t)pave.video_codec << endl;
	o << "\tHeader Size: " << (uint32_t)pave.header_size << endl;
	o << "\tPayload Size: " << (uint32_t)pave.payload_size << endl;
	o << "\tEncoded Size: " << (uint32_t)pave.encoded_stream_width << ", "
		<< (uint32_t)pave.encoded_stream_height << endl;
	o << "\tDisplay Size: " << (uint32_t)pave.display_width
		<< (uint32_t)pave.display_height << endl;
	o << "\tFrame Number: " << pave.frame_number << endl;
	o << "\tTimestamp: " << (uint32_t)pave.timestamp << endl;
	o << "\tTotal Chunks: " << (uint32_t)pave.total_chunks << endl;
	o << "\tChunk Index: " << (uint32_t)pave.chunk_index << endl;
	o << "\tFrame Type: " << (uint32_t)pave.frame_type << endl;
	o << "\tControl: " << (uint32_t)pave.control << endl;
	o << "\tStream Byte Position: " << pave.stream_byte_position_lw << ", "
		<< pave.stream_byte_position_uw << endl;
	o << "\tStream ID: " << (uint32_t)pave.stream_id << endl;
	o << "\tTotal Slices: " << (uint32_t)pave.total_slices << endl;
	o << "\tSlice Index: " << (uint32_t)pave.slice_index << endl;
	o << "\tHeader2 Size: " << (uint32_t)pave.header2_size << endl;
	o << "\tHeader1 Size: " << (uint32_t)pave.header1_size << endl;
	o << "\tAdvertised Size: " << (uint32_t)pave.advertised_size << endl;
}
