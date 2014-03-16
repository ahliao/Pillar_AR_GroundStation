#ifndef _ARDRONE_VIDEO_P_HPP_
#define _ARDRONE_VIDEO_P_HPP_

#include "ardrone_video.h"
#include "ardrone_constants.h"
#include "util.h"

// Workaround for UINT64_CA
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
	#include <libavcodec/avcodec.h>
	#include <libavformat/avformat.h>
	#include <libswscale/swscale.h>
}

// OpenCV headers
#include <opencv2/opencv.hpp>
#include "socket_p.h"
#include <opencv2/core/core.hpp>

typedef struct {
	// PaVE header
	uint8_t signature[4];
	uint8_t version;
	uint8_t video_codec;
	uint16_t header_size;
	uint32_t payload_size;
	uint16_t encoded_stream_width;
	uint16_t encoded_stream_height;
	uint16_t display_width;
	uint16_t display_height;
	uint32_t frame_number;
	uint32_t timestamp;
	uint8_t total_chunks;
	uint8_t chunk_index;
	uint8_t frame_type;
	uint8_t control;
	uint32_t stream_byte_position_lw;
	uint32_t stream_byte_position_uw;
	uint16_t stream_id;
	uint8_t total_slices;
	uint8_t slice_index ;
	uint8_t header2_size;
	uint8_t header1_size;

	uint8_t reserved2[2];

	uint32_t advertised_size;

	// padding
	uint8_t reserved3[12];
	uint8_t reserved4[4];
} __attribute__ ((packed)) parrot_video_encapsulation_t;

// Frame types
typedef enum {
	FRAME_TYPE_UNKNOWN = 0,
	FRAME_TYPE_IDR_FRAME, // headers followed by I-Frame
	FRAME_TYPE_I_FRAME,	  // frame independent of other frames
	FRAME_TYPE_P_FRAME,	  // delta frame
	FRAME_TYPE_HEADERS
} parrot_video_encapsulation_frametypes_t;

class ARDrone2Video
{
	public:
		ARDrone2Video();
		~ARDrone2Video();
		bool start(const Address &address);
		bool isStarted() const;
		bool wakeup();
		bool fetch();
		bool end();
		void latestImage(cv::Mat &image) const;

	private:
		static void initAV()
		{
			if (s_init) return;
			av_register_all();
			avcodec_register_all();
			s_init = true;
		}

		bool setupSocket(Socket &socket) const;

		// adjust the dimensions for AR Drone 2
		void adjustDimensions(const parrot_video_encapsulation_t &pave);

		// Print out the Pave header to the ostream o
		void printPave(const parrot_video_encapsulation_t &pave, std::ostream &o);

		Socket m_socket;
		Address m_address;
		static bool s_init;
		AVCodecContext *m_codecCtx;
		AVFrame *m_frame;
		AVFrame *m_frameBgr;
		SwsContext *m_convertCtx;
		uint8_t *m_bufferBgr;
		cv::Mat m_img;
		uint32_t lastFrame;

		enum {
			Normal,
			WaitForIFrame
		} m_state;
};

#endif
