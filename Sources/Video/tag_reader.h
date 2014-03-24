/**
 *  Scans an image for April Tags
 *  Language: C++
 *  Written by: Alex Liao
*/

#ifndef TAG_READER_H
#define TAG_READER_H

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

extern "C" {
// Include the Libraries for AprilTags
#include "apriltags/apriltag.h"
#include "apriltags/image_u8.h"
#include "apriltags/tag36h11.h"
#include "apriltags/zarray.h"
}

struct TagData
{
	double distance;
	double angle;
	double x;
	double y;
	int id;
};

class TagReader
{
	public:
		// Constructor and Destructor
		TagReader();
		~TagReader();

		// REQUIRES: allocated Mat and TagData
		// MODIFIES: data
		// EFFECTS:  Finds April Tags and stores info in data
		void process_Mat(const cv::Mat& img, TagData &data);

		// REQUIRES: allocated Mats and TagData
		// MODIFIES: data, outimg
		// EFFECTS:  Finds April Tags and stores info in data
		//			 draws data and lines onto outimg
		void process_Mat(const cv::Mat& img, TagData &data, cv::Mat& outimg);

	private:
		// The width and height of the frame
		int frameWidth;
		int frameHeight;

		// The midpoints of the frame
		int frameMidX;
		int frameMidY;
		
		// Linear coeffiecents for altitude
		double distance_M;
		int distance_B;

		// April tag family and detector
		april_tag_family_t *tf;
		april_tag_detector_t *td;

		// Data from the QR code and its position/angle
		int qr_length;		// The length of the code in pixels
		double qr_distance; // How far the qr code is (altitude)
		double qr_angle;	// Angle of the code from the right x-axis
		double qr_angle_deg;// Same as above but in degrees
		double dis2Mid;		// Distance from the camera middle to code
		double theta1;		// the arctan of the y' and x' axes
		double theta2;		// the angle between the two axes
		double theta2_deg;	// theta2 in radians
		double x_d;
		double y_d;
		int rel_x, rel_y;	// The data from the QR Code
};

#endif
