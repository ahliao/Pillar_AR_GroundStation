// File: TagReader.cpp
// Written by: Pillar Technologies MDP Team 2014
// Scans OpenCV's Mat image for any April Tags
// NOTE: Tags have to be different

#include "tag_reader.h"

// Include the Libraries for AprilTags
extern "C" {
#include "apriltags/apriltag.h"
#include "apriltags/image_u8.h"
#include "apriltags/tag36h11.h"
#include "apriltags/zarray.h"
}

#ifndef PI
#define PI 3.1415926535897932384626
#endif

// The number of rows and cols in the map matrix
int map_length = 1;

// The spacing between each tag (x and y spacings are the same)
// Unit of measurement: meters
float map_tag_spacing = 1; // Default is 1 meter

// Constructor
TagReader::TagReader()
{
	frameWidth = 640;
	frameHeight = 360;
	frameMidX = 320;
	frameMidY = 180;
	distance_M = -0.15;
	distance_B = 90;

	// Instantiate a tag family and pass to detector
	tf = tag36h11_create();
	td = april_tag_detector_create(tf);
	td->nthreads = 4;
	td->seg_sigma = 0.8;
	td->min_mag = 0;
	td->seg_decimate = 3; // stops the lag spikes
	// Set to 1 for better detection but slower rate
}

TagReader::~TagReader()
{
	if (td != NULL)
	    april_tag_detector_destroy(td);
	if (tf != NULL)
	    tag36h11_destroy(tf);
}

// REQUIRES: allocated Mat and TagData
// MODIFIES: data
// EFFECTS:  Finds April Tags and stores info in data
void TagReader::process_Mat(const cv::Mat& img, std::vector<TagData> *data)
{
	// Clear the vector
	data->clear();

	// TagData struct
	TagData tag;

	// Form an image_u8 with the image
	image_u8_t *im = image_u8_create_from_rgb3(img.cols, img.rows, 
		(uint8_t *)img.data, img.step);
	zarray_t * detections = april_tag_detector_detect(td, im);

	int i = 0;
	for (i = 0; i < zarray_size(detections); ++i) {
		// Init the default values of the TagData
		tag.distance = -1;
		tag.angle = -1;
		tag.rel_x = -1;
		tag.rel_y = -1;
		tag.img_x = -1;
		tag.img_y = -1;
		tag.id = -1;

		april_tag_detection_t *det;
		zarray_get(detections, i, &det);

		//printf("%i code\n", i);
		//printf("detection %3d: id %4d, hamming %d, goodness %f\n", i,
		//		det->id, det->hamming, det->goodness);

		// Get the coordinates based on the id detected
		rel_x = det->id % map_length * map_tag_spacing;
		rel_y = det->id / map_length * map_tag_spacing;

		// get the four corners
		// Find the angle between the lines
		std::vector<cv::Point> vp;
		for (int i = 0; i < 4; ++i) {
			vp.push_back(cv::Point(det->p[i][0], det->p[i][1]));
		}
		cv::RotatedRect r = minAreaRect(vp);
		cv::Point2f rectvp[4];
		r.points(rectvp);

		// Get the distance from the code to the camera
		qr_length = sqrt((vp[0].x - vp[1].x) * (vp[0].x - vp[1].x) +
				(vp[0].y - vp[1].y) * (vp[0].y - vp[1].y));
		qr_distance = qr_length * distance_M + distance_B;
		//printf("Length: %i\n", qr_length);
		//printf("Distance: %f\n", qr_distance);

		// Get the angle of the square/rectangle
		qr_angle = r.angle;
		if (vp[0].x < vp[1].x && vp[0].y < vp[3].y &&
				vp[0].y < vp[2].y && vp[0].x > vp[3].x) qr_angle += 90;
		else if (vp[0].x > vp[1].x && vp[0].y < vp[1].y &&
				vp[0].y > vp[3].y && vp[0].x > vp[2].x) qr_angle += 180;
		else if (vp[2].x > vp[1].x && vp[0].x < vp[3].x &&
				vp[0].y > vp[2].y && vp[2].y < vp[1].y) qr_angle += 270;
		else if (vp[0].x < vp[2].x && vp[1].y < vp[3].y &&
				vp[1].x > vp[0].x && vp[2].y < vp[3].y) qr_angle += 360;

		else if (vp[0].x == vp[3].x && vp[0].y == vp[1].y &&
				vp[0].x < vp[1].x && vp[0].y < vp[3].y) qr_angle = 0;
		else if (vp[0].x - 1 <= vp[1].x && vp[0].y - 1 <= vp[3].y &&
				vp[0].x > vp[3].x && vp[0].y < vp[1].y) qr_angle = 90;
		else if (vp[0].x == vp[3].x && vp[0].y == vp[1].y &&
				vp[0].x > vp[1].x && vp[0].y > vp[3].y) qr_angle = 180;
		else if (vp[0].x <= vp[1].x && vp[0].y == vp[3].y &&
				vp[0].x < vp[3].x && vp[0].y > vp[1].y) qr_angle = 270;

		// Store the angle
		qr_angle_deg = qr_angle;
		qr_angle = qr_angle * PI / 180;
		cv::Point mid((vp[0].x + vp[2].x)/2, (vp[0].y + vp[2].y)/2);
		
		// Get the relative location based on the data of the QR code
		// Relative position (in pixel)
		dis2Mid = sqrt((mid.x - frameMidX) * (mid.x - frameMidX) + 
				(mid.y - frameMidY) * (mid.y - frameMidY));
		
		// Get the relative position tag (in pixels)
		theta1 = atan2(frameMidY - mid.y, frameMidX - mid.x);
		theta2 = qr_angle - theta1;
		x_d = dis2Mid * cos(theta2);
		y_d = dis2Mid * sin(theta2);

		// Convert the position to meters
		//x_d = x_d * distance_M + distance_B;
		//y_d = y_d * distance_M + distance_B;
		
		// TODO: take average position for multiple tags
		//x_ab = x_d + rel_x;
		//y_ab = y_d + rel_y;

		// put the data into the QR_Data struct
		tag.distance = qr_distance;
		tag.angle = qr_angle;
		tag.rel_x = x_d;
		tag.rel_y = y_d;
		tag.img_x = mid.x;
		tag.img_y = mid.y;
		tag.id = det->id;
		tag.side_length = qr_length;
		data->push_back(tag);

		april_tag_detection_destroy(det);
	}
	image_u8_destroy(im);
	zarray_destroy(detections);
}

// REQUIRES: allocated Mats and TagData
// MODIFIES: data, outimg
// EFFECTS:  Finds April Tags and stores info in data
//			 draws data and lines onto outimg
void TagReader::process_Mat(const cv::Mat& img, std::vector<TagData> *data, cv::Mat& outimg)
{
	// copy the img to outimg
	outimg = img.clone();

	// Clear the vector
	data->clear();

	// TagData struct
	TagData tag;

	// Form an image_u8 with the image
	image_u8_t *im = image_u8_create_from_rgb3(img.cols, img.rows, 
		(uint8_t *)img.data, img.step);
	zarray_t * detections = april_tag_detector_detect(td, im);

	int i = 0;
	for (i = 0; i < zarray_size(detections); ++i) {
		// Init the default values of the TagData
		tag.distance = -1;
		tag.angle = -1;
		tag.rel_x = -1;
		tag.rel_y = -1;
		tag.img_x = -1;
		tag.img_y = -1;
		tag.id = -1;

		april_tag_detection_t *det;
		zarray_get(detections, i, &det);

		//printf("%i code\n", i);
		//printf("detection %3d: id %4d, hamming %d, goodness %f\n", i,
		//		det->id, det->hamming, det->goodness);

		// Get the coordinates based on the id detected
		//rel_x = det->id % map_length * map_tag_spacing;
		//rel_y = det->id / map_length * map_tag_spacing;

		// get the four corners
		// Find the angle between the lines
		std::vector<cv::Point> vp;
		for (int i = 0; i < 4; ++i) {
			vp.push_back(cv::Point(det->p[i][0], det->p[i][1]));
		}
		cv::RotatedRect r = minAreaRect(vp);
		cv::Point2f rectvp[4];
		r.points(rectvp);
		for (int i = 0; i < 4; ++i)
			cv::line(outimg, rectvp[i], rectvp[(i+1)%4], 
					cv::Scalar(55, 255, 0), 5);

		// Get the distance from the code to the camera
		qr_length = sqrt((vp[0].x - vp[1].x) * (vp[0].x - vp[1].x) +
				(vp[0].y - vp[1].y) * (vp[0].y - vp[1].y));
		qr_distance = qr_length * distance_M + distance_B;
		//printf("Length: %i\n", qr_length);
		//printf("Distance: %f\n", qr_distance);

		// Get the angle of the square/rectangle
		qr_angle = r.angle;
		if (vp[0].x < vp[1].x && vp[0].y < vp[3].y &&
				vp[0].y < vp[2].y && vp[0].x > vp[3].x) qr_angle += 90;
		else if (vp[0].x > vp[1].x && vp[0].y < vp[1].y &&
				vp[0].y > vp[3].y && vp[0].x > vp[2].x) qr_angle += 180;
		else if (vp[2].x > vp[1].x && vp[0].x < vp[3].x &&
				vp[0].y > vp[2].y && vp[2].y < vp[1].y) qr_angle += 270;
		else if (vp[0].x < vp[2].x && vp[1].y < vp[3].y &&
				vp[1].x > vp[0].x && vp[2].y < vp[3].y) qr_angle += 360;

		else if (vp[0].x == vp[3].x && vp[0].y == vp[1].y &&
				vp[0].x < vp[1].x && vp[0].y < vp[3].y) qr_angle = 0;
		else if (vp[0].x - 1 <= vp[1].x && vp[0].y - 1 <= vp[3].y &&
				vp[0].x > vp[3].x && vp[0].y < vp[1].y) qr_angle = 90;
		else if (vp[0].x == vp[3].x && vp[0].y == vp[1].y &&
				vp[0].x > vp[1].x && vp[0].y > vp[3].y) qr_angle = 180;
		else if (vp[0].x <= vp[1].x && vp[0].y == vp[3].y &&
				vp[0].x < vp[3].x && vp[0].y > vp[1].y) qr_angle = 270;

		// Draw a line for the angle
		qr_angle_deg = qr_angle;
		qr_angle = qr_angle * PI / 180;
		// TODO: use the tag 'c' parameter instead
		cv::Point mid((vp[0].x + vp[2].x)/2, (vp[0].y + vp[2].y)/2);
		cv::Point p2(mid.x + 25*cos(qr_angle), (mid.y - 25*sin(qr_angle)));
		
		// Draw a line for the angle
		line(outimg, mid, p2, cv::Scalar(100, 100, 0), 3);

		// Get the relative location based on the data of the QR code
		// Relative position (in pixel)
		dis2Mid = sqrt((mid.x - frameMidX) * (mid.x - frameMidX) + 
				(mid.y - frameMidY) * (mid.y - frameMidY));
		
		// Get the relative position tag (in pixels)
		theta1 = atan2(frameMidY - mid.y, frameMidX - mid.x);
		theta2 = qr_angle - theta1;
		x_d = dis2Mid * cos(theta2);
		y_d = dis2Mid * sin(theta2);
		/*theta1 = atan2(frameMidY - mid.y, frameMidX - mid.x) * 180 / PI;
		theta2_deg = 90 - theta1 - qr_angle_deg; 
		theta2 = theta2_deg * PI / 180;
		x_d = dis2Mid * sin(theta2);
		y_d = dis2Mid * cos(theta2);*/
		line(outimg, mid, cv::Point(frameMidX, frameMidY), cv::Scalar(5, 5, 5), 2);
		cv::Point ptb1(mid.x + dis2Mid * cos(theta1) - y_d * sin(theta1 + theta2),
				mid.y + dis2Mid * sin(theta1) + y_d * cos(theta1 + theta2));
		line(outimg, mid, ptb1, cv::Scalar(0, 200, 1), 2);
		cv::Point ptb2(mid.x + dis2Mid * cos(theta1),
				mid.y + dis2Mid * sin(theta1));
		line(outimg, ptb1, ptb2, cv::Scalar(0, 200, 1), 2);

		// Convert the position to meters
		//x_d = x_d * distance_M + distance_B;
		//y_d = y_d * distance_M + distance_B;
		
		// TODO: take average position for multiple tags
		//x_ab = x_d + rel_x;
		//y_ab = y_d + rel_y;

		// put the data into the QR_Data struct
		tag.distance = qr_distance;
		tag.angle = qr_angle;
		tag.rel_x = x_d;
		tag.rel_y = y_d;
		tag.img_x = mid.x;
		tag.img_y = mid.y;
		tag.id = det->id;
		data->push_back(tag);

		april_tag_detection_destroy(det);
	}

	image_u8_destroy(im);
	zarray_destroy(detections);
}
