#ifndef NAVDATA_COMMON_H
#define NAVDATA_COMMON_H
enum State
{
	Disconnected = 0,
	Landed,
	Flying
};

struct navdata_option_t
{
	// Navdata block ('option') identifier
	uint16_t tag;

	// set this to the size of this structure
	uint16_t size;

	uint8_t data[1];
} __attribute__((packed));

struct navdata_t {
	// Always set to NAVDATA_HEADER
	uint32_t header;

	// Bit mask built from def_ardrone_state_mask_t
	uint32_t ardrone_state;

	// Sequence number, incremented for each sent packet
	uint32_t sequence;
	uint32_t vision_defined;

	navdata_option_t options[1];
} __attribute__((packed));

struct navdata_mat3_t
{
	float _00;
	float _01;
	float _02;
	float _10;
	float _11;
	float _12;
	float _20;
	float _21;
	float _22;
} __attribute__((packed));

struct navdata_vec3_t
{
	float _0;
	float _1;
	float _2;
} __attribute__((packed));

struct navdata_demo_t
{
	uint16_t    tag;					  /*!< Navdata block ('option') identifier */
	uint16_t    size;					  /*!< set this to the size of this structure */

	// Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum.
	uint32_t ctrl_state;

	// battery voltage filtered (mV)
	uint32_t vbat_flying_percentage;

	// UAV's pitch in milli-degrees
	float pitch;

	// UAV's roll in milli-degrees
	float roll;

	// UAV's yaw in milli-degrees
	float yaw;

	// UAV's altitude in centimeters
	int32_t altitude;

	// UAV's estimated linear velocity
	navdata_vec3_t velocity;
	//float   vx;                     /*!< UAV's estimated linear velocity */
	//float   vy;                     /*!< UAV's estimated linear velocity */
	//float   vz;                     /*!< UAV's estimated linear velocity */

	uint32_t frameIndex;

	navdata_mat3_t detection_cam_rotation;
	navdata_vec3_t detection_cam_translation;
	uint32_t tagIndex;
	uint32_t detectionType;

	navdata_mat3_t drone_cam_rotation;
	navdata_vec3_t drone_cam_translation;
} __attribute__((packed));

struct navdata_raw_t
{
	uint16_t accelX;
	uint16_t accelY;
	uint16_t accelZ;

	uint16_t gyroX;
	uint16_t gyroY;
	uint16_t gyroZ;

	int16_t gyroX_110;
	int16_t gyroY_110;

	uint32_t batteryMilliVolt;

	uint16_t usEchoStart;
	uint16_t usEchoEnd;
	uint16_t usEchoAssociation;
	uint16_t usEchoDistance;

	uint16_t usCurveTime;
	uint16_t usCurveValue;
	uint16_t usCurveRef;

	uint16_t echoFlagIni;
	uint16_t echoNum;
	uint16_t echoSum;
        
	int32_t altTemp;
	int16_t   gradient;
} __attribute__((packed));

struct navdata_phys_t
{
	 float accelTemp;
	 uint16_t gyroTemp;

	 navdata_vec3_t accel;
         navdata_vec3_t gyro;

	 uint32_t alim3V3;
	 uint32_t vrefEpson;
	 uint32_t vrefIDG;
} __attribute__((packed));

#endif
