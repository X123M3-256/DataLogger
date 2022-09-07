#include<stdint.h>

typedef struct __attribute__((__packed__))
{
uint32_t itow;
uint16_t year;
uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t min;
uint8_t sec;
uint8_t valid;
uint32_t time_acc;
int32_t nanoseconds;
uint8_t fix_type;
uint8_t flags;
uint8_t flags_2;
uint8_t num_sv;
int32_t longitude;
int32_t latitude;
int32_t height;
int32_t height_msl;
uint32_t horizontal_acc;
uint32_t vertical_acc;
int32_t velocity_n;
int32_t velocity_e;
int32_t velocity_d;
int32_t ground_speed;
int32_t ground_heading;
uint32_t speed_acc;
uint32_t heading_acc;
uint16_t position_dop;
uint8_t flags_3;
uint8_t reserved[5];
int32_t heading;
int16_t magnetic_declination;
uint16_t magnetic_declination_acc;
}ubx_nav_pvt_t;

typedef struct __attribute__((__packed__))
{
uint8_t clss;
uint8_t id;
}ubx_ack_ack_t;

typedef struct __attribute__((__packed__))
{
uint8_t clss;
uint8_t id;
}ubx_ack_nack_t;

typedef struct __attribute__((__packed__))
{
uint8_t port;
uint8_t reserved_1;
uint16_t tx_ready;
uint32_t mode;
uint32_t baud_rate;
uint16_t in_proto_mask;
uint16_t out_proto_mask;
uint16_t flags;
uint16_t reserved_2;
}ubx_cfg_prt_t;

typedef struct __attribute__((__packed__))
{
uint8_t clss;
uint8_t id;
uint8_t rate;
}ubx_cfg_msg_t;

typedef struct __attribute__((__packed__))
{
uint16_t measurement_rate;
uint16_t navigation_rate;
uint16_t time_reference;
}ubx_cfg_rate_t;

enum
{
UBX_CFG_NAV5_DYNAMIC_MODEL=0x1,
UBX_CFG_NAV5_MINIMUM_ELEVATION=0x2,
UBX_CFG_NAV5_POSITION_FIX_MODE=0x4,
UBX_CFG_NAV5_DR_LIMIT=0x8,
UBX_CFG_NAV5_POSITION_MASK=0x10,
UBX_CFG_NAV5_TIME_MASK=0x20,
UBX_CFG_NAV5_STATIC_HOLD_MASK=0x40,
UBX_CFG_NAV5_DGNSS_MASK=0x80,
UBX_CFG_NAV5_CNO_THRESHOLD=0x100,
UBX_CFG_NAV5_UTC_STANDARD=0x400
};

enum 
{
UBX_DYNAMIC_MODEL_PORTABLE=0,
UBX_DYNAMIC_MODEL_STATIONARY=2,
UBX_DYNAMIC_MODEL_PEDESTRIAN=3,
UBX_DYNAMIC_MODEL_AUTOMOTIVE=4,
UBX_DYNAMIC_MODEL_SEA=5,
UBX_DYNAMIC_MODEL_AIRBORNE_1G=6,
UBX_DYNAMIC_MODEL_AIRBORNE_2G=7,
UBX_DYNAMIC_MODEL_AIRBORNE_4G=8
};

enum
{
UBX_FIX_MODE_2D_ONLY=1,
UBX_FIX_MODE_3D_ONLY=2,
UBX_FIX_MODE_AUTO=3
};

enum
{
UBX_UTC_STANDARD_AUTOMATIC=0,
UBX_UTC_STANDARD_GPS=1,
UBX_UTC_STANDARD_GALILEO=2,
UBX_UTC_STANDARD_GLONASS=3,
UBX_UTC_STANDARD_BEIDOU=4,
UBX_UTC_STANDARD_NAVIC=5,
};

typedef struct __attribute__((__packed__))
{
uint16_t mask;
uint8_t dynModel;
uint8_t fixMode;
int32_t fixedAlt;		//cm
uint32_t fixedAltVar;		//cm^2
int8_t minElev;			//deg
uint8_t drLimit;		//s
uint16_t pDop;			//0.1
uint16_t tDop;			//0.1
uint16_t pAcc;			//m
uint16_t tAcc;			//m
uint8_t staticHoldThresh;	//cm/s
uint8_t dgnssTimeout;		//s
uint8_t cnoThreshNumSVs;	
uint8_t cnoThresh;		//dBHz
uint8_t reserved1[2];
uint8_t staticHoldMaxDist;	//m
uint8_t utcStandard;
uint8_t reserved2[5];
}ubx_cfg_nav5_t;

typedef struct __attribute__((__packed__))
{
uint8_t clss;
uint8_t id;
uint16_t length;
union
	{
	ubx_ack_ack_t ack_ack;
	ubx_ack_nack_t ack_nack;
	ubx_nav_pvt_t nav_pvt;
	ubx_cfg_prt_t cfg_prt;
	ubx_cfg_msg_t cfg_msg;
	ubx_cfg_rate_t cfg_rate;
	ubx_cfg_nav5_t cfg_nav5;
	};
}ubx_t;

void ubx_send(ubx_t* ubx);
bool ubx_receive(ubx_t* ubx);


typedef struct
{
uint64_t timestamp;
uint32_t num_sv;
double lon;//TODO these can't all be doubles
double lat;
double alt;
double vel_n;
double vel_e;
double vel_d;
double horz_acc;
double vert_acc;
double vel_acc;
double dop;
}gps_sample_t;



bool gps_init(bool gps_already_configured=false);
