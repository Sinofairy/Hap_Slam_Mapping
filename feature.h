#include <string.h>
#include <Eigen/Dense>
#include <vector>

#define CLASS_NUM  20
#define MAX_PS_NUM 14
#define PARKINGSLOT_NUM  4
#define MAX_PS_PTS_NUM 2048
#define MAX_MOS_NUM  20
#define MAX_MOS_CONTOUR_NUM 500
#define MAX_MOS_PTS_NUM 2048


#ifndef _INCLUDE_HEADER_
#define _INCLUDE_HEADER_

// Input Data from IMU and Wheels and GNSS
typedef struct OdomData_
{
	double time = 0.0;
	Eigen::Matrix4d pose;
	Eigen::Matrix4d delta_pose;
	Eigen::Vector3d vel;
}OdomData;

//Input pointsets Data from Mos and Psd
typedef struct Point2d_
{
    int32_t x = 0;
    int32_t y = 0;
}Point2d;

typedef struct SegObjBox_
{
    Point2d p[4];
}SegObjBox;

typedef struct ParkSlot_
{
	int32_t iID;
	int8_t direction;
	int8_t type;
	SegObjBox imgPsdBox;
	SegObjBox realPsdBox;
	int32_t ptPsdNum;
	Point2d PsdPointSets[MAX_PS_PTS_NUM];
}ParkSlot;

typedef struct PsdData_
{
	double time = 0.0;
	int8_t ivalid;
	char PsdName[100];
	/*string PsdName;*/
	int8_t iParkNum;
	ParkSlot stParkLot[MAX_PS_NUM];
}PsdData;

typedef struct MosObject_
{
	int32_t iID;
	int8_t type;
	SegObjBox imgMosBox;
	int32_t boxPtNum;
	int32_t ptMosNum;
	int32_t contourPtNum;
	Point2d contourPts[MAX_MOS_CONTOUR_NUM];
	Point2d MosPointSets[MAX_MOS_PTS_NUM];
}MosObject;

typedef struct MosData_
{
	double time = 0.0;
	int8_t ivalid;
	char MosName[100];
	int8_t iMosNum;
	MosObject stMosObj[MAX_MOS_NUM];
}MosData;

//typedef struct SegFeatureInPicture_
//{
//    double time = 0.0;  //TimeStamp
//    string name;       //image name
//	int8_t ObjType = 0;    //object type
//    vector<SegObjBox> segobjects; //seg results
//}SegFeatureInPicture;

typedef struct key_value_color_
{
	uint8_t key;
	const char * value;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} key_value_color;

static const key_value_color id_label_map[CLASS_NUM] = 
{
	{ 0, "_background_", 0, 0, 0 },
	{ 1, "parkingLine", 150, 0, 0 },
	{ 2, "whiteSolid", 128, 64, 128 },
	{ 3, "whiteDotted", 232, 35, 244 },
	{ 4, "yellowSolid", 156, 250, 102 },
	{ 5, "yellowDotted", 153, 153, 50 },
	{ 6, "crossing", 153, 0, 153 },
	{ 7, "speedBump", 30, 170, 240 },
	{ 8, "straight", 0, 220, 220 },
	{ 9, "left", 35, 142, 107 },
	{ 10, "right", 152, 251, 152 },
	{ 11, "turn", 180, 130, 0 },
	{ 12, "straightLeft", 60, 20, 220 },
	{ 13, "straightRight", 0, 0, 255 },
	{ 14, "straightLeftRight", 0, 142, 0 },
	{ 15, "straightTurn", 0, 0, 90 },
	{ 16, "leftRight", 100, 60, 0 },
	{ 17, "leftTurn", 100, 80, 201 },
	{ 18, "leftCurve", 200, 0, 80 },
	{ 19, "rightCurve", 32, 11, 119 }
};

static const key_value_color ps_label_map[PARKINGSLOT_NUM] = 
{
	{ 0, "parallel", 255, 0, 0 },
	{ 1, "vertical", 0, 255, 0 },
	{ 2, "diagonal_without_rightangle", 128, 255, 128 },
	{ 3, "diagonal_with_rightangle", 128, 0, 128 }
};

#endif