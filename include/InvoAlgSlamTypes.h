/*  BEGIN_FILE_HDR
************************************************************************************************
*   NOTICE
*   This software is the property of INVO Co.,Ltd.. Any information contained in this
*   doc should not be reproduced, or used, or disclosed without the written authorization from
*   INVO Co.,Ltd..
************************************************************************************************
*   File Name       : InvoAlgSlamTypes.h
************************************************************************************************
*   END_FILE_HDR*/
#ifndef INVO_ALG_SLAM_TYPES_H_
#define INVO_ALG_SLAM_TYPES_H_

/*-------------------------------------------------------------------------------
 * Includes
 -------------------------------------------------------------------------------*/
#include "InvoAlgCommon.h"

/*-------------------------------------------------------------------------------
 * Macros
 -------------------------------------------------------------------------------*/
#define INVO_SLAM_BOXPOINT_NUM              (4)
#define INVO_SLAM_NAME_MAX                  (50)
#define INVO_SLAM_MAP_NUM_MAX               (100)
#define INVO_SLAM_OBJ_NUM_MAX               (100)
#define INVO_SLAM_ROAD_NUM_MAX              (1000)

/*-------------------------------------------------------------------------------
 * Types
 -------------------------------------------------------------------------------*/
typedef enum
{
    INVO_SLAM_FILE_OBJECT = 0,                                      /* map of object */
    INVO_SLAM_FILE_ROAD,                                            /* map of road */
    INVO_SLAM_FILE_POINT_SET,                                       /* map of point set */
    INVO_SLAM_FILE_TRAJECTORY,                                      /* map of trajectory, used for preview */
    INVO_SLAM_FILE_POSE,                                            /* map of pose */
    INVO_SLAM_FILE_SIMULATION,                                      /* simulation data */
    INVO_SLAM_FILE_NUMBER,                                          /* map file number */
} INVO_SLAM_FILE_TYPE;

typedef enum
{
    INVO_SLAM_TYPE_PARKING_PARALLEL = 0,                            /* parking parallel */
    INVO_SLAM_TYPE_PARKING_PERPEN,                                  /* parking perpen */
    INVO_SLAM_TYPE_PARKING_PARA_SLANT,                              /* parking slant parallelogram */
    INVO_SLAM_TYPE_PARKING_RECT_SLANT,                              /* parking slant rect */
    INVO_SLAM_TYPE_CROSSING,                                        /* crossing */
    INVO_SLAM_TYPE_SPEEDBUMP,                                       /* speed bump */
    INVO_SLAM_TYPE_ARROW_STRAIGHT,                                  /* arrow straight */
    INVO_SLAM_TYPE_ARROW_LEFT,                                      /* arrow left */
    INVO_SLAM_TYPE_ARROW_RIGHT,                                     /* arrow right */
    INVO_SLAM_TYPE_ARROW_TURN,                                      /* arrow turn */
    INVO_SLAM_TYPE_ARROW_STRAIGHT_LEFT,                             /* arrow straight left */
    INVO_SLAM_TYPE_ARROW_STRAIGHT_RIGHT,                            /* arrow straight right */
    INVO_SLAM_TYPE_ARROW_STRAIGHT_LEFT_RIGHT,                       /* arrow straight left right */
    INVO_SLAM_TYPE_ARROW_STRAIGHT_TURN,                             /* arrow straight turn */
    INVO_SLAM_TYPE_ARROW_LEFT_RIGHT,                                /* arrow left right */
    INVO_SLAM_TYPE_ARROW_LEFT_TURN,                                 /* arrow left turn */
    INVO_SLAM_TYPE_ARROW_LEFT_CURVE,                                /* arrow left curve */
    INVO_SLAM_TYPE_ARROW_RIGHT_CURVE,                               /* arrow right curve */
    INVO_SLAM_TYPE_LINE_DEFAULT,                                    /* normal line */
    INVO_SLAM_TYPE_LINE_WHITE_SOLID,                                /* white solid line */
    INVO_SLAM_TYPE_LINE_YELLOW_SOLID,                               /* yellow solid line */
    INVO_SLAM_TYPE_CAR,                                             /* car */
    INVO_SLAM_TYPE_TRUCK,                                           /* truck */
    INVO_SLAM_TYPE_BICYCLE,                                         /* bicycle */
    INVO_SLAM_TYPE_TRICYCLE,                                        /* tricycle */
    INVO_SLAM_TYPE_OBSTACLE,                                        /* obstacle */
    INVO_SLAM_TYPE_PEDESTRIAN,                                      /* pedestrian */
    INVO_SLAM_TYPE_PILLAR,                                          /* pillar */
    INVO_SLAM_TYPE_LIMITER,                                         /* limiter */
    INVO_SLAM_TYPE_NUMBER
} INVO_SLAM_OBJECT_TYPE;

typedef uint32_t tInvoSlamMapId;

typedef struct
{
    float                       rotate_angle[3];                    /* unit:deg,  rotate angle */
    float                       translate[3];                       /* unit:m, translate */
    float                       Matrix[4][4];                       /* Matrix list */
} tInvoSlamWorldPose;                                               /* used for display */

typedef struct
{
    tInvoAlgImagePoint          Location;                           /* car location in preview */
    float                       direction;                          /* car direction in preview */
} tInvoSlamImagePose;                                               /* used for preview */

typedef struct
{
    float                       distance;                           /* route distance [m] */
    uint32_t                    obj_count[INVO_SLAM_TYPE_NUMBER];   /* count of each object type */
} tInvoSlamTravelInfo;                                              /* used for display */

typedef struct
{
    tInvoSlamMapId              id;                                 /* map id */
    char                        name[INVO_SLAM_NAME_MAX];           /* map name */
    char                        file[INVO_SLAM_FILE_NUMBER][INVO_SLAM_NAME_MAX];
                                                                    /* map file [path + name] */
    tInvoAlgGpsData             start;                              /* route start gps */
    tInvoSlamTravelInfo         travel;                             /* travel information */
} tInvoSlamMapInfo;                                                 /* used for display */

typedef struct
{
    uint32_t                    count;                              /* map count [0,INVO_SLAM_MAP_NUM_MAX] */
    tInvoSlamMapInfo            map[INVO_SLAM_MAP_NUM_MAX];         /* map list */
} tInvoSlamMapList;                                                 /* used for display */

typedef struct
{
    tInvoAlgWorldPoint          left;                               /* road box position */
    tInvoAlgWorldPoint          right;                              /* road box position */
} tInvoSlamRoadInfo;                                                /* used for display */

typedef struct
{
    uint32_t                    count;                              /* road count [0,INVO_SLAM_ROAD_NUM_MAX] */
    tInvoSlamRoadInfo           road[INVO_SLAM_ROAD_NUM_MAX];       /* road list */
} tInvoSlamRoadList;                                                /* used for display */

typedef struct
{
    INVO_SLAM_OBJECT_TYPE       type;                               /* object type */
    tInvoAlgWorldPoint          center;                             /* object center */
    float                       direction;                          /* object direction */
    tInvoAlgWorldPoint          box_point[INVO_SLAM_BOXPOINT_NUM];  /* object box position */
} tInvoSlamObjectInfo;                                              /* used for display */

typedef struct
{
    uint32_t                    count;                              /* object count [0,INVO_SLAM_OBJ_NUM_MAX] */
    tInvoSlamObjectInfo         object[INVO_SLAM_OBJ_NUM_MAX];      /* object list */
} tInvoSlamObjectList;                                              /* used for display */

#endif /* INVO_ALG__TYPES_H_ */
