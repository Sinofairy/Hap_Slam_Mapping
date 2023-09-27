/*  BEGIN_FILE_HDR
************************************************************************************************
*   NOTICE
*   This software is the property of INVO Co.,Ltd.. Any information contained in this
*   doc should not be reproduced, or used, or disclosed without the written authorization from
*   INVO Co.,Ltd..
************************************************************************************************
*   File Name       : InvoAlgCommon.h
************************************************************************************************
*   END_FILE_HDR*/
#ifndef INVO_ALG_COMMON_H_
#define INVO_ALG_COMMON_H_

/*-------------------------------------------------------------------------------
 * Includes
 -------------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

/*-------------------------------------------------------------------------------
 * Macros
 -------------------------------------------------------------------------------*/
#ifndef FALSE
#define FALSE                       (0u)
#endif

#ifndef TRUE
#define TRUE                        (!FALSE)
#endif

#ifndef MAX
#define MAX(A, B)                   ((A) > (B) ? (A) : (B))
#endif

#ifndef MIN
#define MIN(A, B)                   ((A) < (B) ? (A) : (B))
#endif

#ifndef ABS
#define ABS(A)                      ((A) >= 0 ? (A) : (-1 * (A)))
#endif

#define INVO_ALG_USS_OBJECT_MAX     (10)
#define INVO_ALG_USS_ANCHOR_MAX     (30)
#define INVO_ALG_BOX_ANCHOR_MAX     (50)

/*-------------------------------------------------------------------------------
 * Types
 -------------------------------------------------------------------------------*/
typedef int32_t i_time;                                     /* Timestamp */

typedef uint8_t i_bool;                                     /* Boolean : TRUE/FALSE */

typedef int32_t i_errno;                                    /* see <errno.h> */

typedef enum
{
    INVO_ALG_CAMERA_CHN_F = 0,                              /* Front */
    INVO_ALG_CAMERA_CHN_R = 1,                              /* Rear */
    INVO_ALG_CAMERA_CHN_D = 2,                              /* Left */
    INVO_ALG_CAMERA_CHN_P = 3,                              /* Right */
    INVO_ALG_CAMERA_CHN_NUM = 4
} INVO_ALG_CAMERA_CHN;

typedef enum
{
    INVO_ALG_CAMERA_TYPE_720P = 0,                          /* 1280x720 */
    INVO_ALG_CAMERA_TYPE_720P_SCALED = 1,                   /* 1280x720(scaled from 1280x960) */
    INVO_ALG_CAMERA_TYPE_960P = 2,                          /* 1280x960 */
    INVO_ALG_CAMERA_TYPE_NUM = 3
} INVO_ALG_CAMERA_TYPE;

typedef enum
{
    INVO_ALG_CAMERA_NORMAL = 0,                             /* normally */
    INVO_ALG_CAMERA_LOST,                                   /* lost sync */
} INVO_ALG_CAMERA_STATE;

typedef enum
{
    INVO_ALG_SOURCE_NV12 = 0,                               /* NV12 */
    INVO_ALG_SOURCE_Y = 1,                                  /* Y */
    INVO_ALG_SOURCE_RGB,                                    /* RGB */
} INVO_ALG_SOURCE_TYPE;

typedef struct
{
    char _1st;
    char _2nd;
    char _3rd;
    char _4th;
} tInvoAlgVersion;

typedef struct
{
    int32_t                 x;                              /* unit : pixel, position of x-axis. */
    int32_t                 y;                              /* unit : pixel, position of y-axis. */
} tInvoAlgPixelPoint;                                       /* original point : left-up corner of image. */

typedef struct
{
    int32_t                 w;                              /* unit : pixel, number of image width. */
    int32_t                 h;                              /* unit : pixel, number of image height. */
} tInvoAlgPixelSize;

typedef struct
{
    tInvoAlgPixelPoint      point;                          /* rectangle left-up point */
    tInvoAlgPixelSize       size;                           /* rectangle size(w/h) */
} tInvoAlgPixelRect;                                        /* rectangle of object */

typedef struct
{
    float                   x;                              /* [0.0, 1.0] percentage of image width. */
    float                   y;                              /* [0.0, 1.0] percentage of image height. */
} tInvoAlgImagePoint;                                       /* original point : left-up corner of image. */

typedef struct
{
    float                   w;                              /* [0.0, 1.0] percentage of image width. */
    float                   h;                              /* [0.0, 1.0] percentage of image height. */
} tInvoAlgImageSize;

typedef struct
{
    tInvoAlgImagePoint      point;                          /* rectangle left-up point */
    tInvoAlgImageSize       size;                           /* rectangle size(w/h) */
} tInvoAlgImageRect;                                        /* rectangle of object */

typedef struct
{
    float                   x;                              /* unit : m, X+: right of vehicle. */
    float                   y;                              /* unit : m, Y+: front of vehicle. */
    float                   z;                              /* unit : m, Z+: above of vehicle. */
} tInvoAlgLocalPoint;

typedef struct
{
    float                   l;                              /* unit : m, length of object. */
    float                   w;                              /* unit : m, width  of object. */
    float                   h;                              /* unit : m, height of object. */
} tInvoAlgLocalSize;

typedef struct
{
    tInvoAlgLocalPoint      point;                          /* rectangle center point*/
    tInvoAlgLocalSize       size;                           /* rectangle size(l/w/h)*/
    float                   angle;                          /* unit : degree, 0 degree : front of vehicle, direction : clockwise. */
} tInvoAlgLocalRect;                                        /* rectangle of object*/

typedef struct
{
    float                   x;                              /* unit : m, X+: right of mapping start vehicle. */
    float                   y;                              /* unit : m, Y+: front of mapping start vehicle. */
    float                   z;                              /* unit : m, Z+: above of mapping start vehicle. */
} tInvoAlgWorldPoint;

typedef struct
{
    i_time                  timer;                          /* image timestamp(ms) */
    INVO_ALG_CAMERA_CHN     chn;                            /* image channel */
    INVO_ALG_CAMERA_STATE   state;                          /* image state */
    INVO_ALG_SOURCE_TYPE    type;                           /* image type */
    tInvoAlgPixelSize       size;                           /* image size(w/h) */
    uint8_t                *y;                              /* image address of Y(NV12) */
    uint8_t                *uv;                             /* image address of UV(NV12) */
    uint8_t                *rgb;                            /* image address of RGB */
} tInvoAlgSource;

typedef struct
{
    tInvoAlgSource          image[INVO_ALG_CAMERA_CHN_NUM]; /* image source list */
} tInvoAlgSourceList;

typedef struct
{
    i_time                  timer;                          /* unit : ms. */
    i_bool                  valid;                          /* TRUE:valid, FALSE:invalid */
    float                   value;                          /* unit : Km/H */
} tInvoAlgSpeed;

typedef struct
{
    i_time                  timer;                          /* unit : ms. */
    i_bool                  valid;                          /* TRUE:valid, FALSE:invalid. */
    i_bool                  calid;                          /* TRUE:calibrated, FALSE:not-calibrated. */
    float                   value;                          /* unit : degree, 0+:right, 0-:left */
} tInvoAlgSteer;

typedef struct
{
    i_time                  timer;                          /* unit : ms. */
    uint8_t                 front_left;                     /* 0-close, 1-open. */
    uint8_t                 front_right;                    /* 0-close, 1-open. */
    uint8_t                 back_left;                      /* 0-close, 1-open. */
    uint8_t                 back_right;                     /* 0-close, 1-open. */
    uint8_t                 trunk;                          /* 0-close, 1-open. */
    uint8_t                 bonnet;                         /* 0-close, 1-open. */
} tInvoAlgDoors;

typedef struct
{
    i_time                  timer;                          /* unit : ms */
    uint8_t                 position;                       /* 0-invalid; 1-D; 2-N; 3-R; 4-P; 5~7-not used; */
    uint8_t                 level;                          /* 0-invalid; 1-D; 2-N; 3-R; 4-P; 5~7-not used; */
} tInvoAlgGears;

typedef struct
{
    i_time                  timer;                          /* unit : ms */
    i_bool                  valid[4];                       /* [FL/FR/RL/RR] FALSE-invalid, TRUE-valid; */
    uint16_t                edges[4];                       /* [FL/FR/RL/RR] 0~4095; */
    float                   speed[4];                       /* [FL/FR/RL/RR] Km/h; */
    uint8_t                 direct[4];                      /* [FL/FR/RL/RR] 0-Standstill; 1-Forward; 2-Backward; 3-Unknown; */
} tInvoAlgWheel;

typedef struct
{
    tInvoAlgSpeed           speed;                          /* speed information */
    tInvoAlgSteer           steer;                          /* steer information */
    tInvoAlgDoors           doors;                          /* doors information */
    tInvoAlgGears           gears;                          /* gears information */
    tInvoAlgWheel           wheel;                          /* wheel information */
} tInvoAlgSignals;

typedef struct
{
    uint8_t                 id;                             /* object id [0, INVO_ALG_USS_OBJECT_MAX - 1] */
    uint8_t                 number;                         /* anchor number [0, INVO_ALG_USS_ANCHOR_MAX - 1] */
    tInvoAlgLocalPoint      anchor[INVO_ALG_USS_ANCHOR_MAX];/* anchor list */
} tInvoAlgUssObject;

typedef struct
{
    i_time                  timer;                          /* timestamp, unit ms */
    INVO_ALG_CAMERA_CHN     channel;                        /* object channel */
    uint32_t                number;                         /* object number [0, INVO_ALG_USS_OBJECT_MAX - 1] */
    tInvoAlgUssObject       object[INVO_ALG_USS_OBJECT_MAX];/* object list */
} tInvoAlgUssObjList;

typedef struct
{
    i_time                  timer;                          /* image timestamp(ms) */
    float                   ax;                             /* unit : m/s2, accelerometer in x axis */
    float                   ay;                             /* unit : m/s2, accelerometer in y axis */
    float                   az;                             /* unit : m/s2, accelerometer in z axis */
    float                   gx;                             /* unit : deg/s, gyroscope in x axis */
    float                   gy;                             /* unit : deg/s, gyroscope in y axis */
    float                   gz;                             /* unit : deg/s, gyroscope in z axis */
} tInvoAlgImuData;

typedef struct
{
    i_time                  timer;                          /* image timestamp(ms) */
    float                   longitude;                      /* longitude(deg [-180,180]) E+ W- */
    float                   latitude;                       /* latitude(deg [-90,90]) N+ S- */
    float                   height;                         /* height(m) */
} tInvoAlgGpsData;

typedef struct
{
    int32_t                 type;                           /* object type */
    uint32_t                x_min;                          /* box left-up x coordinate */
    uint32_t                y_min;                          /* box left-up y coordinate */
    uint32_t                x_max;                          /* box right-bottom x coordinate */
    uint32_t                y_max;                          /* box right-bottom y coordinate */
    float                   score;                          /* object score */
} tInvoAlgAnchorBBox;

typedef struct
{
    i_time                  timer;                          /* time */
    uint32_t                number;                         /* object number */
    tInvoAlgAnchorBBox      anchor[INVO_ALG_BOX_ANCHOR_MAX];/* each object information */
} tInvoAlgAnchorList;

typedef struct
{
    uint8_t                 type;                           /* 0:720 1:960 scale to 720  2:960 */
    uint8_t                 flag;                           /* 0:no-use-table-scale, 1:use-table-scale */
    uint32_t                height;                         /* unit : pixel, image height */
    uint32_t                width;                          /* unit : pixel, image width */
    uint8_t                 direct_mapping_cnt;             /* direct mapping count */
    uint8_t                 invers_mapping_cnt;             /* invers mapping count */
    double                  direct_mapping[5];              /* direct mapping coefficients */
    double                  invers_mapping[30];             /* invers mapping coefficients */
    double                  int_param[5];                   /* internal params */
    double                  valid_range[4];                 /* unit : pixel, valid range: valid width, valid height, start x, start y */
    double                  table_scale;                    /* scale coefficients */
} tInvoAlgCameraInternal;

typedef struct
{
    int32_t                 position;                       /* 0:front 1:back 2:left 3:right */
    double                  ext_param[6];                   /* ext params:x,y,z,,pan,rot,title */
    double                  ext_mtx_v2c[9];                 /* rotation matrix: camera to vehicle */
    double                  ext_mtx_c2v[9];                 /* rotation matrix: vehicle to camera */
} tInvoAlgCameraExternal;

typedef struct
{
    tInvoAlgCameraInternal  internal[INVO_ALG_CAMERA_CHN_NUM];
    tInvoAlgCameraExternal  external[INVO_ALG_CAMERA_CHN_NUM];
} tInvoAlgCameraParams;

typedef struct
{
    float                   length;                         /* unit : m, from head to end */
    float                   width;                          /* unit : m, from left to right */
    int32_t                 pulse_counter_max_value;        /* pulse counter max value */
    float                   tread[2];                       /* 0:front wheel thread, 1:back wheel thread */
    float                   front_overhang;                 /* unit : m, front overhang */
    float                   rear_overhang;                  /* unit : m, rear overhang */
    float                   wheel_base;                     /* unit : m, wheel base */
    float                   wheel_width;                    /* unit : m, wheel width */
    int32_t                 ackerman_num;                   /* ackerman coefficient count */
    int32_t                 ackerman_id;                    /* 0:fixed ackerman coefficient 1:dynamic ackerman coefficient */
    float                   pos_error;                      /* unit : m, unit pulse distance */
    float                   ackerman_coe;                   /* fixed ackerman coefficient */
    float                   ackerman_x[200];                /* steering wheel angle */
    float                   ackerman_y[200];                /* dynamic ackerman coefficient */
    float                   max_left_angle;                 /* max left steering wheel angle */
    float                   max_right_angle;                /* max right steering wheel angle  */
} tInvoAlgVehicleParam;

typedef enum
{
    INVO_ALG_VEHICLE_GAC_A60 = 0,
    /* TBD : define other vehicle of GAC */

    INVO_ALG_VEHICLE_BYD_HAN = 100,
    INVO_ALG_VEHICLE_BYD_HAN_W = 101,
    /* TBD : define other vehicle of BYD */

    INVO_ALG_VEHICLE_CHANA_S201 = 200,
    /* TBD : define other vehicle of CHANA */

    INVO_ALG_VEHICLE_NETA_S = 300
    /* TBD : define other vehicle of NETA */

} INVO_ALG_VEHICLE_TYPE;

/*-------------------------------------------------------------------------------
 * Functions
 -------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

i_errno InvoAlg_GetDefaultParams(              /* function : get default parameters */
            INVO_ALG_VEHICLE_TYPE type,        /* input    : vehicle type */
            tInvoAlgVehicleParam *vehicle,     /* output   : vehicle parameters */
            tInvoAlgCameraParams *camera       /* output   : camera parameters */
            );                                 /* return   : [0] success, [EINVAL] vehicle type not support */

#ifdef __cplusplus
}
#endif

#endif /*INVO_ALG_COMMON_H_*/
