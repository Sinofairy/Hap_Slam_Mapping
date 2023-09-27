/*  BEGIN_FILE_HDR
************************************************************************************************
*   NOTICE
*   This software is the property of INVO Co.,Ltd.. Any information contained in this
*   doc should not be reproduced, or used, or disclosed without the written authorization from
*   INVO Co.,Ltd..
************************************************************************************************
*   File Name       : InvoAlgMosTypes.h
************************************************************************************************
*   END_FILE_HDR*/
#ifndef INVO_ALG_MOS_TYPES_H_
#define INVO_ALG_MOS_TYPES_H_

/*-------------------------------------------------------------------------------
 * Includes
 -------------------------------------------------------------------------------*/
#include "InvoAlgCommon.h"

/*-------------------------------------------------------------------------------
 * Macros
 -------------------------------------------------------------------------------*/
#define INVO_MOS_BOXPOINT_NUM               (4)
#define INVO_MOS_CONTOUR_NUM                (1024)
#define INVO_MOS_POINTSET_NUM               (4096)
#define INVO_MOS_OBJECT_NUM                 (20)

/*-------------------------------------------------------------------------------
 * Types
 -------------------------------------------------------------------------------*/
typedef enum
{
    INVO_MOS_TYPE_CROSSING,                                     /* crossing */
    INVO_MOS_TYPE_SPEEDBUMP,                                    /* speedBump */
    INVO_MOS_TYPE_ARROW_STRAIGHT,                               /* arrow straight */
    INVO_MOS_TYPE_ARROW_LEFT,                                   /* arrow left */
    INVO_MOS_TYPE_ARROW_RIGHT,                                  /* arrow right */
    INVO_MOS_TYPE_ARROW_TURN,                                   /* arrow turn */
    INVO_MOS_TYPE_ARROW_STRAIGHT_LEFT,                          /* arrow straight left */
    INVO_MOS_TYPE_ARROW_STRAIGHT_RIGHT,                         /* arrow straight right */
    INVO_MOS_TYPE_ARROW_STRAIGHT_LEFT_RIGHT,                    /* arrow straight left right */
    INVO_MOS_TYPE_ARROW_STRAIGHT_TURN,                          /* arrow straight turn */
    INVO_MOS_TYPE_ARROW_LEFT_RIGHT,                             /* arrow left right */
    INVO_MOS_TYPE_ARROW_LEFT_TURN,                              /* arrow left turn */
    INVO_MOS_TYPE_ARROW_LEFT_CURVE,                             /* arrow left curve */
    INVO_MOS_TYPE_ARROW_RIGHT_CURVE,                            /* arrow right curve */
} INVO_MOS_OBJECT_TYPE;


typedef struct
{
	uint32_t                counter_num;                        /* counter number */
    tInvoAlgImagePoint      box_point[INVO_MOS_BOXPOINT_NUM];   /* box-point position */
	tInvoAlgImagePoint      counter[INVO_MOS_CONTOUR_NUM];      /* counter position */
} tInvoMosImageInfo;

typedef struct
{
    tInvoAlgLocalPoint      box_point[INVO_MOS_BOXPOINT_NUM];   /* box-point position */
    uint32_t                counter_num;                        /* counter number */
    tInvoAlgLocalPoint      counter[INVO_MOS_CONTOUR_NUM];      /* counter position */
    float                   angle;                              /* arrow direction */
} tInvoMosLocalInfo;

typedef struct
{
    INVO_MOS_OBJECT_TYPE    type;                               /* object type */
    uint32_t                track_id;                           /* object track id */
    float                   confidence;                         /* [0.0, 1.0] */
    tInvoMosImageInfo       image_info;                         /* object position in image coordinate */
    tInvoMosLocalInfo       local_info;                         /* object position in local coordinate */
} tInvoMosObj;

typedef struct
{   
    bool                    ivalid;
    i_time                  timer;                              /* timestamp, unit ms */
    uint32_t                number;                             /* object number */
    tInvoMosObj             objects[INVO_MOS_OBJECT_NUM];       /* object list */
} tInvoMosObjList;

#endif /* INVO_ALG_MOS_TYPES_H_ */
