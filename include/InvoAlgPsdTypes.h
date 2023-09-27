/*  BEGIN_FILE_HDR
************************************************************************************************
*   NOTICE
*   This software is the property of INVO Co.,Ltd.. Any information contained in this
*   doc should not be reproduced, or used, or disclosed without the written authorization from
*   INVO Co.,Ltd..
************************************************************************************************
*   File Name       : InvoAlgPsdTypes.h
************************************************************************************************
*   END_FILE_HDR*/
#ifndef INVO_ALG_PSD_TYPES_H_
#define INVO_ALG_PSD_TYPES_H_

/*-------------------------------------------------------------------------------
 * Includes
 -------------------------------------------------------------------------------*/
#include "InvoAlgCommon.h"

/*-------------------------------------------------------------------------------
 * Macros
 -------------------------------------------------------------------------------*/
#define INVO_PSD_BOXPOINT_NUM               (4)
#define INVO_PSD_POINTSET_NUM               (8)
#define INVO_PSD_PARKSLOT_NUM               (14)

/*-------------------------------------------------------------------------------
 * Types
 -------------------------------------------------------------------------------*/
typedef enum
{
    INVO_PSD_TYPE_PARALLEL = 0,             /* parallel */
    INVO_PSD_TYPE_PERPEN,                   /* perpen */
    INVO_PSD_TYPE_PARA_SLANT,               /* slant no 90 degree*/
    INVO_PSD_TYPE_RECT_SLANT,               /* slant 90 degree*/
} INVO_PSD_TYPE;

typedef enum
{
    INVO_PSD_SIDE_NONE = 0,                 /* no side */
    INVO_PSD_SIDE_LEFT,                     /* on the left of vehicle */
    INVO_PSD_SIDE_RIGHT,                    /* on the right of vechile */
    INVO_PSD_SIDE_BOTH                      /* vechile is in the parkslot */
} INVO_PSD_SIDE;

typedef enum
{
    INVO_PSD_ST_INACTIVE = 0,
    INVO_PSD_ST_DETECTED,
    INVO_PSD_ST_TRACKING,
    INVO_PSD_ST_TRACING,
    INVO_PSD_ST_VANISHING
} INVO_PSD_STATUS;

typedef enum
{
    INVO_PSD_DIR_NONE = 0,
    INVO_PSD_DIR_IN,
    INVO_PSD_DIR_OUT
} INVO_PSD_DIRECT_TYPE;

typedef enum
{
    INVO_PSD_METHOD_INVALID = 0,
    INVO_PSD_METHOD_SEGMENTATION = 0x01,
    INVO_PSD_METHOD_CORNER = 0x02,
    INVO_PSD_METHOD_SEG_CORNER = 0x03
} INVO_PSD_METHOD_TYPE;

typedef enum
{
    INVO_PSD_AP_DETECTED = 0,
    INVO_PSD_AP_TRACKED,
    INVO_PSD_AP_ESTIMATED
} INVO_PSD_ANCHORPOINT_ST;

typedef struct
{
    int32_t                 iPointSetsNum;
    tInvoAlgImagePoint      box_point[INVO_PSD_BOXPOINT_NUM];   /* box-point position */
    INVO_PSD_ANCHORPOINT_ST box_state[INVO_PSD_BOXPOINT_NUM];   /* box-point state */
    tInvoAlgImagePoint      point_set[INVO_PSD_POINTSET_NUM];   /* point-set position */
} tInvoPsdImageInfo;

typedef struct
{
    int32_t                 iPointSetsNum;
    tInvoAlgLocalPoint      box_point[INVO_PSD_BOXPOINT_NUM];   /* box-point position */
    tInvoAlgLocalPoint      point_set[INVO_PSD_POINTSET_NUM];   /* box-point state */
} tInvoPsdLocalInfo;

typedef struct
{
    uint32_t                index;                              /* parkslot index, [0, INVO_PSD_PARKSLOT_NUM - 1] */
    i_bool                  valid;                              /* parkslot valid, TRUE:valid, FALSE:invalid */
    INVO_PSD_TYPE           type;                               /* parkslot type */
    INVO_PSD_SIDE           side;                               /* parkslot side */
    INVO_PSD_STATUS         state;                              /* parkslot state */
    INVO_PSD_DIRECT_TYPE    direct;                             /* parkslot direct */
    INVO_PSD_METHOD_TYPE    method;                             /* detect method */
    float                   confidence;                         /* [0.0, 1.0] */
    char                    identifier[100];                    /* English, Chinese, Digtal. */
    tInvoPsdImageInfo       image_info;                         /* parkslot position in image coordinate */
    tInvoPsdLocalInfo       local_info;                         /* parkslot position in local coordinate */
} tInvoPsdObj;

typedef struct
{
    bool                    ivalid;
    i_time                  timer;                              /* timestamp, unit ms */
    uint32_t                number;                             /* parkslot number */
    tInvoPsdObj             parkslots[INVO_PSD_PARKSLOT_NUM];   /* parkslot list */
} tInvoPsdObjList;

typedef struct
{
    i_time                  timer;                              /* timestamp, unit ms */
    uint32_t                index;                              /* parkslot index, [0, INVO_PSD_PARKSLOT_NUM - 1] */
    INVO_PSD_TYPE           type;                               /* parkslot type */
    INVO_PSD_SIDE           side;                               /* parkslot side */
    INVO_PSD_STATUS         state;                              /* parkslot state */
    INVO_PSD_DIRECT_TYPE    direct;                             /* parkslot direct */
    tInvoAlgLocalPoint      box_point[INVO_PSD_BOXPOINT_NUM];   /* box-point position */
} tInvoPsdSelect;

#endif /* INVO_ALG_PSD_TYPES_H_ */
