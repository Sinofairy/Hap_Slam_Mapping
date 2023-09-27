/*  BEGIN_FILE_HDR
************************************************************************************************
*   NOTICE
*   This software is the property of INVO Co.,Ltd.. Any information contained in this
*   doc should not be reproduced, or used, or disclosed without the written authorization from
*   INVO Co.,Ltd..
************************************************************************************************
*   File Name       : InvoAlgSlamIf.h
************************************************************************************************
*   END_FILE_HDR*/
#ifndef INVO_ALG_SLAM_IF_H_
#define INVO_ALG_SLAM_IF_H_

/*-------------------------------------------------------------------------------
 * Includes
 -------------------------------------------------------------------------------*/
#include <deque>
#include <string>
#include "InvoAlgCommon.h"
#include "InvoAlgFmTypes.h"
#include "InvoAlgSlamTypes.h"

/*-------------------------------------------------------------------------------
 * NameSpace
 -------------------------------------------------------------------------------*/
using namespace std;

/*-------------------------------------------------------------------------------
 * Functions
 -------------------------------------------------------------------------------*/
i_errno InvoSlam_Init(                                                      /* function : init variables */
            const tInvoAlgVehicleParam              &vehicle_para,          /* input    : vehicle parameters */
            const string                            &path,                  /* input    : root path */
            const string                            &index_file,            /* input    : index file name */
            const string                            &config_file,           /* input    : config file path */
            tInvoSlamMapList                        &map_list,              /* output   : map info list */
            tInvoAlgVersion                         &version                /* output   : version */
            );                                                              /* return   : [0] success, [others] failed reason */
                                                                           
i_errno InvoSlam_TaskSearching(                                             /* function : search mapped parking place */
            const tInvoAlgGpsData                   &gps,                   /* input    : gps signal */
            deque<tInvoSlamMapId>                   &matched_map_id         /* output   : matched map id list */
            );                                                              /* return   : [0] success, [others] failed reason */

i_errno InvoSlam_TaskLearning(                                              /* function : Learning */
            const deque<tInvoAlgWheel>              &wheel,                 /* input    : wheel signal */
            const deque<tInvoAlgImuData>            &imu,                   /* input    : imu signal */
            const deque<tInvoAlgGpsData>            &gps,                   /* input    : gps signal */
            const tInvoAlgGears                     &gear,                  /* input    : current gear signal */
            const deque<tInvoFmOdList>              &od_list,               /* input    : od perception info */
            const deque<tInvoFmPsdList>             &psd_list,              /* input    : psd perception info */
            const deque<tInvoFmLdList>              &line_list,             /* input    : line perception info */
            const deque<tInvoFmMosList>             &mos_list,              /* input    : mos perception info */
            tInvoSlamObjectList                     &objects,               /* output   : object info */
            tInvoSlamRoadList                       &roads,                 /* output   : road info */
            tInvoSlamTravelInfo                     &travel_info,           /* output   : travel information */
            tInvoSlamWorldPose                      &world_pose,            /* output   : pose in world */
            i_bool                                  &is_parking_done        /* output   : [0] still parking [1] parking done */
            );                                                              /* return   : [0] success, [others] failed reason */

i_errno InvoSlam_TaskLocating(                                              /* function : Locating */
            const tInvoSlamMapId                    &map_id,                /* input    : map id */
            const deque<tInvoAlgWheel>              &wheel,                 /* input    : wheel signal */
            const deque<tInvoAlgImuData>            &imu,                   /* input    : imu signal */
            const deque<tInvoAlgGpsData>            &gps,                   /* input    : gps signal */
            const tInvoAlgGears                     &gear,                  /* input    : current gear signal */
            const deque<tInvoFmOdList>              &od_list,               /* input    : od perception info */
            const deque<tInvoFmPsdList>             &psd_list,              /* input    : psd perception info */
            const deque<tInvoFmLdList>              &line_list,             /* input    : line perception info */
            const deque<tInvoFmMosList>             &mos_list,              /* input    : mos perception info */
            tInvoSlamObjectList                     &objects,               /* output   : object info */
            tInvoSlamRoadList                       &roads,                 /* output   : road info */
            tInvoSlamTravelInfo                     &travel_info,           /* output   : travel information */
            tInvoSlamWorldPose                      &world_pose,            /* output   : pose in world */
            tInvoSlamImagePose                      &image_pose,            /* output   : pose in image */
            i_bool                                  &is_parking_done        /* output   : [0] still parking [1] parking done */
            );                                                              /* return   : [0] success, [others] failed reason */

i_errno InvoSlam_TaskComplete(                                              /* function : complete last process and close map file */
            const i_bool                            &confirm,               /* input    : used for mapping, TRUE-confirm, FALSE-cancel */
            tInvoSlamMapList                        &map_list               /* output   : map list */
            );                                                              /* return   : [0] success, [others] failed reason */

i_errno InvoSlam_DeInit(                                                    /* function : deinit variables and  resource */
            void                                                            /* input    : no parameter */
            );                                                              /* return   : [0] success, [others] failed reason */

#endif /* INVO_ALG_SLAM_IF_H_ */
