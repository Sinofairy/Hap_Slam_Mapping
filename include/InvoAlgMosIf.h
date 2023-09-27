/*  BEGIN_FILE_HDR
************************************************************************************************
*   NOTICE
*   This software is the property of INVO Co.,Ltd.. Any information contained in this
*   doc should not be reproduced, or used, or disclosed without the written authorization from
*   INVO Co.,Ltd..
************************************************************************************************
*   File Name       : InvoAlgMosIf.h
************************************************************************************************
*   END_FILE_HDR*/
#ifndef INVO_ALG_MOS_IF_H_
#define INVO_ALG_MOS_IF_H_

/*-------------------------------------------------------------------------------
 * Includes
 -------------------------------------------------------------------------------*/
#include "InvoAlgMosTypes.h"

/*-------------------------------------------------------------------------------
 * Functions
 -------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

i_errno InvoMos_Init(                               /* function : init variables and malloc resource */
    tInvoAlgVehicleParam const *vehicle_para,       /* input    : vehicle parameters */
    tInvoAlgVersion            *version             /* output   : version */
    );                                              /* return   : [0] success, [others] failed reason */

i_errno InvoMos_Task(                               /* function : execute post process */
    i_time                      timer,              /* input    : source image timestamp(ms) */
    uint16_t const             *mos_tensor,         /* input    : mos tensor list */
    tInvoAlgAnchorList const   *pcd_anchor,         /* input    : pcd anchor list */
    tInvoAlgSignals const      *signal,             /* input    : vehicle signals */
    tInvoMosObjList            *objects             /* output   : object list */
    );                                              /* return   : [0] success, [others] failed reason */

i_errno InvoMos_DeInit(                             /* function : deinit variables and  resource */
    void                                            /* input    : no parameter */
    );                                              /* return   : [0] success, [others] failed reason */

#ifdef __cplusplus
}
#endif

#endif /* INVO_ALG_MOS_IF_H_ */
