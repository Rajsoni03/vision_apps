/*
*
* Copyright (c) 2017 Texas Instruments Incorporated
*
* All rights reserved not granted herein.
*
* Limited License.
*
* Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
* license under copyrights and patents it now or hereafter owns or controls to make,
* have made, use, import, offer to sell and sell ("Utilize") this software subject to the
* terms herein.  With respect to the foregoing patent license, such license is granted
* solely to the extent that any such patent is necessary to Utilize the software alone.
* The patent license shall not apply to any combinations which include this software,
* other than combinations with devices manufactured by or for TI ("TI Devices").
* No hardware patent is licensed hereunder.
*
* Redistributions must preserve existing copyright notices and reproduce this license
* (including the above copyright notice and the disclaimer and (if applicable) source
* code license limitations below) in the documentation and/or other materials provided
* with the distribution
*
* Redistribution and use in binary form, without modification, are permitted provided
* that the following conditions are met:
*
* *       No reverse engineering, decompilation, or disassembly of this software is
* permitted with respect to any software provided in binary form.
*
* *       any redistribution and use are licensed by TI for use only with TI Devices.
*
* *       Nothing shall obligate TI to provide you with source code for the software
* licensed and provided to you in object code.
*
* If software source code is provided to you, modification and redistribution of the
* source code are permitted provided that the following conditions are met:
*
* *       any redistribution and use of the source code, including any resulting derivative
* works, are licensed by TI for use only with TI Devices.
*
* *       any redistribution and use of any object code compiled from the source code
* and any resulting derivative works, are licensed by TI for use only with TI Devices.
*
* Neither the name of Texas Instruments Incorporated nor the names of its suppliers
*
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* DISCLAIMER.
*
* THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
* OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


#include "surround_sfm_ogmap_priv.h"

#include <stdio.h>

PTK_INS_RetCode PTK_INS_getIMUPosesFromVelAndAtt(uint64_t *timestamps, uint32_t num, PTK_RigidTransform *M_in_i0)
{
    uint32_t tt;
    PTK_INS_RetCode retCode;
    PTK_INS_Record record_prv;
    PTK_INS_Record record_cur;
    PTK_RigidTransform R_enu_iprv;
    PTK_RigidTransform R_enu_icur;
    PTK_RigidTransform R_iprv_enu;
    PTK_RigidTransform R_icur_enu;
    PTK_RigidTransform M_iprv_icur;
    PTK_RigidTransform M_icur_iprv;
    PTK_RigidTransform R_iprv_icur;

    float deltaT;

    if (num == 0)
    {
        return PTK_INS_RETURN_CODE_OK;
    }

    if (num == 1)
    {
        PTK_RigidTransform_makeIdentity(M_in_i0);
        return PTK_INS_RETURN_CODE_OK;
    }

    PTK_RigidTransform_makeIdentity(M_in_i0);
    retCode = PTK_INS_getRecordLinearInterp(PTK_INS_RECORD_TYPE_INSPVA, timestamps[0], &record_prv);
    if (retCode != PTK_INS_RETURN_CODE_OK)
        return retCode;


    PTK_INS_getIMUtoENU(&record_prv, &R_enu_iprv);

    PTK_RigidTransform_invert(&R_iprv_enu, &R_enu_iprv);

    for (tt = 1; tt < num; tt++)
    {
        //current record
        retCode = PTK_INS_getRecordLinearInterp(PTK_INS_RECORD_TYPE_INSPVA, timestamps[tt], &record_cur);
        if (retCode != PTK_INS_RETURN_CODE_OK)
            return retCode;

        if (timestamps[tt] <= timestamps[tt - 1])
            return PTK_INS_RETURN_CODE_ILLEGAL_ARGIN;

        deltaT = (float)(timestamps[tt] - timestamps[tt - 1])*1e-6f;

        PTK_INS_getIMUtoENU(&record_cur, &R_enu_icur);
        PTK_RigidTransform_invert(&R_icur_enu, &R_enu_icur);

        //rotation given by IMUtoENU at prv and current
        PTK_RigidTransform_compose(&R_iprv_icur, &R_iprv_enu, &R_enu_icur); //assuming ENU at cur and prv is the same
        M_iprv_icur = R_iprv_icur;

        //translation such that origin of cur in prv frame is v*deltaT
        PTK_Vector v_enu, v_iprv;//velocities
        PTK_Point_set(&v_enu, record_prv.data.inspva.velocity.east, record_prv.data.inspva.velocity.north, record_prv.data.inspva.velocity.up);
        PTK_Point_transform(&v_iprv, &v_enu, &R_iprv_enu);
        PTK_RigidTransform_setTranslation(&M_iprv_icur, v_iprv.x*deltaT, v_iprv.y*deltaT, v_iprv.z*deltaT);


        //update
        PTK_RigidTransform_invert(&M_icur_iprv, &M_iprv_icur);
        PTK_RigidTransform_compose(&M_in_i0[tt], &M_icur_iprv, &M_in_i0[tt-1]);

        record_prv = record_cur;
        R_enu_iprv = R_enu_icur;
        R_iprv_enu = R_icur_enu;
    }

    return PTK_INS_RETURN_CODE_OK;
}
