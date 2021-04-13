/*=======================================================================
*
*            Texas Instruments Internal Reference Software
*
*                           EP Systems Lab
*                    Perception & Analytics Lab R&D
*
*         Copyright (c) 2015 Texas Instruments, Incorporated.
*                        All Rights Reserved.
*
*
*          FOR TI INTERNAL USE ONLY. NOT TO BE REDISTRIBUTED.
*
*                 TI Confidential - Maximum Restrictions
*
*
*
*=======================================================================
*
*  File: lens_distortion_correction.c
*
=======================================================================*/

#include "TI/tivx_srv.h"
#include "lens_distortion_correction.h"
#include "srv_common.h"
#include <math.h>
#include <stdio.h>




 LDC_status LDC_UndistToDist(LensDistortionCorrection* ldc, dtype point_in[2], dtype point_out[2])
{
#if LDC_LIB_DATA_TYPE!=0 && LDC_LIB_DATA_TYPE!=1
        "LDC_LIB_DATA_TYPE must be 0 (float) or 1 (double) in lens_distortion_correction.h"
#endif
#if LDC_LIB_U2D_LUT_TYPE!=0 && LDC_LIB_U2D_LUT_TYPE!=1 && LDC_LIB_U2D_LUT_TYPE!=2
        "LDC_LIB_U2D_LUT_TYPE must be 0, 1 or 2"
#endif

        LDC_status status;
        dtype diffX, diffY;
        dtype lut_in_val;
        dtype lut_out_val;

        diffX = point_in[0] - ldc->distCenterX;
        diffY = point_in[1] - ldc->distCenterY;

#if LDC_LIB_U2D_LUT_TYPE == 0 || LDC_LIB_U2D_LUT_TYPE == 2
        dtype ru;
        #if LDC_LIB_DATA_TYPE==0 
                ru = sqrtf(diffX*diffX + diffY*diffY);
                lut_in_val = atanf(ru*ldc->distFocalLengthInv);
        #elif LDC_LIB_DATA_TYPE==1
                ru = sqrt(diffX*diffX + diffY*diffY);
                lut_in_val = atan(ru*ldc->distFocalLengthInv);
        #endif
#elif LDC_LIB_U2D_LUT_TYPE == 1
        #if LDC_LIB_DATA_TYPE==0 
                lut_in_val = sqrtf(diffX*diffX + diffY*diffY);
        #elif LDC_LIB_DATA_TYPE==1
                lut_in_val = sqrt(diffX*diffX + diffY*diffY);
        #endif
#endif

        lut_out_val = lut_lookup_floating(ldc->lut_u2d, lut_in_val, ldc->lut_u2d_indMax, ldc->lut_u2d_stepInv, &status);

#if LDC_LIB_U2D_LUT_TYPE == 0 || LDC_LIB_U2D_LUT_TYPE == 1
        point_out[0] = diffX * lut_out_val + ldc->distCenterX;
        point_out[1] = diffY * lut_out_val + ldc->distCenterY;
#elif LDC_LIB_U2D_LUT_TYPE == 2
        if (ru==0)
        {
                point_out[0] = ldc->distCenterX;
                point_out[1] = ldc->distCenterY;
        }
        else
        {
                point_out[0] = lut_out_val * diffX / ru + ldc->distCenterX;
                point_out[1] = lut_out_val * diffY / ru + ldc->distCenterY;
        }
#endif

        return status;
}


#if LDC_LIB_DATA_TYPE<2 
 dtype lut_lookup_floating(dtype *lut, dtype inval, int32_t indMax, dtype stepInv, LDC_status *status)
{
        *status = LDC_STATUS_OK;
        dtype ind = inval * stepInv;
        if (ind >= (dtype)indMax)
        {
                *status = LDC_STATUS_FAIL;
                return lut[indMax];
        }


#if LDC_LIB_INTERPOLATION_METHOD==0
        return lut[(int32_t)ind];
#elif LDC_LIB_INTERPOLATION_METHOD==1
        return lut[(int32_t)(ind + 0.5)];
#elif LDC_LIB_INTERPOLATION_METHOD==2
        int32_t N = (int32_t)ind;
        dtype indMinN = ind - (dtype)N;
        return (1.0f - indMinN)*lut[N] + indMinN * lut[N + 1];
#endif
}
#endif


