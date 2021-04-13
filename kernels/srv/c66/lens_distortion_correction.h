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
*  File: lens_distortion_correction.h
*
=======================================================================*/

#ifndef LENS_DISTORTION_CORRECTION_INCLUDED
#define LENS_DISTORTION_CORRECTION_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif


        /*user parameters*/
#define LDC_LIB_U2D_LUT_TYPE            2       /* 0: theta[rad]->rd/ru, 1: ru->rd/ru, 2: theta[rad]->rd */
#define LDC_LIB_D2U_LUT_TYPE            0       /* 0: rdSquared->ru/rd */
#define LDC_LIB_INTERPOLATION_METHOD    2       /* 0: 'previous', 1: 'nearest', 2:'linear' */
#define MAX_FP_ALL                   80
#define CALMAT_DUMMY                 (128 - ((MAX_INPUT_CAMERAS*4)+4))

#define BINARIZE_OFFSET_SMALL_WINDOW     50
#define MIN_MAX_CHART_RATIO   5   // in svCheckAllDirections()
#define MIN_CHART_SIZE        20  // in svCheckPatternProfileHiDia()
#define HIGH_MID_TH           5.0 // in svCheckPatternProfileHiDia()
#define LOW_MID_TH            0.2 // in svCheckPatternProfileHiDia()
#define SIDE_TH               4.0 // in svCheckPatternProfileHiDia()

#define MAX_HOR_LENGTH_RATIO  2.5 // in svCheckFPCondition()
#define MAX_VER_LENGTH_RATIO  2.5 // in svCheckFPCondition()
#define MAX_DIA_LENGTH_RATIO  5.0 // in svCheckFPCondition()
#define MIN_HOR_LENGTH        15
#define MIN_VER_LENGTH        30
#define MIN_CHART_LENGTH      150
#define NUM_PT_BOUNDARY       128
//#define DBL_MAX               0x1.fffffffffffffp1023


/* The below struct is for reading the LENS.BIN file */
#if 1
typedef struct
{
    int32_t ldcLUT_numCams;
    /**< Num of cameras */
    int32_t ldcLUT_distortionCenters[2*LDC_MAX_NUM_CAMERAS];
    /**< Num of Lens Distortion centres */
    float ldcLUT_focalLength;
    /**< Lens focal length */

    int32_t U2D_type;
    /**< Lens Undistort to Distort type (must match macro LDC_LIB_U2D_LUT_TYPE)*/
    int32_t ldcLUT_U2D_length;
    /**< Lens Undistort to Distort length */
    float ldcLUT_U2D_step;
    /**< Lens Undistort to Distort step */
    float ldcLUT_U2D_table[LDC_U2D_TABLE_MAX_LENGTH];
    /**< Lens Undistort to Distort table */

    int32_t D2U_type;
    /**< Lens Distort to Undistort type (must match macro LDC_LIB_D2U_LUT_TYPE)*/
    int32_t ldcLUT_D2U_length;
    /**< Lens Distort to Undistort length */
    float ldcLUT_D2U_step;
    /**< Lens Distort to Undistort step */
    float ldcLUT_D2U_table[LDC_D2U_TABLE_MAX_LENGTH];
    /**< Lens Distort to Undistort table */
}ldc_lensParameters;
#endif

/*================================================
Data Structure
================================================== */

/*status flag*/
typedef enum {
        LDC_STATUS_OK,
        LDC_STATUS_FAIL
} LDC_status;

/*main struct*/

 LDC_status LDC_DistToUndist(LensDistortionCorrection* ldc, dtype point_in[2], dtype point_out[2], dtype *rdSq_out);
 dtype lut_lookup_floating(dtype *lut, dtype inval, int32_t indMax, dtype stepInv, LDC_status *status);


#endif
