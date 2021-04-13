/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                            ****                                          |**
**|                            ****                                          |**
**|                            ******o***                                    |**
**|                      ********_///_****                                   |**
**|                      ***** /_//_/ ****                                   |**
**|                       ** ** (__/ ****                                    |**
**|                           *********                                      |**
**|                            ****                                          |**
**|                            ***                                           |**
**|                                                                          |**
**|         Copyright (c) 2007-2012 Texas Instruments Incorporated           |**
**|                        ALL RIGHTS RESERVED                               |**
**|                                                                          |**
**| Permission to use, copy, modify, or distribute this software,            |**
**| whether in part or in whole, for any purpose is forbidden without        |**
**| a signed licensing agreement and NDA from Texas Instruments              |**
**| Incorporated (TI).                                                       |**
**|                                                                          |**
**| TI makes no representation or warranties with respect to the             |**
**| performance of this computer program, and specifically disclaims         |**
**| any responsibility for any damages, special or consequential,            |**
**| connected with the use of this program.                                  |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/
#include "VXLIB_types.h"
#include "VXLIB_inlines.h"

#include "VXLIB_triangulatePoints_i32f_o32f_types.h"
#include "VXLIB_triangulatePoints_i32f_o32f_cn.h"

//#include "VXLIB_triangulatePoints_i32f_o32f.h"

static void makeTriangMatrix(VXLIB_F32 curTrack[],
                             VXLIB_F32 normTCamExtPrm[],
                             VXLIB_F32 matrixA[],
                             VXLIB_F32 matrixb[],
                             uint8_t curTrackLength[],
                             int32_t totalTracks);


static void findTriangWt(VXLIB_F32 outXcam[],
                         VXLIB_F32 normCamExtPrm[],
                         VXLIB_F32 weights[],
                         uint8_t curTrackLength[],
                         int32_t totalTracks);

static void solveMatEq3x3(VXLIB_F32 matAtAPtr[],
                          VXLIB_F32 matBPtr[],
                          VXLIB_F32 resultPtr[],
                          int32_t valid[],
                          int32_t totalTracks);

static void solveMatEq3x3Double(VXLIB_F32 matAtAPtr[],
                                VXLIB_F32 matBPtr[],
                                VXLIB_F32 resultPtr[],
                                int32_t valid[],
                                int32_t totalTracks);

static void solveMatEq3x3DoubleSelect(VXLIB_F32 matAtAPtr[],
                                      VXLIB_F32 matBPtr[],
                                      VXLIB_F32 resultPtr[],
                                      int32_t valid[],
                                      int32_t totalTracks);

static void getPseudoMatrices(VXLIB_F32 matrixA[],
                              VXLIB_F32 matrixb[],
                              VXLIB_F32 matrixP_AtA[],
                              VXLIB_F32 matrixP_Atb[],
                              int32_t totalTracks);

static void getFinalMatrices(VXLIB_F32 matrixP_AtA[],
                             VXLIB_F32 matrixP_Atb[],
                             VXLIB_F32 matrixAtA[],
                             VXLIB_F32 matrixAtb[],
                             VXLIB_F32 weight[],
                             int32_t totalTracks);


/**
* @func makeTriangMatrix
*
* @par Description:
*   This sub module forms initial data matrix for triangulation (Ax=b). Initial data
*   matrix are A & b. Size of data Matrix A is 2Nx3, and of b is 2Nx1, where
*   N is the total number of feature points in a given track. Two tracks are worked
*   together hence data matrix of two tracks are interleaved. For ease of access of
*   data, matrix A is formed in the form of 3x2N, and matrix b is formed in the
*   form of 1x2N. Maximum possible of feature points in a track is VXLIB_TRIANG_MAX_POINTS_IN_TRACK.
*   If a track doesn't have maximum number of feature points then some portion of
*   A & b remains un-initialized. Every feature point of track generates 2 columns
*   in matrix A & b. Storing the matrix A & b in transpose fashion also helps in
*   calculating AtA, and Atb.
*
* @par
*   @param [in]  curTrack      :Current track data after multiplying with intrinsic parameters.
*                               Two track data are clubbed together. x/y-coordinates of two
*                               tracks are placed together. Used in forming initial data matrices.
*                               Lets assume two tracks are {x0y0 x1y1 x2x2 x3y3 x4y4 x5y5} &
*                               {u0v0 u1v1 u2v2 u3v3 u4v4 u5v5}, then packed information is
*                               like {x0u0 y0v0 x1u1 y1v1 x2u2 y2v2 .. so on}
*   @param [in]  normCamExtPrm :Normalized projection matrix
*   @param [out] matrixA       :Output data matrix A of equation Ax=b
*   @param [out] matrixb       :Output data matrix b of equation Ax=b
*   @param [in]  curTrackLength:Array of values for track length
*   @param [in]  totalTracks   : Total number of tracks
*
* @par Assumptions:
*    -   NA
*
* @par Implementation Notes:
*    - NA
*
*/
CODE_SECTION(makeTriangMatrix,  ".text:ansi")
static void makeTriangMatrix(VXLIB_F32 curTrack[],
                             VXLIB_F32 normCamExtPrm[],
                             VXLIB_F32 matrixA[],
                             VXLIB_F32 matrixb[],
                             uint8_t curTrackLength[],
                             int32_t totalTracks)
{
    uint32_t    i, k, l;
    uint32_t    pitch = VXLIB_TRIANG_MAT_COL * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    VXLIB_F32   *normCamExtPrmL;
    uint32_t    curTrackLengthL;
    uint32_t    pmatrixIndx;

    for( l = 0; l < totalTracks; l += VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {

            curTrackLengthL = curTrackLength[l + k];
            pmatrixIndx    = VXLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthL;
            normCamExtPrmL = normCamExtPrm + pmatrixIndx * VXLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE;

            for( i = (VXLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthL); i < VXLIB_TRIANG_MAX_POINTS_IN_TRACK; i++ ) {

                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        0 * pitch + k]  =
                    ((curTrack[i * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + 0 + k] *
                      normCamExtPrmL[0]) -
                     normCamExtPrmL[1]);

                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        1 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR   +
                        0 * pitch + k]  =
                    ((curTrack[i * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                      normCamExtPrmL[0]) -
                     normCamExtPrmL[2]);

                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        1 * pitch + k]  =
                    ((curTrack[i * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + 0 + k] *
                      normCamExtPrmL[3]) -
                     normCamExtPrmL[4]);

                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        1 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR   +
                        1 * pitch + k]  =
                    ((curTrack[i * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                      normCamExtPrmL[3]) -
                     normCamExtPrmL[5]);

                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        2 * pitch + k]  =
                    ((curTrack[i * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + 0 + k] *
                      normCamExtPrmL[6]) -
                     normCamExtPrmL[8]);

                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        1 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR   +
                        2 * pitch + k]  =
                    ((curTrack[i * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                      normCamExtPrmL[6]) -
                     normCamExtPrmL[9]);

                matrixb[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k]   =
                    -((curTrack[i * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + 0 + k] *
                       normCamExtPrmL[7]) -
                      normCamExtPrmL[10]);

                matrixb[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k]    =
                    -((curTrack[i * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                       normCamExtPrmL[7]) -
                      normCamExtPrmL[11]);

                normCamExtPrmL += (VXLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE);

            }

            for( i=0; i < (VXLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthL); i++ ) {

                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        0 * pitch + k]              =  0;
                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        1 * pitch + k]              =  0;
                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        2 * pitch + k]              =  0;
                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        1 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR   +
                        0 * pitch + k]              =  0;
                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        1 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR   +
                        1 * pitch + k]              =  0;
                matrixA[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        1 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR   +
                        2 * pitch + k]              =  0;
                matrixb[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k]
                    =  0;

                matrixb[2 * i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR +
                        VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] = 0;
            }
        }

        curTrack += VXLIB_TRIANG_MAX_POINTS_IN_TRACK * 2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        matrixA += (VXLIB_TRIANG_MAT_ROW *
                    VXLIB_TRIANG_MAT_COL *
                    VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);

        matrixb += (VXLIB_TRIANG_MAT_COL *
                    VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
    }
}

/**
* @func findTriangWt
*
* @par Description:
*   This API Calculates weights for data matrices ( A & b) after each iteration of
*   triangulation. After weighting, AtA and Atb are again calculated to find new
*   refined 3D location of given track. Weights are also packed together for two tracks.
*
* @par
*   @param [in]  outXcam          :3-D output generated after triangulation API. two 3D outputs are
*                               clubbed together. X(or Y or Z) co-ordinates are clubbed together.
*                               e.g X1X0Y1Y0Z1Z0X3X2Y3Y2Z3Z2....so on
*   @param [in]  normCamExtPrm :Normalized projection matrix
*   @param [out] weights       :Output data matrix A of equation Ax=b
*   @param [in]  curTrackLength:Array of values for track length
*   @param [in]  totalTracks   :Total number of tracks
*
* @par Assumptions:
*    -   NA
*
* @par Implementation Notes:
*    - NA
*
*/
CODE_SECTION(findTriangWt,  ".text:ansi")
static void findTriangWt(VXLIB_F32 outXcam[],
                         VXLIB_F32 normCamExtPrm[],
                         VXLIB_F32 weights[],
                         uint8_t curTrackLength[],
                         int32_t totalTracks)
{
    int32_t     i, k, l;
    int32_t     pmatrixIndx;
    VXLIB_F32    W, oneByW;
    VXLIB_F32   *normCamExtPrmL;
    int32_t     curTrackLengthL;

    pmatrixIndx    = VXLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLength[0];
    normCamExtPrmL = normCamExtPrm + pmatrixIndx * VXLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE;

    for( l = 0; l < totalTracks; l += VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            curTrackLengthL = curTrackLength[l + k];
            pmatrixIndx    = VXLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthL;
            normCamExtPrmL = normCamExtPrm + pmatrixIndx * VXLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE;

            for( i = (VXLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthL); i < VXLIB_TRIANG_MAX_POINTS_IN_TRACK; i++ ) {

                W        = (normCamExtPrmL[12] * outXcam[0 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k]);
                W       += (normCamExtPrmL[13] * outXcam[1 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k]);
                W       += (normCamExtPrmL[14] * outXcam[2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k]);
                W       += (normCamExtPrmL[15] * 1.0f);
                /* Even if the change in weight is small then continue doing triangulation
                * dont break
                */
                oneByW                              = VXLIB_oneByXF32(W);
                weights[i * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] =  oneByW * oneByW;
                normCamExtPrmL        += VXLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE;
            }
        }

        weights += VXLIB_TRIANG_MAX_POINTS_IN_TRACK * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        outXcam += (VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR * 3);
    }
}

/**
* @func solveMatEq3x3
*
* @par Description:
*   This API finds the solution x for Ax = b. Solution used here is
*   based on normal inverse calculation and multiplying with b. All
*   data A & b are interleaved for two tracks.
*
* @par
*   @param [in]  matAtAPtr  : Matrix A of the equation Ax=b.
*   @param [in]  matBPtr    : Matrix b of the equation Ax=b.
*   @param [out] resultPtr  : Final result of solution x
*   @param [out] valid      : Validity of each 3D point generated. It is
*                             possible that matrix A might be singular in that case
*                             solution can not be found.
*   @param [in]  totalTracks: Total number of tracks
*
* @par Assumptions:
*    -   NA
*
* @par Implementation Notes:
*    - NA
*
*/
CODE_SECTION(solveMatEq3x3,  ".text:ansi")
static void solveMatEq3x3(VXLIB_F32 matAtAPtr[],
                          VXLIB_F32 matBPtr[],
                          VXLIB_F32 resultPtr[],
                          int32_t valid[],
                          int32_t totalTracks)
{
    uint32_t    inPitchA = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    inPitchB = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    outPitch = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    VXLIB_F32    det, invDet;
    VXLIB_F32    d1, d2, d3, d4, d5, d6;
    VXLIB_F32    a1, a2, a3, b2, b3, c3;
    VXLIB_F32    atB1, atB2, atB3;
    uint32_t    k, l;

    /* a1 a2 a3
    b2 b3
    c3
    */
    for( l = 0; l < totalTracks; l+=VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            a1    = matAtAPtr[0 * inPitchA + k];
            a2    = matAtAPtr[1 * inPitchA + k];
            a3    = matAtAPtr[2 * inPitchA + k];
            b2    = matAtAPtr[3 * inPitchA + k];
            b3    = matAtAPtr[4 * inPitchA + k];
            c3    = matAtAPtr[5 * inPitchA + k];

            atB1  = matBPtr[0 * inPitchB + k];
            atB2  = matBPtr[1 * inPitchB + k];
            atB3  = matBPtr[2 * inPitchB + k];

            d1    = b2 * c3 - b3 * b3;
            d2    = b3 * a3 - a2 * c3;
            d3    = a2 * b3 - b2 * a3;
            d4    = a1 * c3 - a3 * a3;
            d5    = a2 * a3 - a1 * b3;
            d6    = a1 * b2 - a2 * a2;

            det   = ((a1 * d1) + (a3 * d3)) + (a2 * d2);

            if( det <= FLT_EPSILON ) {
                valid[k] = 0;
            }

            invDet= VXLIB_oneByXF32(det);

            resultPtr[0 * outPitch + k] = invDet * (d1 * atB1 + (d2 * atB2 + d3 * atB3));
            resultPtr[1 * outPitch + k] = invDet * (d2 * atB1 + (d4 * atB2 + d5 * atB3));
            resultPtr[2 * outPitch + k] = invDet * (d3 * atB1 + (d5 * atB2 + d6 * atB3));

        }

        matAtAPtr  += VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        matBPtr    += VXLIB_TRIANG_MAT_ROW * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        resultPtr  += 3 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        valid      += VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    }
}

/**
* @func SFM_TI_solveMatEq3x3Double
*
* @par Description:
*   This API finds the solution x for Ax = b. Solution used here is
*   based on normal inverse calculation and multiplying with b. All
*   data A & b are interleaved for two tracks.
*
* @par
*   @param [in]  matAtAPtr  : Matrix A of the equation Ax=b.
*   @param [in]  matBPtr    : Matrix b of the equation Ax=b.
*   @param [out] resultPtr  : Final result of solution x
*   @param [out] valid      : Validity of each 3D point generated. It is
*                             possible that matrix A might be singular in that case
*                             solution can not be found.
*   @param [in]  totalTracks: Total number of tracks
*
* @par Assumptions:
*    -   NA
*
* @par Implementation Notes:
*    - NA
*
*/
CODE_SECTION(solveMatEq3x3Double,  ".text:ansi")
static void solveMatEq3x3Double(VXLIB_F32 matAtAPtr[],
                                VXLIB_F32 matBPtr[],
                                VXLIB_F32 resultPtr[],
                                int32_t valid[],
                                int32_t totalTracks)
{
    uint32_t    inPitchA = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    inPitchB = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    outPitch = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    VXLIB_D64    det, invDet;
    VXLIB_D64    d1, d2, d3, d4, d5, d6;
    VXLIB_D64    a1, a2, a3, b2, b3, c3;
    VXLIB_D64    atB1, atB2, atB3;
    uint32_t    k, l;

    /* a1 a2 a3
    b2 b3
    c3
    */
    for( l = 0; l < totalTracks; l+= VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            a1    = matAtAPtr[0 * inPitchA + k];
            a2    = matAtAPtr[1 * inPitchA + k];
            a3    = matAtAPtr[2 * inPitchA + k];
            b2    = matAtAPtr[3 * inPitchA + k];
            b3    = matAtAPtr[4 * inPitchA + k];
            c3    = matAtAPtr[5 * inPitchA + k];

            atB1  = matBPtr[0 * inPitchB + k];
            atB2  = matBPtr[1 * inPitchB + k];
            atB3  = matBPtr[2 * inPitchB + k];

            d1    = b2 * c3 - b3 * b3;
            d2    = b3 * a3 - a2 * c3;
            d3    = a2 * b3 - b2 * a3;
            d4    = a1 * c3 - a3 * a3;
            d5    = a2 * a3 - a1 * b3;
            d6    = a1 * b2 - a2 * a2;

            det   = (a1 * d1) + (a3 * d3) + (a2 * d2);

            if( det <= DBL_EPSILON ) {
                valid[k] = 0;
            }

            invDet= VXLIB_oneByXF32(det);

            resultPtr[0 * outPitch + k] = invDet * (d1 * atB1 + d2 * atB2 + d3 * atB3);
            resultPtr[1 * outPitch + k] = invDet * (d2 * atB1 + d4 * atB2 + d5 * atB3);
            resultPtr[2 * outPitch + k] = invDet * (d3 * atB1 + d5 * atB2 + d6 * atB3);

        }

        matAtAPtr  += VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        matBPtr    += VXLIB_TRIANG_MAT_ROW * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        resultPtr  += 3 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        valid      += VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    }
}

/**
* @func solveMatEq3x3DoubleSelect
*
* @par Description:
*   This API finds the solution x for Ax = b. Solution used here is
*   based on normal inverse calculation and multiplying with b. All
*   data A & b are interleaved for two tracks.
*
* @par
*   @param [in]  matAtAPtr  : Matrix A of the equation Ax=b.
*   @param [in]  matBPtr    : Matrix b of the equation Ax=b.
*   @param [out] resultPtr  : Final result of solution x
*   @param [out] valid      : Validity of each 3D point generated. It is
*                             possible that matrix A might be singular in that case
*                             solution can not be found.
*   @param [in]  totalTracks  : Total number of tracks
*
* @par Assumptions:
*    -   NA
*
* @par Implementation Notes:
*    - NA
*
*/
CODE_SECTION(solveMatEq3x3DoubleSelect,  ".text:ansi")
static void solveMatEq3x3DoubleSelect(VXLIB_F32 matAtAPtr[],
                                      VXLIB_F32 matBPtr[],
                                      VXLIB_F32 resultPtr[],
                                      int32_t valid[],
                                      int32_t totalTracks)
{
    uint32_t    inPitchA = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    inPitchB = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    outPitch = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    VXLIB_D64    det, invDet;
    VXLIB_D64    d1, d2, d3, d4, d5, d6;
    VXLIB_D64    a1, a2, a3, b2, b3, c3;
    VXLIB_D64    atB1, atB2, atB3;
    uint32_t    k, l;

    /* a1 a2 a3
    b2 b3
    c3
    */
    for( l = 0; l < totalTracks; l+=VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            if( valid[k] == 1 ) {
                continue;
            }

            a1    = matAtAPtr[0 * inPitchA + k];
            a2    = matAtAPtr[1 * inPitchA + k];
            a3    = matAtAPtr[2 * inPitchA + k];
            b2    = matAtAPtr[3 * inPitchA + k];
            b3    = matAtAPtr[4 * inPitchA + k];
            c3    = matAtAPtr[5 * inPitchA + k];

            atB1  = matBPtr[0 * inPitchB + k];
            atB2  = matBPtr[1 * inPitchB + k];
            atB3  = matBPtr[2 * inPitchB + k];

            d1    = b2 * c3 - b3 * b3;
            d2    = b3 * a3 - a2 * c3;
            d3    = a2 * b3 - b2 * a3;
            d4    = a1 * c3 - a3 * a3;
            d5    = a2 * a3 - a1 * b3;
            d6    = a1 * b2 - a2 * a2;

            det   = (a1 * d1) + (a3 * d3) + (a2 * d2);

            if( det <= DBL_MIN ) {
                valid[k] = 0x0;
            } else {
                valid[k] = 0xffffffff;
            }

            invDet= VXLIB_oneByXF32(det);

            resultPtr[0 * outPitch + k] = invDet * (d1 * atB1 + d2 * atB2 + d3 * atB3);
            resultPtr[1 * outPitch + k] = invDet * (d2 * atB1 + d4 * atB2 + d5 * atB3);
            resultPtr[2 * outPitch + k] = invDet * (d3 * atB1 + d5 * atB2 + d6 * atB3);

        }

        matAtAPtr  += VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        matBPtr    += VXLIB_TRIANG_MAT_ROW * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        resultPtr  += 3 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
        valid      += VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    }
}

/**
* @func getPseudoMatrices
*
* @par Description:
*   This API finds the partial solution of AtA, and Atb
*
* @par
*   @param [in]  matrixA      : Original data matrix A
*   @param [in]  matrixb      : Original data matrix b
*   @param [out] matrixP_AtA  : Partial calculated AtA
*   @param [out] matrixP_Atb  : Partial calculated Atb
*   @param [in]  totalTracks  : Total number of tracks
*
*
*
* @par Assumptions:
*    -   NA
*
* @par Implementation Notes:
*    -   NA
*
*/
CODE_SECTION(getPseudoMatrices,  ".text:ansi")
static void getPseudoMatrices(VXLIB_F32 matrixA[],
                              VXLIB_F32 matrixb[],
                              VXLIB_F32 matrixP_AtA[],
                              VXLIB_F32 matrixP_Atb[],
                              int32_t totalTracks)
{
    int32_t    inPitchA  = 2 * VXLIB_TRIANG_MAX_POINTS_IN_TRACK * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    int32_t    outPitchA = VXLIB_TRIANG_MAX_POINTS_IN_TRACK * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    int32_t    outPitchB = VXLIB_TRIANG_MAX_POINTS_IN_TRACK * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    int32_t     rowPair[2 * VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA] = { 0, 0, 0, 1, 0, 2, 1, 1, 1, 2, 2, 2 };
    int32_t     row_i, row_j;
    VXLIB_F32    acc;

    uint32_t    i = 0;
    uint32_t    j = 0;
    uint32_t    k = 0;
    uint32_t    l = 0;

    for( l = 0; l < totalTracks; l += VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            for( i = 0; i < VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA; i++ ) {

                row_i = rowPair[2 * i];
                row_j = rowPair[2 * i + 1];
                acc   = 0;

                for( j = 0; j < VXLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {

                    acc  =  matrixA[row_i * inPitchA + 2 * j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                           matrixA[row_j * inPitchA + 2 * j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k];

                    acc +=  matrixA[row_i * inPitchA + (2 * j + 1) * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                           matrixA[row_j * inPitchA + (2 * j + 1) * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k];

                    matrixP_AtA[i * outPitchA + j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] = acc;
                }
            }

            for( i = 0; i < 3; i++ ) {
                for( j = 0; j < VXLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {
                    acc  =  matrixA[i * inPitchA + 2 * j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                           matrixb[2 * j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k];

                    acc +=  matrixA[i * inPitchA + (2 * j + 1) * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                           matrixb[(2 * j + 1) * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k];

                    matrixP_Atb[i * outPitchB + j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] = acc;
                }
            }
        }

        matrixA     += VXLIB_TRIANG_MAT_ROW *
                       VXLIB_TRIANG_MAT_COL *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

        matrixb     += VXLIB_TRIANG_MAT_COL *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

        matrixP_AtA += VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR *
                       VXLIB_TRIANG_MAX_POINTS_IN_TRACK;

        matrixP_Atb += VXLIB_TRIANG_MAT_ROW *
                       VXLIB_TRIANG_MAX_POINTS_IN_TRACK *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    }
}

/**
* @func getFinalMatrices
*
* @par Description:
*   This API finds actual AtA , Atb from partial calculated
*   AtA & Atb
*
* @par
*   @param [in]   matrixP_AtA      : Partially calculated AtA
*   @param [in]   matrixP_Atb      : Partially calculated Atb
*   @param [out]  matrixAtA        : Final calculated AtA
*   @param [out]  matrixAtb        : Final Calculated Atb
*   @param [in]   weight           : Weights for scaling matrixP_AtA,matrixP_Atb
*                                    before processing
*   @param [in]  totalTracks       : Total number of tracks
*
*
*
* @par Assumptions:
*    -   NA
*
* @par Implementation Notes:
*    -   NA
*
*/
CODE_SECTION(getFinalMatrices,  ".text:ansi")
static void getFinalMatrices(VXLIB_F32 *matrixP_AtA,
                             VXLIB_F32 *matrixP_Atb,
                             VXLIB_F32 *matrixAtA,
                             VXLIB_F32 *matrixAtb,
                             VXLIB_F32 *weight,
                             int32_t totalTracks)
{
    uint32_t    inPitchA = VXLIB_TRIANG_MAX_POINTS_IN_TRACK * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    inPitchB = VXLIB_TRIANG_MAX_POINTS_IN_TRACK * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    uint32_t    outPitchA = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    outPitchB = VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    uint32_t    i, j, k, l;
    VXLIB_F32    acc;

    for( l = 0; l < totalTracks; l += VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            for( i = 0; i < VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA; i++ ) {
                acc = 0;

                for( j = 0; j < VXLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {
                    acc +=  matrixP_AtA[i * inPitchA + j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                           weight[j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k];
                }

                matrixAtA[i * outPitchA + k] = acc;
            }

            for( i = 0; i < 3; i++ ) {
                acc = 0;

                for( j = 0; j < VXLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {
                    acc +=  matrixP_Atb[i * inPitchB + j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k] *
                           weight[j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k];
                }

                matrixAtb[i * outPitchB + k] = acc;
            }
        }

        weight      += VXLIB_TRIANG_MAX_POINTS_IN_TRACK *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

        matrixAtA   += VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

        matrixAtb   += VXLIB_TRIANG_MAT_ROW *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

        matrixP_AtA += VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR *
                       VXLIB_TRIANG_MAX_POINTS_IN_TRACK;

        matrixP_Atb += VXLIB_TRIANG_MAT_ROW *
                       VXLIB_TRIANG_MAX_POINTS_IN_TRACK *
                       VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    }
}


CODE_SECTION(VXLIB_triangulatePoints_i32f_o32f_cn,  ".text:ansi")
VXLIB_STATUS VXLIB_triangulatePoints_i32f_o32f_cn(VXLIB_F32 curTrack[],
                                             VXLIB_F32 camExtPrm[],
                                             uint8_t   curTrackLength[],
                                             uint32_t  maxIter,
                                             VXLIB_F32 scratch[],
                                             uint32_t  totalTracks,
                                             uint32_t  precisionFlag,
                                             VXLIB_F32 outXcam[],
                                             int32_t   outValid[])
{
    uint32_t    i, j, l, k;
    uint8_t     maxLength;

    VXLIB_F32   *matrixA;   /*[VXLIB_TRIANG_MAT_ROW][VXLIB_TRIANG_MAT_COL][VXLIB_TRIANG_NUM_TRACKS_TOGATHER]*/
    VXLIB_F32   *matrixb;   /*[VXLIB_TRIANG_MAT_COL][VXLIB_TRIANG_NUM_TRACKS_TOGATHER]*/
    VXLIB_F32   *matrixP_AtA; /*[VXLIB_TRIANG_NUM_TRACKS_TOGATHER][VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA*VXLIB_TRIANG_MAX_POINTS_IN_TRACK]*/
    VXLIB_F32   *matrixP_Atb; /*[VXLIB_TRIANG_NUM_TRACKS_TOGATHER][VXLIB_TRIANG_MAT_ROW*VXLIB_TRIANG_MAX_POINTS_IN_TRACK]*/

    VXLIB_F32   *matrixAtA; /*[VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA][VXLIB_TRIANG_NUM_TRACKS_TOGATHER]*/
    VXLIB_F32   *matrixAtb; /*[VXLIB_TRIANG_MAT_AROW][VXLIB_TRIANG_NUM_TRACKS_TOGATHER]*/
    VXLIB_F32   *weights;   /*[VXLIB_TRIANG_MAX_POINTS_IN_TRACK][VXLIB_TRIANG_NUM_TRACKS_TOGATHER]*/

    VXLIB_STATUS    status = VXLIB_SUCCESS;
#if defined(VXLIB_CHECK_PARAMS)
    status = VXLIB_triangulatePoints_i32f_o32f_checkParams(curTrack, camExtPrm, curTrackLength, maxIter, scratch, totalTracks, precisionFlag, outXcam, outValid);
    if( status == VXLIB_SUCCESS )
#endif
    {

    matrixA     = scratch;
    matrixb     = (VXLIB_F32 *)matrixA + (VXLIB_TRIANG_MAT_ROW * VXLIB_TRIANG_MAT_COL * totalTracks);
    matrixP_AtA = (VXLIB_F32 *)matrixb + (VXLIB_TRIANG_MAT_COL * totalTracks);
    matrixP_Atb = (VXLIB_F32 *)matrixP_AtA + (VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * VXLIB_TRIANG_MAX_POINTS_IN_TRACK * totalTracks);

    matrixAtA   = (VXLIB_F32 *)scratch;
    matrixAtb   = (VXLIB_F32 *)matrixAtA + (VXLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * totalTracks);
    weights     = (VXLIB_F32 *)matrixAtb + (VXLIB_TRIANG_MAX_POINTS_IN_TRACK * totalTracks);


    makeTriangMatrix((VXLIB_F32 *)curTrack,
                     (VXLIB_F32 *)camExtPrm,
                     (VXLIB_F32 *)matrixA,
                     matrixb,
                     curTrackLength,
                     totalTracks);

    getPseudoMatrices(matrixA,
                      matrixb,
                      matrixP_AtA,
                      matrixP_Atb,
                      totalTracks);

    for( l = 0; l < totalTracks; l+= VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            outValid[l + k] = 0x1;
            maxLength      = curTrackLength[l + k];

            for( j = 0; j < VXLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {
                if( j >= (VXLIB_TRIANG_MAX_POINTS_IN_TRACK - maxLength)) {
                    weights[(l * VXLIB_TRIANG_MAX_POINTS_IN_TRACK) + ((j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 1.0;
                } else {
                    weights[(l * VXLIB_TRIANG_MAX_POINTS_IN_TRACK) + ((j * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 0.0;
                }
            }
        }
    }

    for( i = 0; i < maxIter; i++ ) {
        getFinalMatrices(matrixP_AtA,
                         matrixP_Atb,
                         matrixAtA,
                         matrixAtb,
                         weights,
                         totalTracks);
        if( precisionFlag == 0x0 ) {
            solveMatEq3x3(matrixAtA,
                          matrixAtb,
                          outXcam,
                          outValid,
                          totalTracks);

            solveMatEq3x3DoubleSelect(matrixAtA,
                                      matrixAtb,
                                      outXcam,
                                      outValid,
                                      totalTracks);

        } else {
            solveMatEq3x3Double(matrixAtA,
                                matrixAtb,
                                outXcam,
                                outValid,
                                totalTracks);
        }

        findTriangWt(outXcam,
                     camExtPrm,
                     weights,
                     curTrackLength,
                     totalTracks);
    }

    for( l = 0; l < totalTracks; l+= VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            if( outValid[l + k] == 0 ) {
                outXcam[(l * 3) + ((0 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 0.0;
                outXcam[(l * 3) + ((1 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 0.0;
                outXcam[(l * 3) + ((2 * VXLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 0.0;
            }
        }
    }
   }
    return (status);
}

/* ======================================================================== */
/*  End of file:  VXLIB_triangulatePoints_cn.c                    */
/* ======================================================================== */

