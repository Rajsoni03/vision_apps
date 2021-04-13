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

#include "float.h"
#include "VLIB_internal.h"
#include "VLIB_utility.h"
#include "VLIB_triangulatePoints.h"
#ifdef VLIB_PROFILE_EN
#include "../../common/VLIB_profile.h"
#endif


static void makeTriangMatrix_ci(VLIB_F32 curTrack[restrict],
                                VLIB_F32 normCamExtPrm[restrict],
                                VLIB_F32 matrixA[restrict],
                                VLIB_F32 matrixb[restrict],
                                uint8_t curTrackLength[restrict],
                                int32_t totalTracks);

static void findTriangWt_ci(VLIB_F32 Xcam[restrict],
                            VLIB_F32 normCamExtPrm[restrict],
                            VLIB_F32 weights[restrict],
                            uint8_t curTrackLength[restrict],
                            int32_t totalTracks);

static void solve3x3MatEq_ci(VLIB_F32 matAtAPtr[restrict],
                             VLIB_F32 matBPtr[restrict],
                             VLIB_F32 resultPtr[restrict],
                             uint8_t valid[restrict],
                             int32_t totalTracks);

static void solve3x3MatEqDouble(VLIB_F32 matAtAPtr[restrict],
                                VLIB_F32 matBPtr[restrict],
                                VLIB_F32 resultPtr[restrict],
                                uint8_t valid[restrict],
                                int32_t totalTracks);

static void solve3x3MatEqDoubleSelect(VLIB_F32 matAtAPtr[restrict],
                                      VLIB_F32 matBPtr[restrict],
                                      VLIB_F32 resultPtr[restrict],
                                      uint8_t valid[restrict],
                                      int32_t totalTracks);

static void getPseudoMatrices_ci(VLIB_F32 matrixA[restrict],
                                 VLIB_F32 matrixb[restrict],
                                 VLIB_F32 matrixP_AtA[restrict],
                                 VLIB_F32 matrixP_Atb[restrict],
                                 uint8_t curTrackLen[restrict],
                                 int32_t totalTracks);

static void getFinalMatrices_ci(VLIB_F32 matrixP_AtA[restrict],
                                VLIB_F32 matrixP_Atb[restrict],
                                VLIB_F32 matrixAtA[restrict],
                                VLIB_F32 matrixAtb[restrict],
                                VLIB_F32 weight[restrict],
                                int32_t totalTracks);
#ifdef VLIB_PROFILE_EN
uint64_t t1, t0, acc0, acc1, acc2, acc3, acc4,acc5,acc6;
#endif


/**
* @func makeTriangMatrix_ci
*
* @par Description:
*   This sub module forms initial data matrix for triangulation (Ax=b). Initial data
*   matrix are A & b. Size of data Matrix A is 2Nx3, and of b is 2Nx1, where
*   N is the total number of feature points in a given track. Two tracks are worked
*   together hence data matrix of two tracks are interleaved. For ease of access of
*   data, matrix A is formed in the form of 3x2N, and matrix b is formed in the
*   form of 1x2N. Maximum possible of feature points in a track is VLIB_TRIANG_MAX_POINTS_IN_TRACK.
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
CODE_SECTION(makeTriangMatrix_ci,  ".text:optimized")
static void makeTriangMatrix_ci(VLIB_F32 curTrack[restrict],
                                VLIB_F32 normCamExtPrm[restrict],
                                VLIB_F32 matrixA[restrict],
                                VLIB_F32 matrixb[restrict],
                                uint8_t curTrackLength[restrict],
                                int32_t totalTracks)
{
    int32_t               i, l, k;
    VLIB_F32             *normCamExtPrmL;
    int32_t               curTrackLengthL0, curTrackLengthL1, curTrackLengthLMax;
    int32_t               pmatrixIndx;
    __float2_t            cur_x1x0, cur_y1y0;
    __float2_t            f2temp1, f2temp2, f2temp3;
    __float2_t            P0_P8, P9_P4, P5_P1, P11_P10, P6_P2, P7_P3;
    int32_t               pitch            = VLIB_TRIANG_MAT_COL * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    VLIB_F32 *restrict    curTrackL = curTrack;
    VLIB_F32 *restrict    matrixAL  = matrixA;
    VLIB_F32 *restrict    matrixbL  = matrixb;

    for( l = 0; l < totalTracks; l += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {

        curTrackLengthL0   = (int32_t)curTrackLength[l + 0];
        curTrackLengthL1   = (int32_t)curTrackLength[l + 1];

        curTrackLengthLMax = max(curTrackLengthL0, curTrackLengthL1);
        curTrackLengthLMax = min(curTrackLengthLMax, VLIB_TRIANG_MAX_POINTS_IN_TRACK);

        pmatrixIndx    = VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthLMax;
        normCamExtPrmL = normCamExtPrm + (pmatrixIndx * VLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE);
        curTrackL      = curTrack;
        matrixAL       = matrixA;
        matrixbL       = matrixb;

        /* In track pair, smaller track will be treated as same length as of larger track.
         * So for smaller track some of the data generated will be dummy. Which will taken
         * care separately while forming final data matrix.
         */
        curTrackL     += ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthLMax) * 2 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
        matrixAL      += ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthLMax) * 2 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
        matrixbL      += ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthLMax) * 2 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);

        for( i = 0; i < curTrackLengthLMax; i++ ) {

            cur_x1x0              = _amem8_f2(curTrackL);
            curTrackL            += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
            cur_y1y0              = _amem8_f2(curTrackL);
            curTrackL            += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

            P0_P8                 = _amem8_f2(normCamExtPrmL);
            normCamExtPrmL       += 2;
            P9_P4                 = _amem8_f2(normCamExtPrmL);
            normCamExtPrmL       += 2;

            f2temp1               = _ftof2(_lof2(P0_P8), _lof2(P0_P8)); /* P8_P8 */
            f2temp2               = _ftof2(_hif2(P0_P8), _hif2(P0_P8)); /* P0_P0 */
            f2temp3               = _ftof2(_lof2(P9_P4), _lof2(P9_P4)); /* P4_P4 */

            _amem8_f2(matrixAL)   = _dsubsp(_dmpysp(f2temp1, cur_x1x0), f2temp2);
            matrixAL             += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

            _amem8_f2(matrixAL)   = _dsubsp(_dmpysp(f2temp1, cur_y1y0), f2temp3);
            matrixAL             += ((VLIB_TRIANG_MAT_COL *
                                      VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) -
                                     VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);

            P5_P1                 = _amem8_f2(normCamExtPrmL);
            normCamExtPrmL       += 2;
            f2temp1               = _ftof2(_hif2(P9_P4), _hif2(P9_P4)); /* P9_P9 */
            f2temp2               = _ftof2(_lof2(P5_P1), _lof2(P5_P1)); /* P1_P1 */
            f2temp3               = _ftof2(_hif2(P5_P1), _hif2(P5_P1)); /* P5_P5 */

            _amem8_f2(matrixAL)   = _dsubsp(_dmpysp(f2temp1, cur_x1x0), f2temp2);
            matrixAL             += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

            _amem8_f2(matrixAL)   = _dsubsp(_dmpysp(f2temp1, cur_y1y0), f2temp3);
            matrixAL             += ((VLIB_TRIANG_MAT_COL * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) -
                                     VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);


            P11_P10               = _amem8_f2(normCamExtPrmL);
            normCamExtPrmL       += 2;

            P6_P2                 = _amem8_f2(normCamExtPrmL);
            normCamExtPrmL       += 2;

            f2temp1               = _ftof2(_lof2(P11_P10), _lof2(P11_P10)); /* P10_P10 */
            f2temp2               = _ftof2(_lof2(P6_P2), _lof2(P6_P2)); /* P2_P2   */
            f2temp3               = _ftof2(_hif2(P6_P2), _hif2(P6_P2)); /* P6_P6   */

            _amem8_f2(matrixAL)   = _dsubsp(_dmpysp(f2temp1, cur_x1x0), f2temp2);
            matrixAL             += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
            _amem8_f2(matrixAL)   = _dsubsp(_dmpysp(f2temp1, cur_y1y0), f2temp3);
            matrixAL             += (VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR - ((2 * VLIB_TRIANG_MAT_COL) *
                                                                             VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR));

            P7_P3                 = _amem8_f2(normCamExtPrmL);
            normCamExtPrmL       += 2;
            f2temp1               = _ftof2(_hif2(P11_P10), _hif2(P11_P10)); /* P11_P11  */
            f2temp2               = _ftof2(_lof2(P7_P3), _lof2(P7_P3)); /* P3_P3    */
            f2temp3               = _ftof2(_hif2(P7_P3), _hif2(P7_P3)); /* P7_P7    */

            _amem8_f2(matrixbL)   = _dsubsp(f2temp2, _dmpysp(f2temp1, cur_x1x0));
            matrixbL             += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
            _amem8_f2(matrixbL)   = _dsubsp(f2temp3, _dmpysp(f2temp1, cur_y1y0));
            matrixbL             += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
            normCamExtPrmL       += 8;
        }

        /* Below code is added to initialize the data matrices matrixA,
         * and matrixb for empty positions. This is done to avoid any
         * NAN x 0.0, INF x 0x0 kind of operation. As these operation
         * will result in invalid output.
         */

        for( k = 0; k < VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            curTrackLengthL0   = (int32_t)curTrackLength[l + k];

            for( i=0; i < (VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthL0); i++ ) {

                matrixA[((2 * i) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) +
                        k]              =  0.0f;
                matrixA[((2 * i) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) +
                        ((1 * pitch) + k)]              =  0.0f;
                matrixA[((2 * i) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) +
                        (((2 * pitch) + k))]              =  0.0f;
                matrixA[((2 * i) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) +
                        ((1 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR)   +
                         +k)]              =  0.0f;
                matrixA[((2 * i) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) +
                        ((1 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR)   +
                         ((1 * pitch) + k))]              =  0.0f;
                matrixA[((2 * i) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) +
                        ((1 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR)   +
                         ((2 * pitch) + k))]              =  0.0f;
                matrixb[((2 * i) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k]
                    =  0.0f;

                matrixb[(((2 * i) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) +
                         (VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR + k))] = 0.0f;
            }
        }

        curTrack  += (VLIB_TRIANG_MAX_POINTS_IN_TRACK * (2 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR));

        matrixA   += (VLIB_TRIANG_MAT_ROW *
                      (VLIB_TRIANG_MAT_COL *
                       VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR));

        matrixb   += (VLIB_TRIANG_MAT_COL *
                      VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
    }
}

/**
* @func findTriangWt_ci
*
* @par Description:
*   This API Calculates weights for data matrices ( A & b) after each iteration of
*   triangulation. After weighting, AtA and Atb are again calculated to find new
*   refined 3D location of given track. Weights are also packed together for two tracks.
*
* @par
*   @param [in]  Xcam          :3-D output generated after triangulation API. two 3D outputs are
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
CODE_SECTION(findTriangWt_ci,  ".text:optimized")
static void findTriangWt_ci(VLIB_F32 Xcam[restrict],
                            VLIB_F32 normCamExtPrm[restrict],
                            VLIB_F32 weights[restrict],
                            uint8_t curTrackLength[restrict],
                            int32_t totalTracks)
{
    int32_t       i, l;
    int32_t       pmatrixIndx;
    VLIB_F32     *normCamExtPrmL;
    int32_t       curTrackLengthL0, curTrackLengthL1, curTrackLengthMax;
    __float2_t    temp1f2, temp2f2, temp3f2, temp4f2;
    __float2_t    X1X0, Y1Y0, Z1Z0, W1W0;


    for( l = 0; l < totalTracks; l += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        curTrackLengthL0  = (int32_t)curTrackLength[l + 0];
        curTrackLengthL1  = (int32_t)curTrackLength[l + 1];
        curTrackLengthMax = max(curTrackLengthL0, curTrackLengthL1);
        curTrackLengthMax = min(curTrackLengthMax, VLIB_TRIANG_MAX_POINTS_IN_TRACK);

        pmatrixIndx       = VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthMax;
        normCamExtPrmL    = normCamExtPrm + (pmatrixIndx * VLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE);

        X1X0            = _amem8_f2(Xcam);
        Xcam           += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

        Y1Y0            = _amem8_f2(Xcam);
        Xcam           += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

        Z1Z0            = _amem8_f2(Xcam);
        Xcam           += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

        for( i = VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLengthMax; i < VLIB_TRIANG_MAX_POINTS_IN_TRACK; i++ ) {
            temp1f2  = _amem8_f2(&normCamExtPrmL[12]);
            temp2f2  = _amem8_f2(&normCamExtPrmL[14]);
            temp3f2  = _amem8_f2(&normCamExtPrmL[16]);
            temp4f2  = _amem8_f2(&normCamExtPrmL[18]);

            temp1f2  = _dmpysp(X1X0, temp1f2);
            temp2f2  = _dmpysp(Y1Y0, temp2f2);
            temp3f2  = _dmpysp(Z1Z0, temp3f2);

            temp2f2  = _daddsp(temp1f2, temp2f2);
            temp3f2  = _daddsp(temp2f2, temp3f2);
            temp4f2  = _daddsp(temp3f2, temp4f2);

            /* Even if the change in weight is small then continue doing triangulation
             * dont break
             */
            W1W0  = VLIB_OneByX1X0F32(temp4f2);
            _amem8_f2(&weights[(i * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR)]) =  _dmpysp(W1W0, W1W0);
            normCamExtPrmL        += VLIB_TRIANG_CAMERA_EXTRENSIC_PARAM_ASIZE;
        }
        weights += (VLIB_TRIANG_MAX_POINTS_IN_TRACK * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
    }
}

/**
* @func solve3x3MatEq_ci
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
CODE_SECTION(solve3x3MatEq_ci,  ".text:optimized")
static void solve3x3MatEq_ci(VLIB_F32 matAtAPtr[restrict],
                             VLIB_F32 matBPtr[restrict],
                             VLIB_F32 resultPtr[restrict],
                             uint8_t valid[restrict],
                             int32_t totalTracks)
{
    int32_t                 ctr;
    __float2_t *restrict    matAtAPtrL = (__float2_t *)matAtAPtr;
    __float2_t *restrict    matBPtrL   = (__float2_t *)matBPtr;
    __float2_t *restrict    resultPtrL = (__float2_t *)resultPtr;
    __float2_t              f2temp1, f2temp2, f2temp3;
    int32_t                 temp;

    for( ctr = 0; ctr < totalTracks; ctr += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        __float2_t    a1_ab, a2_ab, a3_ab, b2_ab, b3_ab, c3_ab;
        __float2_t    d1_ab, d2_ab, d3_ab, d4_ab, d5_ab, d6_ab;
        __float2_t    det_ab, invDet_ab;
        __float2_t    atb1_ab, atb2_ab, atb3_ab;

        a1_ab = _amem8_f2(matAtAPtrL);
        matAtAPtrL++;

        a2_ab = _amem8_f2(matAtAPtrL);
        matAtAPtrL++;

        a3_ab = _amem8_f2(matAtAPtrL);
        matAtAPtrL++;

        b2_ab = _amem8_f2(matAtAPtrL);
        matAtAPtrL++;

        b3_ab = _amem8_f2(matAtAPtrL);
        matAtAPtrL++;

        c3_ab = _amem8_f2(matAtAPtrL);
        matAtAPtrL++;

        /* Co-factor calculation Starts
        */
        /* Row 1 b2*c3 - b3*b3*/
        d1_ab = _dsubsp(_dmpysp(b2_ab, c3_ab), _dmpysp(b3_ab, b3_ab));

        /* Row 1 b3*a3 - a2*c3*/
        d2_ab = _dsubsp(_dmpysp(b3_ab, a3_ab), _dmpysp(a2_ab, c3_ab));

        /* Row 1 a2*b3 - b2*a3*/
        d3_ab = _dsubsp(_dmpysp(a2_ab, b3_ab), _dmpysp(b2_ab, a3_ab));

        /* Row 1 a1*c3 - a3*a3*/
        d4_ab = _dsubsp(_dmpysp(a1_ab, c3_ab), _dmpysp(a3_ab, a3_ab));

        /* Row 1 a2*a3 - a1*b3 */
        d5_ab = _dsubsp(_dmpysp(a2_ab, a3_ab), _dmpysp(a1_ab, b3_ab));

        /* Row 1 a1*b2 - a2*a2*/
        d6_ab = _dsubsp(_dmpysp(a1_ab, b2_ab), _dmpysp(a2_ab, a2_ab));

        /* Co-factor calculation Ends
        */
        det_ab    = _daddsp(_daddsp(_dmpysp(a1_ab, d1_ab), _dmpysp(a3_ab, d3_ab)), _dmpysp(a2_ab, d2_ab));

        invDet_ab = _ftof2(_rcpsp(_hif2(det_ab)), _rcpsp(_lof2(det_ab)));
        invDet_ab = _dmpysp(invDet_ab, _dsubsp(_ftof2(2.0f, 2.0f), _dmpysp(det_ab, invDet_ab)));
        invDet_ab = _dmpysp(invDet_ab, _dsubsp(_ftof2(2.0f, 2.0f), _dmpysp(det_ab, invDet_ab)));

        if( _lof2(det_ab) >= FLT_EPSILON ) {
            temp = 1;
        } else {
            temp = 0;
        }

        valid[0] = (uint8_t)temp;

        if( _hif2(det_ab) >= FLT_EPSILON ) {
            temp = 1;
        } else {
            temp = 0;
        }

        valid[1] = (uint8_t)temp;

        atb1_ab   = _amem8_f2(matBPtrL);
        matBPtrL++;

        atb2_ab   = _amem8_f2(matBPtrL);
        matBPtrL++;

        atb3_ab   = _amem8_f2(matBPtrL);
        matBPtrL++;

        f2temp1   = _dmpysp(d1_ab, atb1_ab);
        f2temp2   = _dmpysp(d2_ab, atb2_ab);
        f2temp3   = _dmpysp(d3_ab, atb3_ab);
        f2temp1   = _daddsp(f2temp1, _daddsp(f2temp2, f2temp3));
        f2temp1   = _dmpysp(invDet_ab, f2temp1);
        _amem8_f2(resultPtrL) = f2temp1;
        resultPtrL++;

        f2temp1   = _dmpysp(d2_ab, atb1_ab);
        f2temp2   = _dmpysp(d4_ab, atb2_ab);
        f2temp3   = _dmpysp(d5_ab, atb3_ab);
        f2temp1   = _daddsp(f2temp1, _daddsp(f2temp2, f2temp3));
        f2temp1   = _dmpysp(invDet_ab, f2temp1);
        _amem8_f2(resultPtrL) = f2temp1;
        resultPtrL++;

        f2temp1   = _dmpysp(d3_ab, atb1_ab);
        f2temp2   = _dmpysp(d5_ab, atb2_ab);
        f2temp3   = _dmpysp(d6_ab, atb3_ab);
        f2temp1   = _daddsp(f2temp1, _daddsp(f2temp2, f2temp3));
        f2temp1   = _dmpysp(invDet_ab, f2temp1);
        _amem8_f2(resultPtrL) = f2temp1;
        resultPtrL++;
        valid      += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    }
}

/**
* @func solve3x3MatEqDouble
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
CODE_SECTION(solve3x3MatEqDouble,  ".text:optimized")
static void solve3x3MatEqDouble(VLIB_F32 matAtAPtr[restrict],
                                VLIB_F32 matBPtr[restrict],
                                VLIB_F32 resultPtr[restrict],
                                uint8_t valid[restrict],
                                int32_t totalTracks)
{
    int32_t    inPitchA = VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    int32_t    inPitchB = VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    int32_t    outPitch = VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    VLIB_D64    det, invDet;
    VLIB_D64    d1, d2, d3, d4, d5, d6;
    VLIB_D64    a1, a2, a3, b2, b3, c3;
    VLIB_D64    atB1, atB2, atB3;
    int32_t     k, l;

    /* a1 a2 a3
    b2 b3
    c3
    */
    for( l = 0; l < totalTracks; l+= VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            a1    = (VLIB_D64)matAtAPtr[(0 * inPitchA) + k];
            a2    = (VLIB_D64)matAtAPtr[(1 * inPitchA) + k];
            a3    = (VLIB_D64)matAtAPtr[(2 * inPitchA) + k];
            b2    = (VLIB_D64)matAtAPtr[(3 * inPitchA) + k];
            b3    = (VLIB_D64)matAtAPtr[(4 * inPitchA) + k];
            c3    = (VLIB_D64)matAtAPtr[(5 * inPitchA) + k];

            atB1  = (VLIB_D64)matBPtr[(0 * inPitchB) + k];
            atB2  = (VLIB_D64)matBPtr[(1 * inPitchB) + k];
            atB3  = (VLIB_D64)matBPtr[(2 * inPitchB) + k];

            d1    = (b2 * c3) - (b3 * b3);
            d2    = (b3 * a3) - (a2 * c3);
            d3    = (a2 * b3) - (b2 * a3);
            d4    = (a1 * c3) - (a3 * a3);
            d5    = (a2 * a3) - (a1 * b3);
            d6    = (a1 * b2) - (a2 * a2);

            det   = (a1 * d1) + ((a3 * d3) + (a2 * d2));

            if( det <= DBL_MIN ) {
                valid[k] = (uint8_t)0;
            } else {
                valid[k] = (uint8_t)1;
            }

            invDet = (VLIB_D64)VLIB_OneByXF32((VLIB_F32)det);

            resultPtr[(0 * outPitch) + k] = (VLIB_F32)(invDet * ((d1 * atB1) + ((d2 * atB2) + (d3 * atB3))));
            resultPtr[(1 * outPitch) + k] = (VLIB_F32)(invDet * ((d2 * atB1) + ((d4 * atB2) + (d5 * atB3))));
            resultPtr[(2 * outPitch) + k] = (VLIB_F32)(invDet * ((d3 * atB1) + ((d5 * atB2) + (d6 * atB3))));

        }

        matAtAPtr  += (VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
        matBPtr    += (VLIB_TRIANG_MAT_ROW * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
        resultPtr  += (3 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
        valid      += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    }
}

/**
* @func solve3x3MatEqDoubleSelect
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
CODE_SECTION(solve3x3MatEqDoubleSelect,  ".text:optimized")
static void solve3x3MatEqDoubleSelect(VLIB_F32 matAtAPtr[restrict],
                                      VLIB_F32 matBPtr[restrict],
                                      VLIB_F32 resultPtr[restrict],
                                      uint8_t valid[restrict],
                                      int32_t totalTracks)
{
    int32_t    inPitchA = VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    int32_t    inPitchB = VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    int32_t    outPitch = VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    VLIB_D64    det, invDet;
    VLIB_D64    d1, d2, d3, d4, d5, d6;
    VLIB_D64    a1, a2, a3, b2, b3, c3;
    VLIB_D64    atB1, atB2, atB3;
    int32_t     k, l;

    /* a1 a2 a3
    b2 b3
    c3
    */
    for( l = 0; l < totalTracks; l+=VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            if( valid[k] == 0 ) {
                a1    = (VLIB_D64)matAtAPtr[(0 * inPitchA) + k];
                a2    = (VLIB_D64)matAtAPtr[(1 * inPitchA) + k];
                a3    = (VLIB_D64)matAtAPtr[(2 * inPitchA) + k];
                b2    = (VLIB_D64)matAtAPtr[(3 * inPitchA) + k];
                b3    = (VLIB_D64)matAtAPtr[(4 * inPitchA) + k];
                c3    = (VLIB_D64)matAtAPtr[(5 * inPitchA) + k];

                atB1  = (VLIB_D64)matBPtr[(0 * inPitchB) + k];
                atB2  = (VLIB_D64)matBPtr[(1 * inPitchB) + k];
                atB3  = (VLIB_D64)matBPtr[(2 * inPitchB) + k];

                d1    = (b2 * c3) - (b3 * b3);
                d2    = (b3 * a3) - (a2 * c3);
                d3    = (a2 * b3) - (b2 * a3);
                d4    = (a1 * c3) - (a3 * a3);
                d5    = (a2 * a3) - (a1 * b3);
                d6    = (a1 * b2) - (a2 * a2);

                det   = (a1 * d1) + ((a3 * d3) + (a2 * d2));

                if( det <= DBL_MIN ) {
                    valid[k] = (uint8_t)0;
                } else {
                    valid[k] = (uint8_t)1;
                }

                invDet = (VLIB_D64)VLIB_OneByXF32((VLIB_F32)det);

                resultPtr[(0 * outPitch) + k] = (VLIB_F32)(invDet * ((d1 * atB1) + ((d2 * atB2) + (d3 * atB3))));
                resultPtr[(1 * outPitch) + k] = (VLIB_F32)(invDet * ((d2 * atB1) + ((d4 * atB2) + (d5 * atB3))));
                resultPtr[(2 * outPitch) + k] = (VLIB_F32)(invDet * ((d3 * atB1) + ((d5 * atB2) + (d6 * atB3))));
            }
        }

        matAtAPtr  += (VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
        matBPtr    += (VLIB_TRIANG_MAT_ROW * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
        resultPtr  += (3 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);
        valid      += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
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
*   @param [in]  curTrackLen  : each track length in frames
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
CODE_SECTION(getPseudoMatrices_ci,  ".text:optimized")
static void getPseudoMatrices_ci(VLIB_F32 matrixA[restrict],
                                 VLIB_F32 matrixb[restrict],
                                 VLIB_F32 matrixP_AtA[restrict],
                                 VLIB_F32 matrixP_Atb[restrict],
                                 uint8_t curTrackLen[restrict],
                                 int32_t totalTracks)
{
    int32_t    inPitchA  = 2 * VLIB_TRIANG_MAX_POINTS_IN_TRACK * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    int32_t    outPitchA = VLIB_TRIANG_MAX_POINTS_IN_TRACK * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
    int32_t    inPitchB  = 0;
    int32_t    outPitchB = VLIB_TRIANG_MAX_POINTS_IN_TRACK * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;

    int32_t       rowPair[2 * VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA] = { 0, 0, 0, 1, 0, 2, 1, 1, 1, 2, 2, 2 };
    int32_t       row_i, row_j;
    __float2_t    acc;

    int32_t               i = 0;
    int32_t               j = 0;
    int32_t               l = 0;
    VLIB_F32 *restrict    matrixAi;
    VLIB_F32 *restrict    matrixAj;
    VLIB_F32 *restrict    matrixP_AtAL;
    VLIB_F32 *restrict    matrixP_AtbL;
    VLIB_F32 *restrict    matrixbL;

    __float2_t    f2temp1, f2temp2;
    int32_t       curTrackLen0, curTrackLen1;

    for( l = 0; l < totalTracks; l += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        {
            curTrackLen0 = (int32_t)curTrackLen[l + 0];
            curTrackLen1 = (int32_t)curTrackLen[l + 1];
            curTrackLen0 = max(curTrackLen0, curTrackLen1);

            curTrackLen0 = VLIB_TRIANG_MAX_POINTS_IN_TRACK;

            for( i = 0; i < VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA; i++ ) {

                row_i    = rowPair[(2 * i)];
                row_j    = rowPair[(2 * i) + 1];
                acc      = _ftof2(0.0, 0.0);
                matrixAi    = (VLIB_F32 *)&matrixA[(row_i * inPitchA) + (2 * ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLen0) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR))];
                matrixAj    = (VLIB_F32 *)&matrixA[(row_j * inPitchA) + (2 * ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLen0) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR))];
                matrixP_AtAL= (VLIB_F32 *)&matrixP_AtA[(i * outPitchA) + ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLen0) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR)];

                for( j = VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLen0; j < VLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {
                    f2temp1 = _amem8_f2(matrixAi);
                    matrixAi += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    f2temp2 = _amem8_f2(matrixAj);
                    matrixAj += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    acc     = _dmpysp(f2temp1, f2temp2);

                    f2temp1 = _amem8_f2(matrixAi);
                    matrixAi += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    f2temp2 = _amem8_f2(matrixAj);
                    matrixAj += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    acc     = _daddsp(acc, _dmpysp(f2temp1, f2temp2));

                    _amem8_f2(matrixP_AtAL) = acc;
                    matrixP_AtAL += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                }
            }

            for( i = 0; i < 3; i++ ) {

                matrixP_AtbL= &matrixP_Atb[(i * outPitchB) + ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLen0) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR)];
                matrixAi    = &matrixA[(i * inPitchA) + (2 * ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLen0) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR))];
                matrixbL    = &matrixb[(i * inPitchB) + (2 * ((VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLen0) * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR))];

                for( j = VLIB_TRIANG_MAX_POINTS_IN_TRACK - curTrackLen0; j < VLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {

                    f2temp1 = _amem8_f2(matrixAi);
                    matrixAi += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    f2temp2 = _amem8_f2(matrixbL);
                    matrixbL += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    acc     = _dmpysp(f2temp1, f2temp2);

                    f2temp1 = _amem8_f2(matrixAi);
                    matrixAi += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    f2temp2 = _amem8_f2(matrixbL);
                    matrixbL += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    acc     = _daddsp(acc, _dmpysp(f2temp1, f2temp2));
                    _amem8_f2(matrixP_AtbL) = acc;
                    matrixP_AtbL += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                }
            }
        }

        matrixA     += (VLIB_TRIANG_MAT_ROW *
                        (VLIB_TRIANG_MAT_COL *
                         VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR));

        matrixb     += (VLIB_TRIANG_MAT_COL *
                        VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);

        matrixP_AtA += (VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA *
                        (VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR *
                         VLIB_TRIANG_MAX_POINTS_IN_TRACK));

        matrixP_Atb += (VLIB_TRIANG_MAT_ROW *
                        (VLIB_TRIANG_MAX_POINTS_IN_TRACK *
                         VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR));

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
CODE_SECTION(getFinalMatrices_ci,  ".text:optimized")
static void getFinalMatrices_ci(VLIB_F32 matrixP_AtA[restrict],
                                VLIB_F32 matrixP_Atb[restrict],
                                VLIB_F32 matrixAtA[restrict],
                                VLIB_F32 matrixAtb[restrict],
                                VLIB_F32 weight[restrict],
                                int32_t totalTracks)
{

    int32_t       i, l, j;
    __float2_t    acc;
    __float2_t    wt10;
    __float2_t    pAtA10;

    for( l = 0; l < totalTracks; l += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        {
            for( i = 0; i < VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA; i++ ) {
                acc = _ftof2(0.0, 0.0);

                for( j = 0; j < VLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {
                    wt10   = _amem8_f2(&weight[j * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR]);
                    pAtA10 = _amem8_f2(matrixP_AtA);
                    matrixP_AtA += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    acc    = _daddsp(acc, _dmpysp(wt10, pAtA10));
                }

                _amem8_f2(matrixAtA) = acc;
                matrixAtA           += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
            }

            for( i = 0; i < 3; i++ ) {
                acc = _ftof2(0.0, 0.0);

                for( j = 0; j < VLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {
                    wt10   = _amem8_f2(&weight[j * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR]);
                    pAtA10 = _amem8_f2(matrixP_Atb);
                    matrixP_Atb += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
                    acc    = _daddsp(acc, _dmpysp(wt10, pAtA10));
                }

                _amem8_f2(matrixAtb) = acc;
                matrixAtb           += VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR;
            }
        }

        weight      += (VLIB_TRIANG_MAX_POINTS_IN_TRACK *
                        VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR);

    }
}

CODE_SECTION(VLIB_triangulatePoints,  ".text:optimized")
int32_t VLIB_triangulatePoints(VLIB_F32 curTrack[restrict],
                               VLIB_F32 camExtPrm[restrict],
                               uint8_t curTrackLength[restrict],
                               int32_t maxIter,
                               VLIB_F32 Xcam[restrict],
                               VLIB_F32 scratch[restrict],
                               int32_t totalTracks,
                               uint8_t validOut[restrict],
                               uint32_t flag)
{
    int32_t    i, j, l, k;
    int32_t    maxLength;

    VLIB_F32 *restrict    matrixA; /*[VLIB_TRIANG_MAT_ROW][VLIB_TRIANG_MAT_COL][VLIB_TRIANG_NUM_TRACKS_TOGATHER]*/
    VLIB_F32 *restrict    matrixb; /*[VLIB_TRIANG_MAT_COL][VLIB_TRIANG_NUM_TRACKS_TOGATHER]*/
    VLIB_F32 *restrict    matrixP_AtA; /*[VLIB_TRIANG_NUM_TRACKS_TOGATHER][VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA*VLIB_TRIANG_MAX_POINTS_IN_TRACK]*/
    VLIB_F32 *restrict    matrixP_Atb; /*[VLIB_TRIANG_NUM_TRACKS_TOGATHER][VLIB_TRIANG_MAT_ROW*VLIB_TRIANG_MAX_POINTS_IN_TRACK]*/

    VLIB_F32 *restrict    matrixAtA; /*[VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA][VLIB_TRIANG_NUM_TRACKS_TOGATHER]*/
    VLIB_F32 *restrict    matrixAtb; /*[VLIB_TRIANG_MAT_AROW][VLIB_TRIANG_NUM_TRACKS_TOGATHER]*/
    VLIB_F32 *restrict    weights; /*[VLIB_TRIANG_MAX_POINTS_IN_TRACK][VLIB_TRIANG_NUM_TRACKS_TOGATHER]*/


    matrixA     = scratch;
    matrixb     = (VLIB_F32 *)matrixA + (VLIB_TRIANG_MAT_ROW * VLIB_TRIANG_MAT_COL * totalTracks);
    matrixP_AtA = (VLIB_F32 *)matrixb + (VLIB_TRIANG_MAT_COL * totalTracks);
    matrixP_Atb = (VLIB_F32 *)matrixP_AtA + (VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * VLIB_TRIANG_MAX_POINTS_IN_TRACK * totalTracks);

    matrixAtA   = (VLIB_F32 *)scratch;
    matrixAtb   = (VLIB_F32 *)matrixAtA + (VLIB_TRIANG_NUM_UNIQUE_ELEMENTS_IN_ATA * totalTracks);
    weights     = (VLIB_F32 *)matrixAtb + (VLIB_TRIANG_MAX_POINTS_IN_TRACK * totalTracks);

#ifdef VLIB_PROFILE_EN
    acc0 = 0;
    acc1 = 0;
    acc2 = 0;
    acc3 = 0;
    acc4 = 0;
    acc5 = 0;
    acc6 = 0;
#endif

#ifdef VLIB_PROFILE_EN
    t0 = PROFILE_READ;
#endif

    makeTriangMatrix_ci((VLIB_F32 *)curTrack,
                        (VLIB_F32 *)camExtPrm,
                        (VLIB_F32 *)matrixA,
                        matrixb,
                        curTrackLength,
                        totalTracks);

#ifdef VLIB_PROFILE_EN
    t1    = PROFILE_READ;
    acc0 += (t1 - t0);
#endif
#ifdef VLIB_PROFILE_EN
    t0 = PROFILE_READ;
#endif
    getPseudoMatrices_ci(matrixA,
                         matrixb,
                         matrixP_AtA,
                         matrixP_Atb,
                         curTrackLength,
                         totalTracks);

#ifdef VLIB_PROFILE_EN
    t1    = PROFILE_READ;
    acc1 += (t1 - t0);
#endif

#ifdef VLIB_PROFILE_EN
    t0 = PROFILE_READ;
#endif
    for( l = 0; l < totalTracks; l+= VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            validOut[l + k] = (uint8_t)0x1;
            maxLength       = (int32_t)curTrackLength[l + k];

            for( j = 0; j < VLIB_TRIANG_MAX_POINTS_IN_TRACK; j++ ) {
                if( j >= (VLIB_TRIANG_MAX_POINTS_IN_TRACK - maxLength)) {
                    weights[(l * VLIB_TRIANG_MAX_POINTS_IN_TRACK) + ((j * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 1.0f;
                } else {
                    weights[(l * VLIB_TRIANG_MAX_POINTS_IN_TRACK) + ((j * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 0.0f;
                }
            }
        }
    }

#ifdef VLIB_PROFILE_EN
    t1    = PROFILE_READ;
    acc5 += (t1 - t0);
#endif

    for( i = 0; i < maxIter; i++ ) {
#ifdef VLIB_PROFILE_EN
        t0 = PROFILE_READ;
#endif
        getFinalMatrices_ci(matrixP_AtA,
                            matrixP_Atb,
                            matrixAtA,
                            matrixAtb,
                            weights,
                            totalTracks);
#ifdef VLIB_PROFILE_EN
        t1    = PROFILE_READ;
        acc2 += (t1 - t0);
#endif
        if( flag == 0x0 ) {
#ifdef VLIB_PROFILE_EN
            t0 = PROFILE_READ;
#endif
            solve3x3MatEq_ci(matrixAtA,
                             matrixAtb,
                             Xcam,
                             validOut,
                             totalTracks);
#ifdef VLIB_PROFILE_EN
            t1    = PROFILE_READ;
            acc3 += (t1 - t0);
#endif

            solve3x3MatEqDoubleSelect(matrixAtA,
                                      matrixAtb,
                                      Xcam,
                                      validOut,
                                      totalTracks);

        } else {
            solve3x3MatEqDouble(matrixAtA,
                                matrixAtb,
                                Xcam,
                                validOut,
                                totalTracks);
        }

#ifdef VLIB_PROFILE_EN
        t0 = PROFILE_READ;
#endif
        findTriangWt_ci(Xcam,
                        camExtPrm,
                        weights,
                        curTrackLength,
                        totalTracks);
#ifdef VLIB_PROFILE_EN
        t1    = PROFILE_READ;
        acc4 += (t1 - t0);
#endif
    }

#ifdef VLIB_PROFILE_EN
    t0 = PROFILE_READ;
#endif
    for( l = 0; l < totalTracks; l+= VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR ) {
        for( k = 0; k < VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR; k++ ) {
            if( validOut[l + k] == 0 ) {
                Xcam[(l * 3) + ((0 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 0.0;
                Xcam[(l * 3) + ((1 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 0.0;
                Xcam[(l * 3) + ((2 * VLIB_TRIANG_NUM_TRACKS_IN_ONE_VECTOR) + k)] = 0.0;
            }
        }
    }

#ifdef VLIB_PROFILE_EN
    t1    = PROFILE_READ;
    acc6 += (t1 - t0);
#endif

#ifdef VLIB_PROFILE_EN
    printf("makeTriangMatrix_ci = %8lld getPseudoMatrices_ci = %8lld initWeight = %8lld getFinalMatrices_ci = %8lld solveMatEq3x3_ci = %8lld findTriangWt_ci = %8lld outputCollection = %8lld\n", acc0,acc1,acc5,acc2,acc3,acc4,acc6);
#endif

    return (1);
}

/* ======================================================================== */
/*  End of file:  VLIB_triangulatePoints.c                                  */
/* ======================================================================== */

