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
**|        Copyright (c) 2007 - 2013 Texas Instruments Incorporated          |**
**|                                                                          |**
**|              All rights reserved not granted herein.                     |**
**|                                                                          |**
**|                         Limited License.                                 |**
**|                                                                          |**
**|  Texas Instruments Incorporated grants a world-wide, royalty-free,       |**
**|  non-exclusive license under copyrights and patents it now or            |**
**|  hereafter owns or controls to make, have made, use, import, offer to    |**
**|  sell and sell ("Utilize") this software subject to the terms herein.    |**
**|  With respect to the foregoing patent license, such license is granted   |**
**|  solely to the extent that any such patent is necessary to Utilize the   |**
**|  software alone.  The patent license shall not apply to any              |**
**|  combinations which include this software, other than combinations       |**
**|  with devices manufactured by or for TI ("TI Devices").  No hardware     |**
**|  patent is licensed hereunder.                                           |**
**|                                                                          |**
**|  Redistributions must preserve existing copyright notices and            |**
**|  reproduce this license (including the above copyright notice and the    |**
**|  disclaimer and (if applicable) source code license limitations below)   |**
**|  in the documentation and/or other materials provided with the           |**
**|  distribution                                                            |**
**|                                                                          |**
**|  Redistribution and use in binary form, without modification, are        |**
**|  permitted provided that the following conditions are met:               |**
**|                                                                          |**
**|    *  No reverse engineering, decompilation, or disassembly of this      |**
**|  software is permitted with respect to any software provided in binary   |**
**|  form.                                                                   |**
**|                                                                          |**
**|    *  any redistribution and use are licensed by TI for use only with    |**
**|  TI Devices.                                                             |**
**|                                                                          |**
**|    *  Nothing shall obligate TI to provide you with source code for      |**
**|  the software licensed and provided to you in object code.               |**
**|                                                                          |**
**|  If software source code is provided to you, modification and            |**
**|  redistribution of the source code are permitted provided that the       |**
**|  following conditions are met:                                           |**
**|                                                                          |**
**|    *  any redistribution and use of the source code, including any       |**
**|  resulting derivative works, are licensed by TI for use only with TI     |**
**|  Devices.                                                                |**
**|                                                                          |**
**|    *  any redistribution and use of any object code compiled from the    |**
**|  source code and any resulting derivative works, are licensed by TI      |**
**|  for use only with TI Devices.                                           |**
**|                                                                          |**
**|  Neither the name of Texas Instruments Incorporated nor the names of     |**
**|  its suppliers may be used to endorse or promote products derived from   |**
**|  this software without specific prior written permission.                |**
**|                                                                          |**
**|  DISCLAIMER.                                                             |**
**|                                                                          |**
**|  THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY      |**
**|  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE       |**
**|  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR      |**
**|  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI'S LICENSORS BE      |**
**|  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR     |**
**|  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF    |**
**|  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR         |**
**|  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,   |**
**|  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE    |**
**|  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,       |**
**|  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                      |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/

#ifndef VLIB_UTILITY_H_
#define VLIB_UTILITY_H_ 1

#include "VLIB_types.h"
//#include "../VLIB_orb.h"
#include "../VLIB_platforms.h"
#include <stdlib.h> /* For qsort func in orb */
#include <math.h> /* For floor function */
#include <float.h> /* For FLT_MIN, FLT_MAX etc.*/



/* Temporary defines */
#define VLIB_CGT6X_COMPILER

#if 0
/* Division utility prototypes */
int32_t VLIB_divS32 (int32_t num, int32_t den);
int16_t VLIB_divS16 (int16_t num, int16_t den);
uint16_t VLIB_divU16 (uint16_t num, uint16_t den);
void VLIB_matrix_inverse_4x4_F32(VLIB_F32 *temp2_F32,
                                 int16_t factor,
                                 VLIB_F32 *inv_temp);
/* Miscellaneous prototypes */
int32_t  VLIB_isqrt (int32_t x);
void VLIB_matrixInverse_4x4 (int16_t *output, int16_t *source, int32_t *buffer, int16_t factor);

void VLIB_cORBFeatures_insertionsort_x(CORBFeature *features, int32_t n_features);

void    VLIB_orb_make_offsets_9(int32_t pixl[], int32_t row_stride);

void    VLIB_orb_make_offsets_12(int32_t pixel[], int32_t row_stride);

int32_t VLIB_compareShort(const void *a, const void *b);
#endif

static inline VLIB_F32 VLIB_OneByXF32(VLIB_F32 x);
static inline __float2_t VLIB_OneByX1X0F32(__float2_t x1x0);
static inline VLIB_F32 VLIB_OneBySqrtXF32(VLIB_F32 x);
static inline VLIB_F32 VLIB_YByXF32(VLIB_F32 y, VLIB_F32 x);
static inline VLIB_F32 VLIB_SqrtXF32(VLIB_F32 x);

static inline VLIB_F32 VLIB_OneByXF32(VLIB_F32 x)
{
    VLIB_F32    TWO = 2.0f;
    VLIB_F32    X;

    VLIB_F32    Big = FLT_MAX;
    VLIB_F32    Y;

    Y = _fabsf(x);

    X = _rcpsp(x);
    X = X  * (TWO - (x * X));
    X = X  * (TWO - (x * X));

    if( Y >= Big ) {
        X = 0.0f;
    }

    if( Y <= FLT_MIN ) {
        X = Big;
    }

    return (X);
}

static inline VLIB_F32 VLIB_YByXF32(VLIB_F32 y, VLIB_F32 x)
{
    VLIB_F32    X;

    X = y * VLIB_OneByXF32(x);

    return (X);
}

static inline __float2_t VLIB_OneByX1X0F32(__float2_t x1x0)
{
    VLIB_F32      Big = FLT_MAX;
    VLIB_F32      Y0, Y1;
    VLIB_F32      X0, X1;
    __float2_t    f2Pkd = _ftof2(2.0f, 2.0f);
    __float2_t    X1X0;


    X0 = _rcpsp(_lof2(x1x0));
    X1 = _rcpsp(_hif2(x1x0));

    X1X0 = _ftof2(X1, X0);
    X1X0 = _dmpysp(X1X0, _dsubsp(f2Pkd, _dmpysp(X1X0, x1x0)));
    X1X0 = _dmpysp(X1X0, _dsubsp(f2Pkd, _dmpysp(X1X0, x1x0)));

    Y0 = _fabsf(_lof2(x1x0));
    Y1 = _fabsf(_hif2(x1x0));

    X0   = _lof2(X1X0);
    X1   = _hif2(X1X0);

    if( Y0 >= Big ) {
        X0 = 0.0f;
    }

    if( Y1 >= Big ) {
        X1 = 0.0f;
    }

    if( Y0 <= FLT_MIN ) {
        X0 = Big;
    }

    if( Y1 <= FLT_MIN ) {
        X1 = Big;
    }

    X1X0 = _ftof2(X1, X0);

    return (X1X0);

}

static inline VLIB_F32 VLIB_OneBySqrtXF32(VLIB_F32 x)
{
    VLIB_F32    normInv, val;
    VLIB_F32    Y;
    VLIB_F32    Big = FLT_MAX;

    Y = _fabsf(x);

    normInv   = _rsqrsp(x);
    val       = normInv * ((3.0f - (x * (normInv * normInv))) * 0.5f);
    normInv   = val * ((3.0f - (x * (val * val))) * 0.5f);

    if( Y >= Big ) {
    	normInv = 0.0f;
    }
    if( Y <= FLT_MIN ) {
    	normInv = Big;
    }

    if(x <= 0.0f){
      normInv = 0.0f;
    }

    return (normInv);
}

static inline VLIB_F32 VLIB_SqrtXF32(VLIB_F32 x)
{
    VLIB_F32    X;

    X = x * VLIB_OneBySqrtXF32(x);

    return (X);
}


#endif /* VLIB_UTILITY_H_ */

/* ======================================================================== */
/*  End of file:  VLIB_utility.h                                            */
/* ======================================================================== */

