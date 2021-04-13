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

#ifndef VLIB_INTERNAL_H_
#define VLIB_INTERNAL_H_

/* MACROS*/
#ifndef WIN32
#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) > (y) ? (x) : (y))
#endif
#define WORD_ALIGNED(x) (_nassert(((int32_t)(x) & 0x3) == 0))
#define DOUBLEWORD_ALIGNED(x) (_nassert(((int32_t)(x) & 0x7) == 0))
#define DWORD_ALIGNED(x) (_nassert(((int32_t)(x) & 0x7) == 0))
#define ABS(x, y) ((((x) - (y)) < 0) ? -((x) - (y)) : ((x) - (y)))
#define SAT(a, lo, hi) (min(max((a), (lo)), (hi)))

/*Pre-processor Defines*/
/*Commonly-Used constants*/
#define PI 3.141592654
#define SQRTof2_Q8 362    /* 1.414213 * 256 = 362.038*/
#define NUMG 3

/* Background Model Constants*/
#define IIR_ALPHA_S32         0x00001000    /* SQ0.31*/
#define IIR_ALPHA_S16         0x10           /* SQ0.15*/
#define CAMERA_NOISE_S32      0x00A00000    /* SQ16.15*/
#define CAMERA_NOISE_S16      0x0A00         /* SQ12.3*/
#define THRESHOLD_FACTOR_S32  0x31fffff9    /* SQ4.27*/
#define THRESHOLD_FACTOR_S16  0x31ff        /* SQ4.11*/

#define PIXELFRACTION  (int32_t)((0.7 * (1 << 12)) + 0.5)
#define SALIENCELENGTH 4

#define YCrCb_OFFSET 1
#define VIDEO_BUFFER_LENGTH 4

#define MIN_16bBG_VALUE 0x0020
#define MAX_16bBG_VALUE 0xff00

/* 48kB remaining in L1 from 112kB total (32kB for Data & Prog Cache)*/
#define ONCHIP_L1D_BYTES (48 * 1024)

#define PIXEL_COUNT (INPUTFRAME_WIDTH * INPUTFRAME_HEIGHT)
#define BUFFER_LINES 6

/* Background Model Constants*/
#define IIR_ALPHA_S32         0x00001000    /* SQ0.31*/
#define IIR_ALPHA_S16         0x10           /*SQ0.15*/
#define CAMERA_NOISE_S32      0x00A00000    /* SQ16.15*/
#define CAMERA_NOISE_S16      0x0A00         /* SQ12.3*/
#define THRESHOLD_FACTOR_S32  0x31fffff9    /* SQ4.27*/
#define THRESHOLD_FACTOR_S16  0x31ff        /* SQ4.11*/

#endif  /* VLIB_INTERNAL_H_ */

/* ======================================================================== */
/*  End of file:  VLIB_internal.h                                           */
/* ======================================================================== */

