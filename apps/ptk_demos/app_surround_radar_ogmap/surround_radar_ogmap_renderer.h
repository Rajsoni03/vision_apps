/*
 *
 * Copyright (c) 2019 Texas Instruments Incorporated
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
#ifndef _SURROUND_RADAR_OGMAP_RENDERER_H_
#define _SURROUND_RADAR_OGMAP_RENDERER_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <perception/perception.h>
#include <perception/gui.h>

#include "surround_radar_ogmap_main.h"

#define SUOGMAP_GRID_ID_ACCUMULATED                  (1U)
#define SUOGMAP_GRID_ID_INST_OCC                     (2U)
#define SUOGMAP_GRID_ID_INST_DS                      (3U)

// Flag bits for the occupancy grid
/** Contains at least one non-ground point. */
#define SUOGMAP_FLAG_OCCUPIED                        (0x00000001)

/** Contains a mixture of ground and drivable points. */
#define SUOGMAP_FLAG_GROUND                          (0x00000002)

/** A square we are or were present in. */
#define SUOGMAP_FLAG_EGO                             (0x00000004)

/** A square we want to consider for parking space search. */
#define SUOGMAP_FLAG_FST                             (0x00000008)

/** A square search candidate to test. */
#define SUOGMAP_FLAG_FSD                             (0x00000010)

/** A square that has a circle of appropriate radius free of obstacles. */
#define SUOGMAP_FLAG_PFSD                            (0x00000020)

/** Flag to indicate cell was updated. */
#define SUOGMAP_FLAG_INST_CHANGED                    (0x00000040)

int32_t SUOGMAP_launchRenderThread(SUOGMAP_Context   * appCntxt);
void    SUOGMAP_exitRenderThread(SUOGMAP_Context   * appCntxt);

#endif // _SURROUND_RADAR_OGMAP_RENDERER_H_

