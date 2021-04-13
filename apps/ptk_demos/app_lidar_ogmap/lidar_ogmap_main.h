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
#ifndef _APP_LIDAR_OGMAP_MAIN_H_
#define _APP_LIDAR_OGMAP_MAIN_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <thread>

#include <perception/perception.h>
#include <perception/gui.h>
#include <perception/dbtools.h>
#include <perception/dbtools/c/virtual_sensor_creator.h>
#include <perception/utils/ptk_semaphore.h>

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/j7.h>
#include <TI/tivx_lidar.h>
#include <TI/tivx_park_assist.h>

#include <perception/drv/lidar_capture.h>
#include <perception/drv/ins_capture.h>

#include <app_ptk_demo_display.h>
#include <lidar_ogmap_applib.h>

/* The following user event is used to set the exit condition. */
#define APP_LIDAR_EVENT_BASE        (50U)
#define APP_LIDAR_GRAPH_COMP_EVENT  (APP_LIDAR_EVENT_BASE + 1U)
#define APP_LIDAR_USER_EVT_EXIT     (APP_LIDAR_EVENT_BASE + 2U)

using namespace std;
using namespace ptk;

typedef struct
{
    /** database config file (defines sensor inputs and outputs to
     *  be saved as virtual sensor) */
    PTK_DBConfig            dbConfig;

    LIDAROGAPPLIB_CreateParams      createParams;

    uint32_t                visualize;
    uint8_t                 useTiap;
    uint8_t                 dumpMap;

    LIDAROGAPPLIB_Handle    lidarHdl;

    vx_context              vxContext;

    Renderer               *renderer;
    MapRenderable          *mapRenderer;
    GridFlagView           *gridFlagView;
    KeyHandlerTable        *keyHandlerTbl;

    /** Stream handle for each sensor.
     *  Handle is null if sensor is not enabled*/
    SensorDataPlayer           *dataPlayer;

    /** Stream handle for each sensor.
     *  Handle is null if output is not enabled */
    vscHandle                   outputStrmHdl;

    PTK_Drv_InsDrvParams        insDrvConfig;
    PTK_Drv_LidarDrvParams      lidarDrvConfig;
    PTK_InsCapHandle            insCapHandle;
    PTK_LidarCapHandle          lidarCapHandle;
    uint8_t                     firstInsPkt;

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    bool                        is_interactive;

    /** Sensor rate control flag. If set to true then the inter-frame delay
     * based on the timestamps associated with the data frames will be used.
     */
    bool                        sensorRateControl;

    /** Input rate control flag. If set to true then a semaphore will be
     * used to control the synchronization between the input data and the
     * graph processing threads.
     */
    bool                        inputRateControl;

    /** For dumping the output map, if configured. */
    uint32_t                    outMapSize;

    /** Total radar frames processed. */
    uint32_t                    totalFrameCount;

    /** Total radar frames processed. */
    uint32_t                    framesProcessed;

    /** Total radar frames dropped. */
    uint32_t                    droppedFrameCnt;

    /** Current sensor data timestamp. */
    uint64_t                    curSensorTs;

    /** Flag to indicate that the graph processing thread should exit. */
    bool                        exitGraphProcess;

    /** Flag to indicate the render thread should exit.. */
    bool                        exitRenderProcess;

    /** Render periodicity in milli-sec. */
    uint64_t                    renderPeriod;

    /** Input data processing thread. */
    std::thread                 inputDataThread;

    /** Event handler thread. */
    std::thread                 evtHdlrThread;

    /** Render thread. */
    std::thread                 renderThreadId;

    /** Semaphore for rate synchronizing the input data and
     * graph processing threads.
     */
    UTILS::Semaphore           *dataReadySem;

    /** Width of the display window. */
    uint32_t                    winWidth;

    /** Height of the display window. */
    uint32_t                    winHeight;

#ifdef PLATFORM_EGL
    /** Dispaly context. */
    PTKDEMO_DisplayContext      dispCntxt;
#endif

    /** Counter for tracking number of times the data has been played.*/
    uint32_t                    runCtr;

    /** File name for exporting performance data. */
    char                       *perfOutFile;

    /** Flag to indicate if checksum needs to be performed on the final
     *  output accumulated map.
     */
    bool                        doChecksum;

    /** Expected value of checksum. This is looked at only if doChecksum
     *  is true.
     */
    uint32_t                    expectedChecksum;

    /** Computed checksum. */
    uint32_t                    computedChecksum;

} LIDAR_AppContext;

int32_t LIDAR_launchCaptureThreads(LIDAR_AppContext   * appCntxt);

#endif // _APP_LIDAR_OGMAP_MAIN_H_

