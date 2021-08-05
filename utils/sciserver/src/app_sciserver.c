/*
 *
 * Copyright (c) 2018 Texas Instruments Incorporated
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

#include <utils/console_io/include/app_log.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/sciclient/sciserver_tirtos.h>
#include <stdio.h>

/** \brief Aligned address at which the X509 header is placed. */
#define SCISERVER_COMMON_X509_HEADER_ADDR (0x41cffb00)

/* High Priority for SCI Server - must be higher than Low priority task */
#define SETUP_SCISERVER_TASK_PRI_HIGH   (5)
/*
 * Low Priority for SCI Server - must be higher than IPC echo test tasks
 * to prevent delay in handling Sciserver requests when test is performing
 * multicore ping/pong.
 */
#define SETUP_SCISERVER_TASK_PRI_LOW    (4)

int32_t appSciserverSciclientInit()
{
    int32_t retVal = CSL_PASS;

    Sciclient_ConfigPrms_t  clientParams;

    retVal = Sciclient_configPrmsInit(&clientParams);

    if(retVal==CSL_PASS)
    {
        retVal = Sciclient_boardCfgParseHeader(
                (uint8_t *) SCISERVER_COMMON_X509_HEADER_ADDR,
                &clientParams.inPmPrms, &clientParams.inRmPrms);
    }

    if(retVal==CSL_PASS)
    {
        retVal = Sciclient_init(&clientParams);
    }

    return retVal;
}

int32_t appSciserverSciclientDeInit()
{
    int32_t retVal = 0;

    retVal = Sciclient_deinit();
    if(retVal!=0)
    {
        appLogPrintf("SCICLIENT: ERROR: Sciclient deinit failed !!!\n");
    }

    return retVal;
}

int32_t appSciserverInit()
{
    int32_t retVal = CSL_PASS;
    Sciserver_TirtosCfgPrms_t serverParams;

    retVal = Sciserver_tirtosInitPrms_Init(&serverParams);

    serverParams.taskPriority[SCISERVER_TASK_USER_LO] = SETUP_SCISERVER_TASK_PRI_LOW;
    serverParams.taskPriority[SCISERVER_TASK_USER_HI] = SETUP_SCISERVER_TASK_PRI_HIGH;

    if(retVal==CSL_PASS)
    {
        retVal = Sciserver_tirtosInit(&serverParams);
    }

    return retVal;
}

void appSciserverDeInit()
{
    Sciserver_tirtosDeinit();

    return;
}
