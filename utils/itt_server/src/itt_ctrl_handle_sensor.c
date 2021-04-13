/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
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

#include "itt_priv.h"
static uint8_t cmd_param_sensor_ctrl[CMD_PARAM_SIZE];

void itt_ctrl_cmdHandlerIssReadSensorReg(char *cmd, uint32_t prmSize)
{
    int32_t status;
    uint8_t * cmd_ptr;
    uint32_t * ptr32;
    uint32_t sensorRegRdWr[3] = {0};
    IMAGE_SENSOR_CTRLCMD ctrlCmd = IMAGE_SENSOR_CTRLCMD_READ_SENSOR_REG;
    char dummy_name[ISS_SENSORS_MAX_NAME] = "dummy";

    memset(sensorRegRdWr, 0U, sizeof(sensorRegRdWr));

    /* alloc tmp buffer for parameters */
    if (prmSize <= sizeof(sensorRegRdWr))
    {
        /* read parameters */
        IttCtrl_readParams((uint8_t *)sensorRegRdWr, prmSize);

        memset(cmd_param_sensor_ctrl, 0xAB, CMD_PARAM_SIZE);
        cmd_ptr = (uint8_t *)cmd_param_sensor_ctrl;

        memcpy(cmd_ptr, dummy_name, strlen(dummy_name)+1);
        cmd_ptr += ISS_SENSORS_MAX_NAME;

        ptr32 = (uint32_t *)cmd_ptr;
        *ptr32 = sensorRegRdWr[0]; /*channel ID */
        cmd_ptr += sizeof(uint32_t);

        memcpy(cmd_ptr, &ctrlCmd, sizeof(IMAGE_SENSOR_CTRLCMD));
        cmd_ptr += sizeof(IMAGE_SENSOR_CTRLCMD);

        ptr32 = (uint32_t *)cmd_ptr;
        *ptr32 = sensorRegRdWr[1]; /*Register Address*/
        cmd_ptr += sizeof(uint32_t);

        status = appRemoteServiceRun(
            APP_IPC_CPU_MCU2_0 ,
            IMAGE_SENSOR_REMOTE_SERVICE_NAME,
            IM_SENSOR_CMD_CTL,
            (void*)cmd_param_sensor_ctrl,
            CMD_PARAM_SIZE,
            0);

        if(status != 0)
        {
            printf("Error : appRemoteServiceRun returned %d \n", status);
        }
        /* send response */
        ptr32 = (uint32_t *)cmd_ptr;
        sensorRegRdWr[2] = *ptr32; /*Remote host should write register Value here */
        ITT_PRINTF("Read 0x%x from register 0x%x \n", sensorRegRdWr[2], sensorRegRdWr[1]);
        IttCtrl_writeParams((uint8_t *)&sensorRegRdWr[2], sizeof(sensorRegRdWr[2]), 0);
    }
    else
    {
        printf(" ITT_CTRL: %s: Insufficient parameters (%d bytes) specified !!!\n", cmd, prmSize);
    }

    /* send response */
    IttCtrl_writeParams(NULL, 0, 0);
}

void itt_ctrl_cmdHandlerIssWriteSensorReg(char *cmd, uint32_t prmSize)
{
    int32_t status;
    uint8_t * cmd_ptr;
    uint32_t * ptr32;
    uint32_t sensorRegRdWr[3] = {0};
    IMAGE_SENSOR_CTRLCMD ctrlCmd = IMAGE_SENSOR_CTRLCMD_WRITE_SENSOR_REG;
    char dummy_name[ISS_SENSORS_MAX_NAME] = "dummy";

    memset(sensorRegRdWr, 0U, sizeof(sensorRegRdWr));

    /* alloc tmp buffer for parameters */
    if (prmSize <= sizeof(sensorRegRdWr))
    {
        /* read parameters */
        IttCtrl_readParams((uint8_t *)sensorRegRdWr, prmSize);

        memset(cmd_param_sensor_ctrl, 0xAB, CMD_PARAM_SIZE);
        cmd_ptr = (uint8_t *)cmd_param_sensor_ctrl;

        memcpy(cmd_ptr, dummy_name, strlen(dummy_name)+1);
        cmd_ptr += ISS_SENSORS_MAX_NAME;

        ptr32 = (uint32_t *)cmd_ptr;
        *ptr32 = sensorRegRdWr[0]; /*channel ID */
        cmd_ptr += sizeof(uint32_t);

        memcpy(cmd_ptr, &ctrlCmd, sizeof(IMAGE_SENSOR_CTRLCMD));
        cmd_ptr += sizeof(IMAGE_SENSOR_CTRLCMD);

        ptr32 = (uint32_t *)cmd_ptr;
        *ptr32 = sensorRegRdWr[1]; /*Register Address*/
        cmd_ptr += sizeof(uint32_t);

        ptr32 = (uint32_t *)cmd_ptr;
        *ptr32 = sensorRegRdWr[2]; /*Register Value*/
        cmd_ptr += sizeof(uint32_t);

        status = appRemoteServiceRun(
            APP_IPC_CPU_MCU2_0 ,
            IMAGE_SENSOR_REMOTE_SERVICE_NAME,
            IM_SENSOR_CMD_CTL,
            (void*)cmd_param_sensor_ctrl,
            CMD_PARAM_SIZE,
            0);

        if(status != 0)
        {
            printf("Error : appRemoteServiceRun returned %d \n", status);
        }
        /* send response */
        ptr32 = (uint32_t *)cmd_ptr;
        sensorRegRdWr[2] = *ptr32; /*Remote host should write register Value here */
        ITT_PRINTF("Read 0x%x from register 0x%x \n", sensorRegRdWr[2], sensorRegRdWr[1]);
        IttCtrl_writeParams((uint8_t *)&sensorRegRdWr[2], sizeof(sensorRegRdWr[2]), 0);
    }
    else
    {
        printf(" ITT_CTRL: %s: Insufficient parameters (%d bytes) specified !!!\n", cmd, prmSize);
    }

    /* send response */
    IttCtrl_writeParams(NULL, 0, 0);
}
