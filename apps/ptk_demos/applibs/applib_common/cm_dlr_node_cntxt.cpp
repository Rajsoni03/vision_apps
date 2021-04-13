/*
 *
 * Copyright (c) 2021 Texas Instruments Incorporated
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
#include <cm_dlr_node_cntxt.h>

static void CM_dlrNodeCleaup(
        CM_DLRNodeCntxt           *dlrObj)
{
    if (dlrObj->handle != NULL)
    {
        int32_t status;

        /* Delete the DLR Model handle. */
        status = DeleteDLRModel(&dlrObj->handle);

        if (status < 0)
        {
            PTK_printf("[%s:%d] DeleteDLRModel() failed. Error [%s].\n",
                        __FUNCTION__, __LINE__, DLRGetLastError());
        }

        dlrObj->state = CM_DLR_NODE_CNTXT_STATE_INVALID;
    }
}

static int32_t CM_dlrGetInputInfo(
        CM_DLRNodeCntxt           *dlrObj)
{
    CM_DLRNodeInputInfo    *input = &dlrObj->input;
    int32_t                 status = 0;

    /* Query the number of inputs. */
    status = GetDLRNumInputs(&dlrObj->handle, &input->numInfo);

    if (status < 0)
    {
        PTK_printf("[%s:%d] GetDLRNumInputs() failed. Error [%s].\n",
                    __FUNCTION__, __LINE__, DLRGetLastError());
    }
    else if (input->numInfo > CM_DLR_MAX_NUM_INPUTS)
    {
        PTK_printf("[%s:%d] Number of inputs [%d] "
                   "exceed [%d].\n",
                    __FUNCTION__, __LINE__,
                    dlrObj->input.numInfo, CM_DLR_MAX_NUM_INPUTS);
        status = -1;
    }

    if (status == 0)
    {
        for (int32_t i = 0; i < input->numInfo; i++)
        {
            CM_DLRIfInfo   *inputInfo;
            int32_t         status1;

            inputInfo = &input->info[i];

            /* Query input name. */
            status1 = GetDLRInputName(&dlrObj->handle,
                                      i,
                                      &inputInfo->name);

            if (status1 < 0)
            {
                PTK_printf("[%s:%d] GetDLRInputName(%d) failed. "
                           "Error [%s].\n",
                            __FUNCTION__, __LINE__, i, DLRGetLastError());
                status = status1;
                break;
            }

            /* Query input dimensions. */
            status1 = GetDLRInputSizeDim(&dlrObj->handle,
                                         i,
                                         &inputInfo->size,
                                         &inputInfo->dim);

            if (status1 < 0)
            {
                PTK_printf("[%s:%d] GetDLRInputSizeDim(%d) failed. "
                           "Error [%s].\n",
                            __FUNCTION__, __LINE__, i, DLRGetLastError());
                status = status1;
                break;
            }

            if (inputInfo->dim > CM_DLR_MAX_NUM_DIMS)
            {
                PTK_printf("[%s:%d] [%d] Number of dimensions [%d] "
                           "exceed [%d].\n",
                            __FUNCTION__, __LINE__, i,
                            inputInfo->dim, CM_DLR_MAX_NUM_DIMS);
                status = status1;
                break;
            }

            /* Query input shape. */
            status1 = GetDLRInputShape(&dlrObj->handle,
                                       i,
                                       inputInfo->shape);

            if (status1 < 0)
            {
                PTK_printf("[%s:%d] GetDLRInputShape(%d) failed. "
                           "Error [%s].\n",
                            __FUNCTION__, __LINE__, i, DLRGetLastError());
                status = status1;
                break;
            }

        } // for (int32_t i = 0; i < dlrObj->input.numInfo; i++)
    }

    return status;
}

static int32_t CM_dlrGetOutputInfo(
        CM_DLRNodeCntxt           *dlrObj)
{
    CM_DLRNodeOutputInfo   *output = &dlrObj->output;
    int32_t                 status = 0;

    /* Query the number of outputs. */
    status = GetDLRNumOutputs(&dlrObj->handle, &output->numInfo);

    if (status < 0)
    {
        PTK_printf("[%s:%d] GetDLRNumOutputs() failed. Error [%s].\n",
                    __FUNCTION__, __LINE__, DLRGetLastError());
    }
    else if (output->numInfo > CM_DLR_MAX_NUM_OUTPUTS)
    {
        PTK_printf("[%s:%d] Number of outputs [%d] "
                   "exceed [%d].\n",
                    __FUNCTION__, __LINE__,
                    output->numInfo, CM_DLR_MAX_NUM_OUTPUTS);
        status = -1;
    }

    if (status == 0)
    {
        for (int32_t i = 0; i < output->numInfo; i++)
        {
            CM_DLRIfInfo   *outputInfo;
            int32_t         status1;

            outputInfo = &dlrObj->output.info[i];

            /* Query output name. */
            status1 = GetDLROutputName(&dlrObj->handle,
                                       i,
                                       &outputInfo->name);

            if (status1 < 0)
            {
                PTK_printf("[%s:%d] GetDLROutputName(%d) failed. "
                           "Error [%s].\n",
                            __FUNCTION__, __LINE__, i, DLRGetLastError());
            }

            /* Query output dimensions. */
            status1 = GetDLROutputSizeDim(&dlrObj->handle,
                                          i,
                                          &outputInfo->size,
                                          &outputInfo->dim);

            if (status1 < 0)
            {
                PTK_printf("[%s:%d] GetDLROutputSizeDim(%d) failed. "
                           "Error [%s].\n",
                            __FUNCTION__, __LINE__, i, DLRGetLastError());
                status = status1;
                break;
            }

            if (outputInfo->dim > CM_DLR_MAX_NUM_DIMS)
            {
                PTK_printf("[%s:%d] [%d] Number of dimensions [%d] "
                           "exceed [%d].\n",
                            __FUNCTION__, __LINE__, i,
                            outputInfo->dim, CM_DLR_MAX_NUM_DIMS);
                status = status1;
                break;
            }

            /* Query output shape. */
            status1 = GetDLROutputShape(&dlrObj->handle,
                                        i,
                                        outputInfo->shape);

            if (status1 < 0)
            {
                PTK_printf("[%s:%d] GetDLROutputSizeDim(%d) failed. "
                           "Error [%s].\n",
                            __FUNCTION__, __LINE__, i, DLRGetLastError());
                status = status1;
                break;
            }

        } // for (int32_t i = 0; i < dlrObj->output.numInfo; i++)
    }

    return status;
}

int32_t CM_dlrNodeCntxtInit(
        CM_DLRNodeCntxt           *dlrObj,
        const CM_DLRCreateParams  *createParams)
{
    int32_t status = 0;

    if (dlrObj == NULL)
    {
        PTK_printf("[%s:%d] Parameter 'dlrObj' NULL.", __FUNCTION__, __LINE__);
        status = -1;
    }
    else if (createParams == NULL)
    {
        PTK_printf("[%s:%d] Parameter 'createParams' NULL.", __FUNCTION__, __LINE__);
        status = -1;
    }
    else if (dlrObj->state != CM_DLR_NODE_CNTXT_STATE_INVALID)
    {
        PTK_printf("[%s:%d] Invalid state.", __FUNCTION__, __LINE__);
        status = -1;
    }

    if (status == 0)
    {
        /* Initialize the context. */
        memset(dlrObj, 0, sizeof(CM_DLRNodeCntxt));

        status = CreateDLRModel(&dlrObj->handle,
                                createParams->modelPath,
                                createParams->devType,
                                createParams->devId);

        if (status < 0)
        {
            PTK_printf("[%s:%d] CreateDLRModel() failed. Error [%s].\n",
                        __FUNCTION__, __LINE__, DLRGetLastError());
        }
        else
        {
            memcpy(&dlrObj->params, createParams, sizeof(CM_DLRCreateParams));
        }
    }

    /* Query the input information. */
    if (status == 0)
    {
        status = CM_dlrGetInputInfo(dlrObj);
    }

    /* Query the output information. */
    if (status == 0)
    {
        status = CM_dlrGetOutputInfo(dlrObj);
    }

    if (status == 0)
    {
        dlrObj->state = CM_DLR_NODE_CNTXT_STATE_INIT;
    }

    if ((status != 0) && (dlrObj->handle != NULL))
    {
        CM_dlrNodeCleaup(dlrObj);
    }

    return status;
}

int32_t CM_dlrNodeCntxtProcess(
        CM_DLRNodeCntxt            *dlrObj,
        const CM_DLRNodeInputInfo  *input,
        CM_DLRNodeOutputInfo       *output)
{
    int32_t status = 0;

    if (dlrObj == NULL)
    {
        PTK_printf("[%s:%d] Parameter 'dlrObj' NULL.", __FUNCTION__, __LINE__);
        status = -1;
    }
    else if (input == NULL)
    {
        PTK_printf("[%s:%d] Parameter 'input' NULL.", __FUNCTION__, __LINE__);
        status = -1;
    }
    else if (output == NULL)
    {
        PTK_printf("[%s:%d] Parameter 'output' NULL.", __FUNCTION__, __LINE__);
        status = -1;
    }
    else if (dlrObj->state != CM_DLR_NODE_CNTXT_STATE_INIT)
    {
        PTK_printf("[%s:%d] Invalid state.", __FUNCTION__, __LINE__);
        status = -1;
    }

    if (status == 0)
    {
        /* Set the inputs. */
        for (int32_t i = 0; i < input->numInfo; i++)
        {
            const CM_DLRIfInfo *info = &input->info[i];

            status = SetDLRInput(&dlrObj->handle,
                                 info->name,
                                 info->shape,
                                 info->data,
                                 info->dim);

            if (status < 0)
            {
                PTK_printf("[%s:%d] SetDLRInput(%d) failed. Error [%s].\n",
                            __FUNCTION__, __LINE__, i, DLRGetLastError());
                break;
            }
        }
    }

    /* Run the model. */
    if (status == 0)
    {
        status = RunDLRModel(&dlrObj->handle);

        if (status < 0)
        {
            PTK_printf("[%s:%d] RunDLRModel() failed. Error [%s].\n",
                        __FUNCTION__, __LINE__, DLRGetLastError());
        }
    }

    /* Read the output. */
    if (status == 0)
    {
        for (int32_t i = 0; i < output->numInfo; i++)
        {
            CM_DLRIfInfo   *info = &output->info[i];

            status = GetDLROutput(&dlrObj->handle, i, info->data);

            if (status < 0)
            {
                PTK_printf("[%s:%d] GetDLROutput(%d) failed. Error [%s].\n",
                            __FUNCTION__, __LINE__, i, DLRGetLastError());
                break;
            }
        }
    }

    return status;
}

int32_t CM_dlrNodeCntxtDumpInfo(
        CM_DLRNodeCntxt  *dlrObj)
{
    int32_t status = 0;

    if (dlrObj == NULL)
    {
        PTK_printf("[%s:%d] Parameter 'dlrObj' NULL.", __FUNCTION__, __LINE__);
        status = -1;
    }
    else if (dlrObj->state != CM_DLR_NODE_CNTXT_STATE_INIT)
    {
        PTK_printf("[%s:%d] Model not initialized yet.", __FUNCTION__, __LINE__);
        status = -1;
    }
    
    if (status == 0)
    {
        CM_DLRNodeInputInfo    *input  = &dlrObj->input;
        CM_DLRNodeOutputInfo   *output = &dlrObj->output;

        PTK_printf("Model Path        = %s\n", dlrObj->params.modelPath);
        PTK_printf("Model Device Type = %d\n", dlrObj->params.devType);
        PTK_printf("Model Device Id   = %d\n", dlrObj->params.devId);

        PTK_printf("Number of Inputs  = %d\n", input->numInfo);

        for (int32_t i = 0; i < input->numInfo; i++)
        {
            CM_DLRIfInfo   *info = &input->info[i];

            PTK_printf("    [%d] Input Name       = %s\n", i, info->name);
            PTK_printf("    [%d] Input Size       = %d\n", i, info->size);
            PTK_printf("    [%d] Input Dim        = %d\n", i, info->dim);
            PTK_printf("    [%d] Input Shape      = ", i);

            for (int32_t j = 0; j < info->dim; j++)
            {
                PTK_printf("[%d] ", info->shape[j]);
            }

            PTK_printf("\n");
        }

        PTK_printf("Number of Outputs  = %d\n", output->numInfo);

        for (int32_t i = 0; i < output->numInfo; i++)
        {
            CM_DLRIfInfo   *info = &output->info[i];

            if (info->name != NULL)
            {
                PTK_printf("    [%d] Output Name      = %s\n", i, info->name);
            }

            PTK_printf("    [%d] Output Size      = %d\n", i, info->size);
            PTK_printf("    [%d] Output Dim       = %d\n", i, info->dim);
            PTK_printf("    [%d] Output Shape     = ", i);

            for (int32_t j = 0; j < info->dim; j++)
            {
                PTK_printf("[%d] ", info->shape[j]);
            }

            PTK_printf("\n");
        }
    }

    return status;
}

int32_t CM_dlrNodeCntxtDeInit(
        CM_DLRNodeCntxt  *dlrObj)
{
    int32_t status = 0;

    if (dlrObj == NULL)
    {
        PTK_printf("[%s:%d] Parameter 'dlrObj' NULL.", __FUNCTION__, __LINE__);
        status = -1;
    }
    else
    {
        if (dlrObj->state == CM_DLR_NODE_CNTXT_STATE_INVALID)
        {
            PTK_printf("[%s:%d] Invalid state.", __FUNCTION__, __LINE__);
            status = -1;
        }
        else
        {
            /* Delete the DLR Model handle. */
            CM_dlrNodeCleaup(dlrObj);
        }

        dlrObj->state = CM_DLR_NODE_CNTXT_STATE_INVALID;
    }

    return status;
}

