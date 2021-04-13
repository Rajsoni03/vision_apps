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
#include <cm_tidl_node_cntxt.h>

#define CM_MAX_TENSOR_DIMS  (4u)
#define CM_TIDL_MAX_PARAMS  (16u)

static vx_enum getVxTensorDatatype(int32_t tidl_datatype)
{
    vx_enum tiovx_datatype = VX_TYPE_INVALID;

    if(tidl_datatype == TIDL_UnsignedChar)
    {
        tiovx_datatype = VX_TYPE_UINT8;
    }
    else if(tidl_datatype == TIDL_SignedChar)
    {
        tiovx_datatype = VX_TYPE_INT8;
    }
    else if(tidl_datatype == TIDL_UnsignedShort)
    {
        tiovx_datatype = VX_TYPE_UINT16;
    }
    else if(tidl_datatype == TIDL_SignedShort)
    {
        tiovx_datatype = VX_TYPE_INT16;
    }
    else if(tidl_datatype == TIDL_UnsignedWord)
    {
        tiovx_datatype = VX_TYPE_UINT32;
    }
    else if(tidl_datatype == TIDL_SignedWord)
    {
        tiovx_datatype = VX_TYPE_INT32;
    }
    else if(tidl_datatype == TIDL_SinglePrecFloat)
    {
        tiovx_datatype = VX_TYPE_FLOAT32;
    }

    return (tiovx_datatype);
}

static vx_user_data_object CM_readConfig(
        vx_context  context,
        vx_char    *vxConfig_file,
        vx_uint32  *numInTensors,
        vx_uint32  *numOutTensors)
{
    FILE               *fp_vxConfig;
    sTIDL_IOBufDesc_t  *ioBufDesc;
    tivxTIDLJ7Params   *tidlParams;
    vx_user_data_object vxConfig;
    vx_uint32           capacity;
    vx_map_id           map_id;
    vx_size             read_count;
    vx_status           vxStatus;

    vxStatus    = VX_SUCCESS;
    vxConfig    = NULL;
    fp_vxConfig = fopen(vxConfig_file, "rb");

    if (fp_vxConfig == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Unable to open [%s] file!", vxConfig_file);
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        fseek(fp_vxConfig, 0, SEEK_END);
        capacity = ftell(fp_vxConfig);
        fseek(fp_vxConfig, 0, SEEK_SET);

        if (capacity != sizeof(sTIDL_IOBufDesc_t))
        {
            VX_PRINT(VX_ZONE_ERROR,
                     "Config file size (%d bytes) does not match size of "
                     "sTIDL_IOBufDesc_t (%d bytes)",
                     capacity,
                     (vx_uint32)sizeof(sTIDL_IOBufDesc_t));

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Create a user struct type for handling vxConfig data*/
        vxConfig = vxCreateUserDataObject(context,
                                        "tivxTIDLJ7Params",
                                        sizeof(tivxTIDLJ7Params),
                                        NULL );

        if (vxConfig == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxCreateUserDataObject() failed");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxMapUserDataObject(vxConfig,
                                       0,
                                       sizeof(tivxTIDLJ7Params),
                                       &map_id,
                                       (void **)&tidlParams,
                                       VX_WRITE_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapUserDataObject() failed");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tivx_tidl_j7_params_init(tidlParams);

        ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

        read_count = fread(ioBufDesc, capacity, 1, fp_vxConfig);

        if (read_count != 1)
        {
            VX_PRINT(VX_ZONE_ERROR, "fread() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            *numInTensors  = ioBufDesc->numInputBuf;
            *numOutTensors = ioBufDesc->numOutputBuf;

            vxUnmapUserDataObject(vxConfig, map_id);
        }
    }

    if (fp_vxConfig != NULL)
    {
        fclose(fp_vxConfig);
    }

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        if (vxConfig != NULL)
        {
            vxReleaseUserDataObject(&vxConfig);
        }
    }

    return vxConfig;
}

static vx_user_data_object CM_readNetwork(
        vx_context  context,
        vx_char    *vxNetwork_file)
{
    FILE               *fp_vxNetwork;
    void               *vxNetwork_buffer;
    vx_user_data_object vxNetwork;
    vx_map_id           map_id;
    vx_uint32           capacity;
    vx_size             read_count;
    vx_status           vxStatus;

    vxStatus         = VX_SUCCESS;
    vxNetwork_buffer = NULL;
    vxNetwork        = NULL;

    fp_vxNetwork = fopen(vxNetwork_file, "rb");

    if (fp_vxNetwork == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Unable to open [%s] file!", vxNetwork_file);
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        fseek(fp_vxNetwork, 0, SEEK_END);
        capacity = ftell(fp_vxNetwork);
        fseek(fp_vxNetwork, 0, SEEK_SET);

        vxNetwork = vxCreateUserDataObject(context,
                                         "TIDL_network",
                                         capacity,
                                         NULL );

        if (vxNetwork == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxCreateUserDataObject() failed");

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxMapUserDataObject(vxNetwork,
                                       0,
                                       capacity,
                                       &map_id,
                                       (void **)&vxNetwork_buffer,
                                       VX_WRITE_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapUserDataObject() failed");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        read_count = fread(vxNetwork_buffer, capacity, 1, fp_vxNetwork);

        if (read_count != 1)
        {
            VX_PRINT(VX_ZONE_ERROR, "fread() failed");

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxUnmapUserDataObject(vxNetwork, map_id);
        }
    }

    if (fp_vxNetwork != NULL)
    {
        fclose(fp_vxNetwork);
    }

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        if (vxNetwork != NULL)
        {
            vxReleaseUserDataObject(&vxNetwork);
        }
    }

    return vxNetwork;
}

static vx_user_data_object CM_setInArgs(vx_context context)
{
    void                *inArgs_buffer;
    vx_user_data_object  inArgs;
    vx_map_id            map_id;
    vx_uint32            capacity;
    vx_status            vxStatus;

    vxStatus      = VX_SUCCESS;
    inArgs_buffer = NULL;
    capacity      = sizeof(TIDL_InArgs);

    inArgs = vxCreateUserDataObject(context,
                                    "TIDL_InArgs",
                                    capacity,
                                    NULL );

    if (inArgs == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "vxCreateUserDataObject() failed");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxMapUserDataObject(inArgs,
                                       0,
                                       capacity,
                                       &map_id,
                                       (void **)&inArgs_buffer,
                                       VX_WRITE_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapUserDataObject() failed");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        TIDL_InArgs *prms = (TIDL_InArgs *)inArgs_buffer;
        prms->iVisionInArgs.size         = sizeof(TIDL_InArgs);
        prms->iVisionInArgs.subFrameInfo = 0;

        vxUnmapUserDataObject(inArgs, map_id);
    }

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        if (inArgs != NULL)
        {
            vxReleaseUserDataObject(&inArgs);
        }
    }

    return inArgs;
}

static vx_user_data_object CM_setOutArgs(vx_context context)
{
    void                *outArgs_buffer;
    vx_user_data_object  outArgs;
    vx_map_id            map_id;
    vx_uint32            capacity;
    vx_status            vxStatus;

    vxStatus       = VX_SUCCESS;
    outArgs_buffer = NULL;
    capacity       = sizeof(TIDL_outArgs);

    outArgs = vxCreateUserDataObject(context,
                                     "TIDL_outArgs",
                                     capacity,
                                     NULL );

    if (outArgs == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "vxCreateUserDataObject() failed");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxMapUserDataObject(outArgs,
                                       0,
                                       capacity,
                                       &map_id,
                                       (void **)&outArgs_buffer,
                                       VX_WRITE_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapUserDataObject() failed");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        TIDL_outArgs *prms = (TIDL_outArgs *)outArgs_buffer;
        prms->iVisionOutArgs.size = sizeof(TIDL_outArgs);

        vxUnmapUserDataObject(outArgs, map_id);
    }

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        if (outArgs != NULL)
        {
            vxReleaseUserDataObject(&outArgs);
        }
    }

    return outArgs;
}

static vx_tensor **alloc2DTensorArray(
        uint32_t    dim1,
        uint32_t    dim2,
        uint32_t   *memSize)
{
    vx_tensor **out;
    vx_tensor  *tmp;
    uint8_t    *ptr;
    uint32_t    size;

    size = dim1 * (sizeof(vx_tensor **) + (sizeof(vx_tensor*) * dim2));
    ptr  = (uint8_t *)tivxMemAlloc(size, TIVX_MEM_EXTERNAL);

    if (ptr == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to allocate output tensor array");

        out = NULL;
    }
    else
    {
        *memSize = size;
        out = (vx_tensor **)ptr;
        ptr += sizeof(vx_tensor **) * dim1;
        tmp = (vx_tensor *)ptr;

        for (uint32_t i = 0; i < dim1; i++, tmp++)
        {
            out[i] = tmp;
        }
    }

    return out;
}

static vx_user_data_object CM_setCreateParams(vx_context context)
{
    void               *createParams_buffer;
    vx_user_data_object createParams;
    vx_map_id           map_id;
    vx_uint32           capacity;
    vx_status           vxStatus;

    vxStatus = VX_SUCCESS;
    capacity = sizeof(TIDL_CreateParams);
    createParams = vxCreateUserDataObject(context,
                                          "TIDL_CreateParams",
                                          capacity,
                                          NULL );

    if (createParams == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "vxCreateUserDataObject() failed");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxMapUserDataObject(createParams,
                                       0,
                                       capacity,
                                       &map_id,
                                       (void **)&createParams_buffer,
                                       VX_WRITE_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapUserDataObject() failed");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        TIDL_CreateParams *prms;

        prms = (TIDL_CreateParams *) createParams_buffer;

        //write create params here
        TIDL_createParamsInit(prms);

        prms->isInbufsPaded             = 1;
        prms->quantRangeExpansionFactor = 1.0;
        prms->quantRangeUpdateFactor    = 0.0;

#if defined(x86_64)
        prms->flowCtrl                  = 1;
#endif /* defined(x86_64) */

        vxUnmapUserDataObject(createParams, map_id);
    }

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        if (createParams != NULL)
        {
            vxReleaseUserDataObject(&createParams);
        }
    }

    return createParams;
}

vx_status CM_tidlNodeCntxtInit(
        CM_TIDLNodeCntxt           *tidlObj,
        vx_context                  context,
        const CM_TIDLCreateParams  *createParams)
{
    uint32_t    numInTensors;
    uint32_t    numOutTensors;
    vx_status   vxStatus;

    vxStatus      = VX_SUCCESS;
    numInTensors  = 0;
    numOutTensors = 0;

    if (tidlObj == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'tidlObj' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (context == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'context' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (createParams == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'createParams' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if ((createParams->pipelineDepth == 0) ||
             (createParams->pipelineDepth > CM_MAX_PIPELINE_DEPTH))
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid pipeline depth.");
        vxStatus = VX_FAILURE;
    }
    else if (tidlObj->state != CM_TIDL_NODE_CNTXT_STATE_INVALID)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid state.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->pipelineDepth = createParams->pipelineDepth;

        /* Create a vx_array object and read the vxConfig data*/
        tidlObj->vxConfig = CM_readConfig(context,
                                          tidlObj->tidlCfgFilePath,
                                          &numInTensors,
                                          &numOutTensors);
        if (tidlObj->vxConfig == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "CM_readConfig() failed");
            vxStatus = VX_FAILURE;
        }
        else if ((numInTensors == 0) || (numInTensors >= CM_MAX_TENSOR_DIMS))
        {
            VX_PRINT(VX_ZONE_ERROR, "numInTensors is invalid.");
            vxStatus = VX_FAILURE;
        }
        else if ((numOutTensors == 0) || (numOutTensors >= CM_MAX_TENSOR_DIMS))
        {
            VX_PRINT(VX_ZONE_ERROR, "numOutTensors is invalid.");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)tidlObj->vxConfig,
                               "TIDLConfig");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Save a copy of number of input/output tensors required as per config */
        tidlObj->numInTensors  = numInTensors;
        tidlObj->numOutTensors = numOutTensors;

        /* Create a vx_tensor object and read the network data */
        tidlObj->vxNetwork = CM_readNetwork(context, tidlObj->tidlNwFilePath);

        if (tidlObj->vxNetwork == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "CM_readNetwork() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)tidlObj->vxNetwork,
                               "TIDLNetwork");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->vxCreateParams = CM_setCreateParams(context);

        if (tidlObj->vxCreateParams == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "CM_setCreateParams() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)tidlObj->vxCreateParams,
                               "CreateParams");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->vxInArgs = CM_setInArgs(context);

        if (tidlObj->vxInArgs == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "CM_setInArgs() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)tidlObj->vxInArgs,
                               "TIDL_InArgs");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->vxOutArgs = CM_setOutArgs(context);

        if (tidlObj->vxOutArgs == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "CM_readConfig() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)tidlObj->vxOutArgs,
                               "TIDL_outArgs");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* TIDL kernel init*/
        tidlObj->vxKernel = tivxAddKernelTIDL(context,
                                              tidlObj->numInTensors,
                                              tidlObj->numOutTensors);

        if (tidlObj->vxKernel == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxAddKernelTIDL() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)tidlObj->vxKernel,
                               "TIDLKernel");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->vxOutputTensor =
            alloc2DTensorArray(tidlObj->pipelineDepth,
                               tidlObj->numOutTensors,
                               &tidlObj->vxOutTensorMemSize);

        if (tidlObj->vxOutputTensor == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "alloc2DTensorArray() failed.");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->vxOutputTensorRefs =
            alloc2DTensorArray(tidlObj->numOutTensors,
                               tidlObj->pipelineDepth,
                               &tidlObj->vxOutTensorRefMemSize);

        if (tidlObj->vxOutputTensorRefs == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "alloc2DTensorArray() failed.");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (uint32_t i = 0; i < tidlObj->pipelineDepth; i++)
        {
            vxStatus =
                CM_tidlNodeCnxtCreateOutputTensors(context,
                                                   tidlObj->vxConfig,
                                                   tidlObj->vxOutputTensor[i]);

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                for (uint32_t j = 0; j < tidlObj->numOutTensors; j++)
                {
                    vxSetReferenceName((vx_reference)tidlObj->vxOutputTensor[i][j],
                                       "TIDLOutputTensor");

                    tidlObj->vxOutputTensorRefs[j][i] =
                                 tidlObj->vxOutputTensor[i][j];
                }
            }
            else
            {
                VX_PRINT(VX_ZONE_ERROR,
                         "CM_tidlNodeCnxtCreateOutputTensors() failed");
                vxStatus = VX_FAILURE;
                break;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->outTensorBaseParamIdx =
            TIVX_KERNEL_TIDL_NUM_BASE_PARAMETERS +
            tidlObj->numInTensors;

        tidlObj->state = CM_TIDL_NODE_CNTXT_STATE_INIT;
    }

    return vxStatus;
}

vx_status CM_tidlNodeCntxtSetup(
        CM_TIDLNodeCntxt   *tidlObj,
        vx_context          context,
        vx_graph            graph,
        vx_tensor          *inputTensor)
{
    vx_reference    params[CM_TIDL_MAX_PARAMS];
    vx_status       vxStatus = VX_SUCCESS;
    uint32_t        num_params = 0;

    if (tidlObj == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'tidlObj' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (context == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'context' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (graph == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'graph' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (inputTensor == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'inputTensor' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (tidlObj->state != CM_TIDL_NODE_CNTXT_STATE_INIT)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid state.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* The 1st param MUST be config array */
        params[num_params++] = (vx_reference)tidlObj->vxConfig;

        /* The 2nd param MUST be network tensor */
        params[num_params++] = (vx_reference)tidlObj->vxNetwork;

        /* The 3rd param MUST be create params */
        params[num_params++] = (vx_reference)tidlObj->vxCreateParams;

        /* The 4th param MUST be inArgs */
        params[num_params++] = (vx_reference)tidlObj->vxInArgs;

        /* The 5th param MUST be outArgs */
        params[num_params++] = (vx_reference)tidlObj->vxOutArgs;

        /* The 6th param MUST be NULL if trace data dump is not enabled */
        params[num_params++] = (vx_reference)NULL;

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (num_params > CM_TIDL_MAX_PARAMS)
            {
                VX_PRINT(VX_ZONE_ERROR,
                         "num_params [%d] exceeds CM_TIDL_MAX_PARAMS "
                         "[%d]. Please adjust the constant.",
                         num_params,
                         CM_TIDL_MAX_PARAMS);

                vxStatus = VX_FAILURE;
            }
        }
    }

    /************************/
    /*      TIDL node       */
    /************************/
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->vxNode = tivxTIDLNode(graph,
                                       tidlObj->vxKernel,
                                       params,
                                       inputTensor,
                                       tidlObj->vxOutputTensor[0]);

        if (tidlObj->vxNode == NULL)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxTIDLNode() failed");
            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)tidlObj->vxNode,
                               "TIDLNode");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        tidlObj->state = CM_TIDL_NODE_CNTXT_STATE_SETUP;
    }

    return vxStatus;
}

vx_status CM_tidlNodeCnxtCreateInputTensors(
        vx_context                  context,
        const vx_user_data_object   config,
        vx_int32                    pad_in_tidl,
        vx_tensor                  *tensors)
{
    tivxTIDLJ7Params   *tidlParams;
    sTIDL_IOBufDesc_t  *ioBufDesc;
    vx_size             input_sizes[CM_MAX_TENSOR_DIMS];
    vx_map_id           map_id_vxConfig;
    int32_t             id;
    vx_status           vxStatus;

    vxStatus = VX_SUCCESS;

    if (context == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'context' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (config == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'config' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (tensors == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'tensors' NULL.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxMapUserDataObject(config,
                                       0,
                                       sizeof(tivxTIDLJ7Params),
                                       &map_id_vxConfig,
                                       (void **)&tidlParams,
                                       VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapUserDataObject() failed.");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

        if (ioBufDesc->numInputBuf > CM_MAX_TENSOR_DIMS)
        {
            VX_PRINT(VX_ZONE_ERROR,
                     "ioBufDesc->numInputBuf [%d] exceeds CM_MAX_TENSOR_DIMS "
                     "[%d]. Please adjust the constant.",
                     ioBufDesc->numInputBuf,
                     CM_MAX_TENSOR_DIMS);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vx_enum data_type;

        for (id = 0; id < ioBufDesc->numInputBuf; id++)
        {
            if (pad_in_tidl == 0x0)
            {
                input_sizes[0] = ioBufDesc->inWidth[id]  +
                                 ioBufDesc->inPadL[id] +
                                 ioBufDesc->inPadR[id];

                input_sizes[1] = ioBufDesc->inHeight[id] +
                                 ioBufDesc->inPadT[id] +
                                 ioBufDesc->inPadB[id];
            }
            else
            {
                /* If padding is done in TIDL then input tensor should be
                 * allocated without padding.
                 */
                input_sizes[0] = ioBufDesc->inWidth[id];
                input_sizes[1] = ioBufDesc->inHeight[id];
            }

            input_sizes[2] = ioBufDesc->inNumChannels[id];

            data_type = getVxTensorDatatype(ioBufDesc->inElementType[id]);

            if (data_type == (vx_enum)VX_TYPE_INVALID)
            {
                VX_PRINT(VX_ZONE_ERROR, "getVxTensorDatatype() failed");
                vxStatus = VX_FAILURE;
                break;
            }

            tensors[id] = vxCreateTensor(context,
                                         3,
                                         input_sizes,
                                         data_type,
                                         0);

            if (tensors[id] == NULL)
            {
                VX_PRINT(VX_ZONE_ERROR, "vxCreateTensor() failed");
                vxStatus = VX_FAILURE;
                break;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxUnmapUserDataObject(config, map_id_vxConfig);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxUnmapUserDataObject() failed");
        }
    }

    return vxStatus;
}

vx_status CM_tidlNodeCnxtCreateOutputTensors(
        vx_context                  context,
        const vx_user_data_object   config,
        vx_tensor                  *tensors)
{
    tivxTIDLJ7Params   *tidlParams;
    sTIDL_IOBufDesc_t  *ioBufDesc;
    vx_size             output_sizes[CM_MAX_TENSOR_DIMS];
    vx_map_id           map_id_vxConfig;
    int32_t             id;
    vx_status           vxStatus;

    vxStatus = VX_SUCCESS;

    if (context == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'context' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (config == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'config' NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (tensors == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'tensors' NULL.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxMapUserDataObject(config,
                                       0,
                                       sizeof(tivxTIDLJ7Params),
                                       &map_id_vxConfig,
                                       (void **)&tidlParams,
                                       VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxMapUserDataObject() failed.");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

        if (ioBufDesc->numOutputBuf > CM_MAX_TENSOR_DIMS)
        {
            VX_PRINT(VX_ZONE_ERROR,
                     "ioBufDesc->numOutputBuf [%d] exceeds CM_MAX_TENSOR_DIMS "
                     "[%d]. Please adjust the constant.",
                     ioBufDesc->numOutputBuf,
                     CM_MAX_TENSOR_DIMS);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vx_enum data_type;

        for (id = 0; id < ioBufDesc->numOutputBuf; id++)
        {
            output_sizes[0] = ioBufDesc->outWidth[id]  +
                              ioBufDesc->outPadL[id] +
                              ioBufDesc->outPadR[id];

            output_sizes[1] = ioBufDesc->outHeight[id] +
                              ioBufDesc->outPadT[id] +
                              ioBufDesc->outPadB[id];

            output_sizes[2] = ioBufDesc->outNumChannels[id];

            data_type = getVxTensorDatatype(ioBufDesc->outElementType[id]);

            if (data_type == (vx_enum)VX_TYPE_INVALID)
            {
                VX_PRINT(VX_ZONE_ERROR, "getVxTensorDatatype() failed");
                vxStatus = VX_FAILURE;
                break;
            }

            tensors[id] = vxCreateTensor(context,
                                         3,
                                         output_sizes,
                                         data_type,
                                         0);

            if (tensors[id] == NULL)
            {
                VX_PRINT(VX_ZONE_ERROR, "vxCreateTensor() failed");
                vxStatus = VX_FAILURE;
                break;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxUnmapUserDataObject(config, map_id_vxConfig);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "vxUnmapUserDataObject() failed");
        }
    }

    return vxStatus;
}

vx_status CM_tidlNodeCntxtDeInit(
        CM_TIDLNodeCntxt  *tidlObj)
{
    vx_status   vxStatus = VX_SUCCESS;

    if (tidlObj == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Parameter 'tidlObj' NULL.");
        vxStatus = VX_FAILURE;
    }
    else
    {
        if (tidlObj->state == CM_TIDL_NODE_CNTXT_STATE_INVALID)
        {
            VX_PRINT(VX_ZONE_ERROR, "Invalid state.");
            vxStatus = VX_FAILURE;
        }
        else
        {
            /* TIDL Graph */
            if (tidlObj->vxNode != NULL)
            {
                vxReleaseNode(&tidlObj->vxNode);
            }

            if (tidlObj->vxConfig != NULL)
            {
                vxReleaseUserDataObject(&tidlObj->vxConfig);
            }

            if (tidlObj->vxNetwork != NULL)
            {
                vxReleaseUserDataObject(&tidlObj->vxNetwork);
            }

            if (tidlObj->vxCreateParams != NULL)
            {
                vxReleaseUserDataObject(&tidlObj->vxCreateParams);
            }

            if (tidlObj->vxInArgs != NULL)
            {
                vxReleaseUserDataObject(&tidlObj->vxInArgs);
            }

            if (tidlObj->vxOutArgs != NULL)
            {
                vxReleaseUserDataObject(&tidlObj->vxOutArgs);
            }

            /* Release the output tensor array. */
            if (tidlObj->vxOutputTensor)
            {
                for (uint32_t i = 0; i < tidlObj->pipelineDepth; i++)
                {
                    for (uint32_t j = 0; j < tidlObj->numOutTensors; j++)
                    {
                        if (tidlObj->vxOutputTensor[i][j] != NULL)
                        {
                            vxReleaseTensor(&tidlObj->vxOutputTensor[i][j]);
                        }
                    }
                }

                /* Release the output tensor array. */
                tivxMemFree(tidlObj->vxOutputTensor,
                            tidlObj->vxOutTensorMemSize,
                            TIVX_MEM_EXTERNAL);

                tidlObj->vxOutputTensor = NULL;
            }

            /* Release the output tensor ref array. */
            if (tidlObj->vxOutputTensorRefs)
            {
                /* Release the output tensor array. */
                tivxMemFree(tidlObj->vxOutputTensorRefs,
                            tidlObj->vxOutTensorRefMemSize,
                            TIVX_MEM_EXTERNAL);

                tidlObj->vxOutputTensorRefs = NULL;
            }

            if (tidlObj->vxKernel != NULL)
            {
                vxRemoveKernel(tidlObj->vxKernel);
            }
        }

        tidlObj->state = CM_TIDL_NODE_CNTXT_STATE_INVALID;
    }

    return vxStatus;
}

