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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#include <cm_dlr_node_cntxt.h>

typedef struct
{
    bool                initDone;
    CM_DLRCreateParams  params;
    CM_DLRNodeCntxt     dlrObj;
    char               *inFile;
    char               *outFile;
    float              *dlrInputBuff;
    int32_t            *dlrOutputBuff;
} AppCntxt;

static AppCntxt gAppCntxt;

static void DLRTAPP_showUsage( char * name)
{
    PTK_printf(" \n");
    PTK_printf("# \n");
    PTK_printf("# %s PARAMETERS [OPTIONAL PARAMETERS]\n", name);
    PTK_printf("# OPTIONS:\n");
    PTK_printf("#  --model       |-m Path to the model directory.\n");
    PTK_printf("#  [--input      |-i Input data file name.]\n");
    PTK_printf("#  [--output     |-o Output data file name.]\n");
    PTK_printf("#  [--help       |-h]\n");
    PTK_printf("# \n");
    PTK_printf("# \n");
    PTK_printf("# (c) Texas Instruments 2019\n");
    PTK_printf("# \n");
    PTK_printf("# \n");
    exit(0);
}

void
DLRTAPP_parseCmdLineArgs(int32_t    argc,
                         char      *argv[],
                         AppCntxt  *appCntxt)
{
    CM_DLRCreateParams *params;
    int32_t longIndex;
    int32_t opt;
    static struct option long_options[] = {
        {"help",   no_argument,       0, 'h' },
        {"model",  required_argument, 0, 'm' },
        {"input",  optional_argument, 0, 'i' },
        {"output", optional_argument, 0, 'o' },
        {0,        0,                 0,  0  }
    };

    params = &appCntxt->params;
    params->devType = DLR_DEVTYPE_CPU;

    while ((opt = getopt_long(argc, argv,"hm:i:o:", 
                   long_options, &longIndex )) != -1) {
        switch (opt)
        {
            case 'm' :
                params->modelPath = optarg;
                break;

            case 'i' :
                appCntxt->inFile = optarg;
                break;

            case 'o' :
                appCntxt->outFile = optarg;
                break;

            case 'h' :
            default:
                DLRTAPP_showUsage(argv[0]);
                exit(-1);

        } // switch (opt)

    } // while ((opt = getopt_long(argc, argv

    if (!params->modelPath)
    {
        DLRTAPP_showUsage(argv[0]);
        exit(-1);
    }

    return;

} // End of ParseCmdLineArgs()

static void dumpData(const char *name, const float *buff, uint32_t numSamples)
{
    PTK_printf("%s: [", name);

    for (uint32_t i = 0; i < numSamples; i++)
    {
        PTK_printf(" %f", buff[i]);
    }

    PTK_printf(" ]\n");
}

static void dumpDataInt(const char *name, const int32_t *buff, uint32_t numSamples)
{
    PTK_printf("%s: [", name);

    for (uint32_t i = 0; i < numSamples; i++)
    {
        PTK_printf(" %d", buff[i]);
    }

    PTK_printf(" ]\n");
}

static int32_t DLRTAPP_readInput(const char *filename, float *buff, uint32_t numSamples)
{
    FILE   *fp = NULL;
    int32_t status = 0;

    fp = fopen(filename, "r");

    if (fp == NULL)
    {
        PTK_printf("Failed to open input file %s\n", filename);
        status = -1;
    }

    if (status == 0)
    {
        for (uint32_t i = 0; i < numSamples; i++)
        {
            fscanf(fp, "%f\n", &buff[i]);
        }
    }

    if (fp != NULL)
    {
        fclose(fp);
    }

    return status;
}

static int32_t DLRTAPP_handleData(AppCntxt    *appCntxt)
{
    CM_DLRNodeCntxt    *dlrObj;
    CM_DLRIfInfo       *info;
    uint32_t            inputSize;
    uint32_t            outputSize;
    int32_t             status;

    status = 0;
    dlrObj = &appCntxt->dlrObj;

    /* Allocate input buffer. */
    info = &dlrObj->input.info[0];
    inputSize = info->size;
    appCntxt->dlrInputBuff = (float *)malloc(inputSize * sizeof(float));

    if (appCntxt->dlrInputBuff == NULL)
    {
        PTK_printf("Allocation of %ld bytes for input failed.\n",
                inputSize * sizeof(float));
        status = -1;
    }
    else
    {
        PTK_printf("Allocated of %ld bytes for input.\n",
                inputSize * sizeof(float));
    }

    if (status == 0)
    {
        /* Allocate input buffer. */
        info = &dlrObj->output.info[0];
        outputSize = info->size;
        appCntxt->dlrOutputBuff = (int32_t *)malloc(outputSize * sizeof(int32_t));

        if (appCntxt->dlrOutputBuff == NULL)
        {
            PTK_printf("Allocation of %ld bytes for output failed.\n",
                    outputSize * sizeof(float));
            status = -1;
        }
        else
        {
            PTK_printf("Allocated of %ld bytes for output.\n",
                    outputSize * sizeof(float));
        }
    }

    /* Read input file. */
    if (status == 0)
    {
        DLRTAPP_readInput(appCntxt->inFile, appCntxt->dlrInputBuff, inputSize);

        dumpData("INPUT", appCntxt->dlrInputBuff, 15);
    }

    /* Process the data. */
    if (status == 0)
    {
        CM_DLRNodeInputInfo    *dlrInput = &dlrObj->input;
        CM_DLRNodeOutputInfo   *dlrOutput = &dlrObj->output;

        dlrInput->info[0].data  = appCntxt->dlrInputBuff;
        dlrOutput->info[0].data = appCntxt->dlrOutputBuff;

        status = CM_dlrNodeCntxtProcess(dlrObj, dlrInput, dlrOutput);

        if (status < 0)
        {
            PTK_printf("[%s:%d] CM_dlrNodeCntxtProcess() failed.\n",
                        __FUNCTION__, __LINE__);
        }
    }

    /* Output a sample of the output. */
    if (status == 0)
    {
        dumpDataInt("OUTPUT", appCntxt->dlrOutputBuff, 15);
    }

    /* Save the output. */
    if ((status == 0) && (appCntxt->outFile != NULL))
    {
        FILE   *fp;

        fp = fopen(appCntxt->outFile, "w");

        if (fp == NULL)
        {
            PTK_printf("Failed to open input file %s\n", appCntxt->outFile);
            status = -1;
        } else
        {
            for (uint32_t i = 0; i < outputSize; i++)
            {
                fprintf(fp, "%d\n", appCntxt->dlrOutputBuff[i]);
            }

            fclose(fp);
        }
    }

    return status;
}

static void DLRTAPP_cleanup(AppCntxt   *appCntxt)
{
    if (appCntxt->initDone == true)
    {
        if (appCntxt->dlrInputBuff)
        {
            free(appCntxt->dlrInputBuff);
        }

        if (appCntxt->dlrOutputBuff)
        {
            free(appCntxt->dlrOutputBuff);
        }

        if (appCntxt->initDone == true)
        {
            int32_t status;

            status = CM_dlrNodeCntxtDeInit(&appCntxt->dlrObj);

            if (status < 0)
            {
                PTK_printf("CM_dlrNodeCntxtDeInit() failed.\n");
            }
        }

        appCntxt->initDone = false;
    }
}

int main(int argc, char **argv)
{
    AppCntxt   *appCntxt = &gAppCntxt;
    PTK_CRT     ptkConfig;
    int32_t     status;

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    PTK_init(&ptkConfig);

    /* Initialize the context. */
    memset(appCntxt, 0, sizeof(AppCntxt));

    DLRTAPP_parseCmdLineArgs(argc, argv, appCntxt);

    status = CM_dlrNodeCntxtInit(&appCntxt->dlrObj, &appCntxt->params);

    if (status < 0)
    {
        PTK_printf("CM_dlrNodeCntxtInit() failed.\n");
    }
    else
    {
        appCntxt->initDone = true;
        status = CM_dlrNodeCntxtDumpInfo(&appCntxt->dlrObj);

        if (status < 0)
        {
            PTK_printf("CM_dlrNodeCntxtDumpInfo() failed.\n");
        }
    }

    /* Check if we have input data to process. */
    if (appCntxt->inFile != NULL)
    {
        status = DLRTAPP_handleData(appCntxt);

        if (status < 0)
        {
            PTK_printf("handleData() failed.\n");
        }
    }

    DLRTAPP_cleanup(appCntxt);

    return status;
}

