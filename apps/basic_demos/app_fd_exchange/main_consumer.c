/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <signal.h>
#include <getopt.h>

#include <VX/vx.h>
#include <TI/tivx.h>

#include <app_init.h>
#include "app_common.h"

typedef struct
{
    /** Verbose flag. */
    uint8_t     verbose;

} App_CmdLineParams;

typedef struct
{
    /** Initialization status. */
    uint8_t                 initDone;

    /** Test pattern tracker. */
    uint32_t                testPattern;

    /* VX context. */
    vx_context              vxContext;

    /** Socket Id. */
    int32_t                 sockId;

    /** Number of image objects. */
    uint32_t                numValidObjs;

    /** Image objects. */
    vx_image                images[APP_MAX_IMAGE_INFO];

    /* Command buffer memory. */
    uint8_t                 cmdBuff[APP_MAX_MSG_BUFF_SIZE];

    /* Response buffer memory. */
    uint8_t                 rspBuff[APP_MAX_MSG_BUFF_SIZE];

} App_Context;

static App_Context gAppCntxt;

static uint32_t App_getNumPlanes(vx_df_image format)
{
    uint32_t numPlanes = 0;

    switch (format)
    {
        case VX_DF_IMAGE_U8:
        case VX_DF_IMAGE_U16:
        case VX_DF_IMAGE_S16:
        case VX_DF_IMAGE_U32:
        case VX_DF_IMAGE_S32:
        case VX_DF_IMAGE_RGB:
        case VX_DF_IMAGE_RGBX:
        case VX_DF_IMAGE_YUYV:
        case VX_DF_IMAGE_UYVY:
            numPlanes = 1;
            break;

        case VX_DF_IMAGE_NV12:
        case VX_DF_IMAGE_NV21:
            numPlanes = 2;
            break;

        case VX_DF_IMAGE_YUV4:
        case VX_DF_IMAGE_IYUV:
            numPlanes = 3;
            break;

        default:
            break;
    }

    return numPlanes;
}

static int32_t App_DeAllocImageObjects(App_Context *appCntxt)
{
    uint32_t    i;

    for (i = 0; i < appCntxt->numValidObjs; i++)
    {
        vxReleaseImage(&appCntxt->images[i]);
    }

    return 0;
}

int32_t App_createObjFromBuffInfo(App_Context              *appCntxt,
                                  const App_ImageBuffDesc  *imgBuffDesc,
                                  int32_t                  *fd,
                                  uint32_t                  numFd)
{
    const App_ImageDesc    *imgDesc;
    const App_ImageParams  *imgParams;
    void                   *phyAddr;
    void                   *ptrs[APP_MAX_IMAGE_PLANE_MAX];
    int32_t                 status;
    vx_status               vxStatus;
    uint32_t                objNum;
    uint32_t                numPlanes;
    uint32_t                j;

    status    = 0;
    objNum    = appCntxt->numValidObjs;
    imgDesc   = &imgBuffDesc->info;
    imgParams = &imgDesc->imgParams;
    numPlanes = App_getNumPlanes(imgParams->format);
    #ifdef QNX
    numFd     = imgParams->numFd;
    #endif

    if (numPlanes != numFd)
    {
        VX_PRINT(VX_ZONE_ERROR, "NumFd and numPlane mis-match\n");
        status = -1;
    }

    if (status == 0)
    {
         /* Create a new image. */
         appCntxt->images[objNum] = vxCreateImage(appCntxt->vxContext,
                                                  imgParams->width,
                                                  imgParams->height,
                                                  imgParams->format);

         if (appCntxt->images[objNum] == NULL)
         {
             VX_PRINT(VX_ZONE_ERROR,
                      "vxCreateImage() failed for Obj [%d]\n", objNum);
             status = -1;
         }
    }

    if (status == 0)
    {
        appCntxt->numValidObjs++;

        VX_PRINT(VX_ZONE_INFO, "Consumer app: OBJ [%d]\n", objNum);
        VX_PRINT(VX_ZONE_INFO, "\tWidth     = %d\n", imgParams->width);
        VX_PRINT(VX_ZONE_INFO, "\tHeight    = %d\n", imgParams->height);
        VX_PRINT(VX_ZONE_INFO, "\tFormat    = %d\n", imgParams->format);
        VX_PRINT(VX_ZONE_INFO, "\tNumPlanes = %d\n", numPlanes);

        for (j = 0; j < numPlanes; j++)
        {
            #ifdef QNX
            vxStatus = tivxMemTranslateFd((uint64_t)imgParams->fd[j],
                              imgDesc->size[j],
                              &ptrs[j],
                              &phyAddr);
            #else
            vxStatus = tivxMemTranslateFd((uint64_t)fd[j],
                                          imgDesc->size[j],
                                          &ptrs[j],
                                          &phyAddr);
            #endif
            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                VX_PRINT(VX_ZONE_ERROR, "tivxMemTranslateFd() failed for "
                         "image [%d] plane [%d]\n", objNum, j);
                status = -1;
                break;
            }

            VX_PRINT(VX_ZONE_INFO, "\tPLANE [%d]:\n", j);
            #ifdef QNX
            VX_PRINT(VX_ZONE_INFO, "\t\tFD        = %p\n", imgParams->fd[j]);
            #else
            VX_PRINT(VX_ZONE_INFO, "\t\tFD        = %ld\n", fd[j]);
            #endif
            VX_PRINT(VX_ZONE_INFO, "\t\tVIRT ADDR = %p\n", ptrs[j]);
            VX_PRINT(VX_ZONE_INFO, "\t\tPHY ADDR  = %p\n", phyAddr);
            VX_PRINT(VX_ZONE_INFO, "==================\n\n");

            /* Check the pattern. */
            {
                uint32_t   *t = (uint32_t*)ptrs[j];

                if (*t != appCntxt->testPattern)
                {
                    VX_PRINT(VX_ZONE_ERROR,
                             "\tPattern mismatch. "
                             "Expected [0x%8.8X] Got [0x%8.8x]\n",
                             appCntxt->testPattern, *t);
                    status = -1;
                    break;
                }

                appCntxt->testPattern *= (j+1);
            }

        }
    }

    if (status == 0)
    {
        /* Import the reference handles. */
        vxStatus =
            tivxReferenceImportHandle((vx_reference)appCntxt->images[objNum],
                                      (const void **)ptrs,
                                      (const uint32_t *)imgDesc->size,
                                      numPlanes);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR,
                     "tivxReferenceImportHandle() failed for image [%d]\n",
                     objNum);
            status = -1;
        }
        else
        {
            VX_PRINT(VX_ZONE_INFO,
                     "tivxReferenceImportHandle() success for image [%d]\n",
                     objNum);
        }
    }

    return status;
}

static void App_intSigHandler(int sig)
{
    App_Context    *appCntxt = &gAppCntxt;

    if (appCntxt->sockId >= 0)
    {
        close(appCntxt->sockId);
    }

    exit(0);
}

static int32_t App_init(App_Context *appCntxt)
{
    int32_t status = 0;

    if (appCntxt->initDone == 0)
    {
        appCntxt->testPattern = APP_TEST_PATTERN_INIT;

        /* Create a socket connection. */
        appCntxt->sockId = AppUtil_netUnixConnect(APP_UNIX_STRM_PATH_NAME);

        if (appCntxt->sockId < 0)
        {
            status = -1;
        }

        if (status == 0)
        {
            status = appCommonInit();
        }

        if (status == 0)
        {
            tivxInit();
            tivxHostInit();

            appCntxt->vxContext = vxCreateContext();

            if (appCntxt->vxContext == NULL)
            {
                VX_PRINT(VX_ZONE_ERROR, "vxCreateContext() failed.\n");

                status = -1;
            }
            else
            {
                appCntxt->initDone = 1;
            }
        }
    }

    return status;
}

static int32_t App_deInit(App_Context *appCntxt)
{
    int32_t status = 0;

    if (appCntxt->initDone == 1)
    {
        status = vxReleaseContext(&appCntxt->vxContext);
        if (status == 0)
        {
            tivxHostDeInit();
            tivxDeInit();
        }
        if (status == 0)
        {
            status = appCommonDeInit();
        }
        if (appCntxt->sockId >= 0)
        {
            close(appCntxt->sockId);
        }

        appCntxt->initDone = 0;
    }

    return status;
}

int32_t App_msgProcThread(App_Context  *appCntxt)
{
    int32_t             fd[APP_MAX_IMAGE_PLANE_MAX];
    uint32_t            numFd;
    uint32_t           *numFdPtr;
    App_GenericRspMsg  *rspMsg;
    App_MsgHdr         *msgHdr;
    uint32_t            done = 0;
    uint32_t            size;
    int32_t             status = 0;

    rspMsg   = (App_GenericRspMsg *)appCntxt->rspBuff;
    numFdPtr = NULL;
    size     = sizeof(App_MsgHdr);

    while (!done)
    {
        /* Wait for a command. */
        status = AppUtil_netReadUnixSock(appCntxt->sockId,
                                         appCntxt->cmdBuff,
                                         size,
                                         fd,
                                         numFdPtr);

        if (status < 0)
        {
            VX_PRINT(VX_ZONE_ERROR, "AppUtil_netReadUnixSock() failed.\n");
            break;
        }

        msgHdr        = (App_MsgHdr *)appCntxt->cmdBuff;
        rspMsg->msgId = msgHdr->msgId;

        switch (msgHdr->msgId)
        {
            case APP_MSGTYPE_HELLO_CMD:
                VX_PRINT(VX_ZONE_INFO,
                         "CONSUMER::Received [APP_MSGTYPE_HELLO_CMD]\n");
                status = 0;
                numFdPtr = &numFd;
                size = sizeof(App_ImageBuffDesc);
                break;

            case APP_MSGTYPE_IMAGE_BUF_CMD:
                VX_PRINT(VX_ZONE_INFO,
                         "CONSUMER::Received [APP_MSGTYPE_IMAGE_BUF_CMD]\n");

                {
                    App_ImageBuffDesc  *imgBuffDesc;

                    imgBuffDesc = (App_ImageBuffDesc *)appCntxt->cmdBuff;

                    status = App_createObjFromBuffInfo(appCntxt,
                                                       imgBuffDesc,
                                                       fd,
                                                       numFd);
                    if (imgBuffDesc->lastObj)
                    {
                        numFdPtr = NULL;
                        done = 1;
                    }
                }
                break;

            default:
                VX_PRINT(VX_ZONE_INFO,
                         "CONSUMER::Received [UNKNOWN MESSAGE]\n");
                status = -1;
                break;
        }

        if (status < 0)
        {
            rspMsg->status = APP_CMD_STATUS_FAILURE;
        }
        else
        {
            rspMsg->status = APP_CMD_STATUS_SUCCESS;
        }

        /* Send the response. */
        status = AppUtil_netWriteUnixSock(appCntxt->sockId,
                                          (uint8_t *)rspMsg,
                                          sizeof(App_GenericRspMsg),
                                          NULL,
                                          0);

        if (status < 0)
        {
            VX_PRINT(VX_ZONE_ERROR, "AppUtil_netWriteUnixSock() failed.\n");
        }

    } /* while (!done) */

    return status;
}

static void App_showUsage( char * name)
{
    printf(" \n");
    printf("# \n");
    printf("# %s PARAMETERS [OPTIONAL PARAMETERS]\n", name);
    printf("# OPTIONS:\n");
    printf("#  [--verbose |-v]\n");
    printf("#  [--help    |-h]\n");
    printf("# \n");
    printf("# \n");
    printf("# (c) Texas Instruments 2020\n");
    printf("# \n");
    printf("# \n");

    exit(0);
}

static void
App_parseCmdLineArgs(int32_t            argc,
                     char              *argv[],
                     App_CmdLineParams *clParams)
{
    int32_t              longIndex;
    int32_t              opt;
    static struct option long_options[] = {
        {"help",         no_argument,       0,  'h' },
        {"verbose",      no_argument,       0,  'v' },
        {0,              0,                 0,   0  }
    };

    while ((opt = getopt_long(argc, argv,"hvc:i:", 
                   long_options, &longIndex )) != -1) {
        switch (opt)
        {
            case 'v' :
                clParams->verbose = 1;
                break;

            case 'h' :
            default:
                App_showUsage(argv[0]);

        } // switch (opt)

    } // while ((opt = getopt_long(argc, argv

    return;
}

int main(int argc, char *argv[])
{
    App_Context        *appCntxt = &gAppCntxt;
    App_CmdLineParams   cmdParams = {0};
    int32_t             status = 0;
    int32_t             status_before_deinit = 0;

    /* Register the signal handler. */
    signal(SIGINT, App_intSigHandler);

    memset(appCntxt, 0, sizeof(App_Context));

    /* Parse command line arguments. */
    App_parseCmdLineArgs(argc, argv, &cmdParams);

    status = App_init(appCntxt);

    if (status < 0)
    {
        VX_PRINT(VX_ZONE_ERROR, "App_init() failed.\n");
    }

    if (cmdParams.verbose)
    {
        tivx_set_debug_zone(VX_ZONE_INFO);
    }

    if (status == 0)
    {
        status = App_msgProcThread(appCntxt);
    }

    if (status < 0)
    {
        VX_PRINT(VX_ZONE_ERROR, "App_msgProcThread() failed.\n");
    }

    /* Deallocate any allocated objects. */
    App_DeAllocImageObjects(appCntxt);

    /* Do not collect return status here unless already passing */
    if (status != 0)
    {
        status_before_deinit = status;
    }

    status = App_deInit(appCntxt);
    if (status < 0)
    {
        VX_PRINT(VX_ZONE_ERROR, "App_deInit() failed.\n");
    }
    else
    if (status_before_deinit)
    {
        status = status_before_deinit;
    }

    return status;
}

