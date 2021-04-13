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

#define APP_STATE_CLI_NOT_CONNECTED     (0U)
#define APP_STATE_CLI_CONNECTED         (1U)
#define APP_STATE_CLI_OBJINFO_XFERD     (2U)

typedef struct
{
    /** Verbose flag. */
    uint8_t     verbose;

} App_CmdLineParams;

typedef struct
{
    /** Initialization status. */
    uint8_t                 initDone;

    /** Producer state. */
    uint8_t                 state;

    /** Test pattern tracker. */
    uint32_t                testPattern;

    /* VX context. */
    vx_context              vxContext;

    /** IPC handle. */
    AppUtil_NetSrvCntxt     svrCntxt;

    /** Server socket Id. */
    int32_t                 cliSockId;

    /** Number of image objects. */
    uint32_t                numValidObjs;

    /** Image format information.*/
    App_ImageParams         imgFormat[APP_MAX_IMAGE_INFO];

    /** Image objects. */
    vx_image                images[APP_MAX_IMAGE_INFO];

    /* Command buffer memory. */
    uint8_t                 cmdBuff[APP_MAX_MSG_BUFF_SIZE];

    /* Response buffer memory. */
    uint8_t                 rspBuff[APP_MAX_MSG_BUFF_SIZE];

} App_Context;

static App_Context gAppCntxt;

void App_clientHdlr(void *usrCntxt, void *cliSock);

static int32_t App_sendMsg(App_Context     *appCntxt,
                           uint8_t         *cmdBuff,
                           uint32_t         cmdBuffSize,
                           int32_t         *fd,
                           uint32_t         numFd,
                           uint8_t         *rspBuff)
{
    App_MsgHdr         *hdr;
    App_GenericRspMsg  *rsp;
    int32_t             status = 0;

    hdr = (App_MsgHdr *)cmdBuff;
    rsp = (App_GenericRspMsg *)rspBuff;

    /* Send the command. */
    status = AppUtil_netWriteUnixSock(appCntxt->cliSockId,
                                      cmdBuff,
                                      cmdBuffSize,
                                      fd,
                                      numFd);

    if (status == APPUTIL_NET_RET_SUCCESS)
    {
        /* Wait for the response. */
        status = AppUtil_netReadUnixSock(appCntxt->cliSockId,
                                         rspBuff,
                                         sizeof(App_GenericRspMsg),
                                         NULL,
                                         NULL);

        if (status == APPUTIL_NET_RET_SUCCESS)
        {
            if ((rsp->msgId != hdr->msgId) ||
                (rsp->status != APP_CMD_STATUS_SUCCESS))
            {
                VX_PRINT(VX_ZONE_INFO,
                         "PRODUCER::Received failure status from client.\n");
                status = -1;
            }
        }
    }
    else
    {
        VX_PRINT(VX_ZONE_INFO, "PRODUCER::AppUtil_netWriteUnixSock() failed.\n");
    }

    return status;
}

static int32_t App_establishSync(App_Context *appCntxt)
{
    App_MsgHdr *hdr;
    int32_t     status = 0;

    hdr = (App_MsgHdr *)appCntxt->cmdBuff;
    hdr->msgId = APP_MSGTYPE_HELLO_CMD;

    VX_PRINT(VX_ZONE_INFO, "PRODUCER::Sending [APP_MSGTYPE_HELLO_CMD]\n");

    status = App_sendMsg(appCntxt,
                         (uint8_t *)hdr,
                         sizeof(App_MsgHdr),
                         NULL,
                         0,
                         appCntxt->rspBuff);

    if (status == 0)
    {
        VX_PRINT(VX_ZONE_INFO,
                 "PRODUCER::Received [APP_MSGTYPE_HELLO_RSP] with status: "
                 "SUCCESS.\n");
    }

    return status;
}

static void App_intSigHandler(int sig)
{
    App_Context    *appCntxt = &gAppCntxt;

    if (appCntxt->cliSockId >= 0)
    {
        close(appCntxt->cliSockId);
    }

    exit(0);
}

static int32_t App_init(App_Context *appCntxt)
{
    int32_t status = 0;

    if (appCntxt->initDone == 0)
    {
        AppUtil_NetSrvCntxt    *svrCntxt;

        appCntxt->testPattern = APP_TEST_PATTERN_INIT;
        svrCntxt              = &appCntxt->svrCntxt;
        svrCntxt->listenCnt   = 1;
        svrCntxt->pathName    = APP_UNIX_STRM_PATH_NAME;
        svrCntxt->auxData     = (void *)appCntxt;
        svrCntxt->cliHdlr     = App_clientHdlr;

        /* Create the socket. */
        status = AppUtil_netCreateSvrSocket(svrCntxt);

        if (status >= 0)
        {
            status = 0;
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
        vxReleaseContext(&appCntxt->vxContext);
        tivxHostDeInit();
        tivxDeInit();

        appCommonDeInit();

        if (appCntxt->cliSockId >= 0)
        {
            close(appCntxt->cliSockId);
        }

        appCntxt->initDone = 0;
    }

    return status;
}

static int32_t App_DeAllocImageObjects(App_Context *appCntxt)
{
    uint32_t    i;
    int32_t     status = 0;

    for (i = 0; i < appCntxt->numValidObjs; i++)
    {
        vxReleaseImage(&appCntxt->images[i]);
    }

    return status;
}

static int32_t App_allocImageObjects(App_Context *appCntxt)
{
    uint32_t    width;
    uint32_t    height;
    int32_t     status;
    vx_df_image format;
    uint32_t    i;

    status    = 0;
    width     = 1280;
    height    = 720;
    format    = VX_DF_IMAGE_YUV4;

    for (i = 0; i < APP_MAX_IMAGE_INFO; i++, width /= 2, height /= 2)
    {
        appCntxt->imgFormat[i].width  = width;
        appCntxt->imgFormat[i].height = height;
        appCntxt->imgFormat[i].format = format;

        appCntxt->images[i] =
            vxCreateImage(appCntxt->vxContext, width, height, format);

        if (appCntxt->images[i] == NULL)
        {
            status = -1;
            break;
        }

        appCntxt->numValidObjs++;
    }

    return status;
}

static int32_t App_sendObjInfo(App_Context         *appCntxt,
                               App_ImageBuffDesc   *imgBuffDesc)
{
    #ifdef LINUX
    int32_t     fd32[APP_MAX_IMAGE_PLANE_MAX];
    #endif
    uint32_t    numEntries;
    int32_t     status;
    uint32_t    i;

    status = 0;

    imgBuffDesc->msgId   = APP_MSGTYPE_IMAGE_BUF_CMD;
    imgBuffDesc->lastObj = 0;

    for (i = 0; i < appCntxt->numValidObjs; i++)
    {
        App_ImageDesc      *imgDesc;
        App_ImageParams    *imgParams;
        void               *phyAddr;
        void               *ptrs[APP_MAX_IMAGE_PLANE_MAX];
        uint64_t            fd64;
        vx_status           vxStatus;
        uint32_t            j;

        imgDesc           = &imgBuffDesc->info;
        imgParams         = &imgDesc->imgParams;
        imgParams->width  = appCntxt->imgFormat[i].width;
        imgParams->height = appCntxt->imgFormat[i].height;
        imgParams->format = appCntxt->imgFormat[i].format;

        /* Export the handles. */
        vxStatus = tivxReferenceExportHandle((vx_reference)appCntxt->images[i],
                                             ptrs,
                                             imgDesc->size,
                                             APP_MAX_IMAGE_PLANE_MAX,
                                             &numEntries);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "tivxReferenceExportHandle() failed for "
                     "image [%d]\n", i);
            status = -1;
            break;
        }
        
        if (i == (appCntxt->numValidObjs - 1))
        {
            imgBuffDesc->lastObj = 1;
        }

        VX_PRINT(VX_ZONE_INFO, "Producer app: OBJ [%d]\n", i);
        VX_PRINT(VX_ZONE_INFO, "\tWidth     = %d\n", imgParams->width);
        VX_PRINT(VX_ZONE_INFO, "\tHeight    = %d\n", imgParams->height);
        VX_PRINT(VX_ZONE_INFO, "\tFormat    = %d\n", imgParams->format);
        VX_PRINT(VX_ZONE_INFO, "\tNumPlanes = %d\n", numEntries);

        for (j = 0; j < numEntries; j++)
        {
            uint32_t   *p;

            /* Translate the virtual addresses to 'fd'. */
            p = (uint32_t *)ptrs[j];

            *p = appCntxt->testPattern;
            appCntxt->testPattern *= (j+1);

            status = tivxMemTranslateVirtAddr(ptrs[j],
                                              &fd64,
                                              &phyAddr);

            if (status < 0)
            {
                VX_PRINT(VX_ZONE_ERROR,
                         "tivxMemTranslateVirtAddr() failed for "
                         "image [%d] plane [%d]\n", i, j);
                break;
            }

            imgParams->fd[j] = fd64;
            #ifdef LINUX
            fd32[j] = (int32_t)fd64;
            #endif

            VX_PRINT(VX_ZONE_INFO, "\tPLANE [%d]:\n", j);
            #ifdef QNX
            VX_PRINT(VX_ZONE_INFO, "\t\tFD        = %p\n", imgParams->fd[j]);
            #else
            VX_PRINT(VX_ZONE_INFO, "\t\tFD        = %ld\n", fd32[j]);
            #endif
            VX_PRINT(VX_ZONE_INFO, "\t\tVIRT ADDR = %p\n", ptrs[j]);
            VX_PRINT(VX_ZONE_INFO, "\t\tPHY ADDR  = %p\n", phyAddr);
            VX_PRINT(VX_ZONE_INFO, "==================\n\n");

        } /* for (j = 0; j < numEntries; j++) */

        imgParams->numFd = numEntries;

        if (status == 0)
        {
            VX_PRINT(VX_ZONE_INFO,
                     "PRODUCER::Sending [APP_MSGTYPE_IMAGE_BUF_CMD]\n");

            #ifdef QNX
            status = App_sendMsg(appCntxt,
                                 (uint8_t *)imgBuffDesc,
                                 sizeof(App_ImageBuffDesc),
                                 NULL,
                                 0,
                                 appCntxt->rspBuff);
            #else
            status = App_sendMsg(appCntxt,
                                 (uint8_t *)imgBuffDesc,
                                 sizeof(App_ImageBuffDesc),
                                 fd32,
                                 numEntries,
                                 appCntxt->rspBuff);
            #endif
            if (status == 0)
            {
                VX_PRINT(VX_ZONE_INFO,
                         "PRODUCER::Received [APP_MSGTYPE_IMAGE_BUF_RSP] with "
                         "status: SUCCESS.\n");
            }
        }

    } /* for (i = 0; i < imgBuffDesc->numObjs; i++) */

    return status;
}

int32_t App_msgProcThread(App_Context  *appCntxt)
{
    uint32_t    done = 0;
    int32_t     status = 0;

    while (!done)
    {
        if (appCntxt->state == APP_STATE_CLI_NOT_CONNECTED)
        {
            status = App_establishSync(appCntxt);

            if (status < 0)
            {
                break;
            }

            appCntxt->state = APP_STATE_CLI_CONNECTED;
        }
        else if (appCntxt->state == APP_STATE_CLI_CONNECTED)
        {
            App_ImageBuffDesc  *imgBuffDesc;

            imgBuffDesc = (App_ImageBuffDesc *)appCntxt->cmdBuff;

            status = App_sendObjInfo(appCntxt, imgBuffDesc);

            if (status < 0)
            {
                break;
            }

            appCntxt->state = APP_STATE_CLI_OBJINFO_XFERD;
        }
        else if (appCntxt->state == APP_STATE_CLI_OBJINFO_XFERD)
        {
            break;
        }

    } /* while (!done) */

    return status;
}

void App_clientHdlr(void *usrCntxt, void *cliSock)
{
    App_Context    *appCntxt;

    appCntxt = (App_Context *)usrCntxt;

    if (appCntxt == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "usrCntxt is NULL.\n");
    }
    else
    {
        appCntxt->cliSockId = *(uint32_t *)cliSock;

        App_msgProcThread(appCntxt);

        appCntxt->svrCntxt.exitFlag = 1;
    }

    VX_PRINT(VX_ZONE_INFO, "Exiting %s\n", __FUNCTION__);
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
    App_Context        *appCntxt;
    App_CmdLineParams   cmdParams = {0};
    int32_t             status;
    int32_t             status_before_deinit = 0;

    appCntxt = &gAppCntxt;
    status   = 0;

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

    /* Create image objects. */
    if (status == 0)
    {
        status = App_allocImageObjects(appCntxt);
    }

    if (status == 0)
    {
        AppUtil_netIterSvr(&appCntxt->svrCntxt);
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

