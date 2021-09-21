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

#include "app_ethfw_priv.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#define ETHAPP_LWIP_TASK_STACKSIZE      (4U * 1024U)

static uint8_t gEthAppLwipStackBuf[ETHAPP_LWIP_TASK_STACKSIZE] __attribute__ ((section(".bss:taskStackSection"))) __attribute__((aligned(32)));

/* lwIP features that EthFw relies on */
#ifndef LWIP_IPV4
#error "LWIP_IPV4 is not enabled"
#endif
#ifndef LWIP_NETIF_STATUS_CALLBACK
#error "LWIP_NETIF_STATUS_CALLBACK is not enabled"
#endif
#ifndef LWIP_NETIF_LINK_CALLBACK
#error "LWIP_NETIF_LINK_CALLBACK is not enabled"
#endif

/* DHCP or static IP */
#define ETHAPP_LWIP_USE_DHCP            (1)
#if !ETHAPP_LWIP_USE_DHCP
#define ETHFW_SERVER_IPADDR(addr)       IP4_ADDR((addr), 192,168,1,200)
#define ETHFW_SERVER_GW(addr)           IP4_ADDR((addr), 192,168,1,1)
#define ETHFW_SERVER_NETMASK(addr)      IP4_ADDR((addr), 255,255,255,0)
#endif

static EthAppObj gEthAppObj =
{
    .enetType = ENET_CPSW_9G,
    .hEthFw = NULL,
    .hUdmaDrv = NULL,
    .instId   = 0U,
};

static EthFw_Port gEthAppPorts[] =
{
    {
        .portNum    = ENET_MAC_PORT_1,
        .vlanCfg = { .portPri = 0U, .portCfi = 0U, .portVID = 0U },
    },
#if defined(SOC_J721E)
    /* On J721E EVM to use all 8 ports simultaneously, we use below configuration
       RGMII Ports - 1,3,4,8. QSGMII ports - 2,5,6,7 */
    {
        .portNum    = ENET_MAC_PORT_3, /* RGMII */
        .vlanCfg = { .portPri = 0U, .portCfi = 0U, .portVID = 0U }
    },
    {
        .portNum    = ENET_MAC_PORT_4, /* RGMII */
        .vlanCfg = { .portPri = 0U, .portCfi = 0U, .portVID = 0U }
    },
    {
        .portNum    = ENET_MAC_PORT_8, /* RGMII */
        .vlanCfg = { .portPri = 0U, .portCfi = 0U, .portVID = 0U }
    },
#if defined(ENABLE_QSGMII_PORTS) //kept it disabled for 6.2
    {
        .portNum    = ENET_MAC_PORT_2, /* QSGMII main */
        .vlanCfg = { .portPri = 0U, .portCfi = 0U, .portVID = 0U }
    },
    {
        .portNum    = ENET_MAC_PORT_5, /* QSGMII sub */
        .vlanCfg = { .portPri = 0U, .portCfi = 0U, .portVID = 0U }
    },
    {
        .portNum    = ENET_MAC_PORT_6, /* QSGMII sub */
        .vlanCfg = { .portPri = 0U, .portCfi = 0U, .portVID = 0U }
    },
    {
        .portNum    = ENET_MAC_PORT_7, /* QSGMII sub */
        .vlanCfg = { .portPri = 0U, .portCfi = 0U, .portVID = 0U }
    },
#endif
#endif
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EthApp_lwipMain(void *a0,
                            void *a1);

static void EthApp_initLwip(void *arg);

static void EthApp_initNetif(void);

static void EthApp_netifStatusCb(struct netif *netif);

static int32_t EthApp_initEthFw(void);

static void EthApp_startSwInterVlan(char *recvBuff,
                                    char *sendBuff);

static void EthApp_startHwInterVlan(char *recvBuff,
                                    char *sendBuff);

void appEthFwEarlyInit()
{
    SemaphoreP_Params semParams;

    /* Create semaphore used to synchronize EthFw and NDK init.
     * EthFw opens the CPSW driver which is required by NDK during NIMU
     * initialization, hence EthFw init must complete first.
     * Currently, there is no control over NDK initialization time and its
     * task runs right away after BIOS_start() hence causing a race
     * condition with EthFw init */
    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    gEthAppObj.hInitSem = SemaphoreP_create(0, &semParams);
}

int32_t appEthFwInit()
{
    int32_t status = ETHAPP_OK;

    appLogPrintf("ETHFW: Init ... !!!\n");

    gEthAppObj.coreId = EnetSoc_getCoreId();

    /* Board related initialization */
    EnetBoard_initEthFw();
    EnetAppUtils_enableClocks(gEthAppObj.enetType, gEthAppObj.instId);

    /* Open UDMA driver */
    gEthAppObj.hUdmaDrv = appUdmaGetObj();
    if (gEthAppObj.hUdmaDrv == NULL)
    {
        appLogPrintf("ETHFW: ERROR: failed to open UDMA driver\n");
        status = -1;
    }

    /* Initialize Ethernet Firmware */
    if (status == ETHAPP_OK)
    {
        status = EthApp_initEthFw();
    }

    /* Initialize lwIP */
    if (status == ENET_SOK)
    {
        TaskP_Params taskParams;

        TaskP_Params_init(&taskParams);
        taskParams.priority  = DEFAULT_THREAD_PRIO;
        taskParams.stack     = &gEthAppLwipStackBuf[0];
        taskParams.stacksize = sizeof(gEthAppLwipStackBuf);
        taskParams.name      = "lwIP main loop";

        TaskP_create(EthApp_lwipMain, &taskParams);
    }

    if (status == ETHAPP_OK)
    {
        appLogPrintf("ETHFW: Init ... DONE !!!\n");
    }
    else
    {
        appLogPrintf("ETHFW: Init ... ERROR !!!\n");
    }


    return status;
}

int32_t appEthFwDeInit()
{
    int32_t status = 0;

    EthFw_deinit(gEthAppObj.hEthFw);

    return status;
}

int32_t appEthFwRemoteServerInit()
{
    int32_t status;

    appLogPrintf("ETHFW: Remove server Init ... !!!\n");

    /* Initialize the Remote Config server (CPSW Proxy Server) */
    status = EthFw_initRemoteConfig(gEthAppObj.hEthFw);
    if (status != ENET_SOK)
    {
        appLogPrintf("ETHFW: Remove server Init ... ERROR (%d) !!! \n", status);
    }
    else
    {
        appLogPrintf("ETHFW: Remove server Init ... DONE !!!\n");
    }

    return status;
}

void LwipifEnetAppCb_getHandle(LwipifEnetAppIf_GetHandleInArgs *inArgs,
                               LwipifEnetAppIf_GetHandleOutArgs *outArgs)
{
    /* Wait for EthFw to be initialized */
    SemaphoreP_pend(gEthAppObj.hInitSem, SemaphoreP_WAIT_FOREVER);

    EthFwCallbacks_lwipifCpswGetHandle(inArgs, outArgs);

    /* Save host port MAC address */
    EnetUtils_copyMacAddr(&gEthAppObj.hostMacAddr[0U],
                          &outArgs->rxInfo[0U].macAddr[0U]);
}

void LwipifEnetAppCb_releaseHandle(LwipifEnetAppIf_ReleaseHandleInfo *releaseInfo)
{
    EthFwCallbacks_lwipifCpswReleaseHandle(releaseInfo);
}

static int32_t EthApp_initEthFw(void)
{
    EthFw_Version ver;
    EthFw_Config ethFwCfg;
    EnetUdma_Cfg dmaCfg;
    int32_t status = ETHAPP_OK;
    int32_t i;

    /* Set EthFw config params */
    EthFw_initConfigParams(gEthAppObj.enetType, &ethFwCfg);
    ethFwCfg.ports = &gEthAppPorts[0];
    ethFwCfg.numPorts = ARRAY_SIZE(gEthAppPorts);

    dmaCfg.rxChInitPrms.dmaPriority = UDMA_DEFAULT_RX_CH_DMA_PRIORITY;
    dmaCfg.hUdmaDrv = gEthAppObj.hUdmaDrv;
    ethFwCfg.cpswCfg.dmaCfg = (void *)&dmaCfg;

    /* Overwrite config params with those for hardware interVLAN */
    EthHwInterVlan_setOpenPrms(&ethFwCfg.cpswCfg);

    for (i = 0U; i < ethFwCfg.numPorts; i++)
    {
        EthHwInterVlan_setVlanConfig(&ethFwCfg.ports[i].vlanCfg,
                                     ethFwCfg.ports[i].portNum);
    }

    /* Initialize the EthFw */
    gEthAppObj.hEthFw = EthFw_init(gEthAppObj.enetType, &ethFwCfg);
    if (gEthAppObj.hEthFw == NULL)
    {
        appLogPrintf("ETHFW: ERROR: failed to initialize the firmware\n");
        status = ETHAPP_ERROR;
    }

    /* Get and print EthFw version */
    if (status == ETHAPP_OK)
    {
        EthFw_getVersion(gEthAppObj.hEthFw, &ver);
        appLogPrintf("ETHFW: Version   : %d.%02d.%02d\n", ver.major, ver.minor, ver.rev);
        appLogPrintf("ETHFW: Build Date: %s %s, %s\n", ver.month, ver.date, ver.year);
        appLogPrintf("ETHFW: Build Time: %s:%s:%s\n", ver.hour, ver.min, ver.sec);
        appLogPrintf("ETHFW: Commit SHA: %s\n\n", ver.commitHash);
    }

    /* Post semaphore so that NDK/NIMU can continue with their initialization */
    SemaphoreP_post(gEthAppObj.hInitSem); /* need to create this */

    return status;
}

/* NIMU callbacks (exact name required) */

bool EthFwCallbacks_isPortLinked(struct netif *netif,
                                 Enet_Handle hEnet)
{
    bool linked = false;
    uint32_t i;

    /* Report port linked as long as any port owned by EthFw is up */
    for (i = 0U; (i < ARRAY_SIZE(gEthAppPorts)) && !linked; i++)
    {
        linked = EnetAppUtils_isPortLinkUp(hEnet,
                                           gEthAppObj.coreId,
                                           gEthAppPorts[i].portNum);
    }

    return linked;
}

static void EthApp_lwipMain(void *a0,
                            void *a1)
{
    err_t err;
    sys_sem_t initSem;

    /* initialize lwIP stack and network interfaces */
    err = sys_sem_new(&initSem, 0);
    LWIP_ASSERT("failed to create initSem", err == ERR_OK);
    LWIP_UNUSED_ARG(err);

    tcpip_init(EthApp_initLwip, &initSem);

    /* we have to wait for initialization to finish before
     * calling update_adapter()! */
    sys_sem_wait(&initSem);
    sys_sem_free(&initSem);

#if (LWIP_SOCKET || LWIP_NETCONN) && LWIP_NETCONN_SEM_PER_THREAD
    netconn_thread_init();
#endif
}

static void EthApp_initLwip(void *arg)
{
    sys_sem_t *initSem;

    LWIP_ASSERT("arg != NULL", arg != NULL);
    initSem = (sys_sem_t*)arg;

    /* init randomizer again (seed per thread) */
    srand((unsigned int)sys_now()/1000);

    /* init network interfaces */
    EthApp_initNetif();

    sys_sem_signal(initSem);
}

static void EthApp_initNetif(void)
{
    ip4_addr_t ipaddr, netmask, gw;
#if ETHAPP_LWIP_USE_DHCP
    err_t err;
#endif

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

#if ETHAPP_LWIP_USE_DHCP
    appLogPrintf("Starting lwIP, local interface IP is dhcp-enabled\n");
#else /* ETHAPP_LWIP_USE_DHCP */
    ETHFW_SERVER_GW(&gw);
    ETHFW_SERVER_IPADDR(&ipaddr);
    ETHFW_SERVER_NETMASK(&netmask);
    appLogPrintf("Starting lwIP, local interface IP is %s\n", ip4addr_ntoa(&ipaddr));
#endif /* ETHAPP_LWIP_USE_DHCP */

    init_default_netif(&ipaddr, &netmask, &gw);

    netif_set_status_callback(netif_default, EthApp_netifStatusCb);

    dhcp_set_struct(netif_default, &gEthAppObj.dhcpNetif);

    netif_set_up(netif_default);

#if ETHAPP_LWIP_USE_DHCP
    err = dhcp_start(netif_default);
    if (err != ERR_OK)
    {
        appLogPrintf("Failed to start DHCP: %d\n", err);
    }
#endif /* ETHAPP_LWIP_USE_DHCP */
}

static void EthApp_netifStatusCb(struct netif *netif)
{
    Enet_MacPort macPort = ENET_MAC_PORT_1;
    int32_t status;

    if (netif_is_up(netif))
    {
        const ip4_addr_t *ipAddr = netif_ip4_addr(netif);

        appLogPrintf("Added interface '%c%c%d', IP is %s\n",
                     netif->name[0], netif->name[1], netif->num, ip4addr_ntoa(ipAddr));

        if (ipAddr->addr != 0)
        {
            gEthAppObj.hostIpAddr = lwip_ntohl(ip_addr_get_ip4_u32(ipAddr));

            /* MAC port used for PTP */
            macPort = ENET_MAC_PORT_3;

            /* Initialize and enable PTP stack */
            EthFw_initTimeSyncPtp(gEthAppObj.hostIpAddr,
                                  &gEthAppObj.hostMacAddr[0U],
                                  ENET_BIT(ENET_MACPORT_NORM(macPort)));

            /* Assign functions that are to be called based on actions in GUI.
             * These cannot be dynamically pushed to function pointer array, as the
             * index is used in GUI as command */
            EnetCfgServer_fxn_table[9] = &EthApp_startSwInterVlan;
            EnetCfgServer_fxn_table[10] = &EthApp_startHwInterVlan;

            /* Start Configuration server */
            status = EnetCfgServer_init(gEthAppObj.enetType);
            EnetAppUtils_assert(ENET_SOK == status);

            /* Start the software-based interVLAN routing */
            EthSwInterVlan_setupRouting(gEthAppObj.enetType, ETH_SWINTERVLAN_TASK_PRI);
        }
    }
    else
    {
        appLogPrintf("Removed interface '%c%c%d'\n", netif->name[0], netif->name[1], netif->num);
    }
}

/* Functions called from Config server library based on selection from GUI */

static void EthApp_startSwInterVlan(char *recvBuff,
                                    char *sendBuff)
{
    EnetCfgServer_InterVlanConfig *pInterVlanCfg;
    int32_t status;

    if (recvBuff != NULL)
    {
        pInterVlanCfg = (EnetCfgServer_InterVlanConfig *)recvBuff;
        status = EthSwInterVlan_addClassifierEntries(pInterVlanCfg);
        EnetAppUtils_assert(ENET_SOK == status);
    }
}

static void EthApp_startHwInterVlan(char *recvBuff,
                                    char *sendBuff)
{
    EnetCfgServer_InterVlanConfig *pInterVlanCfg;

    if (recvBuff != NULL)
    {
        pInterVlanCfg = (EnetCfgServer_InterVlanConfig *)recvBuff;
        EthHwInterVlan_setupRouting(gEthAppObj.enetType, pInterVlanCfg);
    }
}
