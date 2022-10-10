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

#if defined(SOC_J784S4)
#include <ti/csl/csl_types.h>
#include <ti/board/board.h>
#endif

#if defined(SOC_AM62A)
#include <ti/board/board.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#endif

/* High Priority for SCI Server - must be higher than Low priority task */
#define SETUP_SCISERVER_TASK_PRI_HIGH   (5)
/*
 * Low Priority for SCI Server - must be higher than IPC echo test tasks
 * to prevent delay in handling Sciserver requests when test is performing
 * multicore ping/pong.
 */
#define SETUP_SCISERVER_TASK_PRI_LOW    (4)


/* This is a HACK to POWER ON DRU's local to C7x */
#if defined(SOC_J784S4)

/* PSC defines*/
#define BOLTON_PSC_BASE             (0x00420000U)

#define PSC_MDCTL00                 (0xA00U)
#define PSC_MDSTAT00                (0x800U)
#define PSC_PDCTL00                 (0x300U)
#define PSC_PDSTAT00                (0x200U)
#define PSC_PTCMD                   (0x120U)
#define PSC_PTSTAT                  (0x128U)
#define PSC_PTCMD_H                 (0x124U)
#define PSC_PTSTAT_H                (0x12CU)

//PSC Parameter definitions
#define PSC_PD_OFF                  (0x0U)
#define PSC_PD_ON                   (0x1U)

#define PSC_SYNCRESETDISABLE        (0x0U)
#define PSC_SYNCRESET               (0x1U)
#define PSC_DISABLE                 (0x2U)
#define PSC_ENABLE                  (0x3U)

#define BOLTON_PSC_MDCTL_BASE       (BOLTON_PSC_BASE + PSC_MDCTL00)
#define BOLTON_PSC_MDSTAT_BASE      (BOLTON_PSC_BASE + PSC_MDSTAT00)
#define BOLTON_PSC_PDCTL_BASE       (BOLTON_PSC_BASE + PSC_PDCTL00)
#define BOLTON_PSC_PDSTAT_BASE      (BOLTON_PSC_BASE + PSC_PDSTAT00)
#define BOLTON_PSC_PTCMD_BASE       (BOLTON_PSC_BASE + PSC_PTCMD)
#define BOLTON_PSC_PTSTAT_BASE      (BOLTON_PSC_BASE + PSC_PTSTAT)
#define BOLTON_PSC_PTCMD            (BOLTON_PSC_PTCMD_BASE)
#define BOLTON_PSC_PTSTAT           (BOLTON_PSC_PTSTAT_BASE)
#define BOLTON_PSC_PTCMD_H          (BOLTON_PSC_BASE + PSC_PTCMD_H)
#define BOLTON_PSC_PTSTAT_H         (BOLTON_PSC_BASE + PSC_PTSTAT_H)

#define PSC_TIMEOUT                 (1000U)

#define BOLT_PD_ANA0                (4U)
#define BOLT_PD_ANA1                (5U)
#define BOLT_PD_ANA2                (6U)
#define BOLT_PD_ANA3                (7U)

#define LPSC_MSMC_L1_0              (14U)
#define LPSC_DRU_0                  (15U)
#define LPSC_ANA_PBIST_0            (16U)
#define LPSC_MSMC_L1_1              (17U)
#define LPSC_DRU_1                  (18U)
#define LPSC_ANA_PBIST_1            (19U)
#define LPSC_MSMC_L1_2              (20U)
#define LPSC_DRU_2                  (21U)
#define LPSC_ANA_PBIST_2            (22U)
#define LPSC_MSMC_L1_3              (23U)
#define LPSC_DRU_3                  (24U)
#define LPSC_ANA_PBIST_3            (25U)

int32_t Board_SetPSCState(uint32_t pd_id, uint32_t md_id, uint32_t pd_state, uint32_t md_state)
{
    uint32_t regVal;
    uint32_t mdctl;
    uint32_t mdstat;
    uint32_t pdctl;
    uint32_t pdstat;
    uint32_t ptcmd;
    uint32_t ptstat;

    uint32_t loop_cnt = 0;
    int32_t ret = 0;

    uint32_t address_offset = 0;

    //Added for support beyond 32-domains
    if(pd_id > 31){
        ptcmd = BOLTON_PSC_PTCMD_H;
        ptstat = BOLTON_PSC_PTSTAT_H;
    }else{
        ptcmd = BOLTON_PSC_PTCMD;
        ptstat = BOLTON_PSC_PTSTAT;
    }

    mdctl = (BOLTON_PSC_MDCTL_BASE + ( 4U * md_id ) + address_offset) ;
    mdstat = (BOLTON_PSC_MDSTAT_BASE + ( 4U * md_id )+ address_offset);
    pdctl = (BOLTON_PSC_PDCTL_BASE + ( 4U * pd_id ) + address_offset);
    pdstat = (BOLTON_PSC_PDSTAT_BASE + ( 4U * pd_id )+ address_offset);

    // If state is already set, do nothing
    if ( (( HW_RD_REG32(pdstat) & 0x1 ) == pd_state) && (( HW_RD_REG32(mdstat) & 0x1f ) == md_state) )
    {
        ret = -1;
        return ret;
    }

    // Wait for GOSTAT to clear
    while( (loop_cnt < PSC_TIMEOUT) && (HW_RD_REG32(ptstat+address_offset) & (0x1 << (pd_id % 32))) != 0 )
    {
        loop_cnt++;
    }

    // Check if we got timeout error while waiting
    if (loop_cnt >= PSC_TIMEOUT)
    {
        ret = -1;
        return ret;
    }

    // Set PDCTL NEXT to new state
    regVal = (HW_RD_REG32(pdctl) & ~(0x1)) | pd_state;
    HW_WR_REG32(pdctl, regVal);

    // Set MDCTL NEXT to new state
    regVal = (HW_RD_REG32(mdctl) & ~(0x1f)) | md_state;
    HW_WR_REG32(mdctl, regVal);

    // Start power transition by setting PTCMD GO to 1
    regVal = (HW_RD_REG32(ptcmd+address_offset)) | (0x1<< (pd_id % 32));
    HW_WR_REG32((ptcmd+address_offset), regVal);

    loop_cnt = 0;

    // Wait for PTSTAT GOSTAT to clear
    while( (loop_cnt < PSC_TIMEOUT) && (HW_RD_REG32(ptstat+address_offset) & (0x1 << (pd_id % 32))) != 0 )
    {
        loop_cnt++;
    }

    // Check if we got timeout error while waiting
    if (loop_cnt >= PSC_TIMEOUT)
    {
        ret = -1;
        return ret;
    }

    // Verify power domain and module domain state got changed
    if ( (( HW_RD_REG32(pdstat) & 0x1 ) == pd_state) && (( HW_RD_REG32(mdstat) & 0x1f ) == md_state) )
    {
        ret = 0;
    }
    if (( HW_RD_REG32(pdstat) & 0x1 ) != pd_state)
    {
        ret = -1;
    }
    if (( HW_RD_REG32(mdstat) & 0x1f ) != md_state)
    {
        ret = -1;
    }

    return ret;
}
#endif

int32_t appSciserverSciclientInit()
{
    int32_t retVal = CSL_PASS;
    Sciclient_ConfigPrms_t  clientParams;

#if defined(SOC_AM62A)
    Board_initCfg   boardCfg;
#endif

    retVal = Sciclient_configPrmsInit(&clientParams);

    if(retVal==CSL_PASS)
    {
        retVal = Sciclient_boardCfgParseHeader(
                (uint8_t *) SCISERVER_COMMON_X509_HEADER_ADDR,
                &clientParams.inPmPrms, &clientParams.inRmPrms);
    }

#if defined(SOC_AM62A)
    if(retVal==CSL_PASS)
    {
        boardCfg = BOARD_INIT_PINMUX_CONFIG | BOARD_INIT_UART_STDIO;
        Board_init(boardCfg);
    }
#endif

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
        #if defined(SOC_AM62A)
        UART_printf("SCICLIENT: ERROR: Sciclient deinit failed !!!\n");
        #else
        appLogPrintf("SCICLIENT: ERROR: Sciclient deinit failed !!!\n");
        #endif
    }

    return retVal;
}

void appSciserverInit(void* arg0, void* arg1)
{
    int32_t retVal = CSL_PASS;
    Sciserver_TirtosCfgPrms_t serverParams;

    #if defined(SOC_AM62A)
    char *version_str = NULL;
    char *rmpmhal_version_str = NULL;
    #endif

    retVal = Sciserver_tirtosInitPrms_Init(&serverParams);

    serverParams.taskPriority[SCISERVER_TASK_USER_LO] = SETUP_SCISERVER_TASK_PRI_LOW;
    serverParams.taskPriority[SCISERVER_TASK_USER_HI] = SETUP_SCISERVER_TASK_PRI_HIGH;

    if(retVal==CSL_PASS)
    {
        retVal = Sciserver_tirtosInit(&serverParams);
    }

    #if defined(SOC_AM62A)
    version_str = Sciserver_getVersionStr();
    rmpmhal_version_str = Sciserver_getRmPmHalVersionStr();
    UART_printf("##DM Built On: %s %s\n", __DATE__, __TIME__);
    UART_printf("##Sciserver Version: %s\n", version_str);
    UART_printf("##RM_PM_HAL Version: %s\n", rmpmhal_version_str);

    if (retVal == CSL_PASS)
    {
        UART_printf("##Starting Sciserver..... PASSED\n");
    }
    else
    {
        UART_printf("Starting Sciserver..... FAILED\n");
    }
    #endif

#if defined(SOC_J784S4)
    Board_SetPSCState(BOLT_PD_ANA0,LPSC_DRU_0, PSC_PD_ON, PSC_ENABLE);
    Board_SetPSCState(BOLT_PD_ANA1,LPSC_DRU_1, PSC_PD_ON, PSC_ENABLE);
    Board_SetPSCState(BOLT_PD_ANA2,LPSC_DRU_2, PSC_PD_ON, PSC_ENABLE);
    Board_SetPSCState(BOLT_PD_ANA3,LPSC_DRU_3, PSC_PD_ON, PSC_ENABLE);
#endif
}

void appSciserverDeInit()
{
    Sciserver_tirtosDeinit();

    return;
}
