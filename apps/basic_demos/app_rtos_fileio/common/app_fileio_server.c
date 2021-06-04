/*
 *
 * Copyright (c) 2017 Texas Instruments Incorporated
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


#define PRINTF(a) printf a
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#define USE_HOST_FILE_IO

#include "ti_file_io.h"
#include "ti_file_io_msg.h"
#include <utils/mem/include/app_mem.h>

typedef struct ServerFileStruct
{
    int available;
    void * fd;
} ServerFileStruct;


int searchFileStruct(struct ServerFileStruct fileStruct[], void* fd_arg)
{ 
    int i;
        for(i = 0; i < MAX_NUM_FILES; i++)
        {
            if( (fileStruct[i].available == 1) && (fileStruct[i].fd == fd_arg))
            {
                return i;
            }
        }
    return -1;
}


int getFreeIndex(struct ServerFileStruct fileStruct[])
{
    int i;
        for(i = 0; i < MAX_NUM_FILES; i++)
        {
            if( fileStruct[i].available == 0 )
            {
                return i;
            }
        }
    return -1;
}
struct ServerFileStruct openFileInfo[MAX_NUM_FILES];

int fileIOServer(send_recv_Struct *msg)
{
    OpStruct_t *opStruct = (OpStruct_t *)msg[0].ptr;
    char *buffer;
    int32_t i, r;
    if (opStruct->opCode == TI_FILEIO_OPCODE_FOPEN)
    {
        //function is fopen
        PRINTF(("Request to open file %s in %s mode. ", opStruct->fileName, opStruct->mode));
        FILE *fp = fopen(opStruct->fileName, opStruct->mode);
        if (fp == NULL)
        {
            fputs("FOPEN : Cannot open file\n", stderr);
        }
        HOSTFILE *hostFile = (HOSTFILE *)msg[1].ptr;
        hostFile->id = fp;

        PRINTF(("File ID =  Ox%x, %d\n", hostFile->id, hostFile->id));

        //set the details in ServerFileStruct openFileInfo
        if (fp)
        {
            i = getFreeIndex(openFileInfo);
            openFileInfo[i].available = 1;
            openFileInfo[i].fd = hostFile->id;
        }
    }

    else if (opStruct->opCode == TI_FILEIO_OPCODE_FCLOSE)
    {
        //function is fclose
        PRINTF(("Request to close file %d\n", opStruct->fid));
        r = fclose((FILE *)(opStruct->fid));
        RETVALUE *retPtr = (RETVALUE *)msg[1].ptr;
        retPtr->ret = r;
        if (r == 0)
        {
            i = searchFileStruct(openFileInfo, opStruct->fid);
            openFileInfo[i].available = 0;
            openFileInfo[i].fd = NULL;
        }
    }

    else if (opStruct->opCode == TI_FILEIO_OPCODE_FREAD)
    {
        //function is fread
        int total = (opStruct->size) * (opStruct->count);
        PRINTF(("Request to read %d bytes from file %d \n", total, opStruct->fid));
        buffer = (char *)msg[1].ptr;

        //check if file is open
        i = searchFileStruct(openFileInfo, opStruct->fid);
        if (i == -1) //if file is not open
        {
            fputs("FREAD : File not open\n", stderr);
        }
        int res = fread(buffer, opStruct->size, opStruct->count, (FILE *)opStruct->fid);
        appMemCacheWbInv(buffer, opStruct->size*opStruct->count);
        if (res != total)
        {
            fputs("FREAD error\n", stderr);
        }
    }

    else if (opStruct->opCode == TI_FILEIO_OPCODE_FWRITE)
    {
        //function is fwrite

        int total = (opStruct->size) * (opStruct->count);
        PRINTF(("Request to write %d bytes to file %d \n", total, opStruct->fid));
        buffer = (char *)msg[1].ptr;

        //check if file is open
        i = searchFileStruct(openFileInfo, opStruct->fid);
        if (i == -1) //if file is not open
        {
            fputs("FWRITE : File not open\n", stderr);
        }

        appMemCacheInv(buffer, opStruct->size*opStruct->count);
        int res = fwrite(buffer, opStruct->size, opStruct->count, (FILE *)opStruct->fid);
        if (res != total)
        {
            fputs("Fwrite error\n", stderr);
        }
    }

    else if (opStruct->opCode == TI_FILEIO_OPCODE_FSEEK)
    {
        //function is fseek

        //check if file is open
        i = searchFileStruct(openFileInfo, opStruct->fid);
        if (i == -1) //if file is not open
        {
            fputs("FSEEK : File not open\n", stderr);
        }

        PRINTF(("Request to seek %d bytes in mode %d from file %d \n", opStruct->offset, opStruct->count, opStruct->fid));

        r = fseek((FILE *)opStruct->fid, opStruct->offset, opStruct->count);

        RETVALUE *retPtr = (RETVALUE *)msg[1].ptr;
        retPtr->ret = r;
    }

    else if (opStruct->opCode == TI_FILEIO_OPCODE_FTELL)
    {
        //function is ftell

        //check if file is open
        i = searchFileStruct(openFileInfo, opStruct->fid);
        if (i == -1) //if file is not open
        {
            fputs("FTELL : File not open\n", stderr);
        }

        PRINTF(("Request to ftell from file %d \n", opStruct->fid));

        long int res = ftell((FILE *)opStruct->fid);

        RETVALUE *retPtr = (RETVALUE *)msg[1].ptr;
        retPtr->ret = res;
    }

    else if (opStruct->opCode == TI_FILEIO_OPCODE_FGETS)
    {
        //function is fgets

        //check if file is open
        i = searchFileStruct(openFileInfo, opStruct->fid);
        if (i == -1) //if file is not open
        {
            fputs("FGETS : File not open\n", stderr);
        }

        //PRINTF(("Request for fgets %d bytes from file %d \n", opStruct->size, opStruct->fid));

        buffer = (char *)msg[1].ptr;
        fgets(buffer, opStruct->size, (FILE *)opStruct->fid);
         appMemCacheWbInv(buffer, opStruct->size);
   }
    else
    {
      fputs("UNSUPORTED OP TYPE : \n", stderr);       
    }
    return 0;
}


int fileIOServerInit(MSG_Q *msg_q)
{
    int i;
    msg_q->numMsg = 0;
    for(i = 0; i < MAX_NUM_FILES; i++)
    {
        openFileInfo[i].available = 0;
        openFileInfo[i].fd = NULL;
    }
    msg_q->sync = (uint32_t)0xE0F12E10U;
    appMemCacheWbInv(msg_q, 0x40000);

    return 0;
}
int fileIOServerRun(MSG_Q *msg_q)
{

    appMemCacheInv(msg_q, 0x40000);
    if (msg_q->numMsg)
    {

        fileIOServer(&msg_q->msgList[0]);
        msg_q->numMsg = 0;
        appMemCacheWbInv(msg_q, 0x40000);
    }
    return 0;
}
