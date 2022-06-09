/*
 *  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _TI_GST_WRAPPER_H_
#define _TI_GST_WRAPPER_H_


/* Standard headers. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>

/* Third-party headers. */
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>


#define MAX_LEN_CMD_STR   4096u
#define MAX_LEN_ELEM_NAME   32u
#define MAX_NUM_CHANNELS     8u
#define MAX_FRAMES_TO_RUN 1800u

typedef struct 
{
    int32_t     width;
    int32_t     height;
    char        format[8];
    uint32_t    size;
    uint8_t     num_channels;

    uint8_t     srcType;
    uint8_t     sinkType;

    char        m_cmdString[MAX_LEN_CMD_STR];
    GstElement *m_pipeline;

    char        m_AppSrcNameArr[MAX_NUM_CHANNELS][MAX_LEN_ELEM_NAME];
    char        m_AppSinkNameArr[MAX_NUM_CHANNELS][MAX_LEN_ELEM_NAME];
    GstElement *m_srcElemArr[MAX_NUM_CHANNELS];
    GstElement *m_sinkElemArr[MAX_NUM_CHANNELS];

    GstBuffer* pulled_buff[MAX_NUM_CHANNELS];
    GstMapInfo pulled_map_info[MAX_NUM_CHANNELS];

    uint32_t    push_count;
    uint32_t    pull_count;

} GstPipeObj;


int32_t app_init_gst_pipe(GstPipeObj *gstPipeInst);

int32_t app_create_gst_pipe(GstPipeObj *gstPipeInst);

int32_t app_start_gst_pipe(GstPipeObj *gstPipeInst);

int32_t push_data_buffer(GstPipeObj *gstPipeInst, void* p_dataArr[]);

int32_t push_EOS(GstPipeObj *gstPipeInst);

int32_t pull_buffer_wait(GstPipeObj *gstPipeInst, void* data_ptr[MAX_NUM_CHANNELS]);

void    pull_buffer_ready(GstPipeObj *gstPipeInst);

int32_t app_stop_gst_pipe(GstPipeObj *gstPipeInst); 

void    app_delete_gst_pipe(GstPipeObj *gstPipeInst);

#endif /* _TI_GST_WRAPPER_H_ */

