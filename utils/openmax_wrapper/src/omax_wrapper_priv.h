/*
 * Copyright (c) 2016-2022 The Khronos Group Inc.
 * Copyright 2022, QNX Software Systems.
 * Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions: 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software. 
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 *
 */

#ifndef _TI_OMAX_WRAPPER_PRIV_H_
#define _TI_OMAX_WRAPPER_PRIV_H_

/* Standard headers. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/procmgr.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <OMX_Core.h>
#include <OMX_Component.h>
#include <OMX_Extension_video_QNX.h>

#include <utils/openmax_wrapper/include/omax_wrapper.h>

// NAL start code length in bytes.

// #define WRAPPER_DEBUG_MODE

#ifdef WRAPPER_DEBUG_MODE
#define WRAPPER_PRINTF(f_, ...) printf((f_), ##__VA_ARGS__)
#else
#define WRAPPER_PRINTF(f_, ...)
#endif

#define WRAPPER_ERROR(f_, ...) printf((f_), ##__VA_ARGS__)

#define OMX_SPEC_VERSION 0x00000001     // OMX Version
#define SET_OMX_VERSION_SIZE( param, size ) {             \
    param.nVersion.nVersion = OMX_SPEC_VERSION;           \
    param.nSize = size;                                   \
}

#if defined (WRAPPER_DEBUG_MODE)
    #define TIMEOUT_WAIT (0x1FFFFFFFFFFF) /* for debug */
#else
    #define TIMEOUT_WAIT (5000 * 1000LL * 1000LL)  /* 5000 ms */
#endif

#define OMAX_MAX_FILE_PATH           (256u)

typedef struct mm_buffer mm_buffer_t;
typedef int32_t (*f_read_frame)(void *handle, mm_buffer_t *buffer);

typedef struct {
    void *addr;
    uint32_t size;
    off64_t offset; /* offset or physical address of the buffer. */
} OmxilEncInputBuffer_t;

typedef struct enc_params_s {
    const char *name;
    int32_t    *value;
    const int32_t range[2];
} enc_params_t;

/*! \brief A Boolean value.
 * This allows 0 to be FALSE, as it is in C, and any non-zero to be TRUE.
 * \code
 * omxil_bool ret = omxil_true_e;
 * if (ret) printf("true!\n");
 * ret = omxil_false_e;
 * if (!ret) printf("false!\n");
 * \endcode
 * This would print both strings.
 * \see omxil_bool
 * \ingroup group_basic_features
 */
typedef enum _omxil_bool_e {
    /*! \brief The "false" value. */
    omxil_false_e = 0,
    /*! \brief The "true" value. */
    omxil_true_e,
} omxil_bool_e;

/*! \brief A formal boolean type with known fixed size.
 * \see omxil_bool_e
 * \ingroup group_basic_features
 */
typedef int32_t omxil_bool;

typedef struct OmxilVideoEncDec_ {
    /* File paths*/
    char out_path[OMAX_MAX_FILE_PATH];
    char in_path[OMAX_MAX_FILE_PATH];
    char conf_path[OMAX_MAX_FILE_PATH];
    int32_t in_fd;
    int32_t out_fd;
    
    OmxilEncInputBuffer_t* input_bufs[CODEC_MAX_BUFFER_DEPTH];
    OMX_U32                input_buf_num;

    int32_t input_format;

    omxil_bool user_request_exit;

    int32_t frame_rate;
    int32_t src_width;
    int32_t src_height;
    int32_t aligned_height;
    int32_t frame_size;
    int32_t src_stride;
    int32_t channelIdx;

    //component
    OMX_HANDLETYPE compHandle;
    OMX_VIDEO_CODINGTYPE compressFmt;
    OMX_U32 inPortIndex;
    OMX_U32 outPortIndex;
    uint16_t numOfPorts; // Data ports
    omxil_bool cmdComplete;
    omxil_bool inPortFlushed;
    omxil_bool outPortFlushed;
    omxil_bool eos_received;
    omxil_bool eos_sent;
    OMX_ERRORTYPE compError;

    //output port
    OMX_U32 outputPortBufSize;
    OMX_U32 nOutputBufs;
    int32_t qOutputBufHdr[CODEC_MAX_BUFFER_DEPTH];
    OMX_BUFFERHEADERTYPE* outputBufHdrList[CODEC_MAX_BUFFER_DEPTH];
    int32_t qOutputBufHdrLastIdx;
    int32_t qOutputBufHdrFirstIdx;
    omxil_bool outBufFull[CODEC_MAX_BUFFER_DEPTH];

    //Input port
    OMX_U32 nInputBufs;
    OMX_U32 inputPortBufSize;
    int32_t qInputBufHdr[CODEC_MAX_BUFFER_DEPTH];
    OMX_BUFFERHEADERTYPE* inputBufHdrList[CODEC_MAX_BUFFER_DEPTH];
    int32_t qInputBufHdrLastIdx;
    int32_t qInputBufHdrFirstIdx;
    omxil_bool inBufEmpty[CODEC_MAX_BUFFER_DEPTH];

    uint32_t srcWidth; // source width, width from configure data, output buffer is allocated with this size
    uint32_t srcHeight; // source height, height from configure data

    //encoder configure parameters
    int32_t bitrate;
    int32_t idr_period;
    int32_t rcmode;

    int32_t input_frame_cnt;

    uint64_t enc_start_time_ms;
    uint64_t enc_eos_time_ms;
    uint64_t enc_time_ms;
    uint64_t enc_frame_rate;

    /* Threads */
    pthread_t encoder_push;
    pthread_mutex_t mutex;
    pthread_cond_t cond;

    int32_t push_count;
    int32_t pull_count;
} OmxilVideoEncDec_t;

typedef struct 
{
    app_codec_wrapper_params_t params;

    int32_t push_count;
    int32_t pull_count;

    OmxilVideoEncDec_t compHandleArray[CODEC_MAX_NUM_CHANNELS];

    int32_t input_format;
} app_omax_wrapper_obj_t;

extern app_omax_wrapper_obj_t g_app_omax_wrapper_obj;


#endif /* _TI_OMAX_WRAPPER_PRIV_H_ */

