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

/* Standard headers. */
#include <string.h>

/* Module headers. */
#include <utils/gst_codec_wrapper/include/gst_wrapper.h>
#include <utils/gst_codec_wrapper/include/gsttiovximagemeta.h>

#define GST_TIMEOUT  100*GST_MSECOND

static void bufferInfoInit(bufferInfo *buf_Info)
{
    uint32_t temp = buf_Info->width * buf_Info->height;

    if (!strcmp(buf_Info->format, "RGB"))
    {
        /* 3 * height * width. */
        buf_Info->num_planes = 3;
        buf_Info->plane_sizes[0] = temp;
        buf_Info->plane_sizes[1] = temp;
        buf_Info->plane_sizes[2] = temp;
        buf_Info->size = 3 * temp;
    }
    else if (!strcmp(buf_Info->format, "NV12"))
    {
        /* 1.5 * height * width. */
        buf_Info->num_planes = 2;
        buf_Info->plane_sizes[0] = temp;
        buf_Info->plane_sizes[1] = temp/2;
        buf_Info->size = temp + temp/2;
    }
    else if (!strcmp(buf_Info->format, "UYVY"))
    {
        /* 2 * height * width. */
        buf_Info->num_planes = 2;
        buf_Info->plane_sizes[0] = temp;
        buf_Info->plane_sizes[1] = temp;
        buf_Info->size = 2 * temp;
    }
}

static void construct_gst_strings(GstPipeObj *gstPipeInst)
{
    int32_t i = 0;

    for (uint8_t ch = 0; ch < gstPipeInst->input.num_channels; ch++)
    {
        if (gstPipeInst->srcType == 0){
            snprintf(gstPipeInst->m_AppSrcNameArr[ch] , MAX_LEN_ELEM_NAME, "myAppSrc%d" , ch);

            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"appsrc format=GST_FORMAT_TIME is-live=true do-timestamp=true block=false name=%s ! queue \n",gstPipeInst->m_AppSrcNameArr[ch]);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)30/1, format=(string)%s, interlace-mode=(string)progressive, colorimetry=(string)bt601 \n",
                                                                                            gstPipeInst->input.width, gstPipeInst->input.height, gstPipeInst->input.format);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! v4l2h264enc bitrate=10000000 \n");
        }
        else if (gstPipeInst->srcType == 1){
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"filesrc location=/home/root/test_video_1080p30.mp4 \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! qtdemux \n");
        }
        else if (gstPipeInst->srcType == 2){
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"videotestsrc is-live=true do-timestamp=true num-buffers=%d \n",MAX_FRAMES_TO_RUN);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)30/1, format=(string)%s, interlace-mode=(string)progressive, colorimetry=(string)bt601 \n",
                                                                                            gstPipeInst->input.width, gstPipeInst->input.height, gstPipeInst->input.format);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! v4l2h264enc bitrate=10000000 \n");
        }

        i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! h264parse \n");

        if (gstPipeInst->sinkType == 0){
            snprintf(gstPipeInst->m_AppSinkNameArr[ch], MAX_LEN_ELEM_NAME, "myAppSink%d", ch);

            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! v4l2h264dec \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! video/x-raw, format=(string)%s \n",
                                                                                        gstPipeInst->output.format);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! tiovxmemalloc pool-size=7 \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! appsink name=%s drop=true wait-on-eos=false max-buffers=4\n",gstPipeInst->m_AppSinkNameArr[ch]);
        }
        else if (gstPipeInst->sinkType == 1){
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! mp4mux \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! filesink location=output_video_%d%d.mp4 \n", gstPipeInst->srcType, ch);
        }
        else if (gstPipeInst->sinkType == 2){
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! v4l2h264dec \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! video/x-raw, format=(string)%s \n",
                                                                                        gstPipeInst->output.format);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! tiovxmemalloc pool-size=7 \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! fakesink \n");
        }
    }
}

static GstElement *findElementByName(GstElement* pipeline,
                                     const char* name)
{
    GstElement *elem;

    elem = gst_bin_get_by_name(GST_BIN(pipeline), name);

    if (elem == NULL)
    {
        printf("gst_wrapper: Could not find element <%s> in the pipeline.\n", name);
    }

    return elem;
}

static int32_t exportgsttiovxbuffer(GstBuffer* buf, void* data_ptr[MAX_NUM_PLANES])
{
    vx_status status = VX_SUCCESS;
    void* p_status = NULL;
    GstTIOVXImageMeta *tiovxmeta = NULL;
    vx_reference img1;
    vx_enum img1_type = VX_TYPE_INVALID;
    uint32_t img1_num_planes = 0;
    uint32_t sizes[MAX_NUM_PLANES] = { 0 };

    tiovxmeta = (GstTIOVXImageMeta *) gst_buffer_iterate_meta(buf, &p_status);
    if (!tiovxmeta)
    {
        printf("gst_wrapper: ERROR: TIOVX meta not found in pulled buffer!\n");
        return -1;
    }
    
    img1 = vxGetObjectArrayItem (tiovxmeta->array, 0);
    status = vxGetStatus((vx_reference) img1 );
    if ( status != VX_SUCCESS)
    {
        printf("gst_wrapper: ERROR: Could not get vx_reference from TIOVX meta!\n");
        return status;
    }
 
    status = vxQueryReference ((vx_reference) img1, (vx_enum) VX_REFERENCE_TYPE, &img1_type, sizeof (vx_enum));
    if (VX_SUCCESS != status) {
        printf("gst_wrapper: ERROR: Failed to verify VX_REFERENCE_TYPE!\n");
        vxReleaseReference (&img1);
        return status;
    }
    else if(VX_TYPE_IMAGE != img1_type) {
        printf("gst_wrapper: ERROR: vx_reference is not a vx_image!\n");
        vxReleaseReference (&img1);
        return VX_ERROR_INVALID_TYPE;
    }

    status = tivxReferenceExportHandle(
                (vx_reference) img1,
                data_ptr,
                sizes,
                MAX_NUM_PLANES,
                &img1_num_planes);
    if (VX_SUCCESS != status) {
        printf("gst_wrapper: ERROR: Could not export handles from vx_image!\n");
        vxReleaseReference (&img1);
        return status;
    }
    
    vxReleaseReference (&img1);
    return status;
}

int32_t app_init_gst_pipe(GstPipeObj *gstPipeInst)
{
    int32_t status = 0;

    /* Construct the Gstreamer command string */
    construct_gst_strings(gstPipeInst);

    gstPipeInst->push_count = 0;
    gstPipeInst->pull_count = 0;

    /* Calculate buffer size for the saved caps */
    bufferInfoInit(&gstPipeInst->input);
    bufferInfoInit(&gstPipeInst->output);

    printf("gst_wrapper: GstCmdString:\n%s\n",gstPipeInst->m_cmdString);

    return status;

}

int32_t app_create_gst_pipe(GstPipeObj *gstPipeInst)
{
    int32_t status = 0;
    GstCaps* caps = NULL;

    gstPipeInst->m_pipeline = gst_parse_launch(gstPipeInst->m_cmdString, NULL);

    if (gstPipeInst->m_pipeline == NULL)
    {
        printf("gst_wrapper: gst_parse_launch() failed:\n%s\n",gstPipeInst->m_cmdString);
        status = -1;
    }

    if (status == 0 && gstPipeInst->srcType==0)
    {
        for (uint8_t ch = 0; ch < gstPipeInst->input.num_channels; ch++)
        {
            /* Setup AppSrc */
            gstPipeInst->m_srcElemArr[ch]  = findElementByName(gstPipeInst->m_pipeline, 
                                                        gstPipeInst->m_AppSrcNameArr[ch]);
            if (gstPipeInst->m_srcElemArr[ch] == NULL)
            {
                printf("gst_wrapper: findElementByName() FAILED!\n");
                status = -1;
            }
            else
            {
                caps = gst_caps_new_simple("video/x-raw",
                                        "width", G_TYPE_INT, gstPipeInst->input.width,
                                        "height", G_TYPE_INT, gstPipeInst->input.height,
                                        "format", G_TYPE_STRING, gstPipeInst->input.format,
                                        NULL);
                if (caps == NULL)
                {
                    printf("gst_wrapper: gst_caps_new_simple() FAILED!\n");
                    status = -1;
                }
                gst_app_src_set_caps (GST_APP_SRC(gstPipeInst->m_srcElemArr[ch]), caps);
            }
        }
    }

    if (status == 0 && gstPipeInst->sinkType==0)
    {
        for (uint8_t ch = 0; ch < gstPipeInst->output.num_channels; ch++)
        {
            /* Setup AppSink */
            gstPipeInst->m_sinkElemArr[ch] = findElementByName(gstPipeInst->m_pipeline, 
                                                    gstPipeInst->m_AppSinkNameArr[ch]);
            if (gstPipeInst->m_sinkElemArr[ch] == NULL)
            {
                printf("gst_wrapper: findElementByName() FAILED!\n");
                status = -1;
            }
        }
    }

    return status;
}

int32_t wrap_buffers(GstPipeObj *gstPipeInst, void* data_ptr[MAX_BUFFER_DEPTH][MAX_NUM_CHANNELS][MAX_NUM_PLANES])
{
    int32_t status = 0;
    uint32_t plane_size = gstPipeInst->input.width * gstPipeInst->input.height;       // For NV12 buffers: plane_size of first plane

    if (gstPipeInst->srcType==0)
    {
        for (uint8_t idx = 0; idx < gstPipeInst->input.buffer_depth && status==0; idx++)
        {
            for (uint8_t ch = 0; ch < gstPipeInst->input.num_channels && status==0; ch++)
            {
                /* Setup Push buffers */
                gstPipeInst->buff[idx][ch] = gst_buffer_new();

                gstPipeInst->mem[idx][ch][0] = gst_memory_new_wrapped (0, data_ptr[idx][ch][0], plane_size, 0, plane_size, NULL, NULL);
                gstPipeInst->mem[idx][ch][1] = gst_memory_new_wrapped (0, data_ptr[idx][ch][1], plane_size/2, 0, plane_size/2, NULL, NULL);

                gst_buffer_append_memory (gstPipeInst->buff[idx][ch], gstPipeInst->mem[idx][ch][0]);
                gst_buffer_append_memory (gstPipeInst->buff[idx][ch], gstPipeInst->mem[idx][ch][1]);

                gstPipeInst->mem[idx][ch][0] = gst_buffer_get_memory (gstPipeInst->buff[idx][ch], 0);
                gstPipeInst->mem[idx][ch][1] = gst_buffer_get_memory (gstPipeInst->buff[idx][ch], 1);

                gst_memory_map(gstPipeInst->mem[idx][ch][0],&gstPipeInst->map_info[idx][ch][0], GST_MAP_WRITE);
                gst_memory_map(gstPipeInst->mem[idx][ch][1],&gstPipeInst->map_info[idx][ch][1], GST_MAP_WRITE);
            }
        }
    }
    return status;
}

int32_t app_start_gst_pipe(GstPipeObj *gstPipeInst)
{
    /* Set pipeline state to PLAYING */
    GstStateChangeReturn ret = gst_element_set_state (gstPipeInst->m_pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        printf("gst_wrapper: gst_element_set_state() FAILED! ... GST pipe not playing.\n");
        return -1;
    }
    return 0;
}

int32_t push_buffer_ready(GstPipeObj *gstPipeInst, uint8_t idx)
{
    int32_t status = 0;
    
    if(gstPipeInst->srcType!=0){
        return -1;
    }
    for (uint8_t ch = 0; ch < gstPipeInst->input.num_channels; ch++)
    {
        gst_memory_unmap(gstPipeInst->mem[idx][ch][0],&gstPipeInst->map_info[idx][ch][0]);
        gst_memory_unmap(gstPipeInst->mem[idx][ch][1],&gstPipeInst->map_info[idx][ch][1]);

        if (status == 0)
        {
            GstFlowReturn ret;

            gst_buffer_ref(gstPipeInst->buff[idx][ch]);
            ret = gst_app_src_push_buffer(GST_APP_SRC(gstPipeInst->m_srcElemArr[ch]), gstPipeInst->buff[idx][ch]);
            if (ret != GST_FLOW_OK)
            {
                printf("gst_wrapper: Pushing buffer to AppSrc returned %d instead of GST_FLOW_OK:%d\n", ret, GST_FLOW_OK);
                status = -1;
            }
        }
    }

    if (status==0) gstPipeInst->push_count++;

    return status;
}

int32_t push_buffer_wait(GstPipeObj *gstPipeInst, uint8_t idx)
{
    int32_t status = 0;
    uint8_t refcount;


    if(gstPipeInst->srcType!=0){
        return -1;
    }
    for (uint8_t ch = 0; ch < gstPipeInst->input.num_channels; ch++)
    {
        refcount = GST_MINI_OBJECT_REFCOUNT_VALUE(&gstPipeInst->buff[idx][ch]->mini_object);
        while (refcount > 1)
        {
            refcount = GST_MINI_OBJECT_REFCOUNT_VALUE(&gstPipeInst->buff[idx][ch]->mini_object);
        }
        refcount = GST_MINI_OBJECT_REFCOUNT_VALUE(&gstPipeInst->mem[idx][ch][1]->mini_object);
        while (refcount > 2)
        {
            refcount = GST_MINI_OBJECT_REFCOUNT_VALUE(&gstPipeInst->mem[idx][ch][1]->mini_object);
        }
        refcount = GST_MINI_OBJECT_REFCOUNT_VALUE(&gstPipeInst->mem[idx][ch][0]->mini_object);
        while (refcount > 2)
        {
            refcount = GST_MINI_OBJECT_REFCOUNT_VALUE(&gstPipeInst->mem[idx][ch][0]->mini_object);
        }

        if(!gst_memory_map(gstPipeInst->mem[idx][ch][0],&gstPipeInst->map_info[idx][ch][0], GST_MAP_WRITE)) 
        { 
            status = -1; 
            break;
        }
        if(!gst_memory_map(gstPipeInst->mem[idx][ch][1],&gstPipeInst->map_info[idx][ch][1], GST_MAP_WRITE)) 
        {
            status = -1;
            break;
        }
    }

    return status;
}

int32_t push_EOS(GstPipeObj *gstPipeInst)
{
    GstFlowReturn ret;
    int32_t status = 0;

    if(gstPipeInst->srcType!=0){
        return -1;
    }
    for (uint8_t ch = 0; ch < gstPipeInst->input.num_channels; ch++)
    {
        ret = gst_app_src_end_of_stream(GST_APP_SRC(gstPipeInst->m_srcElemArr[ch]   ));
        if (ret != GST_FLOW_OK)
        {
            printf("gst_wrapper: Pushing EOS to AppSrc returned %d instead of GST_FLOW_OK:%d\n", ret, GST_FLOW_OK);
            status = -1;
        }
    }

    return status;
}

int32_t pull_buffer_wait(GstPipeObj *gstPipeInst, uint8_t idx)
{
    GstSample *out_sample = NULL;
    int32_t status = 0;

    if(gstPipeInst->sinkType!=0){
        return -1;
    }
    for (uint8_t ch = 0; ch < gstPipeInst->output.num_channels; ch++)
    {
        /* Pull Sample from AppSink element */
        out_sample = gst_app_sink_try_pull_sample(GST_APP_SINK(gstPipeInst->m_sinkElemArr[ch]),GST_TIMEOUT);
        if(out_sample)
        {
            gstPipeInst->pulled_buff[idx][ch] = gst_sample_get_buffer(out_sample);

            status = exportgsttiovxbuffer(gstPipeInst->pulled_buff[idx][ch], gstPipeInst->pulled_data_ptr[idx][ch]);
            if (status != 0){
                break;
            }

            gst_buffer_ref(gstPipeInst->pulled_buff[idx][ch]);
            gst_sample_unref(out_sample);
        }
        else if(gst_app_sink_is_eos(GST_APP_SINK(gstPipeInst->m_sinkElemArr[ch])))
        {
            status = 1;
            break;
        }
        else
        {
            printf("gst_wrapper: WARNING: gst_app_sink_pull_sample() FAILED!\n");
            status = -1;
            break;
        }
    }

    if (status==0) gstPipeInst->pull_count++;

    return status;
}

int32_t pull_buffer_ready(GstPipeObj *gstPipeInst, uint8_t idx)
{
    if(gstPipeInst->sinkType!=0){
        return -1;
    }
    for (uint8_t ch = 0; ch < gstPipeInst->output.num_channels; ch++)
    {
        if ( gstPipeInst->pulled_buff[idx][ch] != NULL )
        {
            for(int32_t p = 0; p < gstPipeInst->output.num_planes; p++)
            {
                gstPipeInst->pulled_data_ptr[idx][ch][p] = NULL;
            }
            gst_buffer_unref(gstPipeInst->pulled_buff[idx][ch]);
            gstPipeInst->pulled_buff[idx][ch] = NULL;
        }
    }
    return 0;
}

int32_t app_stop_gst_pipe(GstPipeObj *gstPipeInst)
{
    GstStateChangeReturn ret;

    if(gstPipeInst->sinkType!=0)
    {
        GstBus     *bus;
        GstMessage *msg;
        bus = gst_element_get_bus (gstPipeInst->m_pipeline);
        msg =
            gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
            GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

        if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_EOS) {
            printf("gst_wrapper: Got EOS from pipeline!\n");
        }
        else if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_ERROR) {
            printf("gst_wrapper: An error occurred! Re-run with the GST_DEBUG=*:WARN environment "
                "variable set for more details.");
        }

        /* Free resources */
        gst_message_unref (msg);
        gst_object_unref (bus);
    }

    /* Set pipeline state to NULL */
    ret = gst_element_set_state (gstPipeInst->m_pipeline, GST_STATE_NULL);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        printf("gst_wrapper: GST pipe set state NULL failed.\n");
        return -1;
    }
    return 0;
}

void app_delete_gst_pipe(GstPipeObj *gstPipeInst)
{
    if(gstPipeInst->sinkType==0)
    {
        for (uint8_t ch = 0; ch < gstPipeInst->output.num_channels; ch++)
        {
            gst_object_unref (gstPipeInst->m_sinkElemArr[ch]);
        }
    }
    if(gstPipeInst->srcType==0)
    {
        for (uint8_t ch = 0; ch < gstPipeInst->input.num_channels; ch++)
        {
            for (uint8_t idx = 0; idx < gstPipeInst->input.buffer_depth; idx++)
            {
                gst_memory_unmap(gstPipeInst->mem[idx][ch][0],&gstPipeInst->map_info[idx][ch][0]);
                gst_memory_unmap(gstPipeInst->mem[idx][ch][1],&gstPipeInst->map_info[idx][ch][1]);

                gst_memory_unref(gstPipeInst->mem[idx][ch][0]);
                gst_memory_unref(gstPipeInst->mem[idx][ch][1]);

                gst_buffer_unref(gstPipeInst->buff[idx][ch]);

            }
            gst_object_unref (gstPipeInst->m_srcElemArr[ch]);
        }
    }
    gst_object_unref (gstPipeInst->m_pipeline);
}


