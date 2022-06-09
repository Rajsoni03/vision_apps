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

#define GST_TIMEOUT  100*GST_MSECOND

static void construct_gst_strings(GstPipeObj *gstPipeInst)
{
    int32_t i = 0;

    for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
    {
        if (gstPipeInst->srcType == 0){
            snprintf(gstPipeInst->m_AppSrcNameArr[ch] , MAX_LEN_ELEM_NAME, "myAppSrc%d" , ch);

            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"appsrc format=GST_FORMAT_TIME is-live=true do-timestamp=true block=false name=%s ! queue \n",gstPipeInst->m_AppSrcNameArr[ch]);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)30/1, format=(string)%s, interlace-mode=(string)progressive, colorimetry=(string)bt601 \n",
                                                                                            gstPipeInst->width, gstPipeInst->height, gstPipeInst->format);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! v4l2h264enc bitrate=10000000 \n");
        }
        else if (gstPipeInst->srcType == 1){
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"filesrc location=/home/root/test_video_1080p30.mp4 \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! qtdemux \n");
        }
        else if (gstPipeInst->srcType == 2){
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"videotestsrc is-live=true do-timestamp=true num-buffers=%d \n",MAX_FRAMES_TO_RUN);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)30/1, format=(string)%s, interlace-mode=(string)progressive, colorimetry=(string)bt601 \n",
                                                                                            gstPipeInst->width, gstPipeInst->height, gstPipeInst->format);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! v4l2h264enc bitrate=10000000 \n");
        }

        i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! h264parse \n");

        if (gstPipeInst->sinkType == 0){
            snprintf(gstPipeInst->m_AppSinkNameArr[ch], MAX_LEN_ELEM_NAME, "myAppSink%d", ch);

            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! v4l2h264dec \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! video/x-raw, format=(string)%s \n",
                                                                                        gstPipeInst->format);
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! appsink name=%s drop=true wait-on-eos=false max-buffers=4\n",gstPipeInst->m_AppSinkNameArr[ch]);
        }
        else if (gstPipeInst->sinkType == 1){
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! mp4mux \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! filesink location=output_video_%d%d.mp4 \n", gstPipeInst->srcType, ch);
        }
        else if (gstPipeInst->sinkType == 2){
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! v4l2h264dec \n");
            i += snprintf(&gstPipeInst->m_cmdString[i], MAX_LEN_CMD_STR-i,"! video/x-raw, format=(string)%s \n",
                                                                                        gstPipeInst->format);
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

int32_t app_init_gst_pipe(GstPipeObj *gstPipeInst)
{
    int32_t status = 0;

    /* Construct the Gstreamer command string */
    construct_gst_strings(gstPipeInst);

    gstPipeInst->push_count = 0;
    gstPipeInst->pull_count = 0;

    /* Calculate buffer size for the saved caps */
    gstPipeInst->size = gstPipeInst->width * gstPipeInst->height;

    if (!strcmp(gstPipeInst->format, "RGB"))
    {
        /* 3 * size * width. */
        gstPipeInst->size *= 3;
    }
    else if (!strcmp(gstPipeInst->format, "NV12"))
    {
        /* 1.5 * size * width. */
        gstPipeInst->size += gstPipeInst->size/2;
    }
    else if (!strcmp(gstPipeInst->format, "UYVY"))
    {
        /* 2 * size * width. */
        gstPipeInst->size *= 2;
    }

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
        for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
        {
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
                                        "width", G_TYPE_INT, gstPipeInst->width,
                                        "height", G_TYPE_INT, gstPipeInst->height,
                                        "format", G_TYPE_STRING, gstPipeInst->format,
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
        for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
        {
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

int32_t push_data_buffer(GstPipeObj *gstPipeInst, void* p_dataArr[])
{
    int32_t status = 0;
    
    for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
    {
        /* Replace by GstBufferPool implementation (dequeue, update, enqueue) */
        GstBuffer *in_buff = NULL;
        in_buff = gst_buffer_new_wrapped (p_dataArr[ch], gstPipeInst->size);
        if (in_buff == NULL)
        {
            printf("gst_wrapper: gst_buffer_new_wrapped() FAILED!\n");
            status = -1;
        }

        if (status == 0)
        {
            GstFlowReturn ret;

            ret = gst_app_src_push_buffer(GST_APP_SRC(gstPipeInst->m_srcElemArr[ch]), in_buff);
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

int32_t push_EOS(GstPipeObj *gstPipeInst)
{
    GstFlowReturn ret;
    int32_t status = 0;

    for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
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

int32_t pull_buffer_wait(GstPipeObj *gstPipeInst, void* data_ptr[MAX_NUM_CHANNELS])
{
    GstSample *out_sample = NULL;
    int32_t status = 0;

    for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
    {
        /* Pull Sample from AppSink element */
        out_sample = gst_app_sink_try_pull_sample(GST_APP_SINK(gstPipeInst->m_sinkElemArr[ch]),GST_TIMEOUT);
        if(out_sample)
        {
            gstPipeInst->pulled_buff[ch] = gst_sample_get_buffer(out_sample);
            gst_buffer_map(gstPipeInst->pulled_buff[ch], &gstPipeInst->pulled_map_info[ch],  GST_MAP_READ);
            data_ptr[ch] = gstPipeInst->pulled_map_info[ch].data;
            gst_buffer_ref(gstPipeInst->pulled_buff[ch]);
            gst_sample_unref(out_sample);
        }
        else if(gst_app_sink_is_eos(GST_APP_SINK(gstPipeInst->m_sinkElemArr[ch])))
        {
            // printf("gst_wrapper: Got EOS from AppSink! Total buffer count: %d\n",gstPipeInst->pull_count);
            data_ptr[ch] = NULL;
            status = 1;
        }
        else
        {
            printf("gst_wrapper: gst_app_sink_pull_sample() FAILED!\n");
            data_ptr[ch] = NULL;
            status = -1;
            break;
        }
    }

    if (status==0) gstPipeInst->pull_count++;

    return status;
}

void pull_buffer_ready(GstPipeObj *gstPipeInst)
{
    for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
    {
        gst_buffer_unmap(gstPipeInst->pulled_buff[ch], &gstPipeInst->pulled_map_info[ch]);
        gst_buffer_unref(gstPipeInst->pulled_buff[ch]);
    }
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
        for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
        {
            gst_object_unref (gstPipeInst->m_sinkElemArr[ch]);
        }
    }
    if(gstPipeInst->srcType==0)
    {
        for (uint8_t ch = 0; ch < gstPipeInst->num_channels; ch++)
        {
            gst_object_unref (gstPipeInst->m_srcElemArr[ch]);
        }
    }
    gst_object_unref (gstPipeInst->m_pipeline);
}


