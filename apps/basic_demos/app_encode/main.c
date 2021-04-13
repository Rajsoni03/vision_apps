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

#include <utils/draw2d/include/draw2d.h>
#include <utils/perf_stats/include/app_perf_stats.h>
#include <utils/console_io/include/app_get.h>
#include <utils/grpx/include/app_grpx.h>
#include <VX/vx_khr_pipelining.h>

#include "app_common.h"
#include "app_sensor_module.h"
#include "app_capture_module.h"
#include "app_viss_module.h"
#include "app_aewb_module.h"
#include "app_ldc_module.h"
#include "app_img_mosaic_module.h"
#include "app_encode_module.h"
#include "app_test.h"

#define APP_BUFFER_Q_DEPTH   (4)
#define APP_PIPELINE_DEPTH   (7)

typedef struct {

    SensorObj     sensorObj;
    CaptureObj    captureObj;
    VISSObj       vissObj;
    AEWBObj       aewbObj;
    LDCObj        ldcObj;
    ImgMosaicObj  imgMosaicObj;
    EncodeObj     encodeObj;

    vx_char output_file_path[APP_MAX_FILE_PATH];

    /* OpenVX references */
    vx_context context;
    vx_graph   graph;

    vx_int32 en_out_img_write;

    vx_uint32 is_interactive;
    vx_uint32 test_mode;

    vx_uint32 num_frames_to_run;

    vx_uint32 num_frames_to_write;
    vx_uint32 num_frames_to_skip;

    tivx_task task;
    vx_uint32 stop_task;
    vx_uint32 stop_task_done;

    app_perf_point_t total_perf;
    app_perf_point_t fileio_perf;
    app_perf_point_t draw_perf;

    int32_t enable_ldc;

    int32_t pipeline;

    int32_t enqueueCnt;
    int32_t dequeueCnt;

    int32_t write_file;

} AppObj;

AppObj gAppObj;

static void app_parse_cmd_line_args(AppObj *obj, vx_int32 argc, vx_char *argv[]);
static vx_status app_init(AppObj *obj);
static void app_deinit(AppObj *obj);
static vx_status app_create_graph(AppObj *obj);
static vx_status app_verify_graph(AppObj *obj);
static vx_status app_run_graph(AppObj *obj);
static vx_status app_run_graph_interactive(AppObj *obj);
static void app_delete_graph(AppObj *obj);
static void app_default_param_set(AppObj *obj);
static void app_update_param_set(AppObj *obj);
static void app_pipeline_params_defaults(AppObj *obj);
static void add_graph_parameter_by_node_index(vx_graph graph, vx_node node, vx_uint32 node_parameter_index);
static vx_int32 calc_grid_size(vx_uint32 ch);
static void set_img_mosaic_params(ImgMosaicObj *imgMosaicObj, vx_uint32 in_width, vx_uint32 in_height, vx_int32 numCh);
static void set_encode_params(AppObj *obj);

static void app_show_usage(vx_int32 argc, vx_char* argv[])
{
    printf("\n");
    printf(" Camera Demo - (c) Texas Instruments 2020\n");
    printf(" ========================================================\n");
    printf("\n");
    printf(" Usage,\n");
    printf("  %s --cfg <config file>\n", argv[0]);
    printf("\n");
}

static char menu[] = {
    "\n"
    "\n ========================="
    "\n Demo : Camera Demo"
    "\n ========================="
    "\n"
    "\n s: Save CSIx, VISS and LDC outputs"
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};

static void app_run_task(void *app_var)
{
    AppObj *obj = (AppObj *)app_var;

    while(!obj->stop_task)
    {
        app_run_graph(obj);
    }
    obj->stop_task_done = 1;
}

static int32_t app_run_task_create(AppObj *obj)
{
    tivx_task_create_params_t params;
    int32_t status;

    tivxTaskSetDefaultCreateParams(&params);
    params.task_main = app_run_task;
    params.app_var = obj;

    obj->stop_task_done = 0;
    obj->stop_task = 0;

    status = tivxTaskCreate(&obj->task, &params);

    return status;
}

static void app_run_task_delete(AppObj *obj)
{
    while(obj->stop_task_done==0)
    {
         tivxTaskWaitMsecs(100);
    }

    tivxTaskDelete(&obj->task);
}

static vx_status app_run_graph_interactive(AppObj *obj)
{
    vx_status status;
    uint32_t done = 0;

    char ch;
    FILE *fp;
    app_perf_point_t *perf_arr[1];

    status = app_run_task_create(obj);
    if(status!=0)
    {
        printf("app_tidl: ERROR: Unable to create task\n");
    }
    else
    {
        appPerfStatsResetAll();
        while(!done)
        {
            printf(menu);
            ch = getchar();
            printf("\n");

            switch(ch)
            {
                case 'p':
                    appPerfStatsPrintAll();
                    tivx_utils_graph_perf_print(obj->graph);
                    appPerfPointPrint(&obj->fileio_perf);
                    appPerfPointPrint(&obj->total_perf);
                    printf("\n");
                    appPerfPointPrintFPS(&obj->total_perf);
                    appPerfPointReset(&obj->total_perf);
                    printf("\n");
                    break;
                case 'e':
                    perf_arr[0] = &obj->total_perf;
                    fp = appPerfStatsExportOpenFile(".", "basic_demos_app_encode");
                    if (NULL != fp)
                    {
                        appPerfStatsExportAll(fp, perf_arr, 1);
                        tivx_utils_graph_perf_export(fp, obj->graph);
                        appPerfStatsExportCloseFile(fp);
                        appPerfStatsResetAll();
                    }
                    else
                    {
                        printf("fp is null\n");
                    }
                    break;
                case 's':
                    obj->write_file = 1;
                    break;
                case 'x':
                    obj->stop_task = 1;
                    done = 1;
                    break;
            }
        }
        app_run_task_delete(obj);
    }
    return status;
}

static void app_set_cfg_default(AppObj *obj)
{
    snprintf(obj->output_file_path,APP_MAX_FILE_PATH, ".");
}

static void app_parse_cfg_file(AppObj *obj, vx_char *cfg_file_name)
{
    FILE *fp = fopen(cfg_file_name, "r");
    vx_char line_str[1024];
    vx_char *token;

    if(fp==NULL)
    {
        printf("# ERROR: Unable to open config file [%s]\n", cfg_file_name);
        exit(0);
    }

    while(fgets(line_str, sizeof(line_str), fp)!=NULL)
    {
        vx_char s[]=" \t";

        if (strchr(line_str, '#'))
        {
            continue;
        }

        /* get the first token */
        token = strtok(line_str, s);
        if(token != NULL)
        {
          if(strcmp(token, "sensor_index")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                obj->sensorObj.sensor_index = atoi(token);
              }
          }
          else
          if(strcmp(token, "num_frames_to_run")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                obj->num_frames_to_run = atoi(token);
              }
          }
          else
          if(strcmp(token, "enable_error_detection")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                obj->captureObj.enable_error_detection = atoi(token);
              }
          }
          else
          if(strcmp(token, "enable_ldc")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                obj->sensorObj.enable_ldc = atoi(token);
                if(obj->sensorObj.enable_ldc > 1)
                  obj->sensorObj.enable_ldc = 1;
              }
          }
          else
          if(strcmp(token, "en_out_img_write")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                obj->en_out_img_write = atoi(token);
                if(obj->en_out_img_write > 1)
                  obj->en_out_img_write = 1;
              }
          }
          else
          if(strcmp(token, "en_out_capture_write")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                obj->captureObj.en_out_capture_write = atoi(token);
                if(obj->captureObj.en_out_capture_write > 1)
                  obj->captureObj.en_out_capture_write = 1;
              }
          }
          else
          if(strcmp(token, "en_out_viss_write")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                obj->vissObj.en_out_viss_write = atoi(token);
                if(obj->vissObj.en_out_viss_write > 1)
                  obj->vissObj.en_out_viss_write = 1;
              }
          }
          else
          if(strcmp(token, "en_out_ldc_write")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                obj->ldcObj.en_out_ldc_write = atoi(token);
                if(obj->ldcObj.en_out_ldc_write > 1)
                  obj->ldcObj.en_out_ldc_write = 1;
              }
          }
          else
          if(strcmp(token, "output_file_path")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                token[strlen(token)-1]=0;
                strcpy(obj->captureObj.output_file_path, token);
                strcpy(obj->vissObj.output_file_path, token);
                strcpy(obj->ldcObj.output_file_path, token);
                strcpy(obj->encodeObj.output_file_path, token);
                strcpy(obj->output_file_path, token);
              }
          }
          else
          if(strcmp(token, "is_interactive")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                token[strlen(token)-1]=0;
                obj->is_interactive = atoi(token);
                if(obj->is_interactive > 1)
                  obj->is_interactive = 1;
              }
              obj->sensorObj.is_interactive = obj->is_interactive;
          }
          else
          if(strcmp(token, "num_cameras_enabled")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                  token[strlen(token)-1]=0;
                  obj->sensorObj.num_cameras_enabled = atoi(token);
              }
          }
          else
          if(strcmp(token, "usecase_option")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                  token[strlen(token)-1]=0;
                  obj->sensorObj.usecase_option = atoi(token);
              }
          }
          else
          if(strcmp(token, "num_frames_to_write")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                  token[strlen(token)-1]=0;
                  obj->num_frames_to_write = atoi(token);
              }
          }
          else
          if(strcmp(token, "num_frames_to_skip")==0)
          {
              token = strtok(NULL, s);
              if(token != NULL)
              {
                  token[strlen(token)-1]=0;
                  obj->num_frames_to_skip = atoi(token);
              }
          }
        }
    }

    fclose(fp);

}

static void app_parse_cmd_line_args(AppObj *obj, vx_int32 argc, vx_char *argv[])
{
    vx_int32 i;
    vx_int8 sensor_override = 0xFF;
    vx_bool set_test_mode = vx_false_e;

    app_set_cfg_default(obj);

    if(argc==1)
    {
        app_show_usage(argc, argv);
        exit(0);
    }

    for(i=0; i<argc; i++)
    {
        if(strcmp(argv[i], "--cfg")==0)
        {
            i++;
            if(i>=argc)
            {
                app_show_usage(argc, argv);
            }
            app_parse_cfg_file(obj, argv[i]);
        }
        else
        if(strcmp(argv[i], "--help")==0)
        {
            app_show_usage(argc, argv);
            exit(0);
        }
        else
        if(strcmp(argv[i], "--test")==0)
        {
            set_test_mode = vx_true_e;
        }
        else
        if(strcmp(argv[i], "--sensor")==0)
        {
            // check to see if there is another argument following --sensor
            if (argc > i+1)
            {
                sensor_override = atoi(argv[i+1]);
                // increment i again to avoid this arg
                i++;
            }
        }
    }
    if(set_test_mode == vx_true_e)
    {
        obj->test_mode = 1;
        obj->captureObj.test_mode = 1;
        obj->is_interactive = 0;
        obj->sensorObj.is_interactive = 0;
        obj->num_frames_to_run = sizeof(checksums_expected[0])/sizeof(checksums_expected[0][0]) + TEST_BUFFER;
        if (sensor_override != 0xFF)
        {
            obj->sensorObj.sensor_index = sensor_override;
        }

        // set the output to a path that is guaranteed to exist
        strcpy(obj->captureObj.output_file_path, "./");
        strcpy(obj->vissObj.output_file_path, "./");
        strcpy(obj->ldcObj.output_file_path, "./");
        strcpy(obj->encodeObj.output_file_path, "./");
        strcpy(obj->output_file_path, "./");
    }

    return;
}

vx_int32 app_encode_main(vx_int32 argc, vx_char* argv[])
{
    vx_status status = VX_SUCCESS;

    AppObj *obj = &gAppObj;

    /*Optional parameter setting*/
    app_default_param_set(obj);

    /*Config parameter reading*/
    app_parse_cmd_line_args(obj, argc, argv);

    /* Querry sensor parameters */
    status = app_querry_sensor(&obj->sensorObj);

    /*Update of parameters are config file read*/
    app_update_param_set(obj);

    if(status == VX_SUCCESS)
    {
        status = app_init(obj);
    }
    if(status == VX_SUCCESS)
    {
        APP_PRINTF("App Init Done! \n");

        status = app_create_graph(obj);

        if(status == VX_SUCCESS)
        {
            APP_PRINTF("App Create Graph Done! \n");

            status = app_verify_graph(obj);

            if(status == VX_SUCCESS)
            {
                APP_PRINTF("App Verify Graph Done! \n");

                if (obj->captureObj.enable_error_detection || obj->test_mode)
                {
                    status = app_send_error_frame(obj->context, &obj->captureObj, &obj->sensorObj);
                }

                if (status == VX_SUCCESS)
                {
                    APP_PRINTF("App Send Error Frame Done! \n");
                    if(obj->is_interactive)
                    {
                        status = app_run_graph_interactive(obj);
                    }
                    else
                    {
                        status = app_run_graph(obj);
                    }
                }
            }
        }

        APP_PRINTF("App Run Graph Done! \n");
    }

    app_delete_graph(obj);

    APP_PRINTF("App Delete Graph Done! \n");

    app_deinit(obj);

    APP_PRINTF("App De-init Done! \n");

    if(obj->test_mode == 1)
    {
        if((test_result == vx_false_e) || (status != VX_SUCCESS))
        {
            printf("\n\nTEST FAILED\n\n");
            status = (status == VX_SUCCESS) ? VX_FAILURE : status;
            print_new_checksum_structs();
        }
        else
        {
            printf("\n\nTEST PASSED\n\n");
        }
    }
    return status;
}

static vx_status app_init(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    /* Create OpenVx Context */
    obj->context = vxCreateContext();
    status = vxGetStatus((vx_reference)obj->context);
    APP_PRINTF("Creating context done!\n");
    if(status == VX_SUCCESS)
    {
        tivxHwaLoadKernels(obj->context);
        tivxImagingLoadKernels(obj->context);
        tivxFileIOLoadKernels(obj->context);
        APP_PRINTF("Kernel loading done!\n");
    }

    /* Initialize modules */
    if(status == VX_SUCCESS)
    {
        /* status not checked in the case that we are waiting for error/test frame */
        app_init_sensor(&obj->sensorObj, "sensor_obj");
        APP_PRINTF("Sensor init done!\n");
    }
    if(status == VX_SUCCESS)
    {
        status = app_init_capture(obj->context, &obj->captureObj, &obj->sensorObj, "capture_obj", APP_BUFFER_Q_DEPTH);
        APP_PRINTF("Capture init done!\n");
    }
    if(status == VX_SUCCESS)
    {
        status = app_init_viss(obj->context, &obj->vissObj, &obj->sensorObj, "viss_obj");
        APP_PRINTF("VISS init done!\n");
    }
    if(status == VX_SUCCESS)
    {
        status = app_init_aewb(obj->context, &obj->aewbObj, &obj->sensorObj, "aewb_obj");
        APP_PRINTF("AEWB init done!\n");
    }
    if((obj->sensorObj.enable_ldc == 1) && (status == VX_SUCCESS))
    {
        status = app_init_ldc(obj->context, &obj->ldcObj, &obj->sensorObj, "ldc_obj");
        APP_PRINTF("LDC init done!\n");
    }
    if(status == VX_SUCCESS)
    {
        status = app_init_img_mosaic(obj->context, &obj->imgMosaicObj, "img_mosaic_obj", APP_BUFFER_Q_DEPTH);
        APP_PRINTF("Img Mosaic init done!\n");
    }
    if(status == VX_SUCCESS)
    {
        status = app_init_encode(obj->context, &obj->encodeObj, "encode_obj");
        APP_PRINTF("Encode init done!\n");
    }
    appPerfPointSetName(&obj->total_perf , "TOTAL");
    appPerfPointSetName(&obj->fileio_perf, "FILEIO");

    return status;
}

static void app_deinit(AppObj *obj)
{
    app_deinit_sensor(&obj->sensorObj);
    APP_PRINTF("Sensor deinit done!\n");

    app_deinit_capture(&obj->captureObj, APP_BUFFER_Q_DEPTH);
    APP_PRINTF("Capture deinit done!\n");

    app_deinit_viss(&obj->vissObj);
    APP_PRINTF("VISS deinit done!\n");

    app_deinit_aewb(&obj->aewbObj);
    APP_PRINTF("AEWB deinit done!\n");

    if(obj->sensorObj.enable_ldc == 1)
    {
        app_deinit_ldc(&obj->ldcObj);
        APP_PRINTF("LDC deinit done!\n");
    }

    app_deinit_img_mosaic(&obj->imgMosaicObj, APP_BUFFER_Q_DEPTH);
    APP_PRINTF("Img Mosaic deinit done!\n");

    app_deinit_encode(&obj->encodeObj);
    APP_PRINTF("Encode deinit done!\n");

    tivxHwaUnLoadKernels(obj->context);
    tivxImagingUnLoadKernels(obj->context);
    tivxFileIOUnLoadKernels(obj->context);
    APP_PRINTF("Kernels unload done!\n");

    vxReleaseContext(&obj->context);
    APP_PRINTF("Release context done!\n");
}

static void app_delete_graph(AppObj *obj)
{
    app_delete_capture(&obj->captureObj);
    APP_PRINTF("Capture delete done!\n");

    app_delete_viss(&obj->vissObj);
    APP_PRINTF("VISS delete done!\n");

    app_delete_aewb(&obj->aewbObj);
    APP_PRINTF("AEWB delete done!\n");

    if(obj->sensorObj.enable_ldc == 1)
    {
        app_delete_ldc(&obj->ldcObj);
        APP_PRINTF("LDC delete done!\n");
    }

    app_delete_img_mosaic(&obj->imgMosaicObj);
    APP_PRINTF("Img Mosaic delete done!\n");

    app_delete_encode(&obj->encodeObj);
    APP_PRINTF("Encode delete done!\n");

    vxReleaseGraph(&obj->graph);
    APP_PRINTF("Graph delete done!\n");
}

static vx_status app_create_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;
    vx_int32 list_depth = (obj->test_mode == 1) ? 3 : 2;
    vx_graph_parameter_queue_params_t graph_parameters_queue_params_list[list_depth];
    vx_int32 graph_parameter_index;

    obj->graph = vxCreateGraph(obj->context);
    status = vxGetStatus((vx_reference)obj->graph);
    if(status == VX_SUCCESS)
    {
        status = vxSetReferenceName((vx_reference)obj->graph, "app_encode_graph");
        APP_PRINTF("Graph create done!\n");
    }

    if(status == VX_SUCCESS)
    {
        status = app_create_graph_capture(obj->graph, &obj->captureObj);
        APP_PRINTF("Capture graph done!\n");
    }

    if(status == VX_SUCCESS)
    {
        status = app_create_graph_viss(obj->graph, &obj->vissObj, obj->captureObj.raw_image_arr[0]);
        APP_PRINTF("VISS graph done!\n");
    }

    if(status == VX_SUCCESS)
    {
        status = app_create_graph_aewb(obj->graph, &obj->aewbObj, obj->vissObj.h3a_stats_arr);
        APP_PRINTF("AEWB graph done!\n");
    }

    vx_int32 idx = 0;
    if(obj->sensorObj.enable_ldc == 1)
    {
        if(status == VX_SUCCESS)
        {
            status = app_create_graph_ldc(obj->graph, &obj->ldcObj, obj->vissObj.output_arr);
            APP_PRINTF("LDC graph done!\n");
        }
        obj->imgMosaicObj.input_arr[idx++] = obj->ldcObj.output_arr;
    }
    else
    {
        obj->imgMosaicObj.input_arr[idx++] = obj->vissObj.output_arr;
    }

    obj->imgMosaicObj.num_inputs = idx;

    if(status == VX_SUCCESS)
    {
        status = app_create_graph_img_mosaic(obj->graph, &obj->imgMosaicObj);
        APP_PRINTF("Img Mosaic graph done!\n");
    }

    if(status == VX_SUCCESS)
    {
        status = app_create_graph_encode(obj->graph, &obj->encodeObj,
        		&obj->imgMosaicObj.output_image[0]);
        APP_PRINTF("Encode graph done!\n");
    }

    if(status == VX_SUCCESS)
    {
        graph_parameter_index = 0;
        add_graph_parameter_by_node_index(obj->graph, obj->captureObj.node, 1);
        obj->captureObj.graph_parameter_index = graph_parameter_index;
        graph_parameters_queue_params_list[graph_parameter_index].graph_parameter_index = graph_parameter_index;
        graph_parameters_queue_params_list[graph_parameter_index].refs_list_size = APP_BUFFER_Q_DEPTH;
        graph_parameters_queue_params_list[graph_parameter_index].refs_list = (vx_reference*)&obj->captureObj.raw_image_arr[0];
        graph_parameter_index++;

        if((obj->en_out_img_write == 1) || (obj->test_mode == 1))
        {
            add_graph_parameter_by_node_index(obj->graph, obj->imgMosaicObj.node, 1);
            obj->imgMosaicObj.graph_parameter_index = graph_parameter_index;
            graph_parameters_queue_params_list[graph_parameter_index].graph_parameter_index = graph_parameter_index;
            graph_parameters_queue_params_list[graph_parameter_index].refs_list_size = APP_BUFFER_Q_DEPTH;
            graph_parameters_queue_params_list[graph_parameter_index].refs_list = (vx_reference*)&obj->imgMosaicObj.output_image[0];
            graph_parameter_index++;
        }

		/* Add Encode Node */
        add_graph_parameter_by_node_index(obj->graph, obj->encodeObj.node, 2);
		graph_parameters_queue_params_list[graph_parameter_index].graph_parameter_index = graph_parameter_index;
		graph_parameters_queue_params_list[graph_parameter_index].refs_list_size = 1;
		graph_parameters_queue_params_list[graph_parameter_index].refs_list =
			(vx_reference*)&obj->encodeObj.bitstream_obj;
		obj->encodeObj.graph_parameter_index = graph_parameter_index;
		graph_parameter_index++;

        if(status == VX_SUCCESS)
        {
            status = vxSetGraphScheduleConfig(obj->graph,
                                VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                graph_parameter_index,
                                graph_parameters_queue_params_list);
        }
        if(status == VX_SUCCESS)
        {
            status = tivxSetGraphPipelineDepth(obj->graph, APP_PIPELINE_DEPTH);
        }
        if(status == VX_SUCCESS)
        {
            status = tivxSetNodeParameterNumBufByIndex(obj->vissObj.node, 6, APP_BUFFER_Q_DEPTH);
        }
        if(status == VX_SUCCESS)
        {
            status = tivxSetNodeParameterNumBufByIndex(obj->vissObj.node, 9, APP_BUFFER_Q_DEPTH);
        }
        if(status == VX_SUCCESS)
        {
            status = tivxSetNodeParameterNumBufByIndex(obj->aewbObj.node, 4, APP_BUFFER_Q_DEPTH);
        }
        if((obj->sensorObj.enable_ldc == 1) && (status == VX_SUCCESS))
        {
            status = tivxSetNodeParameterNumBufByIndex(obj->ldcObj.node, 7, APP_BUFFER_Q_DEPTH);
        }
        if(status == VX_SUCCESS)
        {
            if(!((obj->en_out_img_write == 1) || (obj->test_mode == 1)))
            {
                status = tivxSetNodeParameterNumBufByIndex(obj->imgMosaicObj.node, 1, APP_BUFFER_Q_DEPTH);
            }
        }
        APP_PRINTF("Pipeline params setup done!\n");
    }

    return status;
}

static vx_status app_verify_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    status = vxVerifyGraph(obj->graph);

    if(status == VX_SUCCESS)
    {
        APP_PRINTF("Grapy verify done!\n");
    }

    #if 1
    if(VX_SUCCESS == status)
    {
      status = tivxExportGraphToDot(obj->graph,".", "vx_app_encode");
    }
    #endif

    /* wait a while for prints to flush */
    tivxTaskWaitMsecs(100);

    return status;
}

static vx_status app_run_graph_for_one_frame_pipeline(AppObj *obj, vx_int32 frame_id)
{
	vx_status status = VX_SUCCESS;

	appPerfPointBegin(&obj->total_perf);

	ImgMosaicObj *imgMosaicObj = &obj->imgMosaicObj;
	CaptureObj *captureObj = &obj->captureObj;
	EncodeObj  *encodeObj = &obj->encodeObj;
    vx_uint32 expected_idx, actual_checksum = 0;
    expected_idx = frame_id - APP_BUFFER_Q_DEPTH - 5;

	if(obj->pipeline <= 0)
	{
		/* Enqueue outputs */
		if(((obj->test_mode == 1) || (obj->en_out_img_write == 1)) && (status == VX_SUCCESS))
		{
			status = vxGraphParameterEnqueueReadyRef(obj->graph, imgMosaicObj->graph_parameter_index, (vx_reference*)&imgMosaicObj->output_image[obj->enqueueCnt], 1);
		}

        if(status == VX_SUCCESS)
        {
		    status = vxGraphParameterEnqueueReadyRef(obj->graph, encodeObj->graph_parameter_index, (vx_reference*)&encodeObj->bitstream_obj, 1);
        }
		/* Enqueue inputs during pipeup dont execute */
        if(status == VX_SUCCESS)
        {
		    status = vxGraphParameterEnqueueReadyRef(obj->graph, captureObj->graph_parameter_index, (vx_reference*)&obj->captureObj.raw_image_arr[obj->enqueueCnt], 1);
        }
		obj->enqueueCnt++;
		obj->enqueueCnt   = (obj->enqueueCnt  >= APP_BUFFER_Q_DEPTH)? 0 : obj->enqueueCnt;
		obj->pipeline++;
	}
	if(obj->pipeline > 0)
	{
		vx_image capture_input_image;
		vx_image mosaic_output_image;
		uint32_t num_refs;

		/* Dequeue input */
        if(status == VX_SUCCESS)
        {
		    status = vxGraphParameterDequeueDoneRef(obj->graph, captureObj->graph_parameter_index, (vx_reference*)&capture_input_image, 1, &num_refs);
        }
		if((obj->en_out_img_write == 1) || (obj->test_mode == 1))
		{
			vx_char output_file_name[APP_MAX_FILE_PATH];

			/* Dequeue output */
            if(status == VX_SUCCESS)
            {
                status = vxGraphParameterDequeueDoneRef(obj->graph, imgMosaicObj->graph_parameter_index, (vx_reference*)&mosaic_output_image, 1, &num_refs);
            }
            if((status == VX_SUCCESS) && (obj->test_mode == 1) && (expected_idx >= 0) && (expected_idx < (sizeof(checksums_expected[0])/sizeof(checksums_expected[0][0]))))
            {
                if(app_test_check_image(mosaic_output_image, checksums_expected[obj->sensorObj.sensor_index][expected_idx], &actual_checksum) == vx_false_e)
                {
                    test_result = vx_false_e;
                }
                populate_gatherer(obj->sensorObj.sensor_index, expected_idx, actual_checksum);
            }
            if((status == VX_SUCCESS) && (obj->en_out_img_write == 1))
            {
                appPerfPointBegin(&obj->fileio_perf);
                snprintf(output_file_name, APP_MAX_FILE_PATH, "%s/mosaic_output_%010d_%dx%d.yuv", obj->output_file_path, (frame_id - APP_BUFFER_Q_DEPTH), imgMosaicObj->out_width, imgMosaicObj->out_height);
                status = writeMosaicOutput(output_file_name, mosaic_output_image);
                appPerfPointEnd(&obj->fileio_perf);
            }

			/* Enqueue output */
            if(status == VX_SUCCESS)
            {
			    status = vxGraphParameterEnqueueReadyRef(obj->graph, imgMosaicObj->graph_parameter_index, (vx_reference*)&mosaic_output_image, 1);
            }
        }

		/* Dequeue Bitstream Object */
        if(status == VX_SUCCESS)
        {
		    status = vxGraphParameterDequeueDoneRef(obj->graph, encodeObj->graph_parameter_index, (vx_reference*)&obj->encodeObj.encoded_image, 1, &num_refs);
        }
		/* Write encoded data to file */
        if(status == VX_SUCCESS)
        {
		    status = writeEncodeOutput(&obj->encodeObj);
        }
		/* Encode the buffer again */
        if(status == VX_SUCCESS)
        {
		    status = vxGraphParameterEnqueueReadyRef(obj->graph, encodeObj->graph_parameter_index, (vx_reference*)&encodeObj->bitstream_obj, 1);
        }
		/* Enqueue input - start execution */
        if(status == VX_SUCCESS)
        {
		    status = vxGraphParameterEnqueueReadyRef(obj->graph, captureObj->graph_parameter_index, (vx_reference*)&capture_input_image, 1);
        }
		obj->enqueueCnt++;
		obj->dequeueCnt++;

		obj->enqueueCnt = (obj->enqueueCnt >= APP_BUFFER_Q_DEPTH)? 0 : obj->enqueueCnt;
		obj->dequeueCnt = (obj->dequeueCnt >= APP_BUFFER_Q_DEPTH)? 0 : obj->dequeueCnt;
	}


	appPerfPointEnd(&obj->total_perf);

	return status;
}

static vx_status app_run_graph(AppObj *obj)
{
    vx_status status = VX_SUCCESS;

    SensorObj *sensorObj = &obj->sensorObj;
    vx_int32 frame_id;

    app_pipeline_params_defaults(obj);

    if(NULL == sensorObj->sensor_name)
    {
        printf("sensor name is NULL \n");
        return VX_FAILURE;
    }
    status = appStartImageSensor(sensorObj->sensor_name, ((1 << sensorObj->num_cameras_enabled) - 1));

    for(frame_id = 0; frame_id < obj->num_frames_to_run; frame_id++)
    {
        if(obj->write_file == 1)
        {
            if((obj->captureObj.en_out_capture_write == 1) && (status == VX_SUCCESS))
            {
                status = app_send_cmd_capture_node(&obj->captureObj, frame_id, obj->num_frames_to_write, obj->num_frames_to_skip);
            }
            if((obj->vissObj.en_out_viss_write == 1) && (status == VX_SUCCESS))
            {
                status = app_send_cmd_viss_node(&obj->vissObj, frame_id, obj->num_frames_to_write, obj->num_frames_to_skip);
            }
            if((obj->ldcObj.en_out_ldc_write == 1) && (status == VX_SUCCESS))
            {
                status = app_send_cmd_ldc_node(&obj->ldcObj, frame_id, obj->num_frames_to_write, obj->num_frames_to_skip);
            }
            obj->write_file = 0;
        }

        if(status == VX_SUCCESS)
        {
            printf("frame_id: %d, num_frames_to_run: %d\n", frame_id, obj->num_frames_to_run);
            status = app_run_graph_for_one_frame_pipeline(obj, frame_id);
        }
        /* user asked to stop processing */
        if((obj->stop_task) || (status != VX_SUCCESS))
        {
            break;
        }
    }
    if(status == VX_SUCCESS)
    {
        status = vxWaitGraph(obj->graph);
    }
    obj->stop_task = 1;

    if(status == VX_SUCCESS)
    {
        status = appStopImageSensor(obj->sensorObj.sensor_name, ((1 << sensorObj->num_cameras_enabled) - 1));
    }
    return status;
}

static void app_pipeline_params_defaults(AppObj *obj)
{
  obj->pipeline       = -APP_BUFFER_Q_DEPTH + 1;
  obj->enqueueCnt     = 0;
  obj->dequeueCnt     = 0;
}


static void set_sensor_defaults(SensorObj *sensorObj)
{
    memset(sensorObj->sensor_name, 0, sizeof(sensorObj->sensor_name));
    strncpy(sensorObj->sensor_name, SENSOR_SONY_IMX390_UB953_D3, sizeof(sensorObj->sensor_name)-1);

    sensorObj->num_sensors_found = 0;
    sensorObj->sensor_features_enabled = 0;
    sensorObj->sensor_features_supported = 0;
    sensorObj->sensor_dcc_enabled = 0;
    sensorObj->sensor_wdr_enabled = 0;
    sensorObj->sensor_exp_control_enabled = 0;
    sensorObj->sensor_gain_control_enabled = 0;
}

static void app_default_param_set(AppObj *obj)
{
    set_sensor_defaults(&obj->sensorObj);

    obj->is_interactive = 1;
    obj->test_mode = 0;
    obj->write_file = 0;

    obj->sensorObj.enable_ldc = 0;
    obj->sensorObj.num_cameras_enabled = 1;
    obj->sensorObj.usecase_option = APP_SENSOR_FEATURE_CFG_UC0;
}

static vx_int32 calc_grid_size(vx_uint32 ch)
{
    if(0==ch)
    {
        return -1;
    }
    else if(1==ch)
    {
        return 1;
    }
    else if(4>=ch)
    {
        return 2;
    }
    else if(9>=ch)
    {
        return 3;
    }
    else if(16>=ch)
    {
        return 4;
    }else
    {
        return -1;
    }
}

static void set_img_mosaic_params(ImgMosaicObj *imgMosaicObj, vx_uint32 in_width, vx_uint32 in_height, vx_int32 numCh)
{
    vx_int32 idx, ch;
    vx_int32 grid_size = calc_grid_size(numCh);

    imgMosaicObj->out_width    = DISPLAY_WIDTH;
    imgMosaicObj->out_height   = DISPLAY_HEIGHT;
    imgMosaicObj->num_inputs   = 1;

    tivxImgMosaicParamsSetDefaults(&imgMosaicObj->params);

    idx = 0;

    for(ch = 0; ch < numCh; ch++)
    {
        vx_int32 winX = ch%grid_size;
        vx_int32 winY = ch/grid_size;

        imgMosaicObj->params.windows[idx].startX  = (winX * (in_width/grid_size));
        imgMosaicObj->params.windows[idx].startY  = (winY * (in_height/grid_size));
        imgMosaicObj->params.windows[idx].width   = in_width/grid_size;
        imgMosaicObj->params.windows[idx].height  = in_height/grid_size;
        imgMosaicObj->params.windows[idx].input_select   = 0;
        imgMosaicObj->params.windows[idx].channel_select = ch;
        idx++;
    }

    imgMosaicObj->params.num_windows  = idx;

    /* Number of time to clear the output buffer before it gets reused */
    imgMosaicObj->params.clear_count  = APP_BUFFER_Q_DEPTH;
    imgMosaicObj->params.enable_overlay = 0;
}

static void set_encode_params(AppObj *obj)
{
	obj->encodeObj.inWidth = obj->sensorObj.image_width;
	obj->encodeObj.inHeight = obj->sensorObj.image_height;
}

static void app_update_param_set(AppObj *obj)
{

    vx_uint16 resized_width, resized_height;
    appIssGetResizeParams(obj->sensorObj.image_width, obj->sensorObj.image_height, DISPLAY_WIDTH, DISPLAY_HEIGHT, &resized_width, &resized_height);

    set_img_mosaic_params(&obj->imgMosaicObj, resized_width, resized_height, obj->sensorObj.num_cameras_enabled);
    set_encode_params(obj);

}

/*
 * Utility API used to add a graph parameter from a node, node parameter index
 */
static void add_graph_parameter_by_node_index(vx_graph graph, vx_node node, vx_uint32 node_parameter_index)
{
    vx_parameter parameter = vxGetParameterByIndex(node, node_parameter_index);

    vxAddParameterToGraph(graph, parameter);
    vxReleaseParameter(&parameter);
}
