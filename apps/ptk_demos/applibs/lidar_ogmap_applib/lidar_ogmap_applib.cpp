#include "lidar_ogmap_applib_priv.h"

static PTK_RigidTransform   gM_ego_lidar{0.0f, 1.0f, 0.0f, 0.20350f,
                                         -1.0f, 0.0f, 0.0f, 0.11280f,
                                          0.0f, 0.0f, 1.0f, 0.06590f,
                                          0.0f, 0.0f, 0.0f, 1.0f};

static vx_status LIDAROGAPPLIB_createGraph(
        LIDAROGAPPLIB_Context  *appCntxt)
{
    PTK_Alg_LidarOgmapParams   *ogConfig;
    PTK_PointCloud             *cloud;
    PTK_LidarMeta              *meta;
    PTK_Map                    *ogMap;
    uint8_t                    *cloudMem;
    uint8_t                    *metaMem;
    uint8_t                    *mapMem;
    PTK_MapConfig               mapConfig[PTK_ALG_LIDAR_OGMAP_NUM_OUT_MAPS];
    PTK_PointCloudConfig        pcConfig;
    PTK_LidarMetaConfig         lidarMetaConfig;
    PTK_Api_MemoryReq           memReq;
    vx_status                   vxStatus;
    uint64_t                    dummyTime;
    int32_t                     status;

    vxStatus  = VX_SUCCESS;
    ogConfig  = &appCntxt->params.ogPfsdCfg.ogConfig;
    dummyTime = 0;
    cloud     = NULL;
    cloudMem  = NULL;
    meta      = NULL;
    metaMem   = NULL;
    ogMap     = NULL;
    mapMem    = NULL;

    status = PTK_Alg_LidarOgmapConfig(ogConfig, &memReq);

    if ((status != PTK_ALG_RET_SUCCESS) ||
        (memReq.numBlks != PTK_ALG_LIDAR_OGMAP_NUM_MEM_REQ_BLKS))
    {
        PTK_printf("[%s:%d] PTK_Alg_LidarOgmapConfig() failed\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Dummy memory space to initialize maps */
        lidarMetaConfig.lasers       = PTK_LIDAR_HDL32E_LASERS;
        lidarMetaConfig.slices       = PTK_LIDAR_HDL32E_SLICES_PER_ROTATION;
        lidarMetaConfig.distance     = PTK_Lidar_DistOffsetTable;
        lidarMetaConfig.scale        = PTK_Lidar_ScaleTable;
        lidarMetaConfig.angle        = PTK_Lidar_AngleOffsetTable;
        lidarMetaConfig.height       = PTK_Lidar_HeightOffsetTable;
        lidarMetaConfig.gatingParams = appCntxt->params.gatingParams;

        PTK_Lidar_Velodyne_setHDL32ETrigTables(&lidarMetaConfig);
        appCntxt->lidarMetaSize = PTK_LidarMeta_getSize(&lidarMetaConfig);

        pcConfig.maxPoints = PTK_LIDAR_HDL32E_POINTS_PER_ROTATION;
        appCntxt->pcSize = PTK_PointCloud_getSize(&pcConfig);

        appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);

        if (appCntxt->vxGraph == NULL)
        {
            PTK_printf("[%s:%d] vxCreateGraph() failed\n",
                        __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGraph,
                               "Lidar Occupancy Grid Graph");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        cloudMem = new uint8_t[appCntxt->pcSize];

        if (cloudMem == NULL)
        {
            PTK_printf("[%s:%d] Memory allocation failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            cloud = PTK_PointCloud_init(cloudMem, &pcConfig);

            if (cloud == NULL)
            {
                PTK_printf("[%s:%d] PTK_PointCloud_init() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        metaMem = new uint8_t[appCntxt->lidarMetaSize];

        if (metaMem == NULL)
        {
            PTK_printf("[%s:%d] Memory allocation failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            meta = PTK_LidarMeta_init(metaMem, &lidarMetaConfig);

            if (meta == NULL)
            {
                PTK_printf("[%s:%d] PTK_LidarMeta_init() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        memset(mapConfig, 0, sizeof(mapConfig));

        /* Get the map configuration. */
        status = PTK_Alg_LidarOgmapGetMapConfig(ogConfig, mapConfig);

        /* According to the LIDAR OGMAP API guide:
         * - the entry with id * PTK_ALG_LIDAR_OGMAP_MEM_BUFF_MAP_ACC_OCC gives the
         *   output accumulated map size.
         * - the entry with id * PTK_ALG_LIDAR_OGMAP_MEM_BUFF_MAP_INST_DS gives the
         *   output instantaneous map size.
         */
        if (status != PTK_ALG_RET_SUCCESS)
        {
            PTK_printf("[%s:%d] PTK_Alg_LidarOgmapGetMapConfig() failed\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            /* Accumulated output map. */
            appCntxt->outAccMapSize =
                memReq.blks[PTK_ALG_LIDAR_OGMAP_MEM_BUFF_MAP_ACC_OCC].size;

            mapMem = new uint8_t[appCntxt->outAccMapSize];

            if (mapMem == NULL)
            {
                PTK_printf("[%s:%d] Memory allocation failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
            else
            {
                ogMap =
                PTK_Map_init(mapMem,
                             &mapConfig[PTK_ALG_LIDAR_OGMAP_OUT_MAP_ACC_OCC]);

                if (ogMap == NULL)
                {
                    PTK_printf("[%s:%d] PTK_Map_init() failed.\n",
                               __FUNCTION__, __LINE__);

                    vxStatus = VX_FAILURE;
                }
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Output map object. */
        appCntxt->vxOutAccMap =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "AccMap",
                                   appCntxt->outAccMapSize,
                                   ogMap);

        if (appCntxt->vxOutAccMap == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxOutAccMap,
                               "LidarOutAccMap");
        }
    }

    if (mapMem != NULL)
    {
        delete [] mapMem;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Instantaneous output map. */
        appCntxt->outInstMapSize =
            memReq.blks[PTK_ALG_LIDAR_OGMAP_MEM_BUFF_MAP_INST_DS].size;

        mapMem = new uint8_t[appCntxt->outInstMapSize];

        if (mapMem == NULL)
        {
            PTK_printf("[%s:%d] Memory allocation failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            ogMap =
                PTK_Map_init(mapMem,
                             &mapConfig[PTK_ALG_LIDAR_OGMAP_OUT_MAP_INST_DS]);

            if (ogMap == NULL)
            {
                PTK_printf("[%s:%d] PTK_Map_init() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    /* Graph parameters. */
    for (uint32_t i = 0; i < appCntxt->pipelineDepth; i++ )
    {
        /* Lidar Point Cloud data. */
        appCntxt->vxLidarPointCloud[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "LidarPointCloud",
                                   appCntxt->pcSize,
                                   cloud);

        if (appCntxt->vxLidarPointCloud[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
            break;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxLidarPointCloud[i],
                               "LidarPointCloud");
        }

        /* Position and reference object. */
        appCntxt->vxPoseAndRef[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_InsPoseAndRef",
                                   sizeof(PTK_InsPoseAndRef),
                                   NULL);

        if (appCntxt->vxPoseAndRef[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
            break;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxPoseAndRef[i],
                               "PoseAndReference");
        }

        /* Lidar Meta data. */
        appCntxt->vxLidarMeta[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "LidarMetaData",
                                   appCntxt->lidarMetaSize,
                                   meta);

        if (appCntxt->vxLidarMeta[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
            break;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxLidarMeta[i],
                               "LidarMetaData");
        }

        /* Output instantaneous map. */
        appCntxt->vxOutInstMap[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "InstMap",
                                   appCntxt->outInstMapSize,
                                   ogMap);

        if (appCntxt->vxOutInstMap[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
            break;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxOutInstMap[i],
                               "LidarOutInstMap");
        }

        /* Output PFSD descriptor object. */
        appCntxt->vxPfsdOutDesc[i] =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_Alg_FsdPfsdPSDesc",
                                   sizeof(PTK_Alg_FsdPfsdPSDesc),
                                   NULL);

        if (appCntxt->vxPfsdOutDesc[i] == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
            break;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxPfsdOutDesc[i],
                               "PfsdOutDesc");
        }

    } // for (i = 0; i < appCntxt->pipelineDepth; i++ )

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Lidar pose. */
        appCntxt->vxM_ego_lidar =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_RigidTransform",
                                   sizeof(PTK_RigidTransform),
                                   &gM_ego_lidar);

        if (appCntxt->vxM_ego_lidar == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxM_ego_lidar,
                               "LidarPose");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Lidar Reference frame. */
        appCntxt->vxRootECEF =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_Position",
                                   sizeof(PTK_Position),
                                   NULL);

        if (appCntxt->vxRootECEF == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxRootECEF,
                               "LidarRootPosition");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Lidar MDC point cloud. */
        appCntxt->vxLidarMdcPointCloud =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "MotionCorrectedLidarPointCloud",
                                   appCntxt->pcSize,
                                   cloud);

        if (appCntxt->vxLidarMdcPointCloud == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxLidarMdcPointCloud,
                               "MotionCorrectedLidarPointCloud");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Inputs to GPC node. */
        appCntxt->vxGpcConfig =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PTK_Lidar_GpcConfig",
                                   sizeof(PTK_Lidar_GpcConfig),
                                   &appCntxt->gpcConfig);

        if (appCntxt->vxGpcConfig == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGpcConfig,
                               "GPCConfig");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Normal vectors.. */
        appCntxt->vxLidarNormalCloud =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "NormalVectors",
                                   appCntxt->pcSize,
                                   cloud);

        if (appCntxt->vxLidarNormalCloud == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxLidarNormalCloud,
                               "NormalVectors");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Point cloud with ground tag. */
        appCntxt->vxLidarGpcPointCloud =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "PointCloudWithGroundTag",
                                   appCntxt->pcSize,
                                   cloud);

        if (appCntxt->vxLidarGpcPointCloud == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxLidarGpcPointCloud,
                               "PointCloudWithGroundTag");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Input to OG node. */
        appCntxt->vxOgConfig =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "tivx_lidar_ogmap_pfsd_params_t",
                                   sizeof(tivx_lidar_ogmap_pfsd_params_t),
                                   &appCntxt->params.ogPfsdCfg);

        if (appCntxt->vxOgConfig == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxOgConfig,
                               "LidarOgmapConfig");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->vxPositionTime =
            vxCreateUserDataObject(appCntxt->vxContext,
                                   "uint64_t",
                                   sizeof(uint64_t),
                                   &dummyTime);

        if (appCntxt->vxPositionTime == NULL)
        {
            PTK_printf("[%s:%d] vxCreateUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxPositionTime,
                               "Timestamp");
        }
    }

    /* vxRootECEF is populated in the appCntxt when it runs. */

    /* Create all nodes. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->mdcNode =
            tivxLidarMdcNode(appCntxt->vxGraph,
                             appCntxt->vxLidarPointCloud[0],
                             appCntxt->vxLidarMeta[0],
                             appCntxt->vxM_ego_lidar,
                             appCntxt->vxRootECEF,
                             appCntxt->vxLidarMdcPointCloud);

        if (appCntxt->mdcNode == NULL)
        {
            PTK_printf("[%s:%d] tivxLidarMdcNode() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->mdcNode,
                               "MotionDistortionCorrection");

            vxStatus = vxSetNodeTarget(appCntxt->mdcNode,
                                       VX_TARGET_STRING,
                                       appCntxt->mdcNodeCore);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxSetNodeTarget() failed.\n",
                           __FUNCTION__, __LINE__);
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->gpcNode =
            tivxLidarGpcNode(appCntxt->vxGraph,
                             appCntxt->vxGpcConfig,
                             appCntxt->vxLidarMdcPointCloud,
                             appCntxt->vxLidarMeta[0],
                             appCntxt->vxM_ego_lidar,
                             appCntxt->vxRootECEF,
                             appCntxt->vxLidarGpcPointCloud,
                             appCntxt->vxLidarNormalCloud);

        if (appCntxt->gpcNode == NULL)
        {
            PTK_printf("[%s:%d] tivxLidarGpcNode() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->gpcNode,
                               "GroundPlaneClassification");

            vxStatus = vxSetNodeTarget(appCntxt->gpcNode,
                                       VX_TARGET_STRING,
                                       appCntxt->gpcNodeCore);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxSetNodeTarget() failed.\n",
                           __FUNCTION__, __LINE__);
            }

        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->ogNode =
            tivxLidarOgmapNode(appCntxt->vxGraph,
                               appCntxt->vxOgConfig,
                               appCntxt->vxLidarGpcPointCloud,
                               appCntxt->vxPoseAndRef[0],
                               appCntxt->vxPfsdOutDesc[0],
                               appCntxt->vxOutAccMap,
                               appCntxt->vxOutInstMap[0]);

        if (appCntxt->ogNode == NULL)
        {
            PTK_printf("[%s:%d] tivxLidarOgmapNode() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->ogNode,
                               "LidarOgmap");

            vxStatus = vxSetNodeTarget(appCntxt->ogNode,
                                       VX_TARGET_STRING,
                                       appCntxt->ogNodeCore);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxSetNodeTarget() failed.\n",
                           __FUNCTION__, __LINE__);
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* set up the pipeline. */
        vxStatus = LIDAROGAPPLIB_setupPipeline(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] LIDAROGAPPLIB_setupPipeline() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxVerifyGraph(appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxVerifyGraph() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (appCntxt->params.exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".",
                                 "vx_applib_lidar_ogmap");
        }

        if (appCntxt->params.rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }
    }

    /** Cache the reference to the accumulated map. This is persistent so it is
     * safe to cache it.
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->perf , "LIDAR OGMAP");

        appCntxt->outAccMap =
            (PTK_Map*)ptkdemo_getUserObjDataPayload(appCntxt->vxOutAccMap);

        if (appCntxt->outAccMap == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (mapMem != NULL)
    {
        delete [] mapMem;
    }

    if (metaMem != NULL)
    {
        delete [] metaMem;
    }

    if (cloudMem != NULL)
    {
        delete [] cloudMem;
    }

    return vxStatus;
}

LIDAROGAPPLIB_Handle LIDAROGAPPLIB_create(
        LIDAROGAPPLIB_CreateParams *params)
{
    LIDAROGAPPLIB_Handle        handle;
    LIDAROGAPPLIB_Context      *appCntxt;
    PTK_Alg_LidarOgmapParams   *ogConfig;
    vx_status                   vxStatus;

    handle   = new LIDAROGAPPLIB_Context();
    vxStatus = VX_SUCCESS;

    if (handle == NULL)
    {
        PTK_printf("[%s:%d] Could not allocate memory for internal context.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt         = (LIDAROGAPPLIB_Context *)handle;
        appCntxt->params = *params;
        ogConfig         = &appCntxt->params.ogPfsdCfg.ogConfig;

        PTK_Lidar_Velodyne_initParser();

        appCntxt->vxContext = params->vxContext;

        appCntxt->gpcConfig.removed_tag = ogConfig->tagPcRemoved;
        appCntxt->gpcConfig.ground_tag  = ogConfig->tagPcGround;
        appCntxt->gpcConfig.z_threshold = 0.55f;
        appCntxt->gpcConfig.ground_dtol = 0.20f;
        appCntxt->gpcConfig.ransac_iter = 45;

        appCntxt->ogNodeCore   = params->ogNodeCore;
        appCntxt->mdcNodeCore  = params->mdcNodeCore;
        appCntxt->gpcNodeCore  = params->gpcNodeCore;

        if (appCntxt->ogNodeCore == nullptr)
        {
            appCntxt->ogNodeCore = LIDAROGAPPLIB_DEFAULT_CORE_MAPPING;
        }

        if (appCntxt->mdcNodeCore == nullptr)
        {
            appCntxt->mdcNodeCore = LIDAROGAPPLIB_DEFAULT_CORE_MAPPING;
        }

        if (appCntxt->gpcNodeCore == nullptr)
        {
            appCntxt->gpcNodeCore = LIDAROGAPPLIB_DEFAULT_CORE_MAPPING;
        }

        /* Set the pipeline depth. */
        if (!params->pipelineDepth ||
            (params->pipelineDepth > LIDAROGAPPLIB_PIPELINE_DEPTH))
        {
            PTK_printf("[%s:%d] Invalid pipeline depth value. "
                       "Passed %d. Allowed range [1..%d].\n",
                       __FUNCTION__,
                       __LINE__,
                       params->pipelineDepth,
                       LIDAROGAPPLIB_PIPELINE_DEPTH);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->pipelineDepth = params->pipelineDepth;

        vxStatus = LIDAROGAPPLIB_createGraph(appCntxt);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] LIDAROGAPPLIB_createGraph() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus != VX_SUCCESS)
    {
        LIDAROGAPPLIB_delete(&handle);
    }

    return handle;
}

vx_status LIDAROGAPPLIB_setRootPosition(
        LIDAROGAPPLIB_Handle        handle,
        const PTK_RigidTransform_d *ecef_w,
        const PTK_Position         *root)
{
    LIDAROGAPPLIB_Context  *appCntxt;
    PTK_InsPoseAndRef      *poseAndRef;
    void                   *ptr;
    vx_status               vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (uint32_t i = 0; i < appCntxt->pipelineDepth; i++)
        {
            /* Get a reference to the Pose and reference buffer. */
            ptr = ptkdemo_getUserObjDataPayload(appCntxt->vxPoseAndRef[i]);

            if (ptr == NULL)
            {
                PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
                break;
            }

            poseAndRef         = (PTK_InsPoseAndRef*)ptr;
            poseAndRef->w2Ecef = *ecef_w;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxCopyUserDataObject(appCntxt->vxRootECEF,
                                        0,
                                        sizeof(PTK_Position),
                                        (void*)root,
                                        VX_WRITE_ONLY,
                                        VX_MEMORY_TYPE_HOST);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxCopyUserDataObject() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    return vxStatus;
}

static void LIDAROGAPPLIB_releaseGraph(LIDAROGAPPLIB_Handle handle)
{
    LIDAROGAPPLIB_Context  *appCntxt;

    appCntxt = (LIDAROGAPPLIB_Context*)handle;

    /* Relase the nodes. */
    if (appCntxt->mdcNode != NULL)
    {
        vxReleaseNode(&appCntxt->mdcNode);
    }

    if (appCntxt->gpcNode != NULL)
    {
        vxReleaseNode(&appCntxt->gpcNode);
    }

    if (appCntxt->ogNode != NULL)
    {
        vxReleaseNode(&appCntxt->ogNode);
    }

    /* Release the parameter memory. */
    for (uint32_t i = 0; i < appCntxt->pipelineDepth; i++ )
    {
        if (appCntxt->vxLidarPointCloud[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxLidarPointCloud[i]);
        }

        if (appCntxt->vxLidarMeta[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxLidarMeta[i]);
        }

        if (appCntxt->vxPoseAndRef[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxPoseAndRef[i]);
        }

        if (appCntxt->vxPfsdOutDesc[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxPfsdOutDesc[i]);
        }

        if (appCntxt->vxOutInstMap[i] != NULL)
        {
            vxReleaseUserDataObject(&appCntxt->vxOutInstMap[i]);
        }
    }

    if (appCntxt->vxM_ego_lidar != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxM_ego_lidar);
    }

    if (appCntxt->vxRootECEF != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxRootECEF);
    }

    if (appCntxt->vxLidarMdcPointCloud != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxLidarMdcPointCloud);
    }

    if (appCntxt->vxGpcConfig != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxGpcConfig);
    }

    if (appCntxt->vxLidarNormalCloud != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxLidarNormalCloud);
    }

    if (appCntxt->vxLidarGpcPointCloud != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxLidarGpcPointCloud);
    }

    if (appCntxt->vxPositionTime != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxPositionTime);
    }

    if (appCntxt->vxOgConfig != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxOgConfig);
    }

    if (appCntxt->vxOutAccMap != NULL)
    {
        vxReleaseUserDataObject(&appCntxt->vxOutAccMap);
    }

    if (appCntxt->params.rtLogEnable == 1)
    {
        tivxLogRtTraceDisable(appCntxt->vxGraph);
    }

    if (appCntxt->vxGraph != NULL)
    {
        /* Release the graph. */
        vxReleaseGraph(&appCntxt->vxGraph);
    }

    return;
}

vx_status LIDAROGAPPLIB_delete(
        LIDAROGAPPLIB_Handle *handle)
{
    if ((handle != NULL) && (*handle != NULL))
    {
        /* Delete the nodes. */
        LIDAROGAPPLIB_releaseGraph(*handle);

        delete *handle;
        *handle = NULL;
    }

    return VX_SUCCESS;
}

static void ADAM_Lidar_removeCarReflection(
        PTK_PointCloud *src,
        PTK_LidarMeta  *meta)
{
    uint32_t slice;
    uint32_t laser;
    uint32_t maxSlices = PTK_LidarMeta_getSlices(meta);
    uint32_t maxLasers = PTK_LidarMeta_getLasers(meta);

    for (slice = 0; slice < maxSlices; ++slice)
    {
        for (laser = 0; laser < maxLasers; ++laser)
        {
            uint32_t idx = PTK_LidarMeta_getPointIndex(meta, slice, laser);

            if (PTK_POINTCLOUD_INVALID_POINT != idx)
            {
                PTK_Point *pt = PTK_PointCloud_refPoint(src, idx);

                // Remove all points inside the box [-2.2,-1.4]x[1.9,0.4]
                if ((pt->x > -2.2f)&&
                    (pt->x < 1.9f) &&
                    (pt->y > -1.4f) &&
                    (pt->y < 0.4f ))
                {
                    // @todo need a tag to use defined by the application to
                    // use for invalid points
                    PTK_PointCloud_tag(src, idx, TAG_POINT_REMOVED);
                    PTK_LidarMeta_setPointIndex(meta,
                                                slice,
                                                laser,
                                                PTK_POINTCLOUD_INVALID_POINT);
                }
            }
        }
    }
}

static vx_status LIDAROGAPPLIB_setupNodes(
        LIDAROGAPPLIB_Context      *appCntxt,
        LIDAROGAPPLIB_graphParams  *gpDesc,
        uint8_t                    *data)
{
    PTK_PointCloud     *cloud;
    PTK_LidarMeta      *meta;
    uint64_t           *positionTime;
    PTK_InsPoseAndRef  *poseAndRef;
    PTK_INS_Record     *curInsRec;
    PTK_INS_Record     *prevInsRec;
    void               *ptr;
    PTK_INS_RetCode     retCode;
    uint64_t            time;
    uint32_t            packetCount;
    uint32_t            point;
    vx_status           vxStatus;

    vxStatus = VX_SUCCESS;

    /* Map cloud and metadata from vx to local system. */
    ptr = ptkdemo_getUserObjDataPayload(gpDesc->vxLidarPointCloud);

    if (ptr == NULL)
    {
        PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        cloud = (PTK_PointCloud *)ptr;

        ptr = ptkdemo_getUserObjDataPayload(gpDesc->vxLidarMeta);

        if (ptr == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        meta = (PTK_LidarMeta *)ptr;

        /* Clear previous data in structures. */
        PTK_PointCloud_clear(cloud);
        PTK_LidarMeta_clear(meta);

        /* Parse the lidar data from TIAD. */
        for (packetCount = 0;
             packetCount < PTK_LIDAR_HDL32E_MAX_PACKETS;
             ++packetCount)
        {
            PTK_Lidar_VelodynePacket *packet;
            packet = (PTK_Lidar_VelodynePacket *)
                     (data + packetCount*sizeof(PTK_Lidar_VelodynePacket));

            PTK_Lidar_Velodyne_parse(packet, cloud, meta);
        }

        /* Clear the meta field that is set with reflection data. */
        for (point = 0; point < PTK_PointCloud_getPointCount(cloud); ++point)
        {
            PTK_Point *pt = PTK_PointCloud_refPoint(cloud, point);
            pt->meta.w = 0;
        }

        ADAM_Lidar_removeCarReflection(cloud, meta);

        /* Copy Reference and Pose Information. The world reference is expected to
         * be valid by this time. It should have been set by
         * LIDAROGAPPLIB_setRootPosition() API.
         */
        /* Get a reference to the Pose and reference buffer. */
        ptr = ptkdemo_getUserObjDataPayload(gpDesc->vxPoseAndRef);

        if (ptr == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        poseAndRef = (PTK_InsPoseAndRef*)ptr;
        curInsRec  = &poseAndRef->curInsRec;
        prevInsRec = &poseAndRef->prevInsRec;
        time       = PTK_LidarMeta_getTimestamp(meta, 0); /* Slice 0 */

        retCode = PTK_INS_getRecordLinearInterp(PTK_INS_RECORD_TYPE_INSPVA,
                                                time,
                                                curInsRec);

        if (( retCode != PTK_INS_RETURN_CODE_OK ) &&
            ( retCode != PTK_INS_RETURN_CODE_RECORD_PARTIALLY_AVAIL ))
        {
            PTK_printf("[%s:%d] PTK_INS_getRecordLinearInterp() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
        else
        {
            retCode = PTK_INS_getRecordBefore(PTK_INS_RECORD_TYPE_INSPVA,
                                              time,
                                              prevInsRec);

            if ( retCode != PTK_INS_RETURN_CODE_OK )
            {
                PTK_printf("[%s:%d] PTK_INS_getRecordBefore() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Set position and heading information for graph. */
        ptr = ptkdemo_getUserObjDataPayload(appCntxt->vxPositionTime);

        if (ptr == NULL)
        {
            PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        positionTime = (uint64_t *)ptr;
        *positionTime = time;
    }

    return vxStatus;
}

vx_status LIDAROGAPPLIB_process(
        LIDAROGAPPLIB_Handle    handle,
        uint8_t                *data)
{
    LIDAROGAPPLIB_Context      *appCntxt;
    vx_user_data_object         obj[LIDAROGAPPLIB_NUM_GRAPH_PARAMS];
    LIDAROGAPPLIB_graphParams   gpDesc;
    vx_status                   vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context*)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = LIDAROGAPPLIB_getFreeParamRsrc(appCntxt, &gpDesc);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] LIDAROGAPPLIB_getFreeParamRsrc() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = LIDAROGAPPLIB_setupNodes(appCntxt, &gpDesc, data);
        
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] LIDAROGAPPLIB_setupNodes() failed.\n",
                       __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        obj[0] = gpDesc.vxLidarPointCloud;
        obj[1] = gpDesc.vxLidarMeta;
        obj[2] = gpDesc.vxPoseAndRef;
        obj[3] = gpDesc.vxPfsdOutDesc;
        obj[4] = gpDesc.vxOutInstMap;

        for (uint32_t i = 0; i < LIDAROGAPPLIB_NUM_GRAPH_PARAMS; i++)
        {
            vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                       i,
                                                       (vx_reference*)&obj[i],
                                                       1);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                PTK_printf("[%s:%d] vxGraphParameterEnqueueReadyRef(%d) "
                           "failed\n", __FUNCTION__, __LINE__, i);
                break;
            }
        }
    }

    return vxStatus;
}

PTK_Map* LIDAROGAPPLIB_getOutAccMap(
        LIDAROGAPPLIB_Handle    handle)
{
    LIDAROGAPPLIB_Context  *appCntxt;
    PTK_Map                *map;

    appCntxt = (LIDAROGAPPLIB_Context*)handle;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        map = NULL;
    }
    else
    {
        map = appCntxt->outAccMap;
    }

    return map;
}

uint32_t LIDAROGAPPLIB_getOutAccMapSize(
        LIDAROGAPPLIB_Handle    handle)
{
    LIDAROGAPPLIB_Context  *appCntxt;
    uint32_t                size;

    appCntxt = (LIDAROGAPPLIB_Context*)handle;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        size = 0;
    }
    else
    {
        size = appCntxt->outAccMapSize;
    }

    return size;
}

int32_t LIDAROGAPPLIB_getOutBuff(LIDAROGAPPLIB_Handle       handle,
                                 LIDAROGAPPLIB_OutputBuff  *buff)
{
    LIDAROGAPPLIB_Context      *appCntxt;
    LIDAROGAPPLIB_graphParams  *desc;
    PTK_InsPoseAndRef          *poseAndRef;
    void                       *ptr;
    vx_status                   vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context*)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        std::unique_lock<std::mutex>    lock(appCntxt->paramRsrcMutex);

        if (appCntxt->outputQ.empty())
        {
            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Get the descriptor. */
            desc = appCntxt->outputQ.front();

            /* Get a reference to the Pose and reference buffer. */
            ptr = ptkdemo_getUserObjDataPayload(desc->vxPoseAndRef);

            if (ptr == NULL)
            {
                PTK_printf("[%s:%d] ptkdemo_getUserObjDataPayload() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        poseAndRef          = (PTK_InsPoseAndRef*)ptr;
        buff->ts            = poseAndRef->curInsRec.timestamp;
        buff->outAccMap     = appCntxt->outAccMap;
        buff->vxOutInstMap  = desc->vxOutInstMap;
        buff->vxPfsdOutDesc = desc->vxPfsdOutDesc;
    }

    return vxStatus;
}

vx_status LIDAROGAPPLIB_releaseOutBuff(
        LIDAROGAPPLIB_Handle    handle)
{
    LIDAROGAPPLIB_Context      *appCntxt;
    LIDAROGAPPLIB_graphParams  *desc;
    vx_status                   vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        std::unique_lock<std::mutex> lock(appCntxt->paramRsrcMutex);

        if (appCntxt->outputQ.empty())
        {
            PTK_printf("[%s:%d] No output buffers available.\n",
                       __FUNCTION__, __LINE__);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            desc = appCntxt->outputQ.front();
            appCntxt->outputQ.pop();

            /* Push the descriptor into the free queue. */
            appCntxt->freeQ.push(desc);
        }
    }

    return vxStatus;
}

vx_status LIDAROGAPPLIB_printStats(
        LIDAROGAPPLIB_Handle    handle)
{
    LIDAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = tivx_utils_graph_perf_print(appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivx_utils_graph_perf_print() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointPrint(&appCntxt->perf);
        appPerfPointPrintFPS(&appCntxt->perf);
    }

    return vxStatus;
}

vx_status LIDAROGAPPLIB_exportStats(
        LIDAROGAPPLIB_Handle    handle,
        FILE                   *fp,
        bool                    exportAll)
{
    LIDAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = tivx_utils_graph_perf_export(fp, appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivx_utils_graph_perf_export() failed\n",
                        __FUNCTION__, __LINE__);
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (exportAll == true)
            {
                app_perf_point_t *perfArr[1] = {&appCntxt->perf};
                appPerfStatsExportAll(fp, perfArr, 1);
            }
        }
    }

    return vxStatus;
}

vx_status LIDAROGAPPLIB_waitGraph(
        LIDAROGAPPLIB_Handle    handle)
{
    LIDAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxWaitGraph(appCntxt->vxGraph);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            PTK_printf("[%s:%d] vxWaitGraph() failed\n",
                        __FUNCTION__, __LINE__);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Wait for the output queue to get flushed. */
        while (appCntxt->freeQ.size() != appCntxt->pipelineDepth)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return vxStatus;
}

vx_status LIDAROGAPPLIB_processEvent(
        LIDAROGAPPLIB_Handle    handle,
        vx_event_t             *event)
{
    LIDAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }

    if ((vxStatus == (vx_status)VX_SUCCESS) &&
        (event->type == VX_EVENT_GRAPH_COMPLETED))
    {
        vx_reference    ref[LIDAROGAPPLIB_NUM_GRAPH_PARAMS];
        uint32_t        numRefs;
        uint32_t        index;
        int32_t         status;
        uint32_t        appValue;
        uint32_t        i;

        appValue = appCntxt->params.vxEvtAppValBase +
                   LIDAROGAPPLIB_GRAPH_COMPLETE_EVENT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            PTK_printf("[%s:%d] Unknown App Value [%d].\n",
                       __FUNCTION__, __LINE__, event->app_value);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->startPerfCapt == false)
            {
                appCntxt->startPerfCapt = true;
            }
            else
            {
                appPerfPointEnd(&appCntxt->perf);
            }

            appPerfPointBegin(&appCntxt->perf);

            for (i = 0; i < LIDAROGAPPLIB_NUM_GRAPH_PARAMS; i++)
            {
                vxStatus = vxGraphParameterDequeueDoneRef(appCntxt->vxGraph,
                                                          i,
                                                          &ref[i],
                                                          1,
                                                          &numRefs);
                if (vxStatus != VX_SUCCESS)
                {
                    PTK_printf("[%s:%d] vxGraphParameterDequeueDoneRef() "
                               "failed\n", __FUNCTION__, __LINE__);

                    break;
                }
            }
        }

        /* The first one to deque is vxLidarPointCloud parameter. Search and
         * identify the resource index.
         */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            index = 255;
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                if (ref[0] == (vx_reference)appCntxt->vxLidarPointCloud[i])
                {
                    index = i;
                    break;
                }
            }

            if (index == 255)
            {
                PTK_printf("[%s:%d] Resource look up failed\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Mark the dequeued resource as free. */
            status = LIDAROGAPPLIB_releaseParamRsrc(appCntxt, index);

            if (status < 0)
            {
                PTK_printf("[%s:%d] LIDAROGAPPLIB_releaseParamRsrc() failed.\n",
                           __FUNCTION__, __LINE__);

                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;

}

vx_status LIDAROGAPPLIB_reset(
        LIDAROGAPPLIB_Handle    handle)
{
    LIDAROGAPPLIB_Context  *appCntxt;
    vx_status               vxStatus;

    appCntxt = (LIDAROGAPPLIB_Context *)handle;
    vxStatus = VX_SUCCESS;

    if (appCntxt == NULL)
    {
        PTK_printf("[%s:%d] Invalid handle.\n",
                   __FUNCTION__, __LINE__);

        vxStatus = VX_FAILURE;
    }
    else
    {
        vxStatus = tivxNodeSendCommand(appCntxt->ogNode,
                                       0,
                                       TIVX_KERNEL_LIDAR_OGMAP_RESET,
                                       NULL,
                                       0);

        if (vxStatus != VX_SUCCESS)
        {
            PTK_printf("[%s:%d] tivxNodeSendCommand() failed.\n",
                       __FUNCTION__, __LINE__);
        }

        /* Reset the accumulated map. */
        PTK_Map_clear(appCntxt->outAccMap);

        /* Reset the performance capture initialization flag. */
        appCntxt->startPerfCapt = false;
    }

    return vxStatus;
}

