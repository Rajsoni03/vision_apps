if [[ -d /ti_fs/ ]]
then
    if [[ -f /ti_fs/vision_apps/test_data/psdkra/app_multi_cam_codec/TI_Custom_1920x1080_5secs.264 ]]
    then
        cp /ti_fs/vision_apps/test_data/psdkra/app_multi_cam_codec/TI_Custom_1920x1080_5secs.264 /tmp/test_data.264
    else
        echo "ERROR=> Test Data Not Found"
        exit -1
    fi
fi

/opt/vision_apps/vx_app_multi_cam_codec.out --cfg /opt/vision_apps/app_multi_cam_codec.cfg