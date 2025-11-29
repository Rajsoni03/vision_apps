# Vision Apps Refactoring


## Todo

- [ ] Move Platform code to a separate repo
- [ ] Merge Camera and File bases apps into a single app with config feature to switch between the two modes.
- [ ] Remove utils/gst_wrapper later as it's used in multicam_codac_app.
- [ ] Move dss utils to video_io later.
- [ ] Move fvid to video_io later.
- [ ] Remove ethfw from vision_apps.
- [ ] merge codec and openmax_wrapper utils later.
- [ ] Move `utils/openmax_wrapper` to firmware code
- [ ] Merge `utils/draw2d` and `utils/grpx` into one
- [ ] Create a Encode/Decode kernel for Codec apps that can handle both linux and QNX.
- [ ] Move `utils/app_init` to platform code
- [ ] Fix Config structure 
- [ ] Each demo should contains it's own README.md file 


## Low Priority Todo
- [ ] Migrate concerto to CMAKE tool
- [ ] `tools/PyTI_PSDK_RTOS` create a new repo and push it into PiPy 
- [ ] `tools/3d_calibration_tool` move it to docs
- [ ] Move tools to a separate repo

### Video_IO --> Multimedia 
 - Encode/Decode Kernel (File I/O)
 - Display
 - Camera Capture -> CSI_RX

