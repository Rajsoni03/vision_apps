echo "!Starting Heterogeneous Test IMX728 + IMX390 for 500 frames!"


ln -s /opt/imaging/imx728/wdr/dcc_viss_wdr.bin dcc_csi0.bin
ln -s /opt/imaging/imx390/wdr/dcc_viss_wdr.bin dcc_csi1.bin

./vx_app_heterogeneous.out 500 0 4 4 0 3687 3072 3072 4770 1389 1024 1024 1954


rm dcc_csi0.bin
rm dcc_csi1.bin