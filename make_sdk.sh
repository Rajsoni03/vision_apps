NUM_PROCS=$(cat /proc/cpuinfo | grep processor | wc -l)
make sdk -j${NUM_PROCS} 2> >(tee /tmp/sdk_build_errors.log)
if [ $? -ne 0 ]
then
	echo ""
	echo "###################################################"
	echo "BUILD ERRORS:"
	echo "###################################################"
	echo ""
	cat /tmp/sdk_build_errors.log
	echo ""
	echo "###################################################"
	echo "Error log file also here: /tmp/sdk_build_errors.log"
	echo "###################################################"
	echo ""
fi
