#
# Utility makefile to copy test data
#
# Edit this file to suit your specific build needs
#

# INSTALL_TEST_DATA macro for TI internal development with test_data repo
# $1 : rootfs path
# $2 : folder in rootfs
define INSTALL_TEST_DATA =
	mkdir -p $(1)/$(2)/test_data
	mkdir -p $(1)/$(2)/test_data/output
	cp $(TIOVX_PATH)/conformance_tests/test_data/*.bmp $(1)/$(2)/test_data
	cp $(TIOVX_PATH)/conformance_tests/test_data/*.txt $(1)/$(2)/test_data
	cp -r $(TIOVX_PATH)/conformance_tests/test_data/harriscorners $(1)/$(2)/test_data/
	cp -r $(TIOVX_PATH)/conformance_tests/test_data/tivx $(1)/$(2)/test_data/
	cp -r $(TIOVX_PATH)/conformance_tests/test_data/psdkra $(1)/$(2)/test_data/
	sync
endef

