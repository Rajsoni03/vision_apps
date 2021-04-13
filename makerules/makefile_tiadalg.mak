#
# Utility makefile to build tiadalg libraries
#
# Edit this file to suit your specific build needs
#

tiadalg:
	$(MAKE) -C $(TIADALG_PATH) all -s

tiadalg_docs:
	$(MAKE) -C $(TIADALG_PATH) doxy_docs -s

tiadalg_clean:
	$(MAKE) -C $(TIADALG_PATH) clean -s

tiadalg_scrub:
	$(MAKE) -C $(TIADALG_PATH) scrub -s

.PHONY: tiadalg tiadalg_docs tiadalg_clean tiadalg_scrub
