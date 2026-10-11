# Build the CaveBat app out of tree against the Crazyflie firmware.
#
# Include tools/make/oot.mk, not the firmware's top-level Makefile: the
# firmware builds with Kbuild, and the old "APP=1 / APP_SOURCE=..." style
# silently does nothing.
#
# What gets compiled is listed in Kbuild and src/Kbuild, not here.
CRAZYFLIE_BASE := /home/lenovo/crazyflie-firmware

OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
