# Build the CaveBat app layer against the Crazyflie firmware.
#
# NOTE: this must include tools/make/oot.mk, NOT the firmware's top-level
# Makefile. The firmware uses the Kbuild build system; the old
# "APP=1 / APP_SOURCE=..." style silently does not work with it.
#
# What actually gets compiled is decided by Kbuild and src/Kbuild,
# not by this file.
CRAZYFLIE_BASE := /home/lenovo/crazyflie-firmware

OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
