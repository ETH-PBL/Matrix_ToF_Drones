
# enable app support
APP=1
#Set the task stack size in 32bit word (4 Bytes). The default is 300 (1.2KBytes)
APP_STACKSIZE=1000
#Set the task priority between 0 and 5. Default is 0 (same as IDLE).
APP_PRIORITY = 0


VPATH += src/
VPATH += src/tof_driver
PROJ_OBJ += test_tof.o
PROJ_OBJ += I2C_expander.o
PROJ_OBJ += ToF_process.o
PROJ_OBJ += vl53l5cx_api.o
PROJ_OBJ += ToFplatform.o
PROJ_OBJ += Fly_control.o
# PROJ_OBJ += morph.o



INCLUDES += -Isrc -Isrc/tof_driver

CRAZYFLIE_BASE=../../../crazyflie-firmware

include $(CRAZYFLIE_BASE)/Makefile
CFLAGS := $(CFLAGS:%-Werror=%-Wno-error)
