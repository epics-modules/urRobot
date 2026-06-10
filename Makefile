#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) $(filter-out $(DIRS), configure)
DIRS := $(DIRS) $(filter-out $(DIRS), urRobotSupport)
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard *App))
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocBoot))

ifeq ($(BUILD_IOCS), YES)
 DIRS += $(wildcard iocs)
endif

urRobotSupport_DEPEND_DIRS = configure
urRobotApp_DEPEND_DIRS = configure urRobotSupport

iocBoot_DEPEND_DIRS += $(filter %App,$(DIRS))
iocs_DEPEND_DIRS += $(filter %App,$(DIRS))

include $(TOP)/configure/RULES_TOP


