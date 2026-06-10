TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# urRobot requires C++17 (std::optional)
HAS_CXX17 := $(shell $(CXX) -std=c++17 -fsyntax-only -x c++ /dev/null 2>/dev/null && echo YES || echo NO)
USR_CXXFLAGS += -std=c++17

ifeq ($(HAS_CXX17),YES)

# PROD_NAME is used by ADCORE to add libraries
# DBD_PREFIX is used by ADCORE to add dbd files
PROD_NAME = urExample
DBD_PREFIX = iocurExample
PROD_IOC = $(PROD_NAME)

endif

# Note that the order in which .dbd files and libraries are added to a list
# variable is important.  For .dbd files, things must be defined before they
# are referenced by a later .dbd file.  For libraries, things must be referenced
# before they are searched for; otherwise they won't be included.

##################################################################
# First, define $(DBD_NAME)_DBD and $(PROD_NAME)_LIBS from the modules
# defined in RELEASE
##################################################################
DBD_SUFFIX_Linux = Linux
DBD_NAME = $(DBD_PREFIX)$(DBD_SUFFIX_$(OS_CLASS))

DBD  += $(DBD_NAME).dbd

$(DBD_NAME)_DBD += base.dbd

ifdef ADCORE
  -include $(ADCORE)/ADApp/commonDriverMakefile
endif

ifdef ADURL
  $(DBD_NAME)_DBD += URLDriverSupport.dbd
  $(PROD_NAME)_LIBS := URLDriver $($(PROD_NAME)_LIBS)
endif

ifdef ASYN
  $(DBD_NAME)_DBD += asyn.dbd
  $(DBD_NAME)_DBD += drvAsynSerialPort.dbd drvAsynIPPort.dbd drvVxi11.dbd devGpib.dbd
  $(PROD_NAME)_LIBS := asyn $($(PROD_NAME)_LIBS)
endif

ifdef MOTOR
  $(DBD_NAME)_DBD += motorSupport.dbd devSoftMotor.dbd
  $(PROD_NAME)_LIBS := motor softMotor $($(PROD_NAME)_LIBS)
endif

ifdef BUSY
  $(DBD_NAME)_DBD += busySupport.dbd
  $(PROD_NAME)_LIBS := busy $($(PROD_NAME)_LIBS)
endif

ifdef STD
  $(DBD_NAME)_DBD += stdSupport.dbd
  $(PROD_NAME)_LIBS := std $($(PROD_NAME)_LIBS)
endif

ifdef LUA
  $(DBD_NAME)_DBD += luaSupport.dbd
  $(PROD_NAME)_LIBS := lua $($(PROD_NAME)_LIBS)
endif

ifdef CALC
  $(DBD_NAME)_DBD += calcSupport.dbd
  $(PROD_NAME)_LIBS := calc $($(PROD_NAME)_LIBS)
endif

ifdef URROBOT
  $(DBD_NAME)_DBD += urRobotSupport.dbd
  $(PROD_NAME)_LIBS := urRobot $($(PROD_NAME)_LIBS)
endif

$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS  += $(DBD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
