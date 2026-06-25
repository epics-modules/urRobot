# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

USR_INCLUDES += -I../spdlog/include

# urRobot requires C++17 (std::optional)
HAS_CXX17 := $(shell $(CXX) -std=c++17 -fsyntax-only -x c++ /dev/null 2>/dev/null && echo YES || echo NO)

ifeq ($(HAS_CXX17),YES)
USR_CXXFLAGS += -std=c++17

# Define IOC library name
LIBRARY_IOC += urRobot

# install urRobotSupport.dbd into <top>/dbd
DBD += urRobotSupport.dbd
urRobotSupport_DBD += urRobot.dbd

# source files to be compiled and added to the library
urRobot_SRCS += dashboard_driver.cpp
urRobot_SRCS += rtde_receive_driver.cpp
urRobot_SRCS += rtde_io_driver.cpp
urRobot_SRCS += rtde_control_driver.cpp
urRobot_SRCS += gripper_driver.cpp

# Libraries needed for urRobot
urRobot_LIBS += asyn
urRobot_LIBS += rtde
urRobot_LIBS += $(EPICS_BASE_IOC_LIBS)

# Boost wiring (see WITH_BOOST / BOOST_* in configure/CONFIG_SITE).
# When BOOST_LIB is set (e.g. a PSPKG package), link Boost from there via the
# *_DIR mechanism; otherwise fall back to the system Boost libraries. The
# resulting names are applied to the urRobot library and to each PROD below.
ifeq ($(WITH_BOOST),YES)
  USR_INCLUDES += -I$(BOOST_INCLUDE)
  ifneq ($(BOOST_EXTERNAL),NO)
    ifdef BOOST_LIB
      boost_system_DIR = $(BOOST_LIB)
      boost_thread_DIR = $(BOOST_LIB)
      BOOST_TGT_LIBS = boost_system boost_thread
    endif
  endif
  ifndef BOOST_TGT_LIBS
    BOOST_TGT_SYS_LIBS = boost_system boost_thread
  endif
endif

urRobot_LIBS     += $(BOOST_TGT_LIBS)
urRobot_SYS_LIBS += $(BOOST_TGT_SYS_LIBS)

# Tool for calibrating the Robotiq Gripper
PROD += calibrate_gripper
calibrate_gripper_SRCS += calibrate_gripper.cpp
calibrate_gripper_LIBS += rtde
calibrate_gripper_LIBS += $(BOOST_TGT_LIBS)
calibrate_gripper_SYS_LIBS += $(BOOST_TGT_SYS_LIBS)
calibrate_gripper_SYS_LIBS += pthread

# Program for testing ur_rtde by itself without asyn
TESTPROD += urtest
urtest_SRCS += urtest.cpp
urtest_LIBS += rtde
urtest_LIBS += $(BOOST_TGT_LIBS)
urtest_SYS_LIBS += $(BOOST_TGT_SYS_LIBS)
urtest_SYS_LIBS += pthread

endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
