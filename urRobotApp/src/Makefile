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
urRobot_SYS_LIBS += boost_thread

# Tool for calibrating the Robotiq Gripper
PROD += calibrate_gripper
calibrate_gripper_SRCS += calibrate_gripper.cpp
calibrate_gripper_LIBS += rtde
calibrate_gripper_SYS_LIBS += boost_thread
calibrate_gripper_SYS_LIBS += pthread

# Program for testing ur_rtde by itself without asyn
TESTPROD += urtest
urtest_SRCS += urtest.cpp
urtest_LIBS += rtde
urtest_SYS_LIBS += boost_thread
urtest_SYS_LIBS += pthread

endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
