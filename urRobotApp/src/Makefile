# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

USR_INCLUDES += -I../spdlog

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
urRobot_SYS_LIBS += boost_system
urRobot_SYS_LIBS += boost_thread

# Tool for calibrating the Robotiq Gripper
PROD += calibrate_gripper
calibrate_gripper_SRCS += calibrate_gripper.cpp
calibrate_gripper_LIBS += rtde

# Program for testing ur_rtde by itself without asyn
TESTPROD += urtest
urtest_SRCS += urtest.cpp
urtest_LIBS += rtde
urtest_SYS_LIBS += boost_system
urtest_SYS_LIBS += boost_thread

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
