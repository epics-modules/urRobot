TOP = ../..
include $(TOP)/configure/CONFIG

SRC_DIRS += ../ur_rtde/src
SRC_DIRS += ../ur_rtde/src/urcl
SRC_DIRS += ../ur_rtde/include
SRC_DIRS += ../generated

USR_INCLUDES += -I../ur_rtde/include
USR_INCLUDES += -I../generated

USR_CXXFLAGS += -std=c++11

LIBRARY = rtde

rtde_SRCS += robot_state.cpp
rtde_SRCS += rtde.cpp
rtde_SRCS += dashboard_client.cpp
rtde_SRCS += dashboard_enums.cpp
rtde_SRCS += script_client.cpp
rtde_SRCS += rtde_control_interface.cpp
rtde_SRCS += rtde_receive_interface.cpp
rtde_SRCS += rtde_io_interface.cpp
rtde_SRCS += robotiq_gripper.cpp

rtde_SRCS_Linux  += script_sender.cpp
rtde_SRCS_Linux  += tcp_server.cpp
rtde_SRCS_Linux  += log.cpp
rtde_SRCS_Linux  += default_log_handler.cpp
rtde_SRCS_Darwin += script_sender.cpp
rtde_SRCS_Darwin += tcp_server.cpp
rtde_SRCS_Darwin += log.cpp
rtde_SRCS_Darwin += default_log_handler.cpp

rtde_SYS_LIBS += boost_system
rtde_SYS_LIBS += boost_thread

INC += ur_rtde/dashboard_client.h
INC += ur_rtde/dashboard_enums.h
INC += ur_rtde/robot_state.h
INC += ur_rtde/robotiq_gripper.h
INC += ur_rtde/rtde.h
INC += ur_rtde/rtde_control_interface.h
INC += ur_rtde/rtde_control_interface_doc.h
INC += ur_rtde/rtde_control_script.h
INC += ur_rtde/rtde_export.h
INC += ur_rtde/rtde_io_interface.h
INC += ur_rtde/rtde_io_interface_doc.h
INC += ur_rtde/rtde_receive_interface.h
INC += ur_rtde/rtde_receive_interface_doc.h
INC += ur_rtde/rtde_utility.h
INC += ur_rtde/script_client.h
INC += urcl/default_log_handler.h
INC += urcl/log.h
INC += urcl/script_sender.h
INC += urcl/tcp_server.h
INC += urcl/tcp_socket.h

include $(TOP)/configure/RULES
