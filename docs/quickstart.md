---
layout: default
title: Quickstart
nav_order: 2
---

# Quickstart

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}

## Build Instructions

**Supported Operating Systems**

- Linux (x86-64) - tested on Red Hat Enterprise Linux (RHEL) 9. Other distributions may
work but have not been tested


**Library Dependencies**

- EPICS Base
- asyn
- [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) (included)

This library has been tested with synApps 6-2-1 (EPICS base 7.0.4.1, asyn R4-42), and synApps 6-3
(EPICS Base 7.0.8, asyn R4-44-2). Other versions may work but have not been sufficiently tested.

The `ur_rtde` library comes pre-built for the supported operating systems.
See `urRobotApp/src/ur_rtde/release.txt` for the current included version of the `ur_rtde` library.

If you need to build the support for another operating system, build the `ur_rtde` library for your OS and replace the `ur_rtde` library files and headers in urRobotApp/src.
Consult the `ur_rtde` library documentation on how to build it from source.

**Build**

1. Clone or download [https://github.com/epics-modules/urRobot](https://github.com/epics-modules/urRobot)
2. Edit configure/RELEASE to correct the paths to `EPICS_BASE` and `ASYN`
3. `make`

## Adding `urRobot` support to an IOC
*See iocs/urExample for a complete example*

**IOC Dependencies**

IOCs using `urRobot` depend on these additional EPICS modules for full functionality:
- asyn
- calc
- lua (optional) for waypoints/paths
- std (optional) for waypoints/paths 
- busy (optional) for waypoints/paths
- motor (optional) for soft motor support


**1\.** Add the path to the urRobot support in configure/RELEASE

```
# file: configure/RELEASE
URROBOT=/path/to/urRobot
```

**2\.** Add rules in xxxApp/src/Makefile

```
# file: xxxApp/src/Makefile
ifdef URROBOT
  $(DBD_NAME)_DBD += urRobotSupport.dbd
  $(PROD_NAME)_LIBS := urRobot $($(PROD_NAME)_LIBS)
endif
```

**3\.** In your startup script, load the example script making sure to fill in the correct IP address for your robot.

```
# file: iocBoot/iocxxx/st.cmd.Linux
iocshLoad("$(URROBOT)/urRobotApp/iocsh/urRobot.iocsh", "PREFIX=$(PREFIX), IP=127.0.0.1")
```

**4\.** If you would like to add support for waypoints and paths, load `paths.iocsh`. Note the additional dependencies for waypoint
and path functionality (lua, std, busy).

```
# file: iocBoot/iocxxx/st.cmd.Linux
iocshLoad("$(URROBOT)/urRobotApp/iocsh/paths.iocsh", "PREFIX=$(PREFIX)")
```

The number of waypoints, actions, and paths loaded can be controlled with optional macros:
`N_WP` (default 30), `N_ACTIONS` (default 10), `N_PATH` (default 10), `N_PATH_WP` (default 30).

For advanced use, you can copy `urRobot.iocsh` to your IOC and load only the specific interfaces
you need. Note that the RTDE Control interface depends on both the Dashboard and RTDE Receive
interfaces being active, and the Gripper interface depends on the Dashboard interface.
The Dashboard, Receive, and I/O interfaces can be loaded independently.

**5\.** Before starting the IOC, using the teach pendant, make sure the robot is powered on, brakes released, in Automatic mode,
and in Remote Control mode. Note, if you are not using the RTDE control interface, all you need to do is make sure the robot
controller is powered on.

At this point you should be able to start the IOC. If all went well,
your IOC console should report messages (beginning with "[info]") saying each interface to the robot was successfully connected:
```bash
# Load robot support with waypoint and path support
iocshLoad("urRobot.iocsh", "PREFIX=bcur:, IP=164.54.104.148")
# Set up UR Dashboard server
URDashboardConfig("dash", "164.54.104.148", "0.1")
[2026-03-05 12:46:05.750] [info] Connected to UR Dashboard server
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/dashboard.db", "P=bcur:, PORT=dash")
# Set up UR RTDE Receive interface
RTDEReceiveConfig("rtde_recv", "164.54.104.148", "0.02")
[2026-03-05 12:46:06.331] [info] Connected to UR RTDE Receive interface
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/rtde_receive.db", "P=bcur:, PORT=rtde_recv")
# Set up UR RTDE I/O interface
RTDEInOutConfig("rtde_io", "164.54.104.148", "0.1")
[2026-03-05 12:46:07.134] [info] Connected to UR RTDE IO interface
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/rtde_io.db", "P=bcur:, PORT=rtde_io")
# Set up UR RTDE Control interface
RTDEControlConfig("rtde_ctrl", "dash", "rtde_recv", "0.02")
[2026-03-05 12:46:08.405] [info] Connected to UR RTDE Control interface
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/rtde_control.db", "P=bcur:, PORT=rtde_ctrl")
# Set up Robotiq Gripper
URGripperConfig("gripper", "dash", "0.02")
[2026-03-05 12:46:09.012] [info] Connected to gripper
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/robotiq_gripper.db", "P=bcur:, MIN_POS=3, MAX_POS=248, AUTO_ACTIVATE=YES, PORT=gripper")
```

If something went wrong, you will see messages in the console (beginning with "[error]") which should explain the issue. The
most common problems are an incorrect IP address or the robot not being powered on and brakes released (if using RTDE control
interface).

**6\.** To start the provided GUIs, use the example `urRobot/iocs/urExample/start_urRobot_gui` script.
You may need to adjust the paths to the display manager executables in this script.
