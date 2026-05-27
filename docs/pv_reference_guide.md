---
layout: default
title: PV Reference Guide
nav_order: 4
---

# PV Reference Guide

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}


This page describes the available process variables included in the databases provided with the urRobot EPICS support module.
Note that the PVs below will typically have a `$(P)` macro in their name for the IOC prefix
which is provided when loading the EPICS databases. e.g. `dbLoadRecords(dashboard.db, "P=MyIOC:")`,
which would result is PVs like "MyIOC:Dashboard:Connected"

Also note that many of the binary output records below are "triggers" and the VAL/RVAL fields do not matter. The associated
functions will be called whenever the record processes. For example, to play the currently loaded program in the
robot controller, `caput Dashboard:Play 1` and `caput Dashboard:Play.PROC 1` (and even `caput Dashboard:Play 0`) will work.

## dashboard.db

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Dashboard:Connected    | bi     | 1 if connected to the dashboard client, otherwise 0     |
| Dashboard:IsProgramSaved    | bi     |  1 if the currently loaded program is saved, otherwise 0    |
| Dashboard:IsInRemoteControl    | bi     | 1 if the controller is in remote control mode, otherwise 0     |
| Dashboard:Running    | bi     | 1 if the currently loaded program is running, otherwise 0    |
| Dashboard:PolyscopeVersion    | stringin     | Polyscope version   |
| Dashboard:SerialNumber    | stringin     | Serial number     |
| Dashboard:ProgramState    | stringin     | Program state     |
| Dashboard:RobotMode    | stringin     | Robot mode     |
| Dashboard:RobotModel    | stringin     | Robot model     |
| Dashboard:LoadedProgram    | stringin     | Current loaded program in the robot controller     |
| Dashboard:SafetyStatus    | stringin     | Safety status     |

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Dashboard:Play    | bo     | Plays the currently loaded program    |
| Dashboard:Stop    | bo     | Stops the running program     |
| Dashboard:Pause    | bo     | Pause running program     |
| Dashboard:Connect    | bo     | Connects to the dashboard server     |
| Dashboard:Disconnect    | bo     | Closes connection to the dashboard server     |
| Dashboard:Shutdown    | bo     | Shuts down robot and controller     |
| Dashboard:ClosePopup    | bo     | Closes popup on the teach pendant     |
| Dashboard:CloseSafetyPopup    | bo     | Closes safety popup on the teach pendant     |
| Dashboard:PowerOn    | bo     | Powers on the robot     |
| Dashboard:PowerOff    | bo     | Powers off the robot     |
| Dashboard:BrakeRelease    | bo     | Releases brakes     |
| Dashboard:UnlockProtectiveStop    | bo     | Unlocks protective stop     |
| Dashboard:RestartSafety    | bo     | Restarts safety, power off robot     |
| Dashboard:Popup    | stringout     | Generates a popup message on the teach pendant with the provided string     |
| Dashboard:LoadURP    | stringout     | Loads a URP program saved in the robot controller, e.g. `caput Dashboard:LoadURP my_program.urp` loads the program "/programs/my_program.urp" from the robot controller if the program exists |

***


## rtde_receive.db

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Receive:Connected    | bi     | 1 if connected to RTDE Receive interface, otherwise 0     |
| Receive:ControllerTimestamp    | ai     | Time since controller started     |
| Receive:SafetyStatusBits    | ai     | Bits 0-10: Is normal mode, Is reduced mode, Is protective stopped, Is recovery mode, Is safeguard stopped, Is system emergency stopped, Is robot emergency stopped, Is emergency stopped, Is violation, Is fault, Is stopped due to safety      |
| Receive:DigitalInputBits    | ai     | Digital input bits (18bits)     |
| Receive:DigitalOutputBits    | ai     | Digital output bits (18bits)     |
| Receive:RuntimeState    | ai     | Bits 0-5: Stopping, Stopped, Playing, Pausing, Paused, Resuming     |
| Receive:RobotMode    | ai     | -1=NO CONTROLLER, 0=DISCONNECTED, 1=CONFIRM SAFETY, 2=BOOTING, 3=POWER OFF, 4=POWER ON, 5=IDLE, 6=BACKDRIVE, 7=RUNNING, 8=UPDATING FIRMWARE      |
| Receive:SafetyMode    | ai     | Safety mode     |
| Receive:AnalogInput0    | ai     | Standard A0 input     |
| Receive:AnalogInput1    | ai     | Standard A1 input     |
| Receive:AnalogOutput0    | ai     | Standard A0 output     |
| Receive:AnalogOutput1    | ai     | Standard A1 output     |
| Receive:SpeedScaling    | ai     | Speed scaling     |
| Receive:TargetSpeedFraction    | ai     | Target speed between 0 and 1 corresponding to a value between 0% and 100% of the maximum speed     |
| Receive:ActualMomentum    | ai     | Norm of Cartesian linear momentum     |
| Receive:ActualMainVoltage    | ai     | Safety control board main voltage     |
| Receive:ActualRobotVoltage    | ai     | Safety control board robot voltage     |
| Receive:ActualRobotCurrent    | ai     | Safety control board robot current     |
| Receive:ActualJointPositions    | waveform     | Actual joint positions     |
| Receive:ActualJointVelocities    | waveform     | Actual joint velocities     |
| Receive:ActualJointCurrents    | waveform     | Actual joint currents     |
| Receive:JointControlCurrents    | waveform     | Joint control currents     |
| Receive:ActualTCPPose    | waveform     | Actual TCP pose (x,y,z,r,p,y)     |
| Receive:ActualTCPSpeed    | waveform     | Actual TCP speed     |
| Receive:ActualTCPForce    | waveform     | Actual TCP force     |
| Receive:Joint1    | ai     | Joint 1 position (deg)  |
| Receive:Joint2    | ai     | Joint 2 position (deg)  |
| Receive:Joint3    | ai     | Joint 3 position (deg)  |
| Receive:Joint4    | ai     | Joint 4 position (deg)  |
| Receive:Joint5    | ai     | Joint 5 position (deg)  |
| Receive:Joint6    | ai     | Joint 6 position (deg)  |
| Receive:PoseX    | ai     | TCP X position (mm)  |
| Receive:PoseY    | ai     | TCP Y position (mm)  |
| Receive:PoseZ    | ai     | TCP Z position (mm)  |
| Receive:PoseRoll    | ai     | TCP roll (deg)  |
| Receive:PosePitch    | ai     | TCP pitch (deg)  |
| Receive:PoseYaw    | ai     | TCP yaw (deg)  |
| Receive:JointModes    | waveform     | Joint control modes     |
| Receive:ActualToolAccelerometer    | waveform     | Tool accelerometer (X,Y,Z)     |
| Receive:TargetJointPositions    | waveform     | Target joint positions     |
| Receive:TargetJointVelocities    | waveform     | Target joint speeds     |
| Receive:TargetJointAccelerations    | waveform     | Target joint accelerations     |
| Receive:TargetJointCurrents    | waveform     | Target joint currents     |
| Receive:TargetJointMoments    | waveform     | Target joint moments     |
| Receive:TargetTCPPose    | waveform     | Target TCP pose (x,y,z,r,p,y)     |
| Receive:TargetTCPSpeed    | waveform     | Target TCP speed     |
| Receive:JointTemperatures    | waveform     | Joint temperatures in celsius     |
| Receive:ActualJointVoltages    | waveform     | Actual joint voltages     |

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Receive:Disconnect    | bo     | Disconnects from the RTDE interface     |
| Receive:Reconnect    | bo     | Tries reconnecting to the RTDE interface     |


***

## rtde_control.db

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Control:Connected    | bi     | RTDE Control connection status     |
| Control:Steady    | bi     | 1 if robot is fully at rest     |
| Control:AsyncMoveDone   | bi     | 1 if async motion and waypoint action are complete, else 0 |
| Control:Moving    | bi     | 1 if robot is in motion, else 0     |
| Control:CustomScriptRunning    | bi     | 1 while a custom URScript is executing     |
| Control:CustomScriptError    | bi     | 1 if the last custom script timed out or failed     |

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Control:Disconnect    | bo     | Disconnects from the RTDE interface     |
| Control:Reconnect    | bo     | Tries reconnecting to the RTDE interface     |
| Control:ReuploadControlScript    | bo     | Reuploads control script to controller     |
| Control:StopControlScript    | bo     | Stops the control script     |
| Control:TeachMode   | bo     | Enables (1) / disables (0) teach (freedrive) mode |
| Control:TriggerProtectiveStop   | bo     | Triggers a protective stop |
| Control:Stop    | dfanout     | Stops any in-progress motion (moveJ or moveL) and aborts any running path |
| Control:moveJ    | bo     | Executes moveJ to the commanded joint angles J1Cmd–J6Cmd     |
| Control:JointSpeed    | ao     | Speed for moveJ (deg/s)     |
| Control:JointAcceleration    | ao     | Acceleration for moveJ (deg/s/s)     |
| Control:JointBlend    | ao     | Blend radius for moveJ (mm)     |
| Control:stopJ    | bo     | Stops an in-progress asynchronous moveJ     |
| Control:AutoMoveJ    | bo     | If 1, moveJ runs automatically when any JxCmd changes     |
| Control:J1Cmd    | ao     | Commanded angle for joint 1 (deg)     |
| Control:J1TweakVal    | ao     | Joint 1 tweak step size     |
| Control:J1TweakFwd    | bo     | Tweak joint 1 forward by J1TweakVal     |
| Control:J1TweakRev    | bo     | Tweak joint 1 backward by J1TweakVal     |
| Control:J2Cmd    | ao     | Commanded angle for joint 2 (deg)     |
| Control:J2TweakVal    | ao     | Joint 2 tweak step size     |
| Control:J2TweakFwd    | bo     | Tweak joint 2 forward by J2TweakVal     |
| Control:J2TweakRev    | bo     | Tweak joint 2 backward by J2TweakVal     |
| Control:J3Cmd    | ao     | Commanded angle for joint 3 (deg)     |
| Control:J3TweakVal    | ao     | Joint 3 tweak step size     |
| Control:J3TweakFwd    | bo     | Tweak joint 3 forward by J3TweakVal     |
| Control:J3TweakRev    | bo     | Tweak joint 3 backward by J3TweakVal     |
| Control:J4Cmd    | ao     | Commanded angle for joint 4 (deg)     |
| Control:J4TweakVal    | ao     | Joint 4 tweak step size     |
| Control:J4TweakFwd    | bo     | Tweak joint 4 forward by J4TweakVal     |
| Control:J4TweakRev    | bo     | Tweak joint 4 backward by J4TweakVal     |
| Control:J5Cmd    | ao     | Commanded angle for joint 5 (deg)     |
| Control:J5TweakVal    | ao     | Joint 5 tweak step size     |
| Control:J5TweakFwd    | bo     | Tweak joint 5 forward by J5TweakVal     |
| Control:J5TweakRev    | bo     | Tweak joint 5 backward by J5TweakVal     |
| Control:J6Cmd    | ao     | Commanded angle for joint 6 (deg)     |
| Control:J6TweakVal    | ao     | Joint 6 tweak step size     |
| Control:J6TweakFwd    | bo     | Tweak joint 6 forward by J6TweakVal     |
| Control:J6TweakRev    | bo     | Tweak joint 6 backward by J6TweakVal     |
| Control:moveL    | bo     | Executes moveL to the commanded TCP pose     |
| Control:LinearSpeed    | ao     | Speed for moveL (mm/s)     |
| Control:LinearAcceleration    | ao     | Acceleration for moveL (mm/s/s)     |
| Control:LinearBlend    | ao     | Blend radius for moveL (mm)     |
| Control:stopL    | bo     | Stops an in-progress asynchronous moveL     |
| Control:AutoMoveL    | bo     | If 1, moveL runs automatically when any PoseCmd changes     |
| Control:PoseXCmd    | ao     | Commanded TCP X (mm)     |
| Control:PoseXTweakVal    | ao     | X tweak step size     |
| Control:PoseXTweakFwd    | bo     | Tweak TCP X forward     |
| Control:PoseXTweakRev    | bo     | Tweak TCP X backward     |
| Control:PoseYCmd    | ao     | Commanded TCP Y (mm)     |
| Control:PoseYTweakVal    | ao     | Y tweak step size     |
| Control:PoseYTweakFwd    | bo     | Tweak TCP Y forward     |
| Control:PoseYTweakRev    | bo     | Tweak TCP Y backward     |
| Control:PoseZCmd    | ao     | Commanded TCP Z (mm)     |
| Control:PoseZTweakVal    | ao     | Z tweak step size     |
| Control:PoseZTweakFwd    | bo     | Tweak TCP Z forward     |
| Control:PoseZTweakRev    | bo     | Tweak TCP Z backward     |
| Control:PoseRollCmd    | ao     | Commanded TCP roll (deg)     |
| Control:PoseRollTweakVal    | ao     | Roll tweak step size     |
| Control:PoseRollTweakFwd    | bo     | Tweak TCP roll forward     |
| Control:PoseRollTweakRev    | bo     | Tweak TCP roll backward     |
| Control:PosePitchCmd    | ao     | Commanded TCP pitch (deg)     |
| Control:PosePitchTweakVal    | ao     | Pitch tweak step size     |
| Control:PosePitchTweakFwd    | bo     | Tweak TCP pitch forward     |
| Control:PosePitchTweakRev    | bo     | Tweak TCP pitch backward     |
| Control:PoseYawCmd    | ao     | Commanded TCP yaw (deg)     |
| Control:PoseYawTweakVal    | ao     | Yaw tweak step size     |
| Control:PoseYawTweakFwd    | bo     | Tweak TCP yaw forward     |
| Control:PoseYawTweakRev    | bo     | Tweak TCP yaw backward     |
| Control:TCPOffset_X    | ao     | TCP offset X (mm)     |
| Control:TCPOffset_Y    | ao     | TCP offset Y (mm)     |
| Control:TCPOffset_Z    | ao     | TCP offset Z (mm)     |
| Control:TCPOffset_Roll    | ao     | TCP offset roll (rad)     |
| Control:TCPOffset_Pitch    | ao     | TCP offset pitch (rad)     |
| Control:TCPOffset_Yaw    | ao     | TCP offset yaw (rad)     |
| Control:CustomScriptFile    | lso     | Path to a URScript file to run on the controller     |
| Control:CustomInlineScript    | lso     | Inline URScript string; executes immediately on write     |
| Control:RunCustomScriptFile    | bo     | Uploads and runs the script file set in CustomScriptFile     |
| Control:CustomScriptTimeout    | ao     | Timeout in seconds for custom script completion (default 10)     |

***

## rtde_io.db

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| IO:SpeedSlider    | ao     | Number between 0 and 1 to set the speed between 0% and 100% of the maximum speed     |
| IO:SetVoltageAO0    | ao     | Analog output voltage 0     |
| IO:SetVoltageAO1    | ao     | Analog output voltage 1     |
| IO:SetCurrentAO0    | ao     | Analog output current 0     |
| IO:SetCurrentAO1    | ao     | Analog output current 1     |
| IO:SetStandardDO0    | bo     | Standard digital output 0     |
| IO:SetStandardDO1    | bo     | Standard digital output 1     |
| IO:SetStandardDO2    | bo     | Standard digital output 2     |
| IO:SetStandardDO3    | bo     | Standard digital output 3     |
| IO:SetStandardDO4    | bo     | Standard digital output 4     |
| IO:SetStandardDO5    | bo     | Standard digital output 5     |
| IO:SetStandardDO6    | bo     | Standard digital output 6     |
| IO:SetStandardDO7    | bo     | Standard digital output 7     |
| IO:SetConfigurableDO0    | bo     | Configurable digital output 0     |
| IO:SetConfigurableDO1    | bo     | Configurable digital output 1     |
| IO:SetConfigurableDO2    | bo     | Configurable digital output 2     |
| IO:SetConfigurableDO3    | bo     | Configurable digital output 3     |
| IO:SetConfigurableDO4    | bo     | Configurable digital output 4     |
| IO:SetConfigurableDO5    | bo     | Configurable digital output 5     |
| IO:SetConfigurableDO6    | bo     | Configurable digital output 6     |
| IO:SetConfigurableDO7    | bo     | Configurable digital output 7     |
| IO:SetToolDO0    | bo     | Tool digital output 0     |
| IO:SetToolDO1    | bo     | Tool digital output 1     |

***

## robotiq_gripper.db

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| RobotiqGripper:Connected    | bi     | Connection status     |
| RobotiqGripper:Calibrated    | bi     | Calibration status     |
| RobotiqGripper:IsActive    | bi     | Gripper active status     |
| RobotiqGripper:IsOpen    | bi     | 1 if gripper open     |
| RobotiqGripper:IsClosed    | bi     | 1 if gripper closed     |
| RobotiqGripper:IsStoppedInner    | bi     | 1 if stopped on inner object     |
| RobotiqGripper:IsStoppedOuter    | bi     | 1 if stopped on outer object     |
| RobotiqGripper:CurrentPosition    | ai     | Current position of gripper  |
| RobotiqGripper:OpenPosition    | ai     | Open position of gripper     |
| RobotiqGripper:ClosedPosition    | ai     | Closed position of gripper     |
| RobotiqGripper:MoveStatus    | mbbi     | Enum strings for gripper motion status |
| RobotiqGripper:MoveStatusRaw    | longin     | Integer representing the gripper motion status |

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| RobotiqGripper:Connect    | bo     | Connects to Robotiq gripper     |
| RobotiqGripper:Activate    | bo     | Activates Robotiq gripper     |
| RobotiqGripper:AutoCalibrate    | bo     | Calibrates open/close positions     |
| RobotiqGripper:Open    | bo     | Opens Robotiq gripper     |
| RobotiqGripper:Close    | bo     | Closes Robotiq gripper     |
| RobotiqGripper:SetPositionRange    | bo     | Sets min/max positions     |
| RobotiqGripper:SetSpeed    | ao     | Default gripper speed     |
| RobotiqGripper:SetForce    | ao     | Default gripper force     |
| RobotiqGripper:PositionUnit    | mbbo     | Enum strings for position units  |
| RobotiqGripper:MinPosition    | longout     | Minimum gripper position     |
| RobotiqGripper:MaxPosition    | longout     | Maximum gripper position     |
| RobotiqGripper:PositionUnitRaw    | longout     | Integer representing position units     |

***

## waypointJ.db

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| WaypointJ:$(N)    | stringout     | Waypoint name     |
| WaypointJ:$(N):Enabled    | bo     | 1 if waypoint is enabled, otherwise 0     |
| WaypointJ:$(N):J1    | ao     | Joint 1 angle for waypoint (deg)    |
| WaypointJ:$(N):J2    | ao     | Joint 2 angle for waypoint (deg)    |
| WaypointJ:$(N):J3    | ao     | Joint 3 angle for waypoint (deg)    |
| WaypointJ:$(N):J4    | ao     | Joint 4 angle for waypoint (deg)    |
| WaypointJ:$(N):J5    | ao     | Joint 5 angle for waypoint (deg)    |
| WaypointJ:$(N):J6    | ao     | Joint 6 angle for waypoint (deg)    |
| WaypointJ:$(N):ActionOpt    | longout     | Selects the waypoint action (0 = none, N = ActionSseqN) |
| WaypointJ:$(N):Speed    | ao     | Speed when moving to waypoint (deg/s)     |
| WaypointJ:$(N):Acceleration    | ao     | Acceleration when moving to waypoint (deg/s/s)     |
| WaypointJ:$(N):Blend    | ao     | Blend radius when moving to waypoint (mm)     |
| WaypointJ:$(N):Reset    | seq     | Sets waypoint to the current robot configuration     |
| WaypointJ:$(N):moveJ    | bo     | Executes a moveJ to the waypoint if the waypoint is enabled    |

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| WaypointJ:$(N):Reached    | luascript     | 1 when robot is at waypoint, otherwise 0     |
| WaypointJ:$(N):Busy    | busy     | 1 when robot is en-route to waypoint, otherwise 0   |
| WaypointJ:$(N):ActionDesc    | stringin     | Description of the selected action     |

***

## waypointL.db

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| WaypointL:$(N)    | stringout     | Waypoint name     |
| WaypointL:$(N):Enabled    | bo     | 1 if waypoint is enabled, otherwise 0     |
| WaypointL:$(N):X    | ao     | X position of waypoint (mm)     |
| WaypointL:$(N):Y    | ao     | Y position of waypoint (mm)     |
| WaypointL:$(N):Z    | ao     | Z position of waypoint (mm)     |
| WaypointL:$(N):Roll    | ao     | Roll angle of waypoint (deg)    |
| WaypointL:$(N):Pitch    | ao     | Pitch angle of waypoint (deg)   |
| WaypointL:$(N):Yaw    | ao     | Yaw angle of waypoint (deg)    |
| WaypointL:$(N):ActionOpt    | longout     | Selects the waypoint action (0 = none, N = ActionSseqN) |
| WaypointL:$(N):Speed    | ao     | Speed when moving to waypoint (mm/s)     |
| WaypointL:$(N):Acceleration    | ao     | Acceleration when moving to waypoint (mm/s/s)     |
| WaypointL:$(N):Blend    | ao     | Blend radius when moving to waypoint (mm)     |
| WaypointL:$(N):Reset    | seq     | Sets waypoint to the current robot configuration     |
| WaypointL:$(N):moveL    | bo     | Executes a moveL to the waypoint if the waypoint is enabled    |

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| WaypointL:$(N):Reached    | luascript     | 1 when robot is at waypoint, otherwise 0     |
| WaypointL:$(N):Busy    | busy     | 1 when robot is en-route to waypoint, otherwise 0   |
| WaypointL:$(N):ActionDesc    | stringin     | Description of the selected action     |

***

## path.db

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Path$(N)    | stringout     | Description of path     |
| Path$(N):Go    | luascript     | Executes path     |

***

## path_waypoint.db

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Path$(N):$(K):Type    | mbbo     | Waypoint type: Linear (waypointL) or Joint (waypointJ)     |
| Path$(N):$(K):Number    | longout     | Waypoint number     |
| Path$(N):$(K):ActionOverride    | longout     | Overrides the waypoint's action for this path point (0 = use waypoint's action)     |
| Path$(N):$(K):Enabled    | bo     | 1 if this path point is enabled, else 0     |

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Path$(N):$(K):Reached    | ai     | 1 if robot is at this waypoint, else 0     |
| Path$(N):$(K):Busy    | ai     | 1 if robot is moving to this waypoint, else 0     |
| Path$(N):$(K):Desc    | stringin     | Description of the selected waypoint     |
| Path$(N):$(K):ActionDesc    | stringout     | Description of the selected waypoint action     |

***

## waypoint_action.db

Each instance of `waypoint_action.db` defines one waypoint action identified by `$(N)`.

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| ActionSseq$(N)    | sseq     | Sequence of steps the action performs. Processed automatically when the robot reaches a waypoint with this action selected.     |
| ActionDoneCalc$(N)    | calcout     | Signals when the action is complete. Configure so that the output is 1 when done and 0 while running.     |
