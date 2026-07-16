/// @file rtde_receive_driver.hpp
/// @brief Read-only asynPortDriver for monitoring UR robot state via ur_rtde.
///
/// Polls the robot's RTDE Receive interface each cycle and publishes joint
/// positions/velocities/currents, TCP pose/speed/force, safety status,
/// temperatures, voltages, and I/O state as asyn parameters. Unit conversions
/// (rad→deg, m→mm) are applied in the poll thread before publishing.

#pragma once
#include "ur_rtde/rtde_receive_interface.h"
#include <asynPortDriver.h>

/// Read-only asynPortDriver that wraps ur_rtde::RTDEReceiveInterface.
///
/// A background poll thread reads all monitored values from the robot each
/// cycle and updates the corresponding asyn parameters via setParam /
/// doCallbacksArray calls. No write operations are performed on the robot;
/// writeInt32 only handles connect/disconnect/reconnect.
class RTDEReceive : public asynPortDriver {
  public:
    RTDEReceive(const char* asyn_port_name, const char* robot_port_name, double poll_period);
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;

    /// Poll thread entry point. Reads all robot state each cycle and
    /// updates asyn parameters.
    void poll(void);

  private:
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    const std::string robot_ip_ = "";
    const double poll_period_ = 0.0; ///< seconds between poll cycles

    /// Connect (or reconnect) to the RTDE Receive interface.
    bool try_connect();

  protected:
    /// asyn parameter indices — each maps to a named parameter string registered
    /// in the constructor via createParam(). See rtde_receive.db for the
    /// corresponding EPICS records.

    /// Connection management
    int disconnectIndex_;
    int reconnectIndex_;
    int isConnectedIndex_;

    /// Robot state (scalars)
    int safetyStatusBitsIndex_; ///< bitmask; value of 1 = normal operation
    int runtimeStateIndex_;
    int robotModeIndex_;
    int safetyModeIndex_;
    int controllerTimestampIndex_;
    int outputIntegerReg12Index_;

    /// Analog I/O (scalars)
    int stdAnalogInput0Index_;
    int stdAnalogInput1Index_;
    int stdAnalogOutput0Index_;
    int stdAnalogOutput1Index_;

    /// Digital I/O (bitmasks)
    int digitalInputBitsIndex_;
    int digitalOutputBitsIndex_;

    /// Actual joint state (float64 arrays, 6 elements each)
    int actualJointPosArrIndex_; ///< degrees (converted from radians)
    int actualJointPosIndex_;
    int actualJointVelIndex_;
    int actualJointCurrentsIndex_;
    int actualJointVoltagesIndex_;
    int jointControlCurrentsIndex_;
    int jointTemperaturesIndex_;
    int jointModesIndex_; ///< int32 array

    /// Target joint state (float64 arrays, 6 elements each)
    int targetJointPosIndex_;
    int targetJointVelIndex_;
    int targetJointAccelIndex_;
    int targetJointCurrentsIndex_;
    int targetJointMomentsIndex_;

    /// Actual TCP state (float64 arrays; pose is 6 elements, accel is 3)
    int actualTCPPoseArrIndex_; ///< x,y,z in mm, rx,ry,rz in rad
    int actualTCPPoseIndex_;
    int actualTCPSpeedIndex_;
    int actualTCPForceIndex_;
    int actualToolAccelIndex_; ///< 3 elements (accelerometer)

    /// Target TCP state (float64 arrays, 6 elements each)
    int targetTCPPoseIndex_;
    int targetTCPSpeedIndex_;

    /// Speed / power scalars
    int speedScalingIndex_;
    int targetSpeedFractionIndex_;
    int actualMomentumIndex_;
    int actualMainVoltageIndex_;
    int actualRobotVoltageIndex_;
    int actualRobotCurrentIndex_;
};
