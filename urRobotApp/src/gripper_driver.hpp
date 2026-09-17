/// @file gripper_driver.hpp
/// @brief asynPortDriver for Robotiq Hand-E gripper control via the ur_rtde library.
///
/// Provides open/close commands, speed and force settings, position readback,
/// auto-calibration, and configurable position units (device, normalized,
/// percent, mm). A background poll thread monitors gripper state including
/// object detection status. Requires the robot to be powered on (checks
/// robot mode via a Dashboard Server connection).

#pragma once
#include "dashboard_driver.hpp"
#include "ur_rtde/robotiq_gripper.h"
#include <asynPortDriver.h>

/// asynPortDriver that wraps ur_rtde::RobotiqGripper for Hand-E gripper control.
///
/// A background poll thread reads gripper state (position, open/closed,
/// object detection) each cycle. This driver requires the URDashboard
/// driver to exist already and holds a pointer to the URDashboard which is
/// used to check that the robot is on before gripper operations.
class URGripper : public asynPortDriver {
  public:
    URGripper(const char* asyn_port_name, const char* dash_drv_name, double poll_period);
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;

    /// Poll thread entry point. Runs forever, reading gripper status
    /// and updating asyn parameters each cycle.
    void poll(void);

  private:
    URDashboard* drv_dashboard_; ///< Pointer to dashboard driver which must exist already
    std::unique_ptr<ur_rtde::RobotiqGripper> gripper_;
    const double poll_period_ = 0.0; ///< seconds between poll cycles

    /// Parameter indices for asyn params in dashboard driver
    int robotModeParamId_ = -1;
    int robotConnectedParamId_ = -1;

    /// Returns true if dashboard is connected and robot is in normal mode
    bool robot_ready() const;

    /// Verify robot is powered on, then connect to the gripper.
    bool try_connect();

  protected:
    /// asyn parameter indices — each maps to a named parameter string registered
    /// in the constructor via createParam(). See gripper.db for the
    /// corresponding EPICS records.

    /// Connection and activation
    int isConnectedIndex_; ///< 1 when gripper connection is active
    int connectIndex_;     ///< trigger gripper connect
    int isActiveIndex_;    ///< 1 when gripper is activated
    int activateIndex_;    ///< trigger gripper activation

    /// Motion commands
    int openIndex_;     ///< open the gripper
    int closeIndex_;    ///< close the gripper
    int setPosIndex_;   ///< set gripper target position
    int moveIndex_;     ///< move gripper to target position
    int setSpeedIndex_; ///< gripper speed (percent)
    int setForceIndex_; ///< gripper force (percent)

    /// Object detection status
    int isOpenIndex_;         ///< 1 when gripper is fully open
    int isClosedIndex_;       ///< 1 when gripper is fully closed
    int isStoppedInnerIndex_; ///< 1 when stopped on inner object
    int isStoppedOuterIndex_; ///< 1 when stopped on outer object
    int moveStatusIndex_;     ///< raw eObjectStatus enum value

    /// Position read-back and calibration
    int openPositionIndex_;    ///< calibrated open position
    int closedPositionIndex_;  ///< calibrated closed position
    int currentPositionIndex_; ///< current gripper position
    int autoCalibrateIndex_;   ///< trigger auto-calibration
    int isCalibratedIndex_;    ///< 1 after successful calibration

    /// Position range and units
    int minPositionIndex_;      ///< min position for native range
    int maxPositionIndex_;      ///< max position for native range
    int setPositionRangeIndex_; ///< apply min/max as native position range
    int unitIndex_;     ///< position unit: 0=device, 1=normalized, 2=percent, 3=mm
};
