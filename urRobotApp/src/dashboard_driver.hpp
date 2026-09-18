/// @file dashboard_driver.hpp
/// @brief asynPortDriver for UR robot management via the Dashboard Server.
///
/// Provides power control, brake release, program loading/execution, popup
/// management, safety recovery, and read-back of robot mode, safety status,
/// and program state. A background poll thread periodically queries the
/// Dashboard Server to keep status parameters up to date.

#pragma once
#include "ur_rtde/dashboard_client.h"
#include <asynPortDriver.h>

constexpr int MAX_CONTROLLERS = 1;

/// asynPortDriver that wraps ur_rtde::DashboardClient for robot management.
///
/// A background poll thread reads connection state, running status, program
/// state, robot mode, safety status, and remote-control mode each cycle.
/// Write operations handle power on/off, brake release, program play/stop/pause,
/// URP loading, popup display/dismiss, and safety recovery commands.
class URDashboard : public asynPortDriver {
  public:
    URDashboard(const char* asyn_port_name, const char* robot_port_name, double poll_period);
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;
    asynStatus writeOctet(asynUser* pasynUser, const char* value, size_t maxChars, size_t* nActual) override;

    /// Poll thread entry point. Runs forever, reading dashboard status
    /// and updating asyn parameters each cycle.
    void poll(void);

    /// Returns the IP address of the robot
    std::string get_ip() { return robot_ip_; }

  private:
    std::unique_ptr<ur_rtde::DashboardClient> ur_dashboard_;
    double poll_period_; ///< seconds between poll cycles
    std::string robot_ip_; ///< robot's IP address useful for other drivers
    bool eseries_ = true;

    /// Connect (or reconnect) to the Dashboard Server on the robot.
    bool try_connect();

  protected:
    /// asyn parameter indices — each maps to a named parameter string registered
    /// in the constructor via createParam(). See dashboard.db for the
    /// corresponding EPICS records.

    /// Connection management
    int isConnectedIndex_; ///< 1 when dashboard connection is active
    int connectIndex_;     ///< trigger dashboard connect
    int disconnectIndex_;  ///< trigger dashboard disconnect
    int shutdownIndex_;    ///< shut down robot and controller

    /// Popup control
    int closePopupIndex_;       ///< dismiss current popup
    int popupIndex_;            ///< display a popup with given text (octet)
    int closeSafetyPopupIndex_; ///< dismiss safety popup

    /// Program control
    int loadURPIndex_; ///< load a .urp program by path (octet)
    int playIndex_;    ///< start loaded program
    int stopIndex_;    ///< stop running program
    int pauseIndex_;   ///< pause running program

    /// Power and safety
    int powerOnIndex_;              ///< power on the robot
    int powerOffIndex_;             ///< power off the robot
    int brakeReleaseIndex_;         ///< release brakes
    int unlockProtectiveStopIndex_; ///< unlock a protective stop
    int restartSafetyIndex_;        ///< restart safety configuration

    /// Readback status (updated by poll thread)
    int isRunningIndex_;         ///< 1 when a program is running
    int polyscopeVersionIndex_;  ///< PolyScope version string (octet)
    int serialNumberIndex_;      ///< robot serial number (octet)
    int robotModeIndex_;         ///< current robot mode string (octet)
    int programStateIndex_;      ///< current program state string (octet)
    int robotModelIndex_;        ///< robot model string (octet)
    int loadedProgramIndex_;     ///< path of currently loaded program (octet)
    int safetyStatusIndex_;      ///< current safety status string (octet)
    int isProgramSavedIndex_;    ///< 1 when loaded program is saved
    int isInRemoteControlIndex_; ///< 1 when robot is in remote control mode
};
