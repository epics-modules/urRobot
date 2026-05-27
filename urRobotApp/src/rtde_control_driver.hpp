/// @file rtde_control_driver.hpp
/// @brief asynPortDriver for UR robot motion control via the ur_rtde library.
///
/// Provides moveJ/moveL commands, waypoint-based motion with optional actions,
/// and monitors async motion progress in a poll thread.

#pragma once
#include "rtde_receive_driver.hpp"
#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/script_client.h"
#include <asynPortDriver.h>
#include <optional>

/// State machine for tracking asynchronous motion progress in the poll thread.
///   Done -> WaitingMotion -> WaitingAction -> Done
/// WaitingAction is only entered when the motion has an associated waypoint action.
enum class AsyncMotionStatus : int { WaitingMotion, WaitingAction, Done };

/// Whether a motion command targets joint space (movej) or Cartesian space (movel).
enum class MotionType : int { Joint, Cartesian };

/// asynPortDriver that wraps ur_rtde::RTDEControlInterface for robot motion.
///
/// A background poll thread monitors async operation progress via
/// getAsyncOperationProgressEx() and drives the AsyncMotionStatus state machine.
/// Safety status bits are checked each poll cycle; active motion is aborted on
/// any safety event.
class RTDEControl : public asynPortDriver {
  public:
    RTDEControl(const char* asyn_port_name, const char* dash_drv_name, const char* recv_drv_name,
                double poll_period);
    asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value) override;
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;
    asynStatus writeOctet(asynUser* pasynUser, const char* value, size_t maxChars, size_t* nActual) override;

    /// Poll thread entry point. Runs forever, checking async motion status
    /// and updating asyn parameters each cycle.
    void poll(void);

  private:
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    std::unique_ptr<ur_rtde::ScriptClient> script_client_;
    RTDEReceive* drv_receive_ = nullptr;
    int safetyStatusBitsParamId_ = -1;
    int motion_done_count_ = 0;

    std::string robot_ip_ = "0.0.0.0";
    std::string dash_drv_name_;
    const double poll_period_ = 0.0; ///< seconds between poll cycles

    /// Check dashboard for RUNNING mode, then connect (or reconnect) RTDE interfaces.
    bool try_connect();

    /// Commanded joint angles (radians)
    std::vector<double> cmd_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /// Commanded end-effector pose: x,y,z (meters), roll,pitch,yaw (radians)
    std::vector<double> cmd_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /// TCP pose offset: x,y,z (meters), roll,pitch,yaw (radians)
    std::vector<double> tcp_offset_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    /// Dynamics for moveJ (all in rad/s, rad/s^2, meters)
    double joint_speed_ = 0.5;
    double joint_accel_ = 1.4;
    double joint_blend_ = 0.0;

    /// Dynamics for moveL (all in m/s, m/s^2, meters)
    double linear_speed_ = 0.05;
    double linear_accel_ = 0.5;
    double linear_blend_ = 0.0;

    /// Custom URScript
    std::string custom_script_path_;
    int custom_script_running_count_ = 0;
    std::chrono::time_point<std::chrono::steady_clock> custom_script_start_time_;
    int outputIntRegId_ = 0;
    bool custom_script_running_ = false;

    /// Tests for completion of the custom script uploaded to the controller.
    /// Upon finishing, reuploads the main RTDE Control script. Note that we
    /// choose to block here until the control script is reuploaded for simplicty.
    /// This typically takes around 100ms.
    /// This will also set the CUSTOM_SCRIPT_ERROR asyn parameter. It is expected
    /// callParamCallbacks will be called after calling poll_custom_script().
    void poll_custom_script();

    /// --- Async motion state machine ---

    AsyncMotionStatus motion_status_ = AsyncMotionStatus::Done;
    bool waypoint_move_ = false; ///< if true, next moveJ/moveL will run waypoint action

    /// A pending motion request, set by writeInt32 and consumed by the poll thread.
    struct MotionTask {
        MotionType type; ///< Joint or Cartesian
        bool action;     ///< true if a waypoint action should run after the move
    };
    std::optional<MotionTask> pending_motion_;

    /// Mark async motion as complete: set status to Done, signal asyncMoveDone=1,
    /// and clear the pending motion task.
    void set_motion_task_done() {
        motion_status_ = AsyncMotionStatus::Done;
        setIntegerParam(asyncMoveDoneIndex_, 1);
        setIntegerParam(motionDoneCountIndex_, ++motion_done_count_);
        pending_motion_.reset();
    }

  protected:
    /// Connection management
    int disconnectIndex_;
    int reconnectIndex_;
    int isConnectedIndex_;
    int isSteadyIndex_;

    /// Joint-space motion
    int moveJIndex_;
    int stopJIndex_;
    int jointCmdIndex_; ///< per-joint commanded angle (addr 0-5)
    int actualQIndex_;  ///< actual joint angles (float64 array)
    int jointSpeedIndex_;
    int jointAccelIndex_;
    int jointBlendIndex_;

    /// Cartesian-space motion
    int moveLIndex_;
    int stopLIndex_;
    int poseCmdIndex_;       ///< per-axis commanded pose (addr 0-5)
    int actualTCPPoseIndex_; ///< actual TCP pose (float64 array)
    int tcpOffsetIndex_;     ///< per-axis TCP offset (addr 0-5)
    int linearSpeedIndex_;
    int linearAccelIndex_;
    int linearBlendIndex_;

    /// Async motion / waypoints
    int asyncMoveDoneIndex_;      ///< 1 when no async motion in progress
    int motionDoneCountIndex_;    ///< Counter for synchronization of path motion
    int waypointMoveIndex_;       ///< if 1, moveJ/moveL will run waypoint action after motion
    int runWaypointActionIndex_;  ///< toggled to trigger waypoint action processing
    int waypointActionDoneIndex_; ///< set to 1 by action chain when action completes

    /// Script / safety
    int reuploadCtrlScriptIndex_;
    int stopCtrlScriptIndex_;
    int teachModeIndex_;
    int triggerProtStopIndex_;
    int customScriptFileIndex_;
    int customInlineScriptIndex_;
    int runCustomScriptFileIndex_;
    int customScriptRunningIndex_;
    int customScriptErrorIndex_;
    int customScriptTimeoutIndex_;
};
