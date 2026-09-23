#include <atomic>
#include <optional>
#include <utility>
#include <array>
#include <exception>
#include <filesystem>
#include <fstream>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <alarm.h>
#include <initHooks.h>
#include <iocsh.h>
#include "rtde_control_driver.hpp"
#include "dashboard_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

// helpers for initHook and debug print wrapper
namespace {
std::atomic<bool> ioc_running{false};

void init_hook_callback(initHookState state) { ioc_running.store(state == initHookAfterIocRunning); }

// debug print wrapper to only print after IOC running
template <typename... Args>
void debug(spdlog::string_view_t fmt, Args&&... args) {
    if (ioc_running.load()) {
        spdlog::debug(fmt, std::forward<Args>(args)...);
    }
}
} // namespace

bool RTDEControl::try_connect() {
    // RTDE class construction automatically tries connecting.
    // If this function hasn't been called or if connecting fails,
    // the rtde_control_ object will be a nullptr
    bool connected = false;
    bool robot_running = false;

    if (drv_receive_) {
        int safety_bits = 1;
        drv_receive_->lock();
        drv_receive_->getIntegerParam(safetyStatusBitsParamId_, &safety_bits);
        drv_receive_->unlock();
        if (safety_bits != 1) {
            spdlog::error("Cannot connect to control interface in current safety state. "
                          "Clear safeguard stop, E-stop, etc. then try again.\n");
            return false;
        }
    } else {
        return false;
    }

    char buffer[128];
    size_t nbytesTransferred;
    int eomReason;
    asynStatus status = pasynOctetSyncIO->readOnce(dash_drv_name_.c_str(), 0, buffer, sizeof(buffer), 1.0,
                                                   &nbytesTransferred, &eomReason, "ROBOT_MODE");
    if (status != asynSuccess) {
        return connected;
    }

    if (strcmp(buffer, "Robotmode: RUNNING") == 0) {
        robot_running = true;
    } else {
        spdlog::error("Unable to connect to control interface in current mode. "
                      "Ensure robot is on, in normal mode, and brakes released, then try again.\n");
        robot_running = false;
        connected = false;
    }

    if (robot_running) {
        if (!rtde_control_) {
            try {
                rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
                if (rtde_control_ && rtde_control_->isConnected()) {
                    spdlog::info("Connected to UR RTDE Control interface");
                    connected = true;
                }
            } catch (const std::exception& e) {
                spdlog::error("Failed to connected to UR RTDE Control interface\n{}", e.what());
                connected = false;
            }
        } else {
            try {
                if (not rtde_control_->isConnected()) {
                    debug("Reconnecting to UR RTDE Control interface");
                    rtde_control_->reconnect();
                    connected = true;
                }
            } catch (const std::exception& e) {
                spdlog::error("Failed to reconnect: {}", e.what());
                connected = false;
            }
        }
    }
    return connected;
}

/// Gets the PolyScope version from the Dashboard driver
std::pair<int, int> get_polyscope_version(URDashboard* dash) {
    int major = 0;
    int minor = 0;

    char version_str[64];
    int id;
    dash->findParam("POLYSCOPE_VERSION", &id);
    dash->lock();
    dash->getStringParam(id, 64, version_str);
    dash->unlock();

    int parsed = std::sscanf(version_str, "%d.%d", &major, &minor);
    if (parsed != 2) {
        throw std::runtime_error("Failed to parse Polyscope version");
    }
    return {major, minor};
}

/// Wraps a URScript in a function and handles integer register to signal completion
std::string wrap_script(const std::string& script) {
    std::string cmd_str;
    std::string line;
    std::stringstream ss(script);
    cmd_str += "def custom_func():\n";
    while (std::getline(ss, line)) {
        cmd_str += "\t" + line + "\n";
    }
    cmd_str += "\twrite_output_integer_register(12, read_output_integer_register(12)+1)\n";
    cmd_str += "end\n";
    return cmd_str;
}

// checks if custom URScript is done or timed out
void RTDEControl::poll_custom_script() {
    int count = 0;
    drv_receive_->lock();
    drv_receive_->getIntegerParam(outputIntRegId_, &count);
    drv_receive_->unlock();

    double script_timeout = 0.0;
    getDoubleParam(customScriptTimeoutIndex_, &script_timeout);

    using namespace std::chrono_literals;
    auto elap = std::chrono::steady_clock::now() - custom_script_start_time_;
    bool script_finished = custom_script_running_count_ != count;
    bool timed_out = (!script_finished && elap >= std::chrono::duration<double>(script_timeout));

    if (script_finished) {
        custom_script_running_ = false;
        setIntegerParam(customScriptRunningIndex_, 0);
        debug("URScript done");
        debug("Reuploading RTDE control script");
        rtde_control_->reuploadScript();
        auto start = std::chrono::steady_clock::now();
        constexpr auto reupload_timeout = 1s;
        while (!rtde_control_->isProgramRunning()) {
            auto elap =
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start);
            if (elap >= reupload_timeout) {
                spdlog::error("Timed out trying to reupload control script");
                break;
            }
            epicsThreadSleep(0.01);
        }
    } else if (timed_out) {
        custom_script_running_ = false;
        setIntegerParam(customScriptRunningIndex_, 0);
        setIntegerParam(customScriptErrorIndex_, 1);
        spdlog::error("URScript timed out: {}", custom_script_path_);
    }
}

static void poll_thread_C(void* pPvt) {
    RTDEControl* pRTDEControl = (RTDEControl*)pPvt;
    pRTDEControl->poll();
}

static void servo_thread_C(void* pPvt) {
    RTDEControl* pRTDEControl = (RTDEControl*)pPvt;
    pRTDEControl->servo_worker();
}

constexpr int NUM_JOINTS = 6;
constexpr int MAX_ADDR = NUM_JOINTS;
constexpr int ASYN_INTERFACE_MASK =
    asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask | asynDrvUserMask;
constexpr int ASYN_INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask;

RTDEControl::RTDEControl(const char* asyn_port_name, const char* dash_drv_name, const char* recv_drv_name,
                         double poll_period, int auto_connect)
    : asynPortDriver(asyn_port_name, MAX_ADDR, ASYN_INTERFACE_MASK, ASYN_INTERRUPT_MASK,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0),
      rtde_control_(nullptr), script_client_(nullptr), dash_drv_name_(dash_drv_name),
      poll_period_(poll_period) {

    createParam("DISCONNECT", asynParamInt32, &disconnectIndex_);
    createParam("RECONNECT", asynParamInt32, &reconnectIndex_);
    createParam("IS_CONNECTED", asynParamInt32, &isConnectedIndex_);
    createParam("IS_STEADY", asynParamInt32, &isSteadyIndex_);
    createParam("SERVOJ_START", asynParamInt32, &servoStartIndex_);
    createParam("SERVOJ_STOP", asynParamInt32, &servoStopIndex_);
    createParam("SERVOJ_STATE", asynParamInt32, &servoStateIndex_);
    createParam("MOVEJ", asynParamInt32, &moveJIndex_);
    createParam("STOPJ", asynParamInt32, &stopJIndex_);
    createParam("ACTUAL_Q", asynParamFloat64Array, &actualQIndex_);
    createParam("JOINT_CMD", asynParamFloat64, &jointCmdIndex_);
    createParam("MOVEL", asynParamInt32, &moveLIndex_);
    createParam("STOPL", asynParamInt32, &stopLIndex_);
    createParam("ACTUAL_TCP_POSE", asynParamFloat64Array, &actualTCPPoseIndex_);
    createParam("POSE_CMD", asynParamFloat64, &poseCmdIndex_);
    createParam("TCP_OFFSET", asynParamFloat64, &tcpOffsetIndex_);
    createParam("REUPLOAD_CONTROL_SCRIPT", asynParamInt32, &reuploadCtrlScriptIndex_);
    createParam("STOP_CONTROL_SCRIPT", asynParamInt32, &stopCtrlScriptIndex_);
    createParam("JOINT_SPEED", asynParamFloat64, &jointSpeedIndex_);
    createParam("JOINT_ACCELERATION", asynParamFloat64, &jointAccelIndex_);
    createParam("JOINT_BLEND", asynParamFloat64, &jointBlendIndex_);
    createParam("LINEAR_SPEED", asynParamFloat64, &linearSpeedIndex_);
    createParam("LINEAR_ACCELERATION", asynParamFloat64, &linearAccelIndex_);
    createParam("LINEAR_BLEND", asynParamFloat64, &linearBlendIndex_);
    createParam("ASYNC_MOVE_DONE", asynParamInt32, &asyncMoveDoneIndex_);
    createParam("WAYPOINT_MOVE", asynParamInt32, &waypointMoveIndex_);
    createParam("RUN_WAYPOINT_ACTION", asynParamInt32, &runWaypointActionIndex_);
    createParam("WAYPOINT_ACTION_DONE", asynParamInt32, &waypointActionDoneIndex_);
    createParam("TEACH_MODE", asynParamInt32, &teachModeIndex_);
    createParam("TRIGGER_PROT_STOP", asynParamInt32, &triggerProtStopIndex_);
    createParam("MOTION_DONE_COUNT", asynParamInt32, &motionDoneCountIndex_);
    createParam("CUSTOM_SCRIPT_FILE", asynParamOctet, &customScriptFileIndex_);
    createParam("CUSTOM_INLINE_SCRIPT", asynParamOctet, &customInlineScriptIndex_);
    createParam("RUN_CUSTOM_SCRIPT_FILE", asynParamInt32, &runCustomScriptFileIndex_);
    createParam("CUSTOM_SCRIPT_RUNNING", asynParamInt32, &customScriptRunningIndex_);
    createParam("CUSTOM_SCRIPT_ERROR", asynParamInt32, &customScriptErrorIndex_);
    createParam("CUSTOM_SCRIPT_TIMEOUT", asynParamFloat64, &customScriptTimeoutIndex_);
    createParam("JOG_START", asynParamInt32, &jogStartIndex_);
    createParam("JOG_STOP", asynParamInt32, &jogStopIndex_);
    createParam("JOG_SPEED", asynParamFloat64, &jogSpeedIndex_);
    createParam("JOG_ACCELERATION", asynParamFloat64, &jogAccelerationIndex_);
    createParam("JOGGING", asynParamInt32, &joggingIndex_);
    createParam("FK_REQUEST", asynParamFloat64Array, &fkRequestIndex_);
    createParam("FK_RESULT", asynParamFloat64Array, &fkResultIndex_);
    createParam("IK_REQUEST", asynParamFloat64Array, &ikRequestIndex_);
    createParam("IK_RESULT", asynParamFloat64Array, &ikResultIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    // Get the controller's IP address from the dashboard driver
    URDashboard* dash = findDerivedAsynPortDriver<URDashboard>(dash_drv_name);
    if (!dash) {
        spdlog::error("Failed to find URDashboard asynPortDriver");
        return;
    }
    robot_ip_ = dash->get_ip();

    // Get the version of Polyscope and connect the script client
    auto [major, minor] = get_polyscope_version(dash);
    script_client_ = std::make_unique<ur_rtde::ScriptClient>(robot_ip_, major, minor);
    script_client_->connect();

    // Save the asyn parameter ID for the safety status bits from receive driver
    drv_receive_ = findDerivedAsynPortDriver<RTDEReceive>(recv_drv_name);
    if (!drv_receive_) {
        spdlog::error("Failed to find RTDEReceive asynPortDriver");
        return;
    }
    drv_receive_->findParam("SAFETY_STATUS_BITS", &safetyStatusBitsParamId_);
    drv_receive_->findParam("OUTPUT_INTEGER_REG12", &outputIntRegId_);

    // Try connecting to the control server on the robot controller
    if (auto_connect) {
        try_connect();
    } else {
        spdlog::info("Deferred connection to UR RTDE Control interface");
    }

    servo_event_ = epicsEventMustCreate(epicsEventEmpty);

    servo_thread_id_ = epicsThreadMustCreate("RTDEControlServo", epicsThreadPriorityMedium,
                                            epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)servo_thread_C,
                                            this);

    epicsThreadCreate("RTDEControlPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void RTDEControl::poll() {
    int run_action_val = 0;

    while (true) {
        lock();

        if (servo_owns_control()) {
            int safety_bits = 1;
            drv_receive_->lock();
            drv_receive_->getIntegerParam(safetyStatusBitsParamId_, &safety_bits);
            drv_receive_->unlock();
            if (safety_bits != 1) {
                servo_should_stop_.store(true, std::memory_order_relaxed);
                epicsEventSignal(servo_event_);
            }
        } else {
            if (rtde_control_ and rtde_control_->isConnected()) {

                setIntegerParam(isConnectedIndex_, 1);
                int is_steady = 0;
                if (!custom_script_running_) {
                    is_steady = rtde_control_->isSteady();
                }
                setIntegerParam(isSteadyIndex_, is_steady);

                int safety_bits = 1;
                drv_receive_->lock();
                drv_receive_->getIntegerParam(safetyStatusBitsParamId_, &safety_bits);
                drv_receive_->unlock();
                if (safety_bits != 1) {
                    if (pending_motion_) {
                        debug("Motion stopped due to safety.");
                        set_motion_task_done();
                    }
                    callParamCallbacks();
                    unlock();
                    epicsThreadSleep(poll_period_);
                    continue;
                }

                if (pending_motion_) {
                    if (motion_status_ == AsyncMotionStatus::Done) {
                        // starting new asynchronous motion
                        if (pending_motion_->type == MotionType::Joint) {
                            rtde_control_->moveJ(cmd_joints_, joint_speed_, joint_accel_, true);
                        } else if (pending_motion_->type == MotionType::Cartesian) {
                            rtde_control_->moveL(cmd_pose_, linear_speed_, linear_accel_, true);
                        }
                        motion_status_ = AsyncMotionStatus::WaitingMotion;
                    } else { // async motion task in progress
                        if (motion_status_ == AsyncMotionStatus::WaitingMotion) {
                            auto op_status = rtde_control_->getAsyncOperationProgressEx();
                            if (!op_status.isAsyncOperationRunning()) {
                                if (pending_motion_->action) {
                                    debug("Waypoint reached. Starting action...");
                                    run_action_val = 1 ^ run_action_val; // ensures action PV processes
                                    setIntegerParam(waypointActionDoneIndex_, 0);
                                    setIntegerParam(runWaypointActionIndex_, run_action_val);
                                    motion_status_ = AsyncMotionStatus::WaitingAction;
                                } else {
                                    debug("Motion complete.");
                                    set_motion_task_done();
                                }
                            }
                        } else if (motion_status_ == AsyncMotionStatus::WaitingAction) {
                            if (custom_script_running_) {
                                poll_custom_script();
                            }
                            int done = 0;
                            getIntegerParam(waypointActionDoneIndex_, &done);
                            if (done) {
                                debug("Waypoint action complete.");
                                set_motion_task_done();
                            }
                        }
                    }
                } else if (custom_script_running_) {
                    poll_custom_script();
                }

            } else {
                setIntegerParam(isConnectedIndex_, 0);
            }
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_period_);
    }
}

void RTDEControl::servo_worker() {
    while (true) {
        epicsEventMustWait(servo_event_);

        lock();
        if (!servo_owns_control()) {
            unlock();
            continue;
        }
        unlock();

        while (!servo_should_stop_.load(std::memory_order_relaxed)) {
            epicsEventMustWait(servo_event_);
        }

        lock();
        servo_state_ = ServoState::Idle;
        setIntegerParam(servoStateIndex_, static_cast<int>(servo_state_));
        unlock();
    }
}

asynStatus RTDEControl::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    int addr = 0;
    getAddress(pasynUser, &addr);

    if (function == tcpOffsetIndex_ && servo_owns_control()) {
        spdlog::warn("TCP offset rejected while servo owns RTDE control interface");
        return asynError;
    }

    if (function == jointCmdIndex_) {
        // convert commanded joint angles to radians
        const double val = value * M_PI / 180.0;
        this->cmd_joints_.at(addr) = val;
    } else if (function == poseCmdIndex_) {
        const double val = (addr >= 3) ? value : (value / 1000.0);
        this->cmd_pose_.at(addr) = val;
    }

    // Dynamics for joint moves (moveJ)
    // convert from deg -> rad
    else if (function == jointSpeedIndex_) {
        this->joint_speed_ = value * M_PI / 180.0;
        debug("Setting joint speed to {:.4f}", joint_speed_);
    } else if (function == jointAccelIndex_) {
        this->joint_accel_ = value * M_PI / 180.0;
        debug("Setting joint acceleration to {:.4f}", joint_accel_);
    } else if (function == jointBlendIndex_) {
        this->joint_blend_ = value / 1000.0;
        debug("Setting joint blend to {:.4f}", joint_blend_);
    }

    // Dynamics for linear moves (moveL)
    // convert from m -> mm
    else if (function == linearSpeedIndex_) {
        this->linear_speed_ = value / 1000.0;
        debug("Setting linear speed to {:.4f}", linear_speed_);
    } else if (function == linearAccelIndex_) {
        this->linear_accel_ = value / 1000.0;
        debug("Setting linear acceleration to {:.4f}", linear_accel_);
    } else if (function == linearBlendIndex_) {
        this->linear_blend_ = value / 1000.0;
        debug("Setting linear blend to {:.4f}", linear_blend_);
    }

    else if (function == jogSpeedIndex_) {
        const double val = (addr >= 3) ? value : (value / 1000.0);
        debug("Setting jog speed[{}] to {:.4f}", addr, val);
        jog_speeds_[addr] = val;
        new_jog_ = true;
    }

    else if (function == jogAccelerationIndex_) {
        new_jog_ = true;
        asynPortDriver::writeFloat64(pasynUser, value);
    }

    else if (function == tcpOffsetIndex_) {
        // convert commanded x,y,z from mm to meters. Assume rx, ry, rz is radians
        const double val = (addr >= 3) ? value : (value / 1000.0);
        this->tcp_offset_.at(addr) = val;
        debug("Setting TCP offset to [{:.4f}] m,rad", fmt::join(tcp_offset_, ","));
        if (rtde_control_ && rtde_control_->isConnected()) {
            rtde_control_->setTcp(this->tcp_offset_);
        }
    }

    else {
        asynPortDriver::writeFloat64(pasynUser, value);
    }

    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        return asynError;
    }
}

asynStatus RTDEControl::writeInt32(asynUser* pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (function == servoStartIndex_) {
        if (servo_state_ != ServoState::Idle || pending_motion_ || custom_script_running_) {
            spdlog::warn("Cannot start servo ownership test in current driver state");
            comm_ok = false;
            goto skip;
        }

        servo_should_stop_.store(false, std::memory_order_relaxed);
        servo_state_ = ServoState::Active;
        setIntegerParam(servoStateIndex_, static_cast<int>(servo_state_));
        epicsEventSignal(servo_event_);
        goto skip;
    }

    if (function == servoStopIndex_) {
        if (servo_owns_control()) {
            servo_should_stop_.store(true, std::memory_order_relaxed);
            epicsEventSignal(servo_event_);
        }
        goto skip;
    }

    if (servo_owns_control()) {
        spdlog::warn("Command rejected while servo owns RTDE control interface");
        comm_ok = false;
        goto skip;
    }

    if (function == reconnectIndex_) {
        comm_ok = try_connect();
        if (comm_ok) {
            // User could have set TCP offset with driver
            // disconnected, so we send it again here
            rtde_control_->setTcp(tcp_offset_);
        }
        goto skip;
    }

    if (!rtde_control_) {
        spdlog::error("RTDE Control interface not initialized");
        comm_ok = false;
        goto skip;
    }
    if (function == disconnectIndex_) {
        debug("Disconnecting from RTDE control interface");
        rtde_control_->disconnect();
        comm_ok = not rtde_control_->isConnected();
        goto skip;
    }

    // Check that it's connected before conntinuing
    if (!rtde_control_->isConnected()) {
        spdlog::error("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }

    if (function == moveJIndex_) {
        if (!pending_motion_) {
            debug("moveJ({:.4f}) rad", fmt::join(cmd_joints_, ","));
            if (rtde_control_->isJointsWithinSafetyLimits(cmd_joints_)) {
                pending_motion_ = MotionTask{MotionType::Joint, waypoint_move_};
                waypoint_move_ = false;
                setIntegerParam(asyncMoveDoneIndex_, 0);
                setIntegerParam(moveJIndex_, 1);
            } else {
                spdlog::warn("Requested joint angles not within safety limits. No action taken.");
                setIntegerParam(moveJIndex_, 0);
            }
        } else {
            spdlog::warn("Motion already in progress...please wait");
            setIntegerParam(moveJIndex_, 0);
        }
    }

    else if (function == moveLIndex_) {
        if (!pending_motion_) {
            debug("moveL({:.4f}) m,rad", fmt::join(cmd_pose_, ","));
            if (rtde_control_->isPoseWithinSafetyLimits(cmd_pose_)) {
                pending_motion_ = MotionTask{MotionType::Cartesian, waypoint_move_};
                waypoint_move_ = false;
                setIntegerParam(asyncMoveDoneIndex_, 0);
                setIntegerParam(moveLIndex_, 1);
            } else {
                spdlog::warn("Requested TCP pose not within safety limits. No action taken.");
                setIntegerParam(moveLIndex_, 0);
            }
        } else {
            spdlog::warn("Motion already in progress...please wait");
            setIntegerParam(moveLIndex_, 0);
        }
    }

    else if (function == waypointMoveIndex_) {
        waypoint_move_ = static_cast<bool>(value);
    }

    else if (function == stopJIndex_) {
        set_motion_task_done();
        debug("Stopping (linear in joint space)");
        rtde_control_->stopJ();
    }

    else if (function == stopLIndex_) {
        set_motion_task_done();
        debug("Stopping (linear in tool space)");
        rtde_control_->stopL();
    }

    else if (function == waypointActionDoneIndex_) {
        setIntegerParam(waypointActionDoneIndex_, value);
    }

    else if (function == reuploadCtrlScriptIndex_) {
        debug("Reuploading control script");
        try {
            rtde_control_->reuploadScript();
        } catch (const std::exception& e) {
            spdlog::error("Failed to reupload control script: {}", e.what());
        }
    }

    else if (function == stopCtrlScriptIndex_) {
        debug("Stopping control script");
        try {
            rtde_control_->stopScript();
        } catch (const std::exception& e) {
            spdlog::error("Failed to stop control script: {}", e.what());
        }
    }

    else if (function == triggerProtStopIndex_) {
        debug("Triggering protective stop");
        rtde_control_->triggerProtectiveStop();
    }

    else if (function == teachModeIndex_) {
        if (value) {
            debug("Enabling teach mode");
            rtde_control_->teachMode();
        } else {
            debug("Disabling teach mode");
            rtde_control_->endTeachMode();
        }
    }

    else if (function == runCustomScriptFileIndex_) {
        if (pending_motion_ && motion_status_ != AsyncMotionStatus::WaitingAction) {
            spdlog::warn("Motion task in progress. Cannot run script.");
            goto skip;
        }

        if (!script_client_->isConnected()) {
            spdlog::error("ScriptClient not connected!");
            goto skip;
        }

        std::ifstream fs(custom_script_path_);
        if (!fs.is_open()) {
            goto skip;
        }

        // Read entire file into a string
        std::string script_str =
            std::string(std::istreambuf_iterator<char>(fs), std::istreambuf_iterator<char>());
        debug("Running custom URScript: {}", custom_script_path_);
        setIntegerParam(customScriptErrorIndex_, 0);
        rtde_control_->stopScript();
        script_client_->sendScriptCommand(wrap_script(script_str));
        custom_script_running_ = true;
        custom_script_start_time_ = std::chrono::steady_clock::now();
        setIntegerParam(customScriptRunningIndex_, 1);
        drv_receive_->lock();
        drv_receive_->getIntegerParam(outputIntRegId_, &custom_script_running_count_);
        drv_receive_->unlock();
    }

    else if (function == jogStartIndex_) {
        if (new_jog_) {
            double accel = 0.0;
            getDoubleParam(jogAccelerationIndex_, &accel);
            rtde_control_->speedL(jog_speeds_, accel, 0.01);
            debug("Starting jog: accel={}, speeds=[{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}]", accel,
                  jog_speeds_[0], jog_speeds_[1], jog_speeds_[2], jog_speeds_[3], jog_speeds_[4],
                  jog_speeds_[5]);
            new_jog_ = false;
        }
        setIntegerParam(joggingIndex_, 1);
    }

    else if (function == jogStopIndex_) {
        debug("Stopping jog");
        rtde_control_->speedStop();
        setIntegerParam(joggingIndex_, 0);
    }

    else {
        asynPortDriver::writeInt32(pasynUser, value);
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        debug("RTDE communication error in RTDEControl::writeInt32");
        return asynError;
    }
}

asynStatus RTDEControl::writeOctet(asynUser* pasynUser, const char* value, size_t maxChars, size_t* nActual) {
    int function = pasynUser->reason;
    bool comm_ok = true;

    if (servo_owns_control()) {
        spdlog::warn("String command rejected while servo owns RTDE control interface");
        comm_ok = false;
        goto skip;
    }

    if (!rtde_control_) {
        spdlog::error("RTDE Control interface not initialized");
        comm_ok = false;
        goto skip;
    }

    if (not rtde_control_->isConnected()) {
        spdlog::error("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }

    if (function == customScriptFileIndex_) {
        // Set the path, and read it every time in callback for runCustomScriptFileIndex_
        if (std::filesystem::exists(value)) {
            custom_script_path_ = value;
            debug("Successfully read URScript file: {}", value);
            setIntegerParam(customScriptErrorIndex_, 0);
        } else {
            spdlog::error("Failed to read URScript file: {}", value);
            setIntegerParam(customScriptErrorIndex_, 1);
        }
    }

    else if (function == customInlineScriptIndex_) {
        if (pending_motion_ && motion_status_ != AsyncMotionStatus::WaitingAction) {
            spdlog::warn("Motion task in progress. Cannot run script.");
            goto skip;
        }

        if (!script_client_->isConnected()) {
            spdlog::error("ScriptClient not connected!");
            goto skip;
        }

        debug("Running inline URScript: {}", value);
        setIntegerParam(customScriptErrorIndex_, 0);
        rtde_control_->stopScript();
        script_client_->sendScriptCommand(wrap_script(value));
        custom_script_running_ = true;
        custom_script_start_time_ = std::chrono::steady_clock::now();
        setIntegerParam(customScriptRunningIndex_, 1);
        drv_receive_->lock();
        drv_receive_->getIntegerParam(outputIntRegId_, &custom_script_running_count_);
        drv_receive_->unlock();
    }

skip:
    *nActual = strlen(value);
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        debug("RTDE communication error in RTDEControl::writeOctet");
        return asynError;
    }
}

asynStatus RTDEControl::writeFloat64Array(asynUser* pasynUser, epicsFloat64* value, size_t nElements) {
    int function = pasynUser->reason;

    if (servo_owns_control()) {
        spdlog::warn("Array command rejected while servo owns RTDE control interface");
        return asynError;
    }

    if (!rtde_control_ || !rtde_control_->isConnected()) {
        spdlog::error("RTDE Control interface not initialized or disconnected");
        return asynError;
    }

    auto set_alarm = [&](int index){
        setParamAlarmStatus(index, epicsAlarmCalc);
        setParamAlarmSeverity(index, epicsSevMajor);
    };

    auto clear_alarm = [&](int index){
        setParamAlarmStatus(index, epicsAlarmNone);
        setParamAlarmSeverity(index, epicsSevNone);
    };

    if (function == fkRequestIndex_) {
        std::vector<double> pose(6, 0.0);
        if (nElements != 6) {
            set_alarm(fkResultIndex_);
            std::array<double, 1> no_data{};
            doCallbacksFloat64Array(no_data.data(), 0, fkResultIndex_, 0);
            return asynError;
        }

        std::vector<double> joints(value, value + nElements);
        for (auto& j : joints) {
            j *= M_PI / 180.0; // convert to rad
        }
        pose = rtde_control_->getForwardKinematics(joints, rtde_control_->getTCPOffset());
        for (size_t i = 0; i < 3; i++) {
            pose[i] *= 1000; // convert m -> mm
        }
        clear_alarm(fkResultIndex_);
        doCallbacksFloat64Array(pose.data(), pose.size(), fkResultIndex_, 0);
    }

    else if (function == ikRequestIndex_) {
        std::vector<double> joints(6, 0.0);
        if (nElements != 6) {
            set_alarm(ikResultIndex_);
            std::array<double, 1> no_data{};
            doCallbacksFloat64Array(no_data.data(), 0, ikResultIndex_, 0);
            return asynError;
        }

        std::vector<double> pose(value, value + nElements);
        for (size_t i = 0; i < 3; i++) {
            pose[i] /= 1000; // convert mm -> m
        }

        if (rtde_control_->getInverseKinematicsHasSolution(pose)) {
            joints = rtde_control_->getInverseKinematics(pose);
            for (auto& j : joints) {
                j *= 180.0 / M_PI; // convert rad -> deg
            }
            clear_alarm(ikResultIndex_);
            doCallbacksFloat64Array(joints.data(), joints.size(), ikResultIndex_, 0);
        } else {
            set_alarm(ikResultIndex_);
            std::array<double, 1> no_data{};
            doCallbacksFloat64Array(no_data.data(), 0, ikResultIndex_, 0);
        }
    }

    return asynSuccess;
}

// register function for iocsh
extern "C" int RTDEControlConfig(const char* asyn_port_name, const char* dash_drv_name,
                                 const char* recv_drv_name, double poll_period, int auto_connect) {
    new RTDEControl(asyn_port_name, dash_drv_name, recv_drv_name, poll_period, auto_connect);
    return asynSuccess;
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Dashboard driver name", iocshArgString};
static const iocshArg urRobotArg2 = {"Receive driver name", iocshArgString};
static const iocshArg urRobotArg3 = {"Poll period", iocshArgDouble};
static const iocshArg urRobotArg4 = {"Auto connect", iocshArgInt};
static const iocshArg* const urRobotArgs[5] = {&urRobotArg0, &urRobotArg1, &urRobotArg2, &urRobotArg3,
                                               &urRobotArg4};
static const iocshFuncDef urRobotFuncDef = {"RTDEControlConfig", 5, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf* args) {
    RTDEControlConfig(args[0].sval, args[1].sval, args[2].sval, args[3].dval, args[4].ival);
}

void RTDEControlRegister(void) {
    initHookRegister(init_hook_callback);
    iocshRegister(&urRobotFuncDef, urRobotCallFunc);
}

extern "C" {
epicsExportRegistrar(RTDEControlRegister);
}
