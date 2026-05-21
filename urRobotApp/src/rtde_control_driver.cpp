#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iocsh.h>
#include <optional>

#include "dashboard_driver.hpp"
#include "rtde_control_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"
#include <asynOctetSyncIO.h>

bool RTDEControl::try_connect() {
    // RTDE class construction automatically tries connecting.
    // If this function hasn't been called or if connecting fails,
    // the rtde_control_ object will be a nullptr
    bool connected = false;
    bool robot_running = false;

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
        spdlog::error("Unable to connect to UR RTDE Control Interface: "
                      "Ensure robot is on, in normal mode, and brakes released");
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
                    spdlog::debug("Reconnecting to UR RTDE Control interface");
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
        spdlog::debug("URScript done: {}", custom_script_path_);
        spdlog::debug("Reuploading RTDE control script");
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

constexpr int NUM_JOINTS = 6;
constexpr int MAX_ADDR = NUM_JOINTS;
constexpr int ASYN_INTERFACE_MASK =
    asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask | asynDrvUserMask;
constexpr int ASYN_INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask;

RTDEControl::RTDEControl(const char* asyn_port_name, const char* dash_drv_name, const char* recv_drv_name,
                         double poll_period)
    : asynPortDriver(asyn_port_name, MAX_ADDR, ASYN_INTERFACE_MASK, ASYN_INTERRUPT_MASK,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0),
      rtde_control_(nullptr), script_client_(nullptr), dash_drv_name_(dash_drv_name),
      poll_period_(poll_period) {

    createParam("DISCONNECT", asynParamInt32, &disconnectIndex_);
    createParam("RECONNECT", asynParamInt32, &reconnectIndex_);
    createParam("IS_CONNECTED", asynParamInt32, &isConnectedIndex_);
    createParam("IS_STEADY", asynParamInt32, &isSteadyIndex_);
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
    createParam("CUSTOM_SCRIPT_PATH", asynParamOctet, &customScriptFileIndex_);
    createParam("RUN_CUSTOM_SCRIPT", asynParamInt32, &runCustomScriptIndex_);
    createParam("CUSTOM_SCRIPT_RUNNING", asynParamInt32, &customScriptRunningIndex_);
    createParam("CUSTOM_SCRIPT_ERROR", asynParamInt32, &customScriptErrorIndex_);
    createParam("CUSTOM_SCRIPT_TIMEOUT", asynParamFloat64, &customScriptTimeoutIndex_);

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
    try_connect();

    epicsThreadCreate("RTDEControlPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void RTDEControl::poll() {
    int run_action_val = 0;

    while (true) {
        lock();

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
                    spdlog::debug("Motion stopped due to safety.");
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
                                spdlog::debug("Waypoint reached. Starting action...");
                                run_action_val = 1 ^ run_action_val; // ensures action PV processes
                                setIntegerParam(waypointActionDoneIndex_, 0);
                                setIntegerParam(runWaypointActionIndex_, run_action_val);
                                motion_status_ = AsyncMotionStatus::WaitingAction;
                            } else {
                                spdlog::debug("Motion complete.");
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
                            spdlog::debug("Waypoint action complete.");
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

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_period_);
    }
}

asynStatus RTDEControl::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    int addr = 0;
    getAddress(pasynUser, &addr);

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

    if (function == jointCmdIndex_) {
        // convert commanded joint angles to radians
        const double val = value * M_PI / 180.0;
        this->cmd_joints_.at(addr) = val;
    } else if (function == poseCmdIndex_) {
        // convert commanded x,y,z to meters and roll, pitch, yaw to radians
        const double val = (addr >= 3) ? (value * M_PI / 180.0) : (value / 1000.0);
        this->cmd_pose_.at(addr) = val;
    }

    else if (function == tcpOffsetIndex_) {
        // convert commanded x,y,z from mm to meters. Assume roll, pitch, yaw is radians
        const double val = (addr >= 3) ? value : (value / 1000.0);
        this->tcp_offset_.at(addr) = val;
        spdlog::debug("Setting TCP offset to [{:.4f}] m,rad", fmt::join(tcp_offset_, ","));
        rtde_control_->setTcp(this->tcp_offset_);
    }

    // Dynamics for joint moves (moveJ)
    // convert from deg -> rad
    else if (function == jointSpeedIndex_) {
        this->joint_speed_ = value * M_PI / 180.0;
        spdlog::debug("Setting joint speed to {:.4f}", joint_speed_);
    } else if (function == jointAccelIndex_) {
        this->joint_accel_ = value * M_PI / 180.0;
        spdlog::debug("Setting joint acceleration to {:.4f}", joint_accel_);
    } else if (function == jointBlendIndex_) {
        this->joint_blend_ = value / 1000.0;
        spdlog::debug("Setting joint blend to {:.4f}", joint_blend_);
    }

    // Dynamics for linear moves (moveL)
    // convert from m -> mm
    else if (function == linearSpeedIndex_) {
        this->linear_speed_ = value / 1000.0;
        spdlog::debug("Setting linear speed to {:.4f}", linear_speed_);
    } else if (function == linearAccelIndex_) {
        this->linear_accel_ = value / 1000.0;
        spdlog::debug("Setting linear acceleration to {:.4f}", linear_accel_);
    } else if (function == linearBlendIndex_) {
        this->linear_blend_ = value / 1000.0;
        spdlog::debug("Setting linear blend to {:.4f}", linear_blend_);
    }

    else {
        asynPortDriver::writeFloat64(pasynUser, value);
    }

skip:
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

    if (function == reconnectIndex_) {
        comm_ok = try_connect();
        goto skip;
    }

    if (!rtde_control_) {
        spdlog::error("RTDE Control interface not initialized");
        comm_ok = false;
        goto skip;
    }
    if (function == disconnectIndex_) {
        spdlog::debug("Disconnecting from RTDE control interface");
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
            spdlog::debug("moveJ({:.4f}) rad", fmt::join(cmd_joints_, ","));
            if (rtde_control_->isJointsWithinSafetyLimits(cmd_joints_)) {
                pending_motion_ = MotionTask{MotionType::Joint, waypoint_move_};
                waypoint_move_ = false;
                setIntegerParam(asyncMoveDoneIndex_, 0);
            } else {
                spdlog::warn("Requested joint angles not within safety limits. No action taken.");
            }
        } else {
            spdlog::warn("Motion already in progress...please wait");
        }
    }

    else if (function == moveLIndex_) {
        if (!pending_motion_) {
            spdlog::debug("moveL({:.4f}) m,rad", fmt::join(cmd_pose_, ","));
            if (rtde_control_->isPoseWithinSafetyLimits(cmd_pose_)) {
                pending_motion_ = MotionTask{MotionType::Cartesian, waypoint_move_};
                waypoint_move_ = false;
                setIntegerParam(asyncMoveDoneIndex_, 0);
            } else {
                spdlog::warn("Requested TCP pose not within safety limits. No action taken.");
            }
        } else {
            spdlog::warn("Motion already in progress...please wait");
        }
    }

    else if (function == waypointMoveIndex_) {
        waypoint_move_ = static_cast<bool>(value);
    }

    else if (function == stopJIndex_) {
        set_motion_task_done();
        spdlog::debug("Stopping (linear in joint space)");
        rtde_control_->stopJ();
    }

    else if (function == stopLIndex_) {
        set_motion_task_done();
        spdlog::debug("Stopping (linear in tool space)");
        rtde_control_->stopL();
    }

    else if (function == waypointActionDoneIndex_) {
        setIntegerParam(waypointActionDoneIndex_, value);
    }

    else if (function == reuploadCtrlScriptIndex_) {
        spdlog::debug("Reuploading control script");
        try {
            rtde_control_->reuploadScript();
        } catch (const std::exception& e) {
            spdlog::error("Failed to reupload control script: {}", e.what());
        }
    }

    else if (function == stopCtrlScriptIndex_) {
        spdlog::debug("Stopping control script");
        try {
            rtde_control_->stopScript();
        } catch (const std::exception& e) {
            spdlog::error("Failed to stop control script: {}", e.what());
        }
    }

    else if (function == triggerProtStopIndex_) {
        spdlog::debug("Triggering protective stop");
        rtde_control_->triggerProtectiveStop();
    }

    else if (function == teachModeIndex_) {
        if (value) {
            spdlog::debug("Enabling teach mode");
            rtde_control_->teachMode();
        } else {
            spdlog::debug("Disabling teach mode");
            rtde_control_->endTeachMode();
        }
    }

    else if (function == runCustomScriptIndex_) {
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
        spdlog::debug("Running custom URScript: {}", custom_script_path_);
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

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("RTDE communication error in RTDEControl::writeInt32");
        return asynError;
    }
}

asynStatus RTDEControl::writeOctet(asynUser* pasynUser, const char* value, size_t maxChars, size_t* nActual) {
    int function = pasynUser->reason;
    bool comm_ok = true;

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
        // Set the path, and read it every time in runCustomScriptIndex_.
        if (std::filesystem::exists(value)) {
            custom_script_path_ = value;
            spdlog::debug("Successfully read URScript file: {}", value);
            setIntegerParam(customScriptErrorIndex_, 0);
        } else {
            spdlog::error("Failed to read URScript file: {}", value);
            setIntegerParam(customScriptErrorIndex_, 1);
        }
    }

skip:
    *nActual = strlen(value);
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("RTDE communication error in RTDEControl::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int RTDEControlConfig(const char* asyn_port_name, const char* dash_drv_name,
                                 const char* recv_drv_name, double poll_period) {
    new RTDEControl(asyn_port_name, dash_drv_name, recv_drv_name, poll_period);
    return asynSuccess;
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Dashboard driver name", iocshArgString};
static const iocshArg urRobotArg2 = {"Receive driver name", iocshArgString};
static const iocshArg urRobotArg3 = {"Poll period", iocshArgDouble};
static const iocshArg* const urRobotArgs[4] = {&urRobotArg0, &urRobotArg1, &urRobotArg2, &urRobotArg3};
static const iocshFuncDef urRobotFuncDef = {"RTDEControlConfig", 4, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf* args) {
    RTDEControlConfig(args[0].sval, args[1].sval, args[2].sval, args[3].dval);
}

void RTDEControlRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEControlRegister);
}
