#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "gripper_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

bool URGripper::robot_ready() const {
    int connected = 0;
    int on = 0;
    if (drv_dashboard_) {
        drv_dashboard_->lock();
        drv_dashboard_->getIntegerParam(robotConnectedParamId_, &connected);
        std::string robot_mode;
        drv_dashboard_->getStringParam(robotModeParamId_, robot_mode);
        if (robot_mode == "Robotmode: IDLE" or robot_mode == "Robotmode: RUNNING") {
            on = true;
        }
        drv_dashboard_->unlock();
    }
    return on && connected;
}

bool URGripper::try_connect() {
    bool connected = false;
    try {
        if (robot_ready()) {
            gripper_->connect();
            gripper_->getVar("STA");
            if (gripper_->isConnected()) {
                spdlog::info("Connected to gripper");
                connected = true;
            } else {
                spdlog::info("not connected");
                connected = false;
            }
        } else {
            spdlog::error("Unable to connect to gripper. Robot must be powered on.");
            connected = false;
        }
    } catch (const std::exception& e) {
        spdlog::error(e.what());
    }
    return connected;
}

static void poll_thread_C(void* pPvt) {
    URGripper* pGripper = (URGripper*)pPvt;
    pGripper->poll();
}

constexpr int MAX_ADDR = 1;
constexpr int ASYN_INTERFACE_MASK = asynInt32Mask | asynFloat64Mask | asynDrvUserMask;
constexpr int ASYN_INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask;

URGripper::URGripper(const char* asyn_port_name, const char* dash_drv_name, double poll_period)
    : asynPortDriver(asyn_port_name, MAX_ADDR, ASYN_INTERFACE_MASK, ASYN_INTERRUPT_MASK,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0),
      drv_dashboard_(findDerivedAsynPortDriver<URDashboard>(dash_drv_name)), poll_period_(poll_period) {

    createParam("CONNECT", asynParamInt32, &connectIndex_);
    createParam("IS_CONNECTED", asynParamInt32, &isConnectedIndex_);
    createParam("IS_OPEN", asynParamInt32, &isOpenIndex_);
    createParam("IS_CLOSED", asynParamInt32, &isClosedIndex_);
    createParam("IS_STOPPED_INNER", asynParamInt32, &isStoppedInnerIndex_);
    createParam("IS_STOPPED_OUTER", asynParamInt32, &isStoppedOuterIndex_);
    createParam("IS_ACTIVE", asynParamInt32, &isActiveIndex_);
    createParam("ACTIVATE", asynParamInt32, &activateIndex_);
    createParam("OPEN", asynParamInt32, &openIndex_);
    createParam("CLOSE", asynParamInt32, &closeIndex_);
    createParam("SET_SPEED", asynParamFloat64, &setSpeedIndex_);
    createParam("SET_FORCE", asynParamFloat64, &setForceIndex_);
    createParam("AUTO_CALIBRATE", asynParamInt32, &autoCalibrateIndex_);
    createParam("OPEN_POSITION", asynParamFloat64, &openPositionIndex_);
    createParam("CLOSED_POSITION", asynParamFloat64, &closedPositionIndex_);
    createParam("CURRENT_POSITION", asynParamFloat64, &currentPositionIndex_);
    createParam("MOVE_STATUS", asynParamInt32, &moveStatusIndex_);
    createParam("SET_POSITION_RANGE", asynParamInt32, &setPositionRangeIndex_);
    createParam("MIN_POSITION", asynParamInt32, &minPositionIndex_);
    createParam("MAX_POSITION", asynParamInt32, &maxPositionIndex_);
    createParam("POSITION_UNIT", asynParamInt32, &positionUnitIndex_);
    createParam("IS_CALIBRATED", asynParamInt32, &isCalibratedIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    if (!drv_dashboard_) {
        spdlog::error("Failed to find URDashboard asynPortDriver");
        return;
    }
    drv_dashboard_->findParam("ROBOT_MODE", &robotModeParamId_);
    drv_dashboard_->findParam("IS_CONNECTED", &robotConnectedParamId_);

    gripper_ = std::make_unique<ur_rtde::RobotiqGripper>(drv_dashboard_->get_ip());

    // attempt to connect to the gripper
    try_connect();

    // create epics polling thread
    epicsThreadCreate("GripperPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void URGripper::poll() {
    while (true) {
        lock();

        if (robot_ready() && gripper_->isConnected()) {
            try {
                setIntegerParam(isConnectedIndex_, 1);
                setIntegerParam(isActiveIndex_, gripper_->isActive());
                setIntegerParam(isOpenIndex_, gripper_->isOpen());
                setIntegerParam(isClosedIndex_, gripper_->isClosed());
                setDoubleParam(currentPositionIndex_, gripper_->getCurrentPosition());
                setDoubleParam(openPositionIndex_, gripper_->getOpenPosition());
                setDoubleParam(closedPositionIndex_, gripper_->getClosedPosition());

                ur_rtde::RobotiqGripper::eObjectStatus move_status = gripper_->objectDetectionStatus();
                setIntegerParam(moveStatusIndex_, move_status);
                switch (move_status) {
                case ur_rtde::RobotiqGripper::eObjectStatus::STOPPED_INNER_OBJECT:
                    setIntegerParam(isStoppedInnerIndex_, 1);
                    setIntegerParam(isStoppedOuterIndex_, 0);
                    break;
                case ur_rtde::RobotiqGripper::eObjectStatus::STOPPED_OUTER_OBJECT:
                    setIntegerParam(isStoppedInnerIndex_, 0);
                    setIntegerParam(isStoppedOuterIndex_, 1);
                    break;
                default:
                    setIntegerParam(isStoppedInnerIndex_, 0);
                    setIntegerParam(isStoppedOuterIndex_, 0);
                    break;
                }
            } catch (const std::exception& e) {
                spdlog::error("Gripper poll error: {}", e.what());
                gripper_->disconnect();
                setIntegerParam(isConnectedIndex_, 0);
            }
        } else {
            gripper_->disconnect();
            setIntegerParam(isConnectedIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_period_);
    }
}

asynStatus URGripper::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    bool comm_ok = true;

    // Check that robot is powered on
    if (!robot_ready()) {
        spdlog::error("Robot must be powered and dashboard connected on to use gripper");
        comm_ok = false;
        goto skip;
    }

    // Check that it's connected before continuing
    if (not gripper_->isConnected()) {
        spdlog::error("Robotiq gripper not connected");
        comm_ok = false;
        goto skip;
    }

    try {
        if (function == setSpeedIndex_) {
            spdlog::debug("Setting speed to {}", value);
            gripper_->setSpeed(value);
        } else if (function == setForceIndex_) {
            spdlog::debug("Setting force to {}", value);
            gripper_->setForce(value);
        }
    } catch (const std::exception& e) {
        spdlog::error("{}", e.what());
        comm_ok = false;
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("Error in Gripper::writeFloat64");
        return asynError;
    }
}

asynStatus URGripper::writeInt32(asynUser* pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    // Check that robot is powered on
    if (!robot_ready()) {
        spdlog::error("Robot must be powered and dashboard connected on to use gripper");
        comm_ok = false;
        goto skip;
    }

    if (function == connectIndex_) {
        spdlog::debug("Connecting to gripper");
        try_connect();
    }

    // Check that gripper connected before continuing
    if (not gripper_->isConnected()) {
        spdlog::error("Robotiq gripper not connected");
        comm_ok = false;
        goto skip;
    }

    try {
        if (function == activateIndex_) {
            spdlog::debug("Activating gripper");
            gripper_->activate();
        } else if (function == openIndex_) {
            if (value == 1) {
                setIntegerParam(openIndex_, 1);
                const bool is_open = gripper_->isOpen();
                const bool stopped_outer = gripper_->objectDetectionStatus() == ur_rtde::RobotiqGripper::eObjectStatus::STOPPED_OUTER_OBJECT;
                if (is_open || stopped_outer) {
                    spdlog::warn("Gripper already open or stopped at outer object");
                    setIntegerParam(openIndex_, 0);
                } else {
                    spdlog::debug("Opening gripper");
                    gripper_->open();
                }
            }
        } else if (function == closeIndex_) {
            if (value == 1) {
                setIntegerParam(closeIndex_, 1);
                const bool is_closed = gripper_->isClosed();
                const bool stopped_inner = gripper_->objectDetectionStatus() == ur_rtde::RobotiqGripper::eObjectStatus::STOPPED_INNER_OBJECT;
                if (is_closed || stopped_inner) {
                    spdlog::warn("Gripper already closed or stopped at inner object");
                    setIntegerParam(closeIndex_, 0);
                } else {
                    spdlog::debug("Closing gripper");
                    gripper_->close();
                }
            }
        } else if (function == setPositionRangeIndex_) {
            int minpos = 0;
            int maxpos = 0;
            getIntegerParam(minPositionIndex_, &minpos);
            getIntegerParam(maxPositionIndex_, &maxpos);
            gripper_->setNativePositionRange(minpos, maxpos);
            spdlog::debug("setNativePositionRange(min={}, max={})", minpos, maxpos);
        } else if (function == minPositionIndex_) {
            spdlog::debug("setting min={}", value);
            setIntegerParam(minPositionIndex_, value);
        } else if (function == maxPositionIndex_) {
            spdlog::debug("setting max={}", value);
            setIntegerParam(maxPositionIndex_, value);
        } else if (function == positionUnitIndex_) {
            constexpr auto epos = ur_rtde::RobotiqGripper::eMoveParameter::POSITION;
            constexpr auto eunit_device = ur_rtde::RobotiqGripper::eUnit::UNIT_DEVICE;
            constexpr auto eunit_normalized = ur_rtde::RobotiqGripper::eUnit::UNIT_NORMALIZED;
            constexpr auto eunit_percent = ur_rtde::RobotiqGripper::eUnit::UNIT_PERCENT;
            constexpr auto eunit_mm = ur_rtde::RobotiqGripper::eUnit::UNIT_MM;
            switch (value) {
            case 0:
                spdlog::debug("Setting position unit to 'Device' (0,255)");
                gripper_->setUnit(epos, eunit_device);
                break;
            case 1:
                spdlog::debug("Setting position unit to 'Normalized' (0,1.0)");
                gripper_->setUnit(epos, eunit_normalized);
                break;
            case 2:
                spdlog::debug("Setting position unit to 'Percent' (0,100%)");
                gripper_->setUnit(epos, eunit_percent);
                break;
            case 3:
                spdlog::debug("Setting position unit to 'mm' (must define range)");
                gripper_->setUnit(epos, eunit_mm);
                break;
            default:
                spdlog::warn("Unit {} undefined, no action taken.", value);
            }
        } else if (function == autoCalibrateIndex_) {
            spdlog::debug("Auto calibrating open/close positions");
            if (gripper_->isActive()) {
                gripper_->autoCalibrate();
                spdlog::debug("Auto calibration done");
                setIntegerParam(isCalibratedIndex_, 1);
            } else {
                spdlog::error("Activate gripper before autocalibrating");
            }
        }
    } catch (const std::exception& e) {
        spdlog::error("{}", e.what());
        comm_ok = false;
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("Error in Gripper::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int URGripperConfig(const char* asyn_port_name, const char* dash_drv_name, double poll_period) {
    new URGripper(asyn_port_name, dash_drv_name, poll_period);
    return asynSuccess;
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Dashboard driver name", iocshArgString};
static const iocshArg urRobotArg2 = {"Poll period", iocshArgDouble};
static const iocshArg* const urRobotArgs[3] = {&urRobotArg0, &urRobotArg1, &urRobotArg2};
static const iocshFuncDef urRobotFuncDef = {"URGripperConfig", 3, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf* args) {
    URGripperConfig(args[0].sval, args[1].sval, args[2].dval);
}

void URGripperRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(URGripperRegister);
}
