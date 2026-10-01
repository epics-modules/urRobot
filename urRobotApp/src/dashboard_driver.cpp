#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <iocsh.h>

#include "dashboard_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

static void poll_thread_C(void* pPvt) {
    URDashboard* pURDashboard = (URDashboard*)pPvt;
    pURDashboard->poll();
}

// wraps the ur_dashboard_->connect() function to fail gracefully
bool URDashboard::try_connect() {
    bool connected = false;
    try {
        ur_dashboard_->connect();
        if (ur_dashboard_->isConnected()) {
            spdlog::info("Connected to UR Dashboard server");
            connected = true;
        }
    } catch (const std::exception& e) {
        spdlog::error(e.what());
    }
    return connected;
}

constexpr int MAX_ADDR = 1;
constexpr int ASYN_INTERFACE_MASK = asynInt32Mask | asynOctetMask | asynDrvUserMask;
constexpr int ASYN_INTERRUPT_MASK = asynInt32Mask | asynOctetMask;

URDashboard::URDashboard(const char* asyn_port_name, const char* robot_ip, double poll_period)
    : asynPortDriver(asyn_port_name, MAX_ADDR, ASYN_INTERFACE_MASK, ASYN_INTERRUPT_MASK,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0),
      ur_dashboard_(std::make_unique<ur_rtde::DashboardClient>(robot_ip)), poll_period_(poll_period),
      robot_ip_(robot_ip) {

    // create asyn parameters
    createParam("IS_CONNECTED", asynParamInt32, &isConnectedIndex_);
    createParam("LOAD_URP", asynParamOctet, &loadURPIndex_);
    createParam("PLAY", asynParamInt32, &playIndex_);
    createParam("STOP", asynParamInt32, &stopIndex_);
    createParam("PAUSE", asynParamInt32, &pauseIndex_);
    createParam("CONNECT", asynParamInt32, &connectIndex_);
    createParam("DISCONNECT", asynParamInt32, &disconnectIndex_);
    createParam("SHUTDOWN", asynParamInt32, &shutdownIndex_);
    createParam("IS_RUNNING", asynParamInt32, &isRunningIndex_);
    createParam("CLOSE_POPUP", asynParamInt32, &closePopupIndex_);
    createParam("POPUP", asynParamOctet, &popupIndex_);
    createParam("CLOSE_SAFETY_POPUP", asynParamInt32, &closeSafetyPopupIndex_);
    createParam("POWER_ON", asynParamInt32, &powerOnIndex_);
    createParam("POWER_OFF", asynParamInt32, &powerOffIndex_);
    createParam("BRAKE_RELEASE", asynParamInt32, &brakeReleaseIndex_);
    createParam("UNLOCK_PROTECTIVE_STOP", asynParamInt32, &unlockProtectiveStopIndex_);
    createParam("RESTART_SAFETY", asynParamInt32, &restartSafetyIndex_);
    createParam("POLYSCOPE_VERSION", asynParamOctet, &polyscopeVersionIndex_);
    createParam("SERIAL_NUMBER", asynParamOctet, &serialNumberIndex_);
    createParam("ROBOT_MODE", asynParamOctet, &robotModeIndex_);
    createParam("PROGRAM_STATE", asynParamOctet, &programStateIndex_);
    createParam("ROBOT_MODEL", asynParamOctet, &robotModelIndex_);
    createParam("LOADED_PROGRAM", asynParamOctet, &loadedProgramIndex_);
    createParam("SAFETY_STATUS", asynParamOctet, &safetyStatusIndex_);
    createParam("IS_PROGRAM_SAVED", asynParamInt32, &isProgramSavedIndex_);
    createParam("IS_IN_REMOTE_CONTROL", asynParamInt32, &isInRemoteControlIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    bool connected = this->try_connect();
    if (connected) {
        setIntegerParam(isConnectedIndex_, 1);
        std::string version_str = ur_dashboard_->polyscopeVersion();
        setStringParam(polyscopeVersionIndex_, version_str);
        ur_rtde::PolyScopeVersion version(version_str);
        if (version.major == 5 && version.minor >= 6) {
            setStringParam(serialNumberIndex_, ur_dashboard_->getSerialNumber());
            eseries_ = true;
        } else {
            setStringParam(serialNumberIndex_, "3.X.X");
            eseries_ = false;
        }
        setStringParam(robotModelIndex_, ur_dashboard_->getRobotModel());
    } else {
        setIntegerParam(isConnectedIndex_, 0);
    }

    epicsThreadCreate("URDashboardPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void URDashboard::poll() {
    while (true) {
        lock();
        try {
            if (ur_dashboard_ && ur_dashboard_->isConnected()) {
                setIntegerParam(isConnectedIndex_, 1);
                setIntegerParam(isRunningIndex_, ur_dashboard_->running());
                setStringParam(programStateIndex_, ur_dashboard_->programState());
                setStringParam(robotModeIndex_, ur_dashboard_->robotmode());
                setStringParam(loadedProgramIndex_, ur_dashboard_->getLoadedProgram());
                setStringParam(safetyStatusIndex_, ur_dashboard_->safetystatus());
                setIntegerParam(isProgramSavedIndex_, ur_dashboard_->isProgramSaved());
                if (eseries_) {
                    setIntegerParam(isInRemoteControlIndex_, ur_dashboard_->isInRemoteControl());
                } else {
                    setIntegerParam(isInRemoteControlIndex_, 1);
                }
            } else {
                setIntegerParam(isConnectedIndex_, 0);
            }
        } catch (const std::exception& e) {
            spdlog::error("Caught exception in dashboard poller: {}", e.what());
            setIntegerParam(isConnectedIndex_, 0);
            setIntegerParam(isInRemoteControlIndex_, 0);
            ur_dashboard_->disconnect();
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_period_);
    }
}

asynStatus URDashboard::writeInt32(asynUser* pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (!ur_dashboard_->isConnected()) {
        if (function == connectIndex_) {
            spdlog::debug("Connecting to dashboard server");
            try_connect();
            callParamCallbacks();
            return asynSuccess;
        } else {
            spdlog::debug("Dashboard is disconnected. No action taken");
            return asynError;
        }
    }

    try {
        if (function == playIndex_) {
            spdlog::debug("Playing loaded program");
            ur_dashboard_->play();
        } else if (function == stopIndex_) {
            spdlog::debug("Stopping current program");
            ur_dashboard_->stop();
        } else if (function == pauseIndex_) {
            spdlog::debug("Pausing current program");
            ur_dashboard_->pause();
        } else if (function == connectIndex_) {
            spdlog::debug("Connecting to dashboard server");
            try_connect();
        } else if (function == disconnectIndex_) {
            spdlog::debug("Disconnecting from dashboard server");
            ur_dashboard_->disconnect();
        } else if (function == shutdownIndex_) {
            spdlog::debug("Shutting down robot and controller");
            ur_dashboard_->shutdown();
        } else if (function == closePopupIndex_) {
            spdlog::debug("Closing popup");
            ur_dashboard_->closePopup();
        } else if (function == closeSafetyPopupIndex_) {
            spdlog::debug("Closing safety popup");
            ur_dashboard_->closeSafetyPopup();
        } else if (function == powerOnIndex_) {
            spdlog::debug("Powering on robot");
            ur_dashboard_->powerOn();
        } else if (function == powerOffIndex_) {
            spdlog::debug("Powering off robot");
            ur_dashboard_->powerOff();
        } else if (function == brakeReleaseIndex_) {
            spdlog::debug("Releasing brakes");
            ur_dashboard_->brakeRelease();
        } else if (function == unlockProtectiveStopIndex_) {
            spdlog::debug("Unlocking protective stop");
            ur_dashboard_->unlockProtectiveStop();
        } else if (function == restartSafetyIndex_) {
            spdlog::debug("Restarting safety configuration");
            ur_dashboard_->restartSafety();
        }
    } catch (const std::exception& e) {
        spdlog::error("{}", e.what());
        comm_ok = false;
    }

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

asynStatus URDashboard::writeOctet(asynUser* pasynUser, const char* value, size_t maxChars, size_t* nActual) {
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    *nActual = strlen(value);

    if (!ur_dashboard_->isConnected()) {
        spdlog::debug("Dashboard is disconnected. No action taken");
        return asynError;
    }

    try {
        if (function == popupIndex_) {
            spdlog::debug("Popup text: {}", value);
            ur_dashboard_->popup(value);
        } else if (function == loadURPIndex_) {
            spdlog::debug("Loading program {}", value);
            ur_dashboard_->loadURP(value);
        }
    } catch (const std::exception& e) {
        spdlog::error("{}", e.what());
        status = asynError;
    }

    callParamCallbacks();
    return status;
}
// register function for iocsh
extern "C" int URDashboardConfig(const char* asyn_port_name, const char* robot_ip, double poll_period) {
    new URDashboard(asyn_port_name, robot_ip, poll_period);
    return asynSuccess;
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg urRobotArg2 = {"Poll period", iocshArgDouble};
static const iocshArg* const urRobotArgs[3] = {&urRobotArg0, &urRobotArg1, &urRobotArg2};
static const iocshFuncDef urRobotFuncDef = {"URDashboardConfig", 3, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf* args) {
    URDashboardConfig(args[0].sval, args[1].sval, args[2].dval);
}

void URDashboardRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(URDashboardRegister);
}
