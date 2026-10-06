local db = require("db")
local epics = require("epics")
local asyn = require("asyn")
local event = require("event")
local osi = require("osi")

local robot = {}

local function get_motion_done_count()
    local count = asyn.getIntegerParam("rtde_ctrl", "MOTION_DONE_COUNT")
    if count == nil then
        error("Could not read MOTION_DONE_COUNT", 2)
    end
    return count
end

local function wait_motion_done(count_before, timeout)
    local deadline = osi.monotonic() + timeout

    while true do
        local safety, err = epics.get(PREFIX .. "Receive:SafetyStatusBits")
        if safety == nil then
            error("Could not read safety status: " .. tostring(err), 2)
        end
        if safety ~= 1 then
            error("Motion stopped by robot safety state", 2)
        end

        if get_motion_done_count() ~= count_before then
            return
        end

        if osi.monotonic() >= deadline then
            epics.put(PREFIX .. "Control:Stop", 1)
            error(string.format("Timed out after %.1f seconds waiting for move", timeout), 2)
        end

        osi.sleep(0.05)
    end
end

function robot.moveJ(target, timeout)
    timeout = timeout or 300.0

    if type(target) ~= "table" or #target ~= 6 then
        error("moveJ target must be length 6", 2)
    end

    local done, err = epics.get(PREFIX .. "Control:AsyncMoveDone")
    if done == nil then
        error("Could not read AsyncMoveDone: " .. tostring(err), 2)
    end
    if done ~= 1 then
        error("Cannot start moveJ: another motion is active", 2)
    end

    for i, value in ipairs(target) do
        if type(value) ~= "number" then
            error(string.format("moveJ target element %d is not a number", i), 2)
        end
        epics.put(string.format("%sControl:J%dCmd", PREFIX, i), value)
    end

    local count_before = get_motion_done_count()

    epics.put(PREFIX .. "Control:moveJ", 1)
    wait_motion_done(count_before, timeout)
end

function robot.moveL(target, timeout)
    timeout = timeout or 300.0

    if type(target) ~= "table" or #target ~= 6 then
        error("moveL target must be length 6", 2)
    end

    local done, err = epics.get(PREFIX .. "Control:AsyncMoveDone")
    if done == nil then
        error("Could not read AsyncMoveDone: " .. tostring(err), 2)
    end
    if done ~= 1 then
        error("Cannot start moveL: another motion is active", 2)
    end

    for i, value in ipairs(target) do
        if type(value) ~= "number" then
            error(string.format("moveL target element %d is not a number", i), 2)
        end
        epics.put(string.format("%sControl:Pose%dCmd", PREFIX, i), value)
    end

    local count_before = get_motion_done_count()

    epics.put(PREFIX .. "Control:moveL", 1)
    wait_motion_done(count_before, timeout)
end

function robot.run_program(prefix, record_name, script_name)
    local err = luaRunFile(
        script_name,
        {
            PREFIX = prefix,
            G_RECORD_NAME = record_name
        },
        {async = 1}
    )
    if err ~= nil then
        error(err, 2)
    end
end


-- The calling script is executed once during IOC startup and again in a new,
-- asynchronous Lua state whenever a registered luascript record is processed.
-- This allows users to create a record and associate it with a Lua function
-- using one robot.register() call, without maintaining a separate database file.
--
-- During IOC startup, G_RECORD_NAME is nil, so robot.register() creates the
-- luascript record. The record's CODE field calls run_program(record_name).
--
-- At runtime, run_program() executes the calling script again and injects the
-- selected record name as G_RECORD_NAME. Each robot.register() call is then
-- evaluated, but only the function whose record_name matches G_RECORD_NAME is
-- executed. Concurrent programs for the same robot prefix are rejected.
-- Automatic command/readback synchronization is disabled while the selected
-- function runs and restored afterward, including when the function fails.
function robot.register(record_name, func)
    if G_RECORD_NAME == nil then
        -- Get the name of the user script
        local info = debug.getinfo(2, "S")
        if info == nil or info.source:sub(1,1) ~= "@" then
            error("robot.register must be called directly from a lua file", 2)
        end
        local script_name = info.source:sub(2)

        event.flag("robot-program-lock:" .. PREFIX):set()

        -- Create the EPICS luascript record
        db.record("luascript", record_name) {
            CODE = string.format("return require('robot').run_program(%q, %q, %q)", PREFIX, record_name, script_name),
            SYNC = "Sync",
        }
    elseif G_RECORD_NAME == record_name then
        local program_lock = event.flag("robot-program-lock:" .. PREFIX)
        if not program_lock:testAndClear() then
            error("Another robot program is already running", 2)
        end

        local joint_sync_disa_pv = PREFIX .. "Control:sync_joint_cmd.DISA"
        local pose_sync_disa_pv = PREFIX .. "Control:sync_pose_cmd.DISA"

        epics.put(joint_sync_disa_pv, 1)
        epics.put(pose_sync_disa_pv, 1)

        local result = table.pack(xpcall(func, debug.traceback))

        epics.put(joint_sync_disa_pv, 0)
        epics.put(pose_sync_disa_pv, 0)
        program_lock:set()

        if not result[1] then
            error(result[2], 0)
        end

        return table.unpack(result, 2, result.n)
    end
end

return robot
