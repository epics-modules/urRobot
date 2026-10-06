local db = require("db")
local epics = require("epics")
local asyn = require("asyn")
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
        local safety, err = epics.get("urExample:Receive:SafetyStatusBits")
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
            epics.put("urExample:Control:Stop", 1)
            error(string.format("Timed out after %.1f seconds waiting for moveJ", timeout), 2)
        end

        osi.sleep(0.05)
    end
end

function robot.moveJ(target, timeout)
    timeout = timeout or 300.0

    if type(target) ~= "table" or #target ~= 6 then
        error("moveJ target must contain exactly six joint positions", 2)
    end

    local done, err = epics.get("urExample:Control:AsyncMoveDone")
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
        epics.put(string.format("urExample:Control:J%dCmd", i), value)
    end

    local count_before = get_motion_done_count()

    epics.put("urExample:Control:moveJ", 1)
    wait_motion_done(count_before, timeout)
end

function robot.run_program(record_name, script_name)
    local err = luaRunFile(
        script_name,
        {G_RECORD_NAME = record_name},
        {async = true}
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
-- At runtime, run_program() executes user_program.lua again and injects the
-- selected record name as G_RECORD_NAME. Each robot.register() call is then
-- evaluated, but only the function whose record_name matches G_RECORD_NAME is
-- executed. No records are created during this asynchronous execution.
function robot.register(record_name, func)
    local info = debug.getinfo(2, "S")
    local script_name = info.source:sub(2)
    if G_RECORD_NAME == nil then
        db.record("luascript", record_name) {
            CODE = string.format("return require('robot').run_program(%q, %q)", record_name, script_name),
            SYNC = "Sync",
        }
    elseif G_RECORD_NAME == record_name then
        return func()
    end
end

return robot
