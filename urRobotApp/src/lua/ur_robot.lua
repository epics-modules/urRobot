--- Lua library for EPICS Universal Robots support
-- This module is used for creating in-IOC robot programs.
-- @module ur_robot

local db = require("db")
local epics = require("epics")
local event = require("event")
local osi = require("osi")

local M = {}

local rbv_sync_disabled = false

-- Injected globals:
---@diagnostic disable-next-line: undefined-global
local g_prefix = PREFIX
---@diagnostic disable-next-line: undefined-global
local g_record_name = G_RECORD_NAME

-- Continually reads a pv, passes its value to fn,
-- returning if fn(pv_value) == true, or if timeout
local function wait_pv(pv_name, fn, timeout)
    local deadline = osi.monotonic() + timeout
    while true do
        local value = epics.get(pv_name)
        if fn(value) then
            return value
        end
        if osi.monotonic() >= deadline then
            error(string.format("Timed out after %.1f seconds waiting for %s", timeout, pv_name), 2)
        end
        osi.sleep(0.05)
    end
end

local function get_motion_done_count()
    local count, err = epics.get(g_prefix .. "Control:motion_done_count")
    if count == nil then
        error("Could not read motion_done_count: " .. tostring(err), 2)
    end
    return count
end

local function ensure_rbv_sync_disabled()
    if rbv_sync_disabled then
        return
    end
    epics.put(g_prefix .. "Control:sync_joint_cmd.DISA", 1)
    epics.put(g_prefix .. "Control:sync_pose_cmd.DISA", 1)
    rbv_sync_disabled = true
end

local function ensure_rbv_sync_enabled()
    if not rbv_sync_disabled then
        return
    end
    epics.put(g_prefix .. "Control:sync_joint_cmd.DISA", 0)
    epics.put(g_prefix .. "Control:sync_pose_cmd.DISA", 0)
    rbv_sync_disabled = false
end

local function wait_motion_done(count_before, timeout, move_name)
    local now = osi.monotonic()
    local acceptance_deadline = now + math.min(0.5, timeout)
    local completion_deadline = now + timeout
    local accepted = false

    while true do
        -- Abort if safety state changed
        local safety, err = epics.get(g_prefix .. "Receive:SafetyStatusBits")
        if safety == nil then
            error("Could not read safety status: " .. tostring(err), 2)
        end
        if safety ~= 1 then
            error("Motion stopped by robot safety state", 2)
        end

        -- This means motion completed
        if get_motion_done_count() ~= count_before then
            return
        end

        now = osi.monotonic()
        -- Check that the move command was accepted.
        -- Reasons for rejection could be:
        -- * Target outside safety limits
        -- * Robot is in state not ready for motion (e.g. e-stop, powered off, etc)
        if not accepted then
            local done, done_err = epics.get(g_prefix .. "Control:AsyncMoveDone")
            if done == nil then
                error("Could not read AsyncMoveDone: " .. tostring(done_err), 2)
            end
            if done == 0 then
                accepted = true
            elseif now >= acceptance_deadline then
                error(move_name .. " was rejected by the controller", 2)
            end
        end

        -- Time out and issue a stop if the move is taking too long to complete
        if now >= completion_deadline then
            epics.put(g_prefix .. "Control:Stop", 1)
            error(string.format("Timed out after %.1f seconds waiting for %s", timeout, move_name), 2)
        end

        osi.sleep(0.05)
    end
end

--- Moves the robot to a joint target and waits for completion.
-- @function moveJ
-- @tparam table target Six joint angles in degrees.
-- @tparam[opt=300.0] number timeout Maximum completion time in seconds.
function M.moveJ(target, timeout)
    timeout = timeout or 300.0

    if type(target) ~= "table" or #target ~= 6 then
        error("moveJ target must be length 6", 2)
    end

    local done, err = epics.get(g_prefix .. "Control:AsyncMoveDone")
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
        epics.put(string.format("%sControl:J%dCmd", g_prefix, i), value)
    end

    local count_before = get_motion_done_count()

    ensure_rbv_sync_disabled()
    local move_err = epics.put(g_prefix .. "Control:moveJ", 1)
    if move_err ~= nil then
        error("Failed to submit moveJ: " .. tostring(move_err), 2)
    end
    wait_motion_done(count_before, timeout, "moveJ")
end

--- Moves the robot linearly to a Cartesian pose and waits for completion.
-- @function moveL
-- @tparam table target Six values: X,Y,Z in millimeters and rotation-vector Rx,Ry,Rz in radians.
-- @tparam[opt=300.0] number timeout Maximum completion time in seconds.
function M.moveL(target, timeout)
    timeout = timeout or 300.0

    if type(target) ~= "table" or #target ~= 6 then
        error("moveL target must be length 6", 2)
    end

    local done, err = epics.get(g_prefix .. "Control:AsyncMoveDone")
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
        epics.put(string.format("%sControl:Pose%dCmd", g_prefix, i), value)
    end

    local count_before = get_motion_done_count()

    ensure_rbv_sync_disabled()
    local move_err = epics.put(g_prefix .. "Control:moveL", 1)
    if move_err ~= nil then
        error("Failed to submit moveL: " .. tostring(move_err), 2)
    end
    wait_motion_done(count_before, timeout, "moveL")
end

--- Runs a URP program on the controller and waits for completion.
-- @function run_urp
-- @tparam string filename URP filename to load.
-- @tparam[opt=300.0] number completion_timeout Maximum execution time in seconds.
function M.run_urp(filename, completion_timeout)
    completion_timeout = completion_timeout or 300.0

    -- Stop control script
    epics.put(g_prefix .. "Control:StopControlScript", 1)

    -- Load the URP program, wait for it to be loaded
    epics.put(g_prefix .. "Dashboard:LoadURP", filename)
    wait_pv(g_prefix .. "Dashboard:LoadedProgram.VAL$", function(value)
        return value:find(filename, 1, true)
    end, 5.0)

    -- Play loaded program, wait for it to start running
    epics.put(g_prefix .. "Dashboard:Play", 1)
    wait_pv(g_prefix .. "Dashboard:Running", function(value)
        return value == 1
    end, 5.0)

    -- Wait for program to be done
    wait_pv(g_prefix .. "Dashboard:Running", function(value) return value == 0 end, completion_timeout)
end

--- Reads the current joint positions.
-- @function get_joints
-- @treturn table Six joint angles in degrees.
function M.get_joints()
    local value, err = epics.get(g_prefix .. "Receive:Joints")
    if value == nil then
        error("Failed to read joints: " .. tostring(err), 2)
    end
    return value
end

--- Reads the current Cartesian TCP pose.
-- @function get_pose
-- @treturn table X,Y,Z in millimeters and rotation-vector Rx,Ry,Rz in radians.
function M.get_pose()
    local value, err = epics.get(g_prefix .. "Receive:Pose")
    if value == nil then
        error("Failed to read pose: " .. tostring(err), 2)
    end
    return value
end

function M.open_gripper()
    epics.put(g_prefix .. "RobotiqGripper:Open", 1)
    wait_pv(g_prefix .. "RobotiqGripper:Open", function(value) return value == 0 end, 5.0)
end

function M.close_gripper()
    epics.put(g_prefix .. "RobotiqGripper:Close", 1)
    wait_pv(g_prefix .. "RobotiqGripper:Close", function(value) return value == 0 end, 5.0)
end

--- Starts a registered program in an asynchronous Lua state.
-- @function run_program
-- @tparam string prefix Robot PV prefix.
-- @tparam string record_name Registered record name to execute.
-- @tparam string script_name Lua script containing the registration.
function M.run_program(prefix, record_name, script_name)
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
-- During IOC startup, g_record_name is nil, so robot.register() creates the
-- luascript record. The record's CODE field calls run_program(record_name).
--
-- At runtime, run_program() executes the calling script again and injects the
-- selected record name as G_RECORD_NAME. Each robot.register() call is then
-- evaluated, but only the function whose record_name matches G_RECORD_NAME is
-- executed. Concurrent programs for the same robot prefix are rejected.
-- Automatic command/readback synchronization is disabled while the selected
-- function runs and restored afterward, including when the function fails.
--- Registers a serialized robot program as a luascript record.
-- @function register
-- @tparam string record_name EPICS record name to create.
-- @tparam function func Program function to execute.
-- @return Values returned by the program function.
function M.register(record_name, func)
    if g_record_name == nil then
        -- Get the name of the user script
        local info = debug.getinfo(2, "S")
        if info == nil or info.source:sub(1,1) ~= "@" then
            error("robot.register must be called directly from a lua file", 2)
        end
        local script_name = info.source:sub(2)

        event.flag("robot-program-lock:" .. g_prefix):set()

        -- Create the EPICS luascript record
        db.record("luascript", record_name) {
            CODE = string.format("return require('ur_robot').run_program(%q, %q, %q)", g_prefix, record_name, script_name),
            SYNC = "Sync",
        }
    elseif g_record_name == record_name then
        local program_lock = event.flag("robot-program-lock:" .. g_prefix)
        if not program_lock:testAndClear() then
            error("Another robot program is already running", 2)
        end

        -- Call the user's function
        local result = table.pack(xpcall(func, debug.traceback))

        -- if command/readback sync was disabled, re-enable it
        ensure_rbv_sync_enabled()

        program_lock:set()

        if not result[1] then
            error(result[2], 0)
        end

        return table.unpack(result, 2, result.n)
    end
end

return M
