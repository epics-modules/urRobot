--- Lua library for EPICS Universal Robots support
-- This module is used for creating in-IOC robot programs.
-- @module ur_robot

local db = require("db")
local epics = require("epics")
local event = require("event")
local osi = require("osi")

local M = {}

local rbv_sync_disabled = false
local program_cancelled = {}

-- Injected globals:
---@diagnostic disable-next-line: undefined-global
local g_prefix = PREFIX
---@diagnostic disable-next-line: undefined-global
local g_record_name = G_RECORD_NAME
---@diagnostic disable-next-line: undefined-global
local g_user_script = G_USER_SCRIPT

local module_info = debug.getinfo(1, "S")
local module_script_name = module_info.source:sub(2)

local function program_lock(prefix)
    return event.flag("robot-program-lock:" .. prefix)
end

local function program_cancel(prefix)
    return event.flag("robot-program-cancel:" .. prefix)
end

local function program_gate(prefix)
    return event.flag("robot-program-gate:" .. prefix)
end

local function acquire_program_gate(prefix)
    local gate = program_gate(prefix)
    while not gate:testAndClear() do
        gate:wait(-1)
    end
    return gate
end

local function check_program_cancelled()
    if g_record_name ~= nil and program_cancel(g_prefix):test() then
        error(program_cancelled, 0)
    end
end

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
        if epics.get(g_prefix .. "Receive:SafetyStatusBits") ~= 1 then
            error("Motion stopped due to safety (or Receive:SafetyStatusBits unreachable)", 2)
        end

        now = osi.monotonic()
        -- Check that the move command was accepted.
        -- Reasons for rejection could be:
        -- * Target outside safety limits
        -- * Robot is in state not ready for motion (e.g. e-stop, powered off, etc)
        if not accepted then
            if epics.get(g_prefix .. "Control:AsyncMoveDone") == 0 then
                accepted = true
            elseif now >= acceptance_deadline then
                error(move_name .. " failed to start", 2)
            end
        end

        -- If the count was incremented, that means the move completed
        if get_motion_done_count() > count_before then
            return
        end

        -- Time out and issue a stop if the move is taking too long to complete
        if now >= completion_deadline then
            epics.put(g_prefix .. "Control:Stop", 1)
            error(string.format("Timed out after %.1f seconds waiting for %s", timeout, move_name), 2)
        end

        osi.sleep(0.05)
    end
end

local function validate_move_target(target)
    if type(target) ~= "table" or #target ~= 6 then
        error("Target must be length 6", 3)
    end

    for i, value in ipairs(target) do
        if type(value) ~= "number" then
            error(string.format("Target element %d is not a number", i), 3)
        end
    end
end

local function robot_ready()
    return epics.get(g_prefix .. "Dashboard:RobotMode") == "Robotmode: RUNNING"
       and epics.get(g_prefix .. "Control:Connected") == 1
       and epics.get(g_prefix .. "Control:AsyncMoveDone") == 1
       and epics.get(g_prefix .. "Receive:RuntimeState") == 2
end

--- Moves the robot to a joint target and waits for completion.
-- @function moveJ
-- @tparam table target Six joint angles in degrees.
-- @tparam[opt=300.0] number timeout Maximum completion time in seconds.
function M.moveJ(target, timeout)
    check_program_cancelled()
    timeout = timeout or 300.0

    validate_move_target(target)

    if not robot_ready() then
        error("Robot not ready for motion", 2)
    end

    -- Set the target values
    for i, value in ipairs(target) do
        epics.put(string.format("%sControl:J%dCmd", g_prefix, i), value)
    end

    -- Get the counter value and disable command/readback syncing
    local count_before = get_motion_done_count()
    ensure_rbv_sync_disabled()

    -- Start motion and block until it's done
    if epics.put(g_prefix .. "Control:moveJ", 1) ~= nil then
        error("Failed to trigger moveJ", 2)
    end
    wait_motion_done(count_before, timeout, "moveJ")
    check_program_cancelled()
end

--- Moves the robot linearly to a Cartesian pose and waits for completion.
-- @function moveL
-- @tparam table target Six values: X,Y,Z in millimeters and rotation-vector Rx,Ry,Rz in radians.
-- @tparam[opt=300.0] number timeout Maximum completion time in seconds.
function M.moveL(target, timeout)
    check_program_cancelled()
    timeout = timeout or 300.0

    validate_move_target(target)

    if not robot_ready() then
        error("Robot not ready for motion", 2)
    end

    -- Set the target values
    for i, value in ipairs(target) do
        epics.put(string.format("%sControl:Pose%dCmd", g_prefix, i), value)
    end

    -- Get the counter value
    -- Disable command/readback syncing
    local count_before = get_motion_done_count()
    ensure_rbv_sync_disabled()

    -- Start motion and block until it's done
    if epics.put(g_prefix .. "Control:moveL", 1) ~= nil then
        error("Failed to trigger moveL", 2)
    end
    wait_motion_done(count_before, timeout, "moveL")
    check_program_cancelled()
end

--- Runs a URP program on the controller and waits for completion.
-- @function run_urp
-- @tparam string filename URP filename to load.
-- @tparam[opt=300.0] number completion_timeout Maximum execution time in seconds.
function M.run_urp(filename, completion_timeout)
    check_program_cancelled()
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
    check_program_cancelled()
end

--- Reads the current joint positions.
-- @function get_joints
-- @treturn table Six joint angles in degrees.
function M.get_joints()
    check_program_cancelled()
    local value, err = epics.get(g_prefix .. "Receive:Joints")
    if value == nil then
        error("Failed to read joints: " .. tostring(err), 2)
    end
    check_program_cancelled()
    return value
end

--- Reads the current Cartesian TCP pose.
-- @function get_pose
-- @treturn table X,Y,Z in millimeters and rotation-vector Rx,Ry,Rz in radians.
function M.get_pose()
    check_program_cancelled()
    local value, err = epics.get(g_prefix .. "Receive:Pose")
    if value == nil then
        error("Failed to read pose: " .. tostring(err), 2)
    end
    check_program_cancelled()
    return value
end

function M.open_gripper()
    check_program_cancelled()
    epics.put(g_prefix .. "RobotiqGripper:Open", 1)
    wait_pv(g_prefix .. "RobotiqGripper:Open", function(value) return value == 0 end, 5.0)
    check_program_cancelled()
end

function M.close_gripper()
    check_program_cancelled()
    epics.put(g_prefix .. "RobotiqGripper:Close", 1)
    wait_pv(g_prefix .. "RobotiqGripper:Close", function(value) return value == 0 end, 5.0)
    check_program_cancelled()
end

--- Starts a registered program in an asynchronous Lua state.
-- @function run_program
-- @tparam string prefix Robot PV prefix.
-- @tparam string record_name Registered record name to execute.
-- @tparam string script_name Lua script containing the registration.
function M.run_program(prefix, record_name, script_name)
    local gate = acquire_program_gate(prefix)
    local lock = program_lock(prefix)
    local cancel = program_cancel(prefix)

    if not lock:testAndClear() then
        gate:set()
        error("Another robot program is already running", 2)
    end

    cancel:clear()
    local err = luaRunFile(
        module_script_name,
        {
            PREFIX = prefix,
            G_RECORD_NAME = record_name,
            G_USER_SCRIPT = script_name,
        },
        {async = 1}
    )

    if err ~= nil then
        lock:set()
    end
    gate:set()

    if err ~= nil then
        error(err, 2)
    end
end

function M.stop_program(prefix)
    local gate = acquire_program_gate(prefix)
    if not program_lock(prefix):test() then
        program_cancel(prefix):set()
    end
    gate:set()
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

        program_lock(g_prefix):set()
        program_gate(g_prefix):set()

        -- Create the EPICS luascript records
        db.record("luascript", record_name) {
            CODE = string.format("return require('ur_robot').run_program(%q, %q, %q)", g_prefix, record_name, script_name),
            SYNC = "Sync",
        }
        db.record("luascript", g_prefix .. "LuaUR:Stop") {
            CODE = string.format("return require('ur_robot').stop_program(%q)", g_prefix),
            SYNC = "Sync",
            FLNK = g_prefix .. "Control:Stop.PROC",
        }
        db.record("bo", g_prefix .. "LuaUR:Running") {
            ZNAM = "Done",
            ONAM = "Running"
        }
    elseif g_record_name == record_name then
        return func()
    end
end

if g_user_script ~= nil then
    -- Sets this module instance to be returned to the user
    -- script's call to require("ur_robot")
    package.loaded["ur_robot"] = M

    -- Traceback for potential user script errors
    local function traceback(err)
        if err == program_cancelled then
            return err
        end
        return debug.traceback(err, 2)
    end

    -- This runs the complete user's script.
    -- The script will call robot.register(record_name, func). This is not during IOC initialization
    -- so g_record_name is not nil, so each call the register will check if record_name == g_record_name,
    -- if so, call func.
    epics.put(g_prefix .. "LuaUR:Running", 1)
    local result = table.pack(xpcall(dofile, traceback, g_user_script))
    epics.put(g_prefix .. "LuaUR:Running", 0)

    -- Restore command/readback sync. Wrap in pcall so errors don't prevent later lock cleanup
    local cleanup_ok, cleanup_err = table.pack(pcall(ensure_rbv_sync_enabled))

    -- Clear cancellation flag; Clear program flag to allow other programs to start
    local gate = acquire_program_gate(g_prefix)
    program_cancel(g_prefix):clear()
    program_lock(g_prefix):set()
    gate:set()

    -- Report errors
    if not cleanup_ok then
        error(cleanup_err, 0)
    end
    if not result[1] and result[2] ~= program_cancelled then
        error(result[2], 0)
    end
end

return M
