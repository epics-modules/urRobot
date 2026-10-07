local epics = require("epics")
local asyn = require("asyn")
local seq = require("seq")

local N = math.floor(tonumber(N))
local KMAX = math.floor(tonumber(KMAX))

luaRegisterState(STATE)

local go_pv              = epics.pv(P .. "Path" .. N .. ":Go")
local stop_path_pv       = epics.pv(P .. "Control:stop_path")
local safety_pv          = epics.pv(P .. "Receive:SafetyStatusBits")
local sync_joint_disa_pv = epics.pv(P .. "Control:sync_joint_cmd.DISA")
local sync_pose_disa_pv  = epics.pv(P .. "Control:sync_pose_cmd.DISA")

local wp_idx = 0
local count_before = 0
local wp_action_original = 0

local wp = {}

local function restore_action_override()
    if wp.action_override ~= 0 then
        wp.action_opt_val.VAL = wp_action_original
        wp.set_action_opt.LNK0 = wp.action_opt.name .. " PP"
        wp.set_action_opt.PROC = 1
    end
end

local function finish_path(msg)
    print(msg)
    go_pv.VAL = 0
    sync_joint_disa_pv.VAL = 0
    sync_pose_disa_pv.VAL = 0
end

local prog = seq.program("path" .. N, { poll = 0.01 })

prog:state("idle", {
    seq.when(function() return go_pv.VAL == 1 end) {
        action = function()
            sync_joint_disa_pv.VAL = 1
            sync_pose_disa_pv.VAL = 1
            stop_path_pv.VAL = 0
            wp_idx = 1
        end,
        next = "check_waypoint",
    },
})

prog:state("check_waypoint", {
    options = { always_enter = true },

    entry = function()
        wp = { action_override = 0 }
    end,

    seq.when(function() return wp_idx > KMAX end) {
        action = function() finish_path("Path " .. N .. " completed!") end,
        next = "idle",
    },

    seq.when(function() return stop_path_pv.VAL == 1 end) {
        action = function() finish_path("Path " .. N .. " stopped") end,
        next = "idle",
    },

    seq.when(function()
        local path_wp = P .. "Path" .. N .. ":" .. wp_idx
        local num = epics.get(path_wp .. ":Number")
        if num <= 0 then return false end
        return epics.get(path_wp .. ":Enabled") ~= 0
    end) {
        next = "configure_move",
    },

    seq.when() {
        action = function() wp_idx = wp_idx + 1 end,
        next = "check_waypoint",
    },
})

prog:state("configure_move", {
    entry = function()
        local path_wp = P .. "Path" .. N .. ":" .. wp_idx
        local wp_type_val = epics.get(path_wp .. ":Type")
        local wp_type = (wp_type_val == 0.0) and "L" or "J"
        local wp_num = math.floor(epics.get(path_wp .. ":Number"))
        local wp_base = P .. "Waypoint" .. wp_type .. ":" .. wp_num

        wp = {
            action_override = math.floor(epics.get(path_wp .. ":ActionOverride")),
            action_opt      = epics.pv(wp_base .. ":ActionOpt"),
            action_opt_val  = epics.pv(path_wp .. ":action_opt_val"),
            set_action_opt  = epics.pv(path_wp .. ":set_action_opt"),
            move            = epics.pv(wp_base .. ":move" .. wp_type),
            busy            = epics.pv(path_wp .. ":Busy"),
        }

        if wp.action_override ~= 0 then
            wp_action_original = math.floor(wp.action_opt.VAL)
            wp.action_opt_val.VAL = wp.action_override
            wp.set_action_opt.LNK0 = wp.action_opt.name .. " PP"
            wp.set_action_opt.PROC = 1
        end

        count_before = asyn.getIntegerParam(CTRL_PORT, "MOTION_DONE_COUNT")
        print("Moving to waypoint " .. wp_base .. "...")
        wp.busy.VAL = 1
        wp.move.PROC = 1
    end,

    seq.when() {
        next = "wait_motion",
    },
})

prog:state("wait_motion", {
    seq.when(function()
        return asyn.getIntegerParam(CTRL_PORT, "MOTION_DONE_COUNT") ~= count_before
    end) {
        next = "finish_waypoint",
    },

    seq.when(function() return stop_path_pv.VAL == 1 end) {
        action = function()
            wp.busy.VAL = 0
            restore_action_override()
            finish_path("Path " .. N .. " stopped")
        end,
        next = "idle",
    },

    seq.when(function() return safety_pv.VAL ~= 1 end) {
        action = function()
            stop_path_pv.VAL = 1
            wp.busy.VAL = 0
            restore_action_override()
            finish_path("Path " .. N .. " stopped (safety)")
        end,
        next = "idle",
    },

    seq.when(seq.delay(300.0)) {
        action = function()
            print("Error: Timeout exceeded waiting for motion to complete")
            wp.busy.VAL = 0
            finish_path("Path " .. N .. " timed out")
        end,
        next = "idle",
    },
})

prog:state("finish_waypoint", {
    entry = function()
        wp.busy.VAL = 0
        restore_action_override()
        wp_idx = wp_idx + 1
    end,

    seq.when() {
        next = "check_waypoint",
    },
})

seq.register(prog)
