-- NOTE: IOC must add $(URROBOT)/urRobotApp/src/lua to LUA_SCRIPT_PATH
epics = require("epics")
osi = require("osi")
asyn = require("asyn")

local function wait_motion_done(port, count_before, path_stop_pv, safety_pv)
    local timeout = 300.0
    local t0 = os.time()
    while true do
        local elap = os.time() - t0
        if elap >= timeout then
            print("Error: Timeout exceeded waiting for motion to complete")
            break
        end
        if asyn.getIntegerParam(port, "MOTION_DONE_COUNT") ~= count_before then
            break
        end
        if epics.get(path_stop_pv) == 1 then break end
        if safety_pv and epics.get(safety_pv) ~= 1 then
            epics.put(path_stop_pv, 1)
            break
        end
        osi.sleep(0.01)
    end
end

-- Moves robot through all enabled points along path
function path_go(args)

    local path_stop_pv = string.format("%sControl:stop_path", args.prefix, args.N)
    local safety_pv = string.format("%sReceive:SafetyStatusBits", args.prefix)
    local sync_joint_disa_pv = string.format("%sControl:sync_joint_cmds.DISA", args.prefix)
    local sync_pose_disa_pv = string.format("%sControl:sync_pose_cmds.DISA", args.prefix)

    -- disable automatic syncing of joint and pose command values
    epics.put(sync_joint_disa_pv, 1)
    epics.put(sync_pose_disa_pv, 1)

    epics.put(path_stop_pv, 0)
    local stopped = false

    for i = 1,args.kmax do

        -- check if Path$(N):Stop is called
        local path_stop_val = epics.get(path_stop_pv)
        if path_stop_val == 1 then
            stopped = true
            break
        end

        local wp_type = epics.get(string.format("%sPath%d:%d:Type", args.prefix, args.N, i))
        wp_type = (wp_type == 0.0) and "L" or "J"
        local wp_num = epics.get(string.format("%sPath%d:%d:Number", args.prefix, args.N, i))
        local wp = string.format("%sWaypoint%s:%d", args.prefix, wp_type, wp_num)
        local wp_action = math.floor(epics.get(string.format("%sPath%d:%d:ActionOverride", args.prefix, args.N, i)))
        local wp_enabled = epics.get(string.format("%sPath%d:%d:Enabled", args.prefix, args.N, i)) ~= 0
        if (wp_num <= 0) then
            wp_enabled = false
        end

        if wp_enabled then
            -- Apply action override
            local action_pv
            local wp_action0
            local action_opt_val_pv
            local set_action_opt_lnk0_pv
            local set_action_opt_proc_pv
            if wp_action ~= 0 then
                action_pv = string.format("%s:ActionOpt", wp)
                wp_action0 = math.floor(epics.get(action_pv))
                action_opt_val_pv = string.format("%sPath%d:%d:action_opt_val", args.prefix, args.N, i)
                set_action_opt_lnk0_pv = string.format("%sPath%d:%d:set_action_opt.LNK0", args.prefix, args.N, i)
                set_action_opt_proc_pv = string.format("%sPath%d:%d:set_action_opt.PROC", args.prefix, args.N, i)
                epics.put(action_opt_val_pv, wp_action)
                epics.put(set_action_opt_lnk0_pv, string.format("%s PP",action_pv))
                epics.put(set_action_opt_proc_pv, 1) -- manually force processing
            end

            -- Snapshot counter, trigger move, wait for counter to change
            print(string.format("Moving to waypoint %s...", wp))
            local move_pv = string.format("%s:move%s.PROC", wp, wp_type)
            local path_busy_pv = string.format("%sPath%d:%d:Busy", args.prefix, args.N, i)
            local count = asyn.getIntegerParam(args.port, "MOTION_DONE_COUNT")
            epics.put(path_busy_pv, 1)
            epics.put(move_pv, 1)
            wait_motion_done(args.port, count, path_stop_pv, safety_pv)
            epics.put(path_busy_pv, 0)

            -- Restore original action
            if wp_action ~= 0 then
                epics.put(action_opt_val_pv, wp_action0)
                epics.put(set_action_opt_lnk0_pv, string.format("%s PP",action_pv))
                epics.put(set_action_opt_proc_pv, 1) -- manually force processing
            end
        end
    end
    if stopped then
        print(string.format("Path %d stopped", args.N))
    else
        print(string.format("Path %d completed!", args.N))
    end

    -- re-enable automatic syncing of joint and pose command values
    epics.put(sync_joint_disa_pv, 0)
    epics.put(sync_pose_disa_pv, 0)
end


function get_wp_info(args)
    local wp_type = epics.get(string.format("%sPath%d:%d:Type", args.prefix, args.N, args.k))
    wp_type = (wp_type == 0.0) and "L" or "J"
    local wp_num = epics.get(string.format("%sPath%d:%d:Number", args.prefix, args.N, args.k))
    if wp_num > 0 then
        local override_action_num = epics.get(string.format("%sPath%d:%d:ActionOverride", args.prefix, args.N, args.k))
        local action_desc
        if override_action_num == 0 then
            action_desc = epics.get(string.format("%sWaypoint%s:%d:ActionDesc", args.prefix, wp_type, wp_num))
        else
            action_desc = epics.get(string.format("%sActionSseq%d.DESC", args.prefix, override_action_num))
        end
        epics.put(string.format("%sPath%d:%d:ActionDesc", args.prefix, args.N, args.k), action_desc)

        local wp_pv = string.format("%sWaypoint%s:%d", args.prefix, wp_type, wp_num)
        local reached_inp_pv = string.format("%sPath%d:%d:Reached.INP",args.prefix,args.N,args.k)
        local wp_reached_pv = string.format("%s:Reached CP",wp_pv)
        epics.put(reached_inp_pv, wp_reached_pv) -- set link to Waypoint:Reached PV
        return epics.get(wp_pv)
    else
        epics.put(string.format("%sPath%d:%d:ActionDesc", args.prefix, args.N, args.k), "")
        return ""
    end
end
