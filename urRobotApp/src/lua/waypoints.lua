-- NOTE: IOC must add $(URROBOT)/urRobotApp/src/lua to LUA_SCRIPT_PATH
epics = require("epics")

-- Returns true if two numbers are within the specified tolerance of each other
local function almost_equal(a, b, tol)
    tol = tol or 1e-6
    if (math.abs(a - b) <= tol) then
        return true
    else
        return false
    end
end


-- Returns true if two tables have equal elements within the tolerance
function table.almost_equal(t1, t2, tol)
    tol = tol or 1e-6
    assert(#t1 == #t2, "Tables must be the same length")

    for i, _ in ipairs(t1) do
        if not almost_equal(t1[i], t2[i], tol) then
            return false
        end
    end
    return true
end

local function joint_waypoint_reached(prefix, num)
    local actual_pose = {
        epics.get(string.format("%sReceive:Joint1", prefix)),
        epics.get(string.format("%sReceive:Joint2", prefix)),
        epics.get(string.format("%sReceive:Joint3", prefix)),
        epics.get(string.format("%sReceive:Joint4", prefix)),
        epics.get(string.format("%sReceive:Joint5", prefix)),
        epics.get(string.format("%sReceive:Joint6", prefix)),
    }

    local waypoint_base = string.format("%sWaypointJ:%d", prefix, num)
    local waypoint_pose = {
        epics.get(string.format("%s:J1",waypoint_base)),
        epics.get(string.format("%s:J2",waypoint_base)),
        epics.get(string.format("%s:J3",waypoint_base)),
        epics.get(string.format("%s:J4",waypoint_base)),
        epics.get(string.format("%s:J5",waypoint_base)),
        epics.get(string.format("%s:J6",waypoint_base))
    }

    local reached = table.almost_equal(waypoint_pose, actual_pose, 0.1)
    return reached and 1 or 0
end

local function cartesian_waypoint_reached(prefix, num)
    local actual_xyz = {
        epics.get(string.format("%sReceive:PoseX", prefix)),
        epics.get(string.format("%sReceive:PoseY", prefix)),
        epics.get(string.format("%sReceive:PoseZ", prefix)),
    }
    local actual_rotvec = {
        epics.get(string.format("%sReceive:PoseRx", prefix)),
        epics.get(string.format("%sReceive:PoseRy", prefix)),
        epics.get(string.format("%sReceive:PoseRz", prefix)),
    }
    local waypoint_base = string.format("%sWaypointL:%d", prefix, num)
    local waypoint_xyz = {
        epics.get(string.format("%s:X", waypoint_base)),
        epics.get(string.format("%s:Y", waypoint_base)),
        epics.get(string.format("%s:Z", waypoint_base)),
    }
    local waypoint_rotvec = {
        epics.get(string.format("%s:Rx", waypoint_base)),
        epics.get(string.format("%s:Ry", waypoint_base)),
        epics.get(string.format("%s:Rz", waypoint_base)),
    }
    local reached_xyz = table.almost_equal(waypoint_xyz, actual_xyz, 1.0)
    local reached_rpy = table.almost_equal(waypoint_rotvec, actual_rotvec, 0.1)
    return (reached_xyz and reached_rpy) and 1 or 0
end

function active_reached(args)
    local wp_type = A
    local wp_num = B
    local wp_reached_pv = AA

    if wp_num <= 0 then return 0 end

    local reached = 0
    if wp_type == 0 then
        reached = joint_waypoint_reached(args.prefix, wp_num)
    else
        reached = cartesian_waypoint_reached(args.prefix, wp_num)
    end
    epics.put(wp_reached_pv, reached)

    return reached
end

function clear_reached()
    local wp_num = A
    if wp_num <= 0 then return end
    local wp_reached_pv = AA
    epics.put(wp_reached_pv, 0)
end
