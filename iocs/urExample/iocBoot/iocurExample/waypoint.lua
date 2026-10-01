local epics = require("epics")

local Space = {
    JOINT = 0,
    CARTESIAN = 1,
}

function change_space(args)
    local P = args.P
    local N = args.N
    local waypoint = string.format("%sWaypoint:%s:", P, N)
    local new_space = A == 0 and Space.JOINT or Space.CARTESIAN
    local last_space = AA == "deg" and Space.JOINT or Space.CARTESIAN

    if last_space == new_space then
        return
    end

    local request
    local units

    if new_space == Space.CARTESIAN then
        request = waypoint .. "request_fk_.PROC"
        units = {"mm", "mm", "mm", "rad", "rad", "rad"}
    else
        request = waypoint .. "request_ik_.PROC"
        units = {"deg", "deg", "deg", "deg", "deg", "deg"}
    end

    epics.put(request, 1)

    for i = 1, 6 do
        epics.put(string.format("%sP%d.EGU", waypoint, i - 1), units[i])
    end
end
