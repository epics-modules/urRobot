local robot = require("ur_robot")
local osi = require("osi")

local function print_table(tab)
    io.write("[")
    for i,v in ipairs(tab) do
        if i == #tab then
            io.write(string.format("%.4f]\n", v))
        else
            io.write(string.format("%.4f,", v))
        end
    end
end

local function test()
    robot.moveJ({-100, -100, -100, -50, 90, 0})
    io.write("Joints = ")
    print_table(robot.get_joints())

    robot.moveJ({-90, -90, -90, -50, 90, 0})
    io.write("Joints = ")
    print_table(robot.get_joints())

    robot.moveL({-140, -500, 570, 0.0, 2.95, -1.07})
    io.write("Pose = ")
    print_table(robot.get_pose())

    robot.moveL({-133, -532, 575, 0.0, 2.95, -1.07})
    io.write("Pose = ")
    print_table(robot.get_pose())
end

robot.register("urExample:tst", test)
