local robot = require("robot")

local function test_moves()
    robot.moveJ({-100, -100, -100, -50, 90, 0})
    robot.moveJ({-90, -90, -90, -50, 90, 0})
    robot.moveL({-140, -500, 570, 0.0, 2.95, -1.07})
    robot.moveL({-133, -532, 575, 0.0, 2.95, -1.07})
end

robot.register("urExample:tst", test_moves)
