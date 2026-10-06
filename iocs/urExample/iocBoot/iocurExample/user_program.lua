local robot = require("robot")

local function test_moves()
    robot.moveJ({-100, -100, -100, -50, 90, 0})
    robot.moveJ({-90, -90, -90, -50, 90, 0})
end

local function greet()
    print("Hello world!")
end

robot.register("urExample:tst", test_moves)
robot.register("urExample:hello", greet)
