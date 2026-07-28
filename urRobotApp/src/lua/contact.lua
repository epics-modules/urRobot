local epics = require("epics")
local osi = require("osi")

-- Moves until contact
-- Moves towards the target (ControlTargetX,Y,Z) and checks for contact of
-- the end-effector against an object
function muc(args)

    -- Set motion target
    -- Provided via input links
    epics.put(args.P .. "Control:PoseXCmd", A)
    epics.put(args.P .. "Control:PoseYCmd", B)
    epics.put(args.P .. "Control:PoseZCmd", C)
    epics.put(args.P .. "Control:PoseRxCmd", D)
    epics.put(args.P .. "Control:PoseRyCmd", E)
    epics.put(args.P .. "Control:PoseRzCmd", F)

    -- Clear stop flag
    epics.put(args.P .. "Control:StopContactDetect", 0)

    -- Start motion
    epics.put(args.P .. "Control:moveL", 1)
    osi.sleep(0.1)
    if epics.get(args.P .. "Control:AsyncMoveDone") == 1 then
        print("MoveUntilContact: Motion failed to start")
        epics.put(args.P .. "Control:ContactError", 2)
        epics.put(args.P .. "Control:MoveUntilContact", 0)
        return
    end

    -- Start contact detection
    epics.put(args.P .. "Control:StartContactDetect", 1)
    osi.sleep(0.1) -- wait until the contact detected flag is cleared

    -- Timeout provided via INPG
    local timeout = G
    local t0 = os.time()
    local success = true

    while true do

        -- If we are in a non "Normal" safety mode, abort
        if epics.get(args.P .. "Receive:SafetyStatusBits") ~= 1 then
            epics.put(args.P .. "Control:StopContactDetect", 1)
            epics.put(args.P .. "Control:Stop", 1)
            epics.put(args.P .. "Control:ContactError", 2)
            success = false
            break
        end

        -- Check timeout
        local elap = os.time() - t0
        if elap >= timeout then
            print("MoveUntilContact: Timeout exceeded waiting for contact")
            epics.put(args.P .. "Control:StopContactDetect", 1)
            epics.put(args.P .. "Control:Stop", 1)
            epics.put(args.P .. "Control:ContactError", 1)
            success = false
            break
        end

        -- Stop detection if requested
        if epics.get(args.P .. "Control:StopContactDetect") == 1 then
            print("MoveUntilContact: Stop requested")
            epics.put(args.P .. "Control:Stop", 1)
            epics.put(args.P .. "Control:ContactError", 2)
            success = false
            break;
        end

        -- Stop if robot has stopped for any other reason
        if epics.get(args.P .. "Control:Moving") == 0 then
            print("MoveUntilContact: Stopping contact detection. Robot is stopped")
            epics.put(args.P .. "Control:StopContactDetect", 1)
            epics.put(args.P .. "Control:ContactError", 2)
            success = false
            break
        end

        -- Break out if contact detected
        if epics.get(args.P .. "Control:ReadContactDetect") == 1 then
            print("MoveUntilContact: Contact detected")
            epics.put(args.P .. "Control:Stop", 1)
            success = true
            break;
        end
        osi.sleep(0.1)
    end

    -- Get the TCP pose at contact position
    if success then
        epics.put(args.P .. "Control:ContactError", 0)
        epics.put(args.P .. "Control:ContactPose.PROC", 1)
    end

    -- Clear busy
    epics.put(args.P .. "Control:MoveUntilContact", 0)
end
