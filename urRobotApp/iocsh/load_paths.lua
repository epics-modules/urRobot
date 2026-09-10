local db = require("db")

NPATHS = tonumber(N)
KMAX = tonumber(K)
for n = 1, NPATHS do
    for k = 1, KMAX do
        local macros = string.format("P=%s,N=%d,K=%d", P, n, k)
        dbLoadRecords("$(URROBOT)/urRobotApp/Db/path_waypoint.db", macros)
    end
end
for n = 1, NPATHS do
    db.record("stringout", P .. "Path" .. n) {
        DESC = "Description of path",
        VAL = "Path " .. n
    }
    db.record("bo", P .. "Path" .. n .. ":Go") {
        ZNAM = "Idle",
        ONAM = "Go",
    }
    luaLoadFile("paths_seq.lua", string.format("P=%s,N=%d,KMAX=%d,CTRL_PORT=%s,STATE=path_seq_%d", P, n, KMAX, CTRL_PORT, n))
end
