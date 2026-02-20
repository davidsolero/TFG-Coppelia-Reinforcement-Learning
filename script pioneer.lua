--[[
================================================================================
ROBOT NAVIGATION SCRIPT v2 - CONTROLADO POR PYTHON
================================================================================

CAMBIOS RESPECTO A v1:
- El bucle principal ya NO elige acciones aleatorias
- Espera comandos de Python via señales
- Python envía: robot_command = "goTo:Hab1", "goTo:C", "stop", etc.
- Lua ejecuta y responde con: robot_status = "idle", "moving", "charging", "arrived", "error"

================================================================================
--]]

--[[
================================================================================
CONFIGURATION PARAMETERS
================================================================================
--]]

local CONFIG = {
    VELOCITY = 0.5,
    ANGULAR_VELOCITY = 1.57,
    RECHARGE_RADIUS = 0.5,
    REALIGN_THRESHOLD = 0.05,
    
    -- BATERÍA - TIEMPOS DE PRUEBA (para testing rápido)
    DISCHARGE_PLATEAU_DURATION = 30,  -- 30 segundos de meseta
    DISCHARGE_FALL_DURATION = 20,     -- 20 segundos de caída
    CHARGE_DURATION = 15,             -- 15 segundos para cargar
    
    -- BATERÍA - TIEMPOS REALISTAS (descomentar para uso real)
    -- DISCHARGE_PLATEAU_DURATION = 1800,
    -- DISCHARGE_FALL_DURATION = 300,
    -- CHARGE_DURATION = 3600,
    
    LOG_LEVEL = 1,
    BATTERY_REPORT_INTERVAL = 10,
}

--[[
================================================================================
SCRIPT PRINCIPAL
================================================================================
--]]

function sysCall_init()
    sim = require('sim')
    objectToFollowPath = sim.getObjectHandle('/PioneerP3DX')
    robot = sim.getObjectHandle('/PioneerP3DX')

    -- Aplicar configuración
    velocity = CONFIG.VELOCITY
    angularVelocity = CONFIG.ANGULAR_VELOCITY
    rechargeRadius = CONFIG.RECHARGE_RADIUS
    realignThreshold = CONFIG.REALIGN_THRESHOLD
    dischargePhase1Duration = CONFIG.DISCHARGE_PLATEAU_DURATION
    dischargePhase2Duration = CONFIG.DISCHARGE_FALL_DURATION
    chargePhase1Duration = CONFIG.CHARGE_DURATION
    logLevel = CONFIG.LOG_LEVEL
    batteryReportInterval = CONFIG.BATTERY_REPORT_INTERVAL
    
    sim.setStepping(true)

    -- Time tracking
    previousSimulationTime = sim.getSimulationTime()

    -- Battery system
    battery = 100.0
    lastBatteryReport = 100.0
    batteryDepleted = false
    batteryAtChargeStart = 100.0
    
    -- Battery state tracking
    isCharging = false
    timeInCurrentPhase = 0
    batteryPhase = 'discharge_plateau'
    
    -- Charging station
    dummyHandle = nil
    local ok, h = pcall(sim.getObjectHandle, 'C')
    if ok and type(h) == 'number' then
        dummyHandle = h
        logMessage(1, 'INIT', 'Charging station found: C')
    else
        logMessage(0, 'ERROR', 'Charging station "C" not found!')
    end

    -- Nodes in the scene
    nodes = {}
    local nodeNames = {'R', 'Hab1', 'Hab2', 'Hab3', 'C'}
    for _, nodeName in ipairs(nodeNames) do
        local ok, h = pcall(sim.getObjectHandle, nodeName)
        if ok and type(h) == 'number' then
            nodes[nodeName] = h
        end
    end
    logMessage(1, 'INIT', 'Loaded ' .. tableLength(nodes) .. ' nodes')

    -- Bidirectional path graph
    graph = {
        R = {Hab1 = '/Path_R_Hab1', Hab2 = '/Path_R_Hab2', Hab3 = '/Path_R_Hab3', C = '/Path_R_C'},
        Hab1 = {R = '/Path_R_Hab1', Hab2 = '/PathHab1_Hab2', Hab3 = '/PathHab1_Hab3', C = '/PathHab1_C'},
        Hab2 = {R = '/Path_R_Hab2', Hab1 = '/PathHab1_Hab2', Hab3 = '/PathHab2_Hab3', C = '/PathHab2_C'},
        Hab3 = {R = '/Path_R_Hab3', Hab1 = '/PathHab1_Hab3', Hab2 = '/PathHab2_Hab3', C = '/PathHab3_C'},
        C = {R = '/Path_R_C', Hab1 = '/PathHab1_C', Hab2 = '/PathHab2_C', Hab3 = '/PathHab3_C'}
    }
    
    -- Current node tracking
    currentNode = 'R'
    
    -- Command system
    sim.setStringSignal('robot_command', '')
    sim.setStringSignal('robot_status', 'idle')
    
    logMessage(1, 'INIT', 'Robot starting at node: ' .. currentNode)
    logMessage(1, 'INIT', 'Waiting for Python commands...')
end

-- =================================
-- LOGGING SYSTEM
-- =================================

function logMessage(level, category, message)
    if level <= logLevel then
        local timeStr = string.format('[%.1fs]', sim.getSimulationTime())
        local prefix = ''
        
        if category == 'ERROR' then
            prefix = '!!! '
        elseif category == 'ACTION' then
            prefix = '>>> '
        elseif category == 'BATTERY' then
            prefix = '[BAT] '
        elseif category == 'NAV' then
            prefix = '[NAV] '
        elseif category == 'INIT' then
            prefix = '[INIT] '
        elseif category == 'CMD' then
            prefix = '[CMD] '
        elseif category == 'DEBUG' then
            prefix = '[DBG] '
        end
        
        sim.addStatusbarMessage(timeStr .. ' ' .. prefix .. message)
    end
end

function tableLength(t)
    local count = 0
    for _ in pairs(t) do count = count + 1 end
    return count
end

-- =================================
-- REMOTE API INTERFACE
-- =================================

function publishState()
    sim.setInt32Signal('robot_battery', math.floor(battery))
    sim.setStringSignal('robot_currentNode', currentNode or 'unknown')
    sim.setStringSignal('robot_batteryPhase', batteryPhase)
    sim.setInt32Signal('robot_isCharging', isCharging and 1 or 0)
    sim.setInt32Signal('robot_batteryDepleted', batteryDepleted and 1 or 0)
end

function setStatus(status)
    sim.setStringSignal('robot_status', status)
end

function getCommand()
    local cmd = sim.getStringSignal('robot_command')
    return cmd
end

function clearCommand()
    sim.setStringSignal('robot_command', '')
end

-- =================================
-- UTILITY FUNCTIONS
-- =================================

function normalizeAngle(a)
    while a > math.pi do a = a - 2*math.pi end
    while a < -math.pi do a = a + 2*math.pi end
    return a
end

function updateBattery(dt)
    if dt <= 0 then return end
    
    timeInCurrentPhase = timeInCurrentPhase + dt
    
    if isCharging then
        if batteryPhase == 'charge_rise' then
            local remainingCharge = 100 - batteryAtChargeStart
            local timeToFull = chargePhase1Duration * (remainingCharge / 100)
            
            if timeToFull > 0 then
                local progress = math.min(timeInCurrentPhase / timeToFull, 1.0)
                battery = batteryAtChargeStart + remainingCharge * progress
            else
                battery = 100
            end
            
            if battery >= 100 then
                battery = 100
                batteryPhase = 'charge_plateau'
                timeInCurrentPhase = 0
                logMessage(2, 'BATTERY', 'Charge complete - entering plateau')
            end
            
        elseif batteryPhase == 'charge_plateau' then
            battery = 100
        end
        
    else
        if batteryPhase == 'discharge_plateau' then
            battery = 100
            
            if timeInCurrentPhase >= dischargePhase1Duration then
                batteryPhase = 'discharge_fall'
                timeInCurrentPhase = 0
                logMessage(2, 'BATTERY', 'Plateau ended - starting discharge')
            end
            
        elseif batteryPhase == 'discharge_fall' then
            local progress = timeInCurrentPhase / dischargePhase2Duration
            battery = 100 * (1 - progress)
            
            if battery <= 0 then
                battery = 0
                batteryPhase = 'discharge_depleted'
            end
            
        elseif batteryPhase == 'discharge_depleted' then
            battery = 0
        end
    end
    
    local currentTier = math.floor(battery / batteryReportInterval) * batteryReportInterval
    local lastTier = math.floor(lastBatteryReport / batteryReportInterval) * batteryReportInterval
    if currentTier ~= lastTier then
        logMessage(1, 'BATTERY', string.format('%d%% [%s]', currentTier, batteryPhase))
        lastBatteryReport = battery
    end
    
    if battery <= 0 and not batteryDepleted then
        batteryDepleted = true
        setStatus('depleted')
        logMessage(0, 'ERROR', 'BATTERY DEPLETED!')
    end
end

function startCharging()
    if isCharging then return end
    
    isCharging = true
    batteryAtChargeStart = battery
    
    if battery >= 100 then
        batteryPhase = 'charge_plateau'
        timeInCurrentPhase = 0
    else
        batteryPhase = 'charge_rise'
        timeInCurrentPhase = 0
    end
    
    logMessage(1, 'BATTERY', string.format('Charging started at %.1f%%', battery))
end

function stopCharging()
    if not isCharging then return end
    
    isCharging = false
    
    if battery >= 100 then
        batteryPhase = 'discharge_plateau'
        timeInCurrentPhase = 0
    elseif battery > 0 then
        batteryPhase = 'discharge_fall'
        timeInCurrentPhase = dischargePhase2Duration * (1 - battery / 100)
    else
        batteryPhase = 'discharge_depleted'
        timeInCurrentPhase = 0
    end
    
    logMessage(1, 'BATTERY', string.format('Charging stopped at %.1f%%', battery))
end

function tryRecharge()
    if not dummyHandle then return end
    
    local rpos = sim.getObjectPosition(robot, -1)
    local dpos = sim.getObjectPosition(dummyHandle, -1)
    local dx = rpos[1] - dpos[1]
    local dy = rpos[2] - dpos[2]
    local dist = math.sqrt(dx*dx + dy*dy)
    
    if dist <= rechargeRadius then
        if not isCharging then
            startCharging()
        end
    else
        if isCharging then
            stopCharging()
        end
    end
end

function isBatteryOk()
    return battery > 0 and not batteryDepleted
end

-- =================================
-- MOVEMENT FUNCTIONS
-- =================================

function rotateTo(targetYaw)
    local euler = sim.getObjectOrientation(robot, -1)
    local currentYaw = euler[3]
    local diff = normalizeAngle(targetYaw - currentYaw)
    
    previousSimulationTime = sim.getSimulationTime()
    
    while math.abs(diff) > 0.01 and not sim.getSimulationStopping() and isBatteryOk() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now
        
        updateBattery(dt)
        tryRecharge()
        publishState()
        
        if not isBatteryOk() then return false end
        
        local dir = (diff > 0) and 1 or -1
        local deltaYaw = dir * angularVelocity * dt
        if math.abs(deltaYaw) > math.abs(diff) then deltaYaw = diff end
        
        currentYaw = currentYaw + deltaYaw
        euler[3] = currentYaw
        sim.setObjectOrientation(robot, -1, euler)
        diff = normalizeAngle(targetYaw - currentYaw)
        sim.step()
    end
    
    return isBatteryOk()
end

function moveToPosition(targetPos)
    local robotPos = sim.getObjectPosition(robot, -1)
    local dx = targetPos[1] - robotPos[1]
    local dy = targetPos[2] - robotPos[2]
    local dist = math.sqrt(dx*dx + dy*dy)
    
    if dist < 0.01 then return true end
    
    local travelDistance = 0
    previousSimulationTime = sim.getSimulationTime()
    
    while travelDistance < dist and not sim.getSimulationStopping() and isBatteryOk() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now
        
        updateBattery(dt)
        tryRecharge()
        publishState()
        
        if not isBatteryOk() then return false end
        
        travelDistance = travelDistance + velocity * dt
        local progress = math.min(travelDistance / dist, 1.0)
        
        local newPos = {
            robotPos[1] + dx * progress,
            robotPos[2] + dy * progress,
            robotPos[3]
        }
        
        sim.setObjectPosition(robot, -1, newPos)
        sim.step()
    end
    
    return isBatteryOk()
end

function realignToNode(nodeName)
    if not nodes[nodeName] then return true end
    
    local nodePos = sim.getObjectPosition(nodes[nodeName], -1)
    local robotPos = sim.getObjectPosition(robot, -1)
    
    local dx = nodePos[1] - robotPos[1]
    local dy = nodePos[2] - robotPos[2]
    local dist = math.sqrt(dx*dx + dy*dy)
    
    if dist < realignThreshold then return true end
    
    logMessage(2, 'NAV', string.format('Realigning to %s (offset: %.2fm)', nodeName, dist))
    
    local targetYaw = math.atan2(dy, dx)
    if not rotateTo(targetYaw) then return false end
    
    if not moveToPosition(nodePos) then return false end
    
    local nodeOrient = sim.getObjectOrientation(nodes[nodeName], -1)
    sim.setObjectOrientation(robot, -1, nodeOrient)
    
    return true
end

function followPath(pathName)
    if not isBatteryOk() then return false end
    
    local pathHandle = nil
    local success = pcall(function()
        pathHandle = sim.getObjectHandle(pathName)
    end)
    if not success or not pathHandle then 
        logMessage(0, 'ERROR', 'Path not found: ' .. pathName)
        return false 
    end
    
    local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(pathHandle, 'PATH'))
    local m = Matrix(#pathData // 7, 7, pathData)
    local pathPositions = m:slice(1,1,m:rows(),3):data()

    local robotPos = sim.getObjectPosition(robot, pathHandle)
    local startPos = {pathPositions[1], pathPositions[2], pathPositions[3]}
    local endPos = {pathPositions[#pathPositions-2], pathPositions[#pathPositions-1], pathPositions[#pathPositions]}

    local function distXY(a,b) return math.sqrt((a[1]-b[1])^2 + (a[2]-b[2])^2) end
    local dStart = distXY(robotPos, startPos)
    local dEnd = distXY(robotPos, endPos)

    if dEnd < dStart then
        local reversed = {}
        for i=#pathPositions,1,-3 do
            table.insert(reversed, pathPositions[i-2])
            table.insert(reversed, pathPositions[i-1])
            table.insert(reversed, pathPositions[i])
        end
        pathPositions = reversed
    end

    local robotStartPos = sim.getObjectPosition(robot, -1)
    local offset = {
        robotStartPos[1]-pathPositions[1], 
        robotStartPos[2]-pathPositions[2], 
        robotStartPos[3]-pathPositions[3]
    }
    for i=1,#pathPositions,3 do
        pathPositions[i] = pathPositions[i] + offset[1]
        pathPositions[i+1] = pathPositions[i+1] + offset[2]
        pathPositions[i+2] = pathPositions[i+2] + offset[3]
    end

    local dx = pathPositions[4] - pathPositions[1]
    local dy = pathPositions[5] - pathPositions[2]
    local targetYaw = math.atan2(dy, dx)
    if not rotateTo(targetYaw) then return false end

    local pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)
    local posAlongPath = 0
    previousSimulationTime = sim.getSimulationTime()

    while posAlongPath < totalLength and not sim.getSimulationStopping() and isBatteryOk() do
        local t = sim.getSimulationTime()
        local dt = t - previousSimulationTime
        previousSimulationTime = t

        updateBattery(dt)
        tryRecharge()
        publishState()
        
        if not isBatteryOk() then return false end

        posAlongPath = posAlongPath + velocity*dt
        if posAlongPath > totalLength then posAlongPath = totalLength end

        local pos = sim.getPathInterpolatedConfig(pathPositions, pathLengths, posAlongPath)
        local aheadPos = sim.getPathInterpolatedConfig(pathPositions, pathLengths, math.min(posAlongPath+0.02, totalLength))
        
        local dx = aheadPos[1]-pos[1]
        local dy = aheadPos[2]-pos[2]
        local yaw = (math.abs(dx)+math.abs(dy) > 1e-6) and math.atan2(dy,dx) or sim.getObjectOrientation(objectToFollowPath,-1)[3]
        local cy = math.cos(yaw*0.5)
        local sy = math.sin(yaw*0.5)
        local quat = {0,0,sy,cy}

        sim.setObjectPosition(objectToFollowPath,-1,pos)
        sim.setObjectQuaternion(objectToFollowPath,-1,quat)
        sim.step()
    end
    
    return isBatteryOk()
end

-- =================================
-- PATH PLANNING
-- =================================

function getClosestNode()
    local robotPos = sim.getObjectPosition(robot,-1)
    local minDist, closest = math.huge, nil
    for node, handle in pairs(nodes) do
        local pos = sim.getObjectPosition(handle, -1)
        local d = math.sqrt((robotPos[1]-pos[1])^2 + (robotPos[2]-pos[2])^2)
        if d < minDist then
            minDist = d
            closest = node
        end
    end
    return closest
end

function getPathLength(pathName)
    local success, result = pcall(function()
        local path = sim.getObjectHandle(pathName)
        local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(path,'PATH'))
        local m = Matrix(#pathData//7,7,pathData)
        local positions = m:slice(1,1,m:rows(),3):data()
        local length = 0
        for i=1,#positions-3,3 do
            local dx = positions[i+3]-positions[i]
            local dy = positions[i+4]-positions[i+1]
            local dz = positions[i+5]-positions[i+2]
            length = length + math.sqrt(dx*dx+dy*dy+dz*dz)
        end
        return length
    end)
    return success and result or 999999
end

function findShortestPath(startNode, targetNode)
    local dist, prev, Q = {}, {}, {}
    for node,_ in pairs(graph) do
        dist[node] = math.huge
        prev[node] = nil
        Q[node] = true
    end
    dist[startNode] = 0

    while next(Q) do
        local u, minDist = nil, math.huge
        for node,_ in pairs(Q) do
            if dist[node] < minDist then u, minDist = node, dist[node] end
        end

        if u == targetNode then break end
        Q[u] = nil

        for neighbor, pathName in pairs(graph[u]) do
            if Q[neighbor] then
                local alt = dist[u] + getPathLength(pathName)
                if alt < dist[neighbor] then
                    dist[neighbor] = alt
                    prev[neighbor] = u
                end
            end
        end
    end

    local path = {}
    local node = targetNode
    while node do
        table.insert(path, 1, node)
        node = prev[node]
    end
    return path
end

-- =================================
-- HIGH-LEVEL ACTIONS
-- =================================

function goTo(targetNode)
    if not isBatteryOk() then 
        setStatus('depleted')
        return false 
    end
    
    if not nodes[targetNode] then
        logMessage(0, 'ERROR', 'Unknown node: ' .. targetNode)
        setStatus('error')
        return false
    end
    
    if not currentNode then
        currentNode = getClosestNode()
    end
    
    if currentNode == targetNode then
        logMessage(1, 'NAV', 'Already at ' .. targetNode)
        setStatus('arrived')
        return true
    end
    
    setStatus('moving')
    
    local nodePath = findShortestPath(currentNode, targetNode)
    if #nodePath == 0 then
        logMessage(0, 'ERROR', 'No path found to ' .. targetNode)
        setStatus('error')
        return false
    end
    
    logMessage(1, 'NAV', 'Route: ' .. table.concat(nodePath, ' -> '))
    
    if not realignToNode(currentNode) then
        logMessage(1, 'NAV', 'Aborted during initial realignment')
        setStatus('depleted')
        return false
    end
    
    for i = 1, #nodePath-1 do
        if not isBatteryOk() then
            logMessage(1, 'NAV', 'Aborted - battery depleted')
            setStatus('depleted')
            return false
        end
        
        local fromNode = nodePath[i]
        local toNode = nodePath[i+1]
        local pathName = graph[fromNode] and graph[fromNode][toNode]
        
        if not pathName then
            logMessage(0, 'ERROR', 'Path not found: ' .. fromNode .. ' -> ' .. toNode)
            setStatus('error')
            return false
        end
        
        logMessage(2, 'NAV', 'Following path: ' .. fromNode .. ' -> ' .. toNode)
        
        if not followPath(pathName) then
            logMessage(1, 'NAV', 'Path following interrupted')
            setStatus('depleted')
            return false
        end
        
        currentNode = toNode
        publishState()
    end
    
    if not realignToNode(targetNode) then
        logMessage(1, 'NAV', 'Aborted during final realignment')
        setStatus('depleted')
        return false
    end
    
    logMessage(1, 'NAV', 'Arrived at ' .. targetNode)
    setStatus('arrived')
    return true
end

function actionStop(duration)
    if not isBatteryOk() then 
        setStatus('depleted')
        return false 
    end
    
    duration = duration or 1
    logMessage(1, 'ACTION', 'Stop for ' .. duration .. ' seconds')
    setStatus('stopped')
    
    local endTime = sim.getSimulationTime() + duration
    
    while sim.getSimulationTime() < endTime and not sim.getSimulationStopping() and isBatteryOk() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now
        
        updateBattery(dt)
        tryRecharge()
        publishState()
        sim.step()
    end
    
    if isBatteryOk() then
        setStatus('idle')
    else
        setStatus('depleted')
    end
    
    return isBatteryOk()
end

function callOperator()
    logMessage(1, 'ACTION', 'OPERATOR CALLED - Teleporting to charging station')
    setStatus('operator_called')
    
    if not dummyHandle then
        logMessage(0, 'ERROR', 'Charging station not found!')
        setStatus('error')
        return
    end
    
    local chargePos = sim.getObjectPosition(dummyHandle, -1)
    local chargeOrient = sim.getObjectOrientation(dummyHandle, -1)
    
    sim.setObjectPosition(robot, -1, chargePos)
    sim.setObjectOrientation(robot, -1, chargeOrient)
    
    currentNode = 'C'
    
    startCharging()
    batteryDepleted = false
    previousSimulationTime = sim.getSimulationTime()
    
    setStatus('charging')
    logMessage(1, 'BATTERY', 'Charging until full...')
    
    while battery < 100.0 and not sim.getSimulationStopping() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now
        
        sim.setObjectPosition(robot, -1, chargePos)
        sim.setObjectOrientation(robot, -1, chargeOrient)
        
        updateBattery(dt)
        publishState()
        sim.step()
    end
    
    setStatus('idle')
    logMessage(1, 'ACTION', 'Charging complete - Ready for new commands')
end

-- =================================
-- COMMAND PARSER
-- =================================

function parseAndExecuteCommand(cmd)
    if not cmd or cmd == '' then
        return false
    end
    
    logMessage(1, 'CMD', 'Received: ' .. cmd)
    clearCommand()
    
    -- Parse command
    local action, param = cmd:match('([^:]+):?(.*)')
    
    if action == 'goTo' and param ~= '' then
        return goTo(param)
        
    elseif action == 'stop' then
        local duration = tonumber(param) or 1
        return actionStop(duration)
        
    elseif action == 'callOperator' then
        callOperator()
        return true
        
    else
        logMessage(0, 'ERROR', 'Unknown command: ' .. cmd)
        setStatus('error')
        return false
    end
end

-- =================================
-- MAIN LOOP - ESPERA COMANDOS DE PYTHON
-- =================================

function sysCall_thread()
    logMessage(1, 'INIT', '=== SIMULATION STARTED ===')
    logMessage(1, 'INIT', '=== WAITING FOR PYTHON COMMANDS ===')
    
    while not sim.getSimulationStopping() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now
        
        -- Siempre actualizar batería y estado
        updateBattery(dt)
        tryRecharge()
        publishState()
        
        -- Verificar si hay batería
        if batteryDepleted then
            -- Esperar comando callOperator de Python o hacerlo automático
            local cmd = getCommand()
            if cmd == 'callOperator' then
                clearCommand()
                callOperator()
            end
        else
            -- Verificar si hay comando pendiente
            local cmd = getCommand()
            if cmd and cmd ~= '' then
                parseAndExecuteCommand(cmd)
            end
        end
        
        sim.step()
    end
    
    logMessage(1, 'INIT', '=== SIMULATION ENDED ===')
end