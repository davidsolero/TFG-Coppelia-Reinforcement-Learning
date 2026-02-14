function sysCall_init()
    sim = require('sim')
    objectToFollowPath = sim.getObjectHandle('/PioneerP3DX')
    robot = sim.getObjectHandle('/PioneerP3DX')

    velocity = 0.5 -- m/s (máximo ~0.5 m/s en terreno plano) --velocidades realistas supongo??
    angularVelocity = 1.57 -- rad/s (~90 grados/s)
    sim.setStepping(true)

    -- Time tracking
    previousSimulationTime = sim.getSimulationTime()

    -- Battery system (modelo de carga/descarga realista)
    battery = 100.0
    lastBatteryReport = 100.0
    batteryDepleted = false
    
    -- Battery discharge model (2 phases)
    dischargePhase1Duration = 1800 -- 30 minutos en meseta al 100%
    dischargePhase2Duration = 300  -- 5 minutos de caída rápida de 100% a 0%
    
    -- Battery charge model (2 phases)
    chargePhase1Duration = 3600    -- 1 hora de subida de 0% a 100%
    
    -- Battery state tracking
    isCharging = false
    timeInCurrentPhase = 0  -- Tiempo acumulado en la fase actual
    batteryPhase = 'discharge_plateau'  -- 'discharge_plateau', 'discharge_fall', 'charge_rise', 'charge_plateau'
    
    -- Charging station (dummy C)
    dummyHandle = nil
    rechargeRadius = 0.5
    local ok, h = pcall(sim.getObjectHandle, 'C')
    if ok and type(h) == 'number' then
        dummyHandle = h
        sim.addStatusbarMessage('Found charging dummy: C')
    else
        sim.addStatusbarMessage('Charging dummy "C" not found')
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
        -- CHARGING MODEL
        if batteryPhase == 'charge_rise' then
            -- Phase 1: Rising from current battery to 100% 
            -- Calculate where we should be in the rise based on time
            local progress = timeInCurrentPhase / chargePhase1Duration
            local targetBattery = battery + (100 - battery) * progress
            
            if targetBattery >= 100 then
                battery = 100
                batteryPhase = 'charge_plateau'
                timeInCurrentPhase = 0
            else
                battery = targetBattery
            end
            
        elseif batteryPhase == 'charge_plateau' then
            -- Phase 2: Stay at 100%
            battery = 100
        end
        
    else
        -- DISCHARGING MODEL
        if batteryPhase == 'discharge_plateau' then
            -- Phase 1: Stay at 100% for 30 minutes
            battery = 100
            
            if timeInCurrentPhase >= dischargePhase1Duration then
                batteryPhase = 'discharge_fall'
                timeInCurrentPhase = 0
            end
            
        elseif batteryPhase == 'discharge_fall' then
            -- Phase 2: Fall from 100% to 0% in 5 minutes
            local progress = timeInCurrentPhase / dischargePhase2Duration
            battery = 100 * (1 - progress)
            
            if battery <= 0 then
                battery = 0
                batteryPhase = 'discharge_depleted'
            end
            
        elseif batteryPhase == 'discharge_depleted' then
            -- Phase 3: Stay at 0%
            battery = 0
        end
    end
    
    -- Report battery at multiples of 10
    local currentTier = math.floor(battery / 10) * 10
    local lastTier = math.floor(lastBatteryReport / 10) * 10
    if currentTier < lastTier then
        sim.addStatusbarMessage(string.format('Battery: %d%%', currentTier))
        lastBatteryReport = battery
    end
    
    -- Mark as depleted if battery runs out
    if battery <= 0 then
        if not batteryDepleted then
            batteryDepleted = true
            sim.addStatusbarMessage('!!! BATTERY DEPLETED !!!')
        end
    end
end

function startCharging()
    if isCharging then return end  -- Already charging
    
    isCharging = true
    
    -- Determine where to start in the charging model based on current battery
    if battery >= 100 then
        batteryPhase = 'charge_plateau'
        timeInCurrentPhase = 0
    else
        batteryPhase = 'charge_rise'
        -- Calculate how much time we need to reach 100% from current battery
        -- We want to continue the rise smoothly
        timeInCurrentPhase = 0
    end
    
    sim.addStatusbarMessage('Started charging from ' .. string.format('%.1f%%', battery))
end

function stopCharging()
    if not isCharging then return end  -- Not charging
    
    isCharging = false
    
    -- Determine where to start in the discharging model based on current battery
    if battery >= 100 then
        batteryPhase = 'discharge_plateau'
        timeInCurrentPhase = 0
    elseif battery > 0 then
        -- We're somewhere in the middle, start the fall phase
        batteryPhase = 'discharge_fall'
        -- Calculate time position in fall phase based on current battery
        timeInCurrentPhase = dischargePhase2Duration * (1 - battery / 100)
    else
        batteryPhase = 'discharge_depleted'
        timeInCurrentPhase = 0
    end
    
    sim.addStatusbarMessage('Stopped charging at ' .. string.format('%.1f%%', battery))
end

function tryRecharge()
    if not dummyHandle then return end
    
    local rpos = sim.getObjectPosition(robot, -1)
    local dpos = sim.getObjectPosition(dummyHandle, -1)
    local dx = rpos[1] - dpos[1]
    local dy = rpos[2] - dpos[2]
    local dz = rpos[3] - dpos[3]
    local dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    if dist <= rechargeRadius then
        -- Inside charging zone
        if not isCharging then
            startCharging()
        end
    else
        -- Outside charging zone
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
    
    -- Only realign if offset is significant (>5cm)
    if dist < 0.05 then return true end
    
    sim.addStatusbarMessage(string.format('Realigning to %s (offset: %.2fm)', nodeName, dist))
    
    -- Rotate towards node
    local targetYaw = math.atan2(dy, dx)
    if not rotateTo(targetYaw) then return false end
    
    -- Move to node
    if not moveToPosition(nodePos) then return false end
    
    -- Final orientation adjustment
    local nodeOrient = sim.getObjectOrientation(nodes[nodeName], -1)
    sim.setObjectOrientation(robot, -1, nodeOrient)
    
    return true
end

function followPath(pathName)
    if not isBatteryOk() then return false end
    
    -- Get path
    local pathHandle = nil
    local success = pcall(function()
        pathHandle = sim.getObjectHandle(pathName)
    end)
    if not success or not pathHandle then return false end
    
    local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(pathHandle, 'PATH'))
    local m = Matrix(#pathData // 7, 7, pathData)
    local pathPositions = m:slice(1,1,m:rows(),3):data()

    -- Determine path direction
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

    -- Offset path to robot position
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

    -- Orient to path start
    local dx = pathPositions[4] - pathPositions[1]
    local dy = pathPositions[5] - pathPositions[2]
    local targetYaw = math.atan2(dy, dx)
    if not rotateTo(targetYaw) then return false end

    -- Follow path
    local pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)
    local posAlongPath = 0
    previousSimulationTime = sim.getSimulationTime()

    while posAlongPath < totalLength and not sim.getSimulationStopping() and isBatteryOk() do
        local t = sim.getSimulationTime()
        local dt = t - previousSimulationTime
        previousSimulationTime = t

        updateBattery(dt)
        tryRecharge()
        
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
    if not isBatteryOk() then return false end
    
    if not currentNode then
        currentNode = getClosestNode()
    end
    
    if currentNode == targetNode then
        sim.addStatusbarMessage('Already at ' .. targetNode)
        return true
    end
    
    local nodePath = findShortestPath(currentNode, targetNode)
    if #nodePath == 0 then
        sim.addStatusbarMessage('No path found to ' .. targetNode)
        return false
    end
    
    sim.addStatusbarMessage('Route: ' .. table.concat(nodePath, ' -> '))
    
    -- Realign to current node
    if not realignToNode(currentNode) then
        sim.addStatusbarMessage('Aborted during initial realignment')
        return false
    end
    
    -- Follow each segment
    for i = 1, #nodePath-1 do
        if not isBatteryOk() then
            sim.addStatusbarMessage('Aborted - battery depleted')
            return false
        end
        
        local fromNode = nodePath[i]
        local toNode = nodePath[i+1]
        local pathName = graph[fromNode] and graph[fromNode][toNode]
        
        if not pathName then
            sim.addStatusbarMessage('Path not found: ' .. fromNode .. ' -> ' .. toNode)
            return false
        end
        
        if not followPath(pathName) then
            sim.addStatusbarMessage('Path following interrupted')
            return false
        end
        
        currentNode = toNode
    end
    
    -- Final realignment
    if not realignToNode(targetNode) then
        sim.addStatusbarMessage('Aborted during final realignment')
        return false
    end
    
    sim.addStatusbarMessage('Arrived at ' .. targetNode)
    return true
end

function actionGoToRoom()
    if not isBatteryOk() then return false end
    
    local rooms = {'Hab1', 'Hab2', 'Hab3', 'C'}
    local randomRoom = rooms[math.random(1, #rooms)]
    sim.addStatusbarMessage('>>> ACTION: Go to ' .. randomRoom)
    return goTo(randomRoom)
end

function actionStop()
    if not isBatteryOk() then return false end
    
    local stopTime = math.random(1, 5)
    sim.addStatusbarMessage('>>> ACTION: Stop for ' .. stopTime .. ' seconds')
    local endTime = sim.getSimulationTime() + stopTime
    
    while sim.getSimulationTime() < endTime and not sim.getSimulationStopping() and isBatteryOk() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now
        
        updateBattery(dt)
        tryRecharge()
        sim.step()
    end
    
    return isBatteryOk()
end

function callOperator()
    sim.addStatusbarMessage('>>> CALLING OPERATOR <<<')
    
    if not dummyHandle then
        sim.addStatusbarMessage('ERROR: Charging station not found!')
        return
    end
    
    -- Teleport to charging station
    local chargePos = sim.getObjectPosition(dummyHandle, -1)
    local chargeOrient = sim.getObjectOrientation(dummyHandle, -1)
    sim.setObjectPosition(robot, -1, chargePos)
    sim.setObjectOrientation(robot, -1, chargeOrient)
    
    currentNode = 'C'
    sim.addStatusbarMessage('Teleported to C')
    
    -- Start charging
    startCharging()
    
    -- Reset flags
    batteryDepleted = false
    
    -- Reset time tracking
    previousSimulationTime = sim.getSimulationTime()
    
    -- Wait while charging until battery is full
    sim.addStatusbarMessage('Charging... (will take ~1 hour from 0%)')
    while battery < 100.0 and not sim.getSimulationStopping() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now
        
        updateBattery(dt)
        sim.step()
    end
    
    sim.addStatusbarMessage('Battery fully charged - ready for new actions')
end

-- =================================
-- MAIN LOOP
-- =================================
function sysCall_thread()
    local actions = {actionGoToRoom}--, actionStop}
    
    while not sim.getSimulationStopping() do
        -- Check if operator needed
        if batteryDepleted then
            callOperator()
        end
        
        -- Execute random action
        if isBatteryOk() then
            local action = actions[math.random(1, #actions)]
            action()
            
            -- Small pause between actions
            if isBatteryOk() then
                local pauseTime = sim.getSimulationTime() + 0.5
                while sim.getSimulationTime() < pauseTime and not sim.getSimulationStopping() and isBatteryOk() do
                    local now = sim.getSimulationTime()
                    local dt = now - previousSimulationTime
                    previousSimulationTime = now
                    updateBattery(dt)
                    sim.step()
                end
            end
        end
    end
end