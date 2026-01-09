function sysCall_init()
    sim = require('sim')
    objectToFollowPath = sim.getObjectHandle('/PioneerP3DX')
    robot = sim.getObjectHandle('/PioneerP3DX')

    velocity = 0.6 -- m/s
    angularVelocity = 0.8 -- rad/s
    sim.setStepping(true)

    -- Initialize time tracking
    previousSimulationTime = sim.getSimulationTime()

    -- Battery
    battery = 100.0
    batteryDrainRate = 1.0 -- % por segundo

    -- Dummy C para recarga
    dummyHandle = nil
    rechargeRadius = 0.5
    local ok, h = pcall(sim.getObjectHandle, 'C')
    if ok and type(h) == 'number' then
        dummyHandle = h
        sim.addStatusbarMessage('Found charging dummy: C')
    else
        sim.addStatusbarMessage('Charging dummy "C" not found')
    end

    -- Nodos usando dummys de la escena
    nodes = {}
    local nodeNames = {'R', 'Hab1', 'Hab2', 'Hab3', 'C'}
    for _, nodeName in ipairs(nodeNames) do
        local ok, h = pcall(sim.getObjectHandle, nodeName)
        if ok and type(h) == 'number' then
            nodes[nodeName] = h
        else
            sim.addStatusbarMessage('Warning: Node dummy "' .. nodeName .. '" not found')
        end
    end

    -- Grafo de caminos bidireccionales
    graph = {
        R = {Hab1 = '/Path_R_Hab1', Hab2 = '/Path_R_Hab2', Hab3 = '/Path_R_Hab3', C = '/Path_R_C'},
        Hab1 = {R = '/Path_R_Hab1', Hab2 = '/PathHab1_Hab2', Hab3 = '/PathHab1_Hab3', C = '/PathHab1_C'},
        Hab2 = {R = '/Path_R_Hab2', Hab1 = '/PathHab1_Hab2', Hab3 = '/PathHab2_Hab3', C = '/PathHab2_C'},
        Hab3 = {R = '/Path_R_Hab3', Hab1 = '/PathHab1_Hab3', Hab2 = '/PathHab2_Hab3', C = '/PathHab3_C'},
        C = {R = '/Path_R_C', Hab1 = '/PathHab1_C', Hab2 = '/PathHab2_C', Hab3 = '/PathHab3_C'}
    }
    
    -- Variable para trackear el nodo actual
    currentNode = 'R' -- Asumimos que empieza en R
end

-- =================================
-- Funciones auxiliares
-- =================================

function normalizeAngle(a)
    while a > math.pi do a = a - 2*math.pi end
    while a < -math.pi do a = a + 2*math.pi end
    return a
end

function updateBattery(dt)
    if dt <= 0 then return end
    battery = battery - batteryDrainRate * dt
    sim.addStatusbarMessage(string.format('Battery: %.1f%%', battery))
    if battery <= 0 then
        battery = 0
        sim.addStatusbarMessage('Battery depleted')
        sim.stopSimulation()
    end
end

function tryRecharge()
    if not dummyHandle then return false end
    local rpos = sim.getObjectPosition(robot, -1)
    local dpos = sim.getObjectPosition(dummyHandle, -1)
    local dx = rpos[1] - dpos[1]
    local dy = rpos[2] - dpos[2]
    local dz = rpos[3] - dpos[3]
    local dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist <= rechargeRadius then
        if battery < 100.0 then
            battery = 100.0
            sim.addStatusbarMessage('Battery recharged to 100% at Dummy C')
        end
        return true
    end
    return false
end

function orientToPathStart(pathPositions)
    local euler = sim.getObjectOrientation(robot, -1)
    local currentYaw = euler[3]

    local dx = pathPositions[4] - pathPositions[1]
    local dy = pathPositions[5] - pathPositions[2]
    local targetYaw = math.atan2(dy, dx)

    local diff = normalizeAngle(targetYaw - currentYaw)
    local t = sim.getSimulationTime()
    previousSimulationTime = t

    while math.abs(diff) > 0.01 and not sim.getSimulationStopping() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now

        updateBattery(dt)
        tryRecharge()
        if sim.getSimulationStopping() then break end

        local dir = (diff > 0) and 1 or -1
        local deltaYaw = dir * angularVelocity * dt
        if math.abs(deltaYaw) > math.abs(diff) then deltaYaw = diff end

        currentYaw = currentYaw + deltaYaw
        euler[3] = currentYaw
        sim.setObjectOrientation(robot, -1, euler)
        diff = normalizeAngle(targetYaw - currentYaw)
        sim.step()
    end
end

-- =================================
-- Función para seguir un path
-- =================================
function followPath(pathName)
    local path = sim.getObjectHandle(pathName)
    local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(path, 'PATH'))
    local m = Matrix(#pathData // 7, 7, pathData)
    local pathPositions = m:slice(1,1,m:rows(),3):data()

    local robotPos = sim.getObjectPosition(robot, path)
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
    local offset = {robotStartPos[1]-pathPositions[1], robotStartPos[2]-pathPositions[2], robotStartPos[3]-pathPositions[3]}
    for i=1,#pathPositions,3 do
        pathPositions[i] = pathPositions[i] + offset[1]
        pathPositions[i+1] = pathPositions[i+1] + offset[2]
        pathPositions[i+2] = pathPositions[i+2] + offset[3]
    end

    local pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)
    local posAlongPath = 0
    previousSimulationTime = sim.getSimulationTime()

    orientToPathStart(pathPositions)

    while posAlongPath < totalLength and not sim.getSimulationStopping() do
        local t = sim.getSimulationTime()
        local dt = t - previousSimulationTime
        previousSimulationTime = t

        updateBattery(dt)
        tryRecharge()
        if sim.getSimulationStopping() then break end

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
end

-- =================================
-- Funciones de planificación de alto nivel
-- =================================

-- Obtener nodo más cercano usando dummys
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

-- Longitud real de un path
function getPathLength(pathName)
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
end

-- Dijkstra para encontrar camino más corto
function findShortestPath(startNode,targetNode)
    local dist, prev, Q = {}, {}, {}
    for node,_ in pairs(graph) do
        dist[node] = math.huge
        prev[node] = nil
        Q[node] = true
    end
    dist[startNode] = 0

    while next(Q) do
        local u,minDist = nil, math.huge
        for node,_ in pairs(Q) do
            if dist[node] < minDist then u,minDist = node,dist[node] end
        end

        if u==targetNode then break end
        Q[u]=nil

        for neighbor,pathName in pairs(graph[u]) do
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
        table.insert(path,1,node)
        node = prev[node]
    end
    return path
end

-- Función de alto nivel: ir a un nodo
function goTo(targetNode)
    if not currentNode then
        currentNode = getClosestNode()
        sim.addStatusbarMessage('Initial node detected: ' .. currentNode)
    end
    
    if currentNode == targetNode then
        sim.addStatusbarMessage('Already at ' .. targetNode)
        return
    end
    
    local nodePath = findShortestPath(currentNode, targetNode)
    if #nodePath == 0 then
        sim.addStatusbarMessage('Error: No path found to ' .. targetNode)
        return
    end
    
    sim.addStatusbarMessage('Planning route from ' .. currentNode .. ' to ' .. targetNode .. ': ' .. table.concat(nodePath, ' -> '))
    
    for i=1,#nodePath-1 do
        local pathName = graph[nodePath[i]] and graph[nodePath[i]][nodePath[i+1]]
        if not pathName then
            sim.addStatusbarMessage('Error: Path not found between ' .. nodePath[i] .. ' and ' .. nodePath[i+1])
            break
        end
        followPath(pathName)
    end
    
    -- Actualizar nodo actual al destino
    currentNode = targetNode
    sim.addStatusbarMessage('Arrived at ' .. currentNode)
end

-- =================================
-- Thread principal
-- =================================
function sysCall_thread()
    -- Todos los nodos disponibles (excluyendo R que es solo inicio)
    local allNodes = {'Hab1', 'Hab2', 'Hab3', 'C'}
    
    -- Loop infinito con decisiones completamente aleatorias
    while not sim.getSimulationStopping() do
        -- Seleccionar un nodo aleatorio
        local randomIndex = math.random(1, #allNodes)
        local targetNode = allNodes[randomIndex]
        
        sim.addStatusbarMessage('=== Random destination selected: ' .. targetNode .. ' ===')
        goTo(targetNode)
        
        -- Pausa aleatoria al llegar (entre 0.5 y 2 segundos)
        local pauseDuration = 0.5 + math.random() * 1.5
        local pauseTime = sim.getSimulationTime() + pauseDuration
        while sim.getSimulationTime() < pauseTime and not sim.getSimulationStopping() do
            tryRecharge() -- Siempre intenta recargar si está cerca de C
            sim.step()
        end
    end
end