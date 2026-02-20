--[[
================================================================================
ROBOT NAVIGATION SCRIPT - DOCUMENTACIÓN
================================================================================

DESCRIPCIÓN GENERAL
-------------------
Este script implementa un sistema de navegación autónoma de alto nivel para un
robot Pioneer P3DX en CoppeliaSim. El robot puede desplazarse entre ubicaciones
predefinidas (nodos) siguiendo rutas (paths) mientras gestiona su batería de
forma realista.

El objetivo principal es servir como base para implementar aprendizaje por
refuerzo (RL) para la toma de decisiones de alto nivel.

ARQUITECTURA DEL SISTEMA
------------------------
El sistema se divide en capas:

    +------------------------------------------+
    |        CAPA DE ALTO NIVEL (RL)           |
    |   Acciones: goTo, stop, callOperator     |
    +------------------------------------------+
                      |
    +------------------------------------------+
    |      CAPA DE PLANIFICACIÓN               |
    |   Dijkstra, findShortestPath             |
    +------------------------------------------+
                      |
    +------------------------------------------+
    |      CAPA DE MOVIMIENTO                  |
    |   followPath, rotateTo, moveToPosition   |
    +------------------------------------------+
                      |
    +------------------------------------------+
    |      CAPA DE SIMULACIÓN                  |
    |   CoppeliaSim API, sim.step()            |
    +------------------------------------------+

NODOS DEL ENTORNO
-----------------
- R: Punto de inicio del robot (no se vuelve a él durante la operación)
- Hab1, Hab2, Hab3: Habitaciones/ubicaciones de trabajo
- C: Estación de carga

CONECTIVIDAD
------------
Todas las habitaciones (Hab1, Hab2, Hab3) y la estación de carga (C) están
conectadas entre sí mediante paths bidireccionales. El robot puede ir
directamente de cualquier nodo a cualquier otro.

El algoritmo de Dijkstra calcula la ruta más corta basándose en la longitud
real de los paths, no en distancia euclidiana.

MODELO DE BATERÍA
-----------------
El sistema implementa un modelo de batería bifásico realista:

DESCARGA (cuando el robot NO está en zona de carga):
    
    Batería
    100% |============== (Fase 1: Meseta) ==============|
         |                                              |\
         |                                              | \
         |                                              |  \ (Fase 2: Caída)
      0% |                                              |   \___________
         +----------------------------------------------+--------------> Tiempo
         0                                          t1              t1+t2
    
    - Fase 1 (discharge_plateau): Batería al 100% durante dischargePhase1Duration
    - Fase 2 (discharge_fall): Caída lineal de 100% a 0% en dischargePhase2Duration
    - Fase 3 (discharge_depleted): Batería a 0%, requiere operario

CARGA (cuando el robot está en zona de carga):
    
    Batería
    100% |                    ========================== (Fase 2: Meseta)
         |                   /
         |                  / (Fase 1: Subida)
         |                 /
      0% |________________/
         +------------------------------------------------> Tiempo
    
    - Fase 1 (charge_rise): Subida lineal desde nivel actual hasta 100%
    - Fase 2 (charge_plateau): Mantiene 100% mientras siga cargando

    Nota: El tiempo de carga es proporcional a la batería que falta.
    Si empieza al 50%, tarda la mitad de chargePhase1Duration.

ACCIONES DE ALTO NIVEL
----------------------
1. actionGoToRoom(): Selecciona habitación aleatoria y navega hacia ella
2. actionStop(): Detiene el robot durante 1-5 segundos (consume batería)
3. callOperator(): Teletransporta a C y recarga (se llama automáticamente al agotar batería)

PARÁMETROS CONFIGURABLES
------------------------
Ver sección CONFIGURATION PARAMETERS más abajo.

INTEGRACIÓN CON RL (FUTURO)
---------------------------
Este script está diseñado para integrarse con Python vía ZeroMQ Remote API.
El agente RL podrá:
- Observar: currentNode, battery, batteryPhase, isCharging
- Actuar: goTo(nodo), stop()
- Recibir recompensa basada en: tiempo operativo, eficiencia de desplazamiento

================================================================================
--]]

--[[
================================================================================
CONFIGURATION PARAMETERS - AJUSTA ESTOS VALORES
================================================================================
--]]

-- MOVIMIENTO
local CONFIG = {
    -- Velocidad lineal del robot (m/s)
    -- Pioneer P3DX real: máximo ~1.2 m/s, recomendado 0.3-0.5 para interiores
    -- Valores: 0.1 (muy lento) | 0.3 (cauteloso) | 0.5 (normal) | 0.8 (rápido)
    VELOCITY = 0.5,
    
    -- Velocidad angular (rad/s)
    -- Pioneer P3DX real: máximo ~5.24 rad/s (~300°/s)
    -- Valores: 0.5 (suave) | 1.57 (90°/s normal) | 3.14 (180°/s rápido)
    ANGULAR_VELOCITY = 1.57,
    
    -- Radio de detección de zona de carga (m)
    -- Qué tan cerca debe estar el robot de C para empezar a cargar
    -- Valores: 0.3 (preciso) | 0.5 (normal) | 1.0 (permisivo)
    RECHARGE_RADIUS = 0.5,
    
    -- Umbral de realineación (m)
    -- Si el robot está más lejos que esto del nodo, se realinea
    -- Valores: 0.02 (muy preciso) | 0.05 (normal) | 0.1 (permisivo)
    REALIGN_THRESHOLD = 0.05,
    
    -- BATERÍA - TIEMPOS REALISTAS
    -- Meseta de descarga: tiempo que la batería se mantiene al 100%
    -- Valores: 1800 (30 min) | 3600 (1 hora) | 7200 (2 horas)
    DISCHARGE_PLATEAU_DURATION = 1800,
    
    -- Caída de descarga: tiempo que tarda en bajar de 100% a 0%
    -- Valores: 300 (5 min rápido) | 600 (10 min) | 1200 (20 min)
    DISCHARGE_FALL_DURATION = 300,
    
    -- Tiempo de carga completa: tiempo para cargar de 0% a 100%
    -- Valores: 1800 (30 min) | 3600 (1 hora) | 7200 (2 horas)
    CHARGE_DURATION = 3600,
    
    -- BATERÍA - TIEMPOS DE PRUEBA (descomentar para testing rápido)
    -- DISCHARGE_PLATEAU_DURATION = 10,
    -- DISCHARGE_FALL_DURATION = 5,
    -- CHARGE_DURATION = 10,
    
    -- MENSAJES
    -- Nivel de verbosidad: 0 (mínimo) | 1 (normal) | 2 (debug)
    LOG_LEVEL = 1,
    
    -- Mostrar batería cada X% (debe ser divisor de 100)
    -- Valores: 5 | 10 | 20 | 25
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
    
    -- Charging station (dummy C)
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
    
    logMessage(1, 'INIT', 'Robot starting at node: ' .. currentNode)
    logMessage(1, 'INIT', 'Configuration loaded - Velocity: ' .. velocity .. ' m/s')
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
        -- DISCHARGING MODEL
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
    
    -- Report battery at intervals
    local currentTier = math.floor(battery / batteryReportInterval) * batteryReportInterval
    local lastTier = math.floor(lastBatteryReport / batteryReportInterval) * batteryReportInterval
    if currentTier ~= lastTier then
        logMessage(1, 'BATTERY', string.format('%d%% [%s]', currentTier, batteryPhase))
        lastBatteryReport = battery
    end
    
    -- Mark as depleted if battery runs out
    if battery <= 0 and not batteryDepleted then
        batteryDepleted = true
        logMessage(0, 'ERROR', 'BATTERY DEPLETED - Calling operator!')
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
        logMessage(1, 'NAV', 'Already at ' .. targetNode)
        return true
    end
    
    local nodePath = findShortestPath(currentNode, targetNode)
    if #nodePath == 0 then
        logMessage(0, 'ERROR', 'No path found to ' .. targetNode)
        return false
    end
    
    logMessage(1, 'NAV', 'Route: ' .. table.concat(nodePath, ' -> '))
    
    if not realignToNode(currentNode) then
        logMessage(1, 'NAV', 'Aborted during initial realignment')
        return false
    end
    
    for i = 1, #nodePath-1 do
        if not isBatteryOk() then
            logMessage(1, 'NAV', 'Aborted - battery depleted')
            return false
        end
        
        local fromNode = nodePath[i]
        local toNode = nodePath[i+1]
        local pathName = graph[fromNode] and graph[fromNode][toNode]
        
        if not pathName then
            logMessage(0, 'ERROR', 'Path not found: ' .. fromNode .. ' -> ' .. toNode)
            return false
        end
        
        logMessage(2, 'NAV', 'Following path: ' .. fromNode .. ' -> ' .. toNode)
        
        if not followPath(pathName) then
            logMessage(1, 'NAV', 'Path following interrupted')
            return false
        end
        
        currentNode = toNode
    end
    
    if not realignToNode(targetNode) then
        logMessage(1, 'NAV', 'Aborted during final realignment')
        return false
    end
    
    logMessage(1, 'NAV', 'Arrived at ' .. targetNode)
    return true
end

function actionGoToRoom()
    if not isBatteryOk() then return false end
    
    local rooms = {'Hab1', 'Hab2', 'Hab3', 'C'}
    local randomRoom = rooms[math.random(1, #rooms)]
    logMessage(1, 'ACTION', 'Go to ' .. randomRoom)
    return goTo(randomRoom)
end

function actionStop()
    if not isBatteryOk() then return false end
    
    local stopTime = math.random(1, 5)
    logMessage(1, 'ACTION', 'Stop for ' .. stopTime .. ' seconds')
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
    logMessage(1, 'ACTION', 'OPERATOR CALLED - Teleporting to charging station')
    
    if not dummyHandle then
        logMessage(0, 'ERROR', 'Charging station not found!')
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
    
    logMessage(1, 'BATTERY', 'Charging until full...')
    while battery < 100.0 and not sim.getSimulationStopping() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now
        
        sim.setObjectPosition(robot, -1, chargePos)
        sim.setObjectOrientation(robot, -1, chargeOrient)
        
        updateBattery(dt)
        sim.step()
    end
    
    logMessage(1, 'ACTION', 'Charging complete - Ready for new actions')
end

-- =================================
-- MAIN LOOP
-- =================================
function sysCall_thread()
    local actions = {actionGoToRoom} --, actionStop}
    
    logMessage(1, 'INIT', '=== SIMULATION STARTED ===')
    
    while not sim.getSimulationStopping() do
        if batteryDepleted then
            callOperator()
        end
        
        if isBatteryOk() then
            local action = actions[math.random(1, #actions)]
            action()
            
            if isBatteryOk() then
                local pauseTime = sim.getSimulationTime() + 0.5
                while sim.getSimulationTime() < pauseTime and not sim.getSimulationStopping() and isBatteryOk() do
                    local now = sim.getSimulationTime()
                    local dt = now - previousSimulationTime
                    previousSimulationTime = now
                    updateBattery(dt)
                    tryRecharge()
                    sim.step()
                end
            end
        end
    end
    
    logMessage(1, 'INIT', '=== SIMULATION ENDED ===')
end