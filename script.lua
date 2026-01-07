function sysCall_init()
    sim = require('sim')
    objectToFollowPath = sim.getObjectHandle('/PioneerP3DX')
    robot = sim.getObjectHandle('/PioneerP3DX')
    -- Lista de caminos a seguir en orden
    pathNames = {
        -- Desde R hacia todas las habitaciones y vuelta
        '/Path_R_Hab1',
        '/Path_R_Hab1',

        '/Path_R_Hab2',
        '/Path_R_Hab2',

        '/Path_R_Hab3',
        '/Path_R_Hab3',

        '/Path_R_C',
        '/Path_R_C',
        
        '/Path_R_Hab1',--nexo
        -- Conexiones entre habitaciones
        '/PathHab1_Hab2',
        '/PathHab1_Hab2',

        '/PathHab1_Hab3',
        '/PathHab1_Hab3',
        '/PathHab1_Hab2',--nexo

        '/PathHab2_Hab3',
        '/PathHab2_Hab3',
        
        '/PathHab2_C',--nexo
        -- Conexiones desde habitaciones al C
        '/PathHab1_C',
        '/PathHab1_C',

        '/PathHab2_C',
        '/PathHab2_C',

        '/PathHab3_C',
        '/PathHab3_C',

        -- Secuencia de prueba combinada (ruta más larga encadenada)
        '/Path_R_Hab1',
        '/PathHab1_Hab2',
        '/PathHab2_Hab3',
        '/PathHab3_C',
        '/PathHab3_C',
        '/PathHab2_Hab3',
        '/PathHab1_Hab2',
        '/Path_R_Hab1',
        '/Path_R_C'
    }

    currentPathIndex = 1
    velocity = 0.6 -- m/s
    angularVelocity = 0.8 -- rad/s, velocidad de rotación del robot
    sim.setStepping(true)

    -- Battery timer (porcentaje)
    battery = 100.0          -- nivel inicial (%) 
    batteryDrainRate = 1.0   -- % por segundo (ajustable)

    -- Charging (dummy R)
    dummyHandle = nil
    rechargeRadius = 0.5 -- metros, distancia para considerar "en" el dummy R
    -- obtener el handle del dummy llamado "R"
    local ok, h = pcall(sim.getObjectHandle, 'R')
    if ok and type(h) == 'number' then
        dummyHandle = h
        sim.addStatusbarMessage('Found charging dummy: R')
    else
        sim.addStatusbarMessage('Charging dummy "R" not found')
    end
end

-- Actualiza la bateria y detiene la simulacion si se agota
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

-- Comprueba si el robot está dentro del dummy R y recarga si es así
function tryRecharge()
    if not dummyHandle then return false end
    -- obtener posiciones en mundo
    local rpos = sim.getObjectPosition(robot, -1)
    local dpos = sim.getObjectPosition(dummyHandle, -1)
    local dx = rpos[1] - dpos[1]
    local dy = rpos[2] - dpos[2]
    local dz = rpos[3] - dpos[3]
    local dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist <= rechargeRadius then
        if battery < 100.0 then
            battery = 100.0
            sim.addStatusbarMessage('Battery recharged to 100% at Dummy R')
        end
        return true
    end
    return false
end

-- Función auxiliar: normaliza ángulo a [-pi, pi]
function normalizeAngle(a)
    while a > math.pi do
        a = a - 2*math.pi
    end
    while a < -math.pi do
        a = a + 2*math.pi
    end
    return a
end

-- Función que hace girar el robot hasta orientarse con el siguiente camino
function orientToPathStart(pathPositions)
    -- Obtener orientación actual local
    euler = sim.getObjectOrientation(robot, -1)
    local currentYaw = euler[3]

    -- Calcular yaw deseado del nuevo camino (según primeros dos puntos)
    local dx = pathPositions[4] - pathPositions[1]
    local dy = pathPositions[5] - pathPositions[2]
    local targetYaw = math.atan2(dy, dx)

    -- Diferencia angular
    local diff = normalizeAngle(targetYaw - currentYaw)
    local t = sim.getSimulationTime()
    previousSimulationTime = t

    -- Girar poco a poco hasta alinearse
    while math.abs(diff) > 0.01 and not sim.getSimulationStopping() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now

        -- Actualizar bateria
        updateBattery(dt)

        -- Intentar recargar si estamos en el dummy R
        tryRecharge()
        if sim.getSimulationStopping() then break end

        -- Sentido del giro
        local dir = (diff > 0) and 1 or -1
        local deltaYaw = dir * angularVelocity * dt

        -- Evitar sobrepasar el ángulo
        if math.abs(deltaYaw) > math.abs(diff) then
            deltaYaw = diff
        end
        currentYaw = currentYaw + deltaYaw
        euler[3] = currentYaw
        sim.setObjectOrientation(robot, -1, euler)
        diff = normalizeAngle(targetYaw - currentYaw)
        sim.step()
    end
end

function followPath(pathName)
    local path = sim.getObjectHandle(pathName)
    local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(path, 'PATH'))
    local m = Matrix(#pathData // 7, 7, pathData)
    pathPositions = m:slice(1,1,m:rows(),3):data()

    -- Obtener posición actual del robot en el marco del path (para comparar correctamente)
    local robotPos = sim.getObjectPosition(robot, path)

    -- Calcular distancia al inicio y al final del camino
    local startPos = {pathPositions[1], pathPositions[2], pathPositions[3]}
    local endPos = {
        pathPositions[#pathPositions - 2],
        pathPositions[#pathPositions - 1],
        pathPositions[#pathPositions]
    }
    local function distXY(a, b) return math.sqrt((a[1]-b[1])^2 + (a[2]-b[2])^2) end
    local dStart = distXY(robotPos, startPos)
    local dEnd = distXY(robotPos, endPos)

    --  Si está más cerca del final, invertir el camino
    if dEnd < dStart then
        local reversed = {}
        for i = #pathPositions, 1, -3 do
            table.insert(reversed, pathPositions[i-2])
            table.insert(reversed, pathPositions[i-1])
            table.insert(reversed, pathPositions[i])
        end
        pathPositions = reversed
    end

    -- Calcular offset entre posición del robot y primer punto del path
    local robotStartPos = sim.getObjectPosition(robot, -1)
    local offset = {
        robotStartPos[1] - pathPositions[1],
        robotStartPos[2] - pathPositions[2],
        robotStartPos[3] - pathPositions[3]
    }
    for i = 1, #pathPositions, 3 do
        pathPositions[i] = pathPositions[i] + offset[1]
        pathPositions[i+1] = pathPositions[i+1] + offset[2]
        pathPositions[i+2] = pathPositions[i+2] + offset[3]
    end

    pathQuaternions = m:slice(1,4,m:rows(),7):data()
    pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)
    posAlongPath = 0
    previousSimulationTime = sim.getSimulationTime()

    -- Girar hacia el camino antes de empezar
    orientToPathStart(pathPositions)

    -- Recorre el camino solo una vez
    while posAlongPath < totalLength and not sim.getSimulationStopping() do
        local t = sim.getSimulationTime()
        local dt = t - previousSimulationTime

        -- Actualizar bateria
        updateBattery(dt)

        -- Intentar recargar si estamos en el dummy R
        tryRecharge()
        if sim.getSimulationStopping() then break end

        posAlongPath = posAlongPath + velocity * dt
        if posAlongPath > totalLength then posAlongPath = totalLength end

        local pos = sim.getPathInterpolatedConfig(pathPositions, pathLengths, posAlongPath)
        local aheadPos = sim.getPathInterpolatedConfig(pathPositions, pathLengths, math.min(posAlongPath + 0.02, totalLength))
        local dx = aheadPos[1] - pos[1]
        local dy = aheadPos[2] - pos[2]
        local yaw
        if math.abs(dx) + math.abs(dy) > 1e-6 then
            yaw = math.atan2(dy, dx)
        else
            local euler = sim.getObjectOrientation(objectToFollowPath, -1)
            yaw = euler[3]
        end
        local cy = math.cos(yaw * 0.5)
        local sy = math.sin(yaw * 0.5)
        local quat = {0.0, 0.0, sy, cy}

        sim.setObjectPosition(objectToFollowPath, -1, pos)
        sim.setObjectQuaternion(objectToFollowPath, -1, quat)
        previousSimulationTime = t
        sim.step()
    end
end


function sysCall_thread()
    for i, pathName in ipairs(pathNames) do
        followPath(pathName)
    end
end