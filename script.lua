function sysCall_init()
    sim = require('sim')

    objectToFollowPath = sim.getObjectHandle('/PioneerP3DX')
    robot = sim.getObjectHandle('/PioneerP3DX')

    -- Lista de caminos a seguir en orden
    pathNames = {
        '/PathHab1_Hab3',
        '/PathHab3_Hab2',
        '/PathHab2_Hab4'
    }

    currentPathIndex = 1
    velocity = 0.2 -- m/s
    angularVelocity = 0.8 -- rad/s, velocidad de rotación del robot
    sim.setStepping(true)
end


-- Función auxiliar: normaliza ángulo a [-pi, pi]
function normalizeAngle(a)
    while a > math.pi do a = a - 2*math.pi end
    while a < -math.pi do a = a + 2*math.pi end
    return a
end


-- Función que hace girar el robot hasta orientarse con el siguiente camino
function orientToPathStart(pathPositions)
    -- Obtener orientación actual
    local euler = sim.getObjectOrientation(robot, -1)
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
    while math.abs(diff) > 0.035 and not sim.getSimulationStopping() do
        local now = sim.getSimulationTime()
        local dt = now - previousSimulationTime
        previousSimulationTime = now

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


-- Función que recorre un camino una sola vez
function followPath(pathName)
    local path = sim.getObjectHandle(pathName)
    local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(path, 'PATH'))
    local m = Matrix(#pathData // 7, 7, pathData)

    pathPositions = m:slice(1,1,m:rows(),3):data()

    -- Calcular offset entre posición del robot y primer punto del path
    local robotStartPos = sim.getObjectPosition(robot, -1)
    local offset = {
        robotStartPos[1] - pathPositions[1],
        robotStartPos[2] - pathPositions[2],
        robotStartPos[3] - pathPositions[3]
    }

    for i = 1, #pathPositions, 3 do
        pathPositions[i]   = pathPositions[i]   + offset[1]
        pathPositions[i+1] = pathPositions[i+1] + offset[2]
        pathPositions[i+2] = pathPositions[i+2] + offset[3]
    end

    pathQuaternions = m:slice(1,4,m:rows(),7):data()
    pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)

    posAlongPath = 0
    previousSimulationTime = sim.getSimulationTime()

    -- ⚙️ Nueva parte: girar hacia el camino antes de empezar
    orientToPathStart(pathPositions)

    -- Recorre el camino solo una vez
    while posAlongPath < totalLength and not sim.getSimulationStopping() do
        local t = sim.getSimulationTime()
        posAlongPath = posAlongPath + velocity * (t - previousSimulationTime)
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
