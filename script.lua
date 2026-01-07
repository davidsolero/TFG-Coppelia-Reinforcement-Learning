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

    -- Índice del camino actual
    currentPathIndex = 1

    velocity = 0.2 -- m/s
    sim.setStepping(true)
end


-- Función auxiliar que recorre un camino completo una sola vez
function followPath(pathName)

    -- Obtener handle del camino actual
    path = sim.getObjectHandle(pathName)

    -- Leer los datos binarios del path y convertirlos a tabla de doubles
    local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(path, 'PATH'))

    -- Crear matriz con 7 columnas (x y z qx qy qz qw)
    local m = Matrix(#pathData // 7, 7, pathData)

    -- Extraer posiciones (x,y,z)
    pathPositions = m:slice(1,1,m:rows(),3):data()

    -- Calcular offset entre posición inicial del robot y primer punto del path
    local robotStartPos = sim.getObjectPosition(robot, -1)
    offset = {
        robotStartPos[1] - pathPositions[1],
        robotStartPos[2] - pathPositions[2],
        robotStartPos[3] - pathPositions[3]
    }

    -- Ajustar pathPositions sumando offset
    for i = 1, #pathPositions, 3 do
        pathPositions[i]   = pathPositions[i]   + offset[1]
        pathPositions[i+1] = pathPositions[i+1] + offset[2]
        pathPositions[i+2] = pathPositions[i+2] + offset[3]
    end

    -- Extraer orientaciones (qx,qy,qz,qw)
    pathQuaternions = m:slice(1,4,m:rows(),7):data()

    -- Calcular longitudes acumuladas y longitud total
    pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)

    -- Inicializar recorrido
    posAlongPath = 0
    previousSimulationTime = sim.getSimulationTime()

    -- Bucle: avanzar por el path hasta llegar al final
    while posAlongPath < totalLength and not sim.getSimulationStopping() do

        local t = sim.getSimulationTime()
        posAlongPath = posAlongPath + velocity * (t - previousSimulationTime)

        if posAlongPath > totalLength then
            posAlongPath = totalLength
        end

        -- Interpolación de posición y orientación
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


-- sysCall_thread() ejecuta secuencialmente todos los caminos de la lista
function sysCall_thread()
    for i, pathName in ipairs(pathNames) do
        followPath(pathName)
    end
end
