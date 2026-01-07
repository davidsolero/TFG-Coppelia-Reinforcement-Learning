-- Script sencillo para que Pioneer siga un único path: Path_R_Hab1
-- Pégalo como child script del propio Pioneer (no hace falta nada más).

function sysCall_init()
    sim.setThreadAutomaticSwitch(true)

    -- NOMBRE DEL PATH (ajusta si tu alias/route es distinto)
    local possibleNames = {'./Path_R_Hab1', '/Path_R_Hab1', 'Path_R_Hab1'}

    -- Buscar path (intenta varias notaciones)
    pathHandle = -1
    for i=1,#possibleNames,1 do
        local h = sim.getObject(possibleNames[i])
        if h ~= -1 then
            pathHandle = h
            break
        end
    end
    if pathHandle == -1 then
        sim.addLog(sim.verbosity_errors, "No se encontró Path_R_Hab1. Revisa el alias/ruta.")
        return
    end

    -- Obtener robot (objeto que contiene este script)
    robot = sim.getObject('.')  -- referencia al propio modelo donde está el script

    -- Buscar los motores (provee varios intentos por si el alias difiere)
    motorLeft = sim.getObject('./leftMotor')
    if motorLeft == -1 then motorLeft = sim.getObject('./Pioneer_p3dx_leftMotor') end
    if motorLeft == -1 then motorLeft = sim.getObject('/Pioneer_p3dx_leftMotor') end

    motorRight = sim.getObject('./rightMotor')
    if motorRight == -1 then motorRight = sim.getObject('./Pioneer_p3dx_rightMotor') end
    if motorRight == -1 then motorRight = sim.getObject('/Pioneer_p3dx_rightMotor') end

    if motorLeft == -1 or motorRight == -1 then
        sim.addLog(sim.verbosity_errors, "No se encontraron los motores (left/right). Comprueba nombres/alias.")
        return
    end

    -- Parámetros de control (ajusta si es necesario)
    baseSpeed = 3.0       -- velocidad base en "unidades" para la rueda (puedes reducir si va muy rápido)
    kp = 2.5              -- ganancia proporcional para corrección angular
    followSpeed = 0.05    -- incremento sobre la distancia a lo largo del path (m por paso)

    -- Inicializar recorrido sobre el path
    pathLength = sim.getPathLength(pathHandle) or 0
    s = 0.0               -- distancia recorrida a lo largo del path
    finished = false

    sim.addLog(sim.verbosity_infos, "Path encontrado. Longitud: "..tostring(pathLength))
end

function sysCall_actuation()
    if finished then return end
    if not pathHandle or pathHandle == -1 then return end

    -- Avanzamos a lo largo del path
    local dt = sim.getSimulationTimeStep()
    s = s + followSpeed * dt * 100  -- escala para que avance razonablemente (ajusta si va lento/rápido)

    -- Si llegamos al final, paramos y marcamos terminado
    if s >= pathLength then
        sim.setJointTargetVelocity(motorLeft, 0)
        sim.setJointTargetVelocity(motorRight, 0)
        finished = true
        sim.addLog(sim.verbosity_infos, "Path completado.")
        return
    end

    -- Obtenemos la posición objetivo en coordenadas absolutas
    local posOnPath, orientOnPath = sim.getPositionOnPath(pathHandle, s)
    if not posOnPath then return end
    -- posOnPath = {x,y,z}

    -- Posición actual del robot (origen mundo)
    local robotPos = sim.getObjectPosition(robot, -1) -- {x,y,z}

    -- Vector objetivo relativo al robot (en plano XY)
    local dx = posOnPath[1] - robotPos[1]
    local dy = posOnPath[2] - robotPos[2]
    local distance = math.sqrt(dx*dx + dy*dy)
    if distance < 0.01 then distance = 0.01 end

    -- Ángulo deseado y orientación actual
    local desiredHeading = math.atan2(dy, dx)     -- en radianes (mundo)
    local robotOri = sim.getObjectOrientation(robot, -1)
    local robotHeading = robotOri[3]              -- yaw

    -- Error de orientación (normalizado entre -pi y pi)
    local error = desiredHeading - robotHeading
    if error > math.pi then error = error - 2*math.pi end
    if error < -math.pi then error = error + 2*math.pi end

    -- Control diferencial básico (velocidad de ruedas = base +/- corrección)
    local leftVel = baseSpeed - kp * error
    local rightVel = baseSpeed + kp * error

    -- Limitar velocidades (por seguridad)
    local vmax = 10
    if leftVel > vmax then leftVel = vmax end
    if rightVel > vmax then rightVel = vmax end
    if leftVel < -vmax then leftVel = -vmax end
    if rightVel < -vmax then rightVel = -vmax end

    sim.setJointTargetVelocity(motorLeft, leftVel)
    sim.setJointTargetVelocity(motorRight, rightVel)
end

function sysCall_cleanup()
    -- nada especial
end
