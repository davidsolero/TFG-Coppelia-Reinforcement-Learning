function sysCall_init()
    sim = require('sim')
    objectToFollowPath = sim.getObjectHandle('/PioneerP3DX')
    robot = sim.getObjectHandle('/PioneerP3DX')
    path = sim.getObjectHandle('/Path_R_Hab1')
    -- Read the custom data block named 'PATH' from the path object.
    -- This block contains serialized double data describing all path points (x,y,z,qx,qy,qz,qw,...).
    -- sim.readCustomDataBlock(path, 'PATH') returns binary data.
    -- sim.unpackDoubleTable(...) converts that binary data into a flat Lua table of doubles.
    local pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(path, 'PATH'))
    -- Create a Matrix from the path data for easy slicing.
    -- Input: rows = number of path points (#pathData // 7), columns = 7, data = pathData.
    -- Output: a Matrix 'm' where each row = [x y z qx qy qz qw].
    local m = Matrix(#pathData // 7, 7, pathData)
    -- Extract the position data (columns 1 to 3) for all rows.
    -- slice(rowStart, colStart, rowEnd, colEnd): returns submatrix.
    -- data() : returns flattened Lua array.
    -- Output: flat list [x1,y1,z1,x2,y2,z2,...].
    pathPositions = m:slice(1,1,m:rows(),3):data()
    -- Calcular offset entre posici?n inicial del robot y primer punto del path
    local robotStartPos = sim.getObjectPosition(robot, -1)
    offset = {
        robotStartPos[1] - pathPositions[1],
        robotStartPos[2] - pathPositions[2],
        robotStartPos[3] - pathPositions[3]
    }
    -- Ajustar pathPositions sumando offset
    for i = 1, #pathPositions, 3 do
        pathPositions[i] = pathPositions[i] + offset[1]
        pathPositions[i+1] = pathPositions[i+1] + offset[2]
        pathPositions[i+2] = pathPositions[i+2] + offset[3]
    end
    -- Extract quaternion data (columns 4 to 7) for all rows.
    -- Output: flat list [qx1,qy1,qz1,qw1,qx2,qy2,qz2,qw2,...].
    pathQuaternions = m:slice(1,4,m:rows(),7):data()
    -- Compute the cumulative distances along the path.
    -- Inputs: pathPositions (x,y,z array), dimension=3 because each point has 3 coordinates.
    -- Outputs: pathLengths (table of cumulative distances per segment),
    -- totalLength (scalar, full path length).
    pathLengths, totalLength = sim.getPathLengths(pathPositions, 3)
    -- Set linear velocity along the path in meters per second.
    -- Input: numeric value (scalar). Output: variable used later in motion equation.
    velocity = 0.2 -- 0.2 cm/s
    -- Initialize distance traveled along path (m).
    posAlongPath = 0
    -- Store current simulation time for time-step calculations.
    -- Output: numeric simulation time in seconds.
    previousSimulationTime = sim.getSimulationTime()
    -- Enable stepping mode for threaded scripts.
    -- When true, we must call sim.step() manually to advance the simulation one step per loop.
    sim.setStepping(true)
end -- sysCall_thread() executes repeatedly (like a loop) until the simulation stops.

function sysCall_thread()
    -- Continue looping until user stops simulation.
    while not sim.getSimulationStopping() do
        -- Read current simulation time (s).
        -- Output: floating-point time in seconds since sim start.
        local t = sim.getSimulationTime()
        -- Update distance along the path using speed × time difference.
        -- (t - previousSimulationTime) gives elapsed time since last step.
        -- '%' (modulo) keeps value within [0, totalLength), so object loops around the path.
        posAlongPath = (posAlongPath + velocity * (t - previousSimulationTime)) % totalLength
        -- Interpolate position at 'posAlongPath' along path.
        -- At every simulation step, it computes the interpolated position and orientation at a given travel distance.
        -- Inputs: pathPositions (flat array), pathLengths (cumulative distances), posAlongPath (distance).
        -- Output: table {x,y,z} in world coordinates.
        local pos = sim.getPathInterpolatedConfig(pathPositions, pathLengths, posAlongPath)
        -- Compute a small lookahead to estimate path tangent direction.
        -- eps defines how far ahead to sample (in meters).
        local eps = 0.02
        -- Interpolate another point slightly ahead along the path.
        local aheadPos = sim.getPathInterpolatedConfig(pathPositions, pathLengths, (posAlongPath + eps) % totalLength)
        -- Compute tangent vector components in the XY plane.
        -- Inputs: current and ahead positions. Outputs: dx, dy (tangent vector).
        local dx = aheadPos[1] - pos[1]
        local dy = aheadPos[2] - pos[2]
        -- Compute yaw angle (rotation about Z) so +X faces along tangent.
        local yaw
        if math.abs(dx) + math.abs(dy) > 1e-6 then
            -- atan2(dy,dx) gives orientation angle of tangent in radians.
            yaw = math.atan2(dy, dx)
        else
            -- fallback: if tangent is tiny, use current object yaw to avoid sudden jumps.
            local euler = sim.getObjectOrientation(objectToFollowPath, -1)
            -- euler[3] = yaw angle in radians for Z-axis rotation.
            yaw = euler[3]
        end
        -- Convert yaw (Z-rotation) to quaternion {qx,qy,qz,qw}.
        -- Inputs: yaw in radians.
        -- Outputs: unit quaternion representing pure yaw rotation (no roll/pitch).
        local cy = math.cos(yaw * 0.5)
        local sy = math.sin(yaw * 0.5)
        local quat = {0.0, 0.0, sy, cy}
        -- Set the object's position in the WORLD frame.
        -- Inputs: handle (objectToFollowPath), reference frame (-1 means world), position {x,y,z}.
        sim.setObjectPosition(objectToFollowPath, -1, pos)
        -- Set the object's orientation (quaternion) in the WORLD frame.
        -- Inputs: handle, reference (-1 = world), quaternion {qx,qy,qz,qw}.
        sim.setObjectQuaternion(objectToFollowPath, -1, quat)
        -- Store current time for next iteration's delta-time computation.
        previousSimulationTime = t
        -- Advance the simulation one step (works only if stepping mode enabled).
        sim.step()
    end
end