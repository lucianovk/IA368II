sim = require'sim'

function sysCall_init()
    model = sim.getObject('..')
    laserHandle = sim.getObject('../sensor')
    jointHandle = sim.getObject('../joint')

    -- cor azul dos pontos
    local blue = {0, 0, 1}
    points = sim.addDrawingObject(sim.drawing_spherepts, 0.01, 0, -1, 100000, nil, nil, nil, blue)

    -- varredura total de 360 graus
    horizontalScanningAngle = 2 * math.pi
    scanningDensity = 2     -- 2 amostras por grau (~720 amostras)

    rangeMax = 10.0         -- tem que bater com o range_max do LaserScan
end

function sysCall_sensing()
    -- se quiser limpar a cada passo, deixa. Se quiser acumular pontos, comenta essa linha.
    sim.addDrawingObjectItem(points, nil)

    local pts = math.floor(horizontalScanningAngle * 180 * scanningDensity / math.pi) + 1
    local p = -horizontalScanningAngle / 2
    local stepSize = math.pi / (scanningDensity * 180)

    local ranges = {}

    for i = 0, pts, 1 do
        sim.setJointPosition(jointHandle, p)
        p = p + stepSize

        local r, dist, pt = sim.handleProximitySensor(laserHandle)
        local d

        if r > 0 then
            d = dist
            -- desenha ponto
            local m = sim.getObjectMatrix(laserHandle)
            pt = sim.multiplyVector(m, pt)
            sim.addDrawingObjectItem(points, pt)
        else
            d = rangeMax
        end

        ranges[#ranges + 1] = d
    end

    -- empacota como binário
    local packed = sim.packFloatTable(ranges)

    -- *** AQUI ESTÁ O PULO DO GATO ***
    -- 3 argumentos: target, propertyName, propertyState
    sim.setBufferProperty(sim.handle_scene, 'signal.myRobot_scan', packed)
end
