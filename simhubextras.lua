-- Simhub telemetry mod, Wotever 2023
local M = {}

local ip = "127.0.0.1"
local port = 9999
local updateRate = 100
local udpSocket = nil
local lastSimHubFrameData = nil
local abs = math.abs
local ffi = require("ffi")

local updateTime = 0
local updateTimer = 0
local timeStamp = 0

local accX
local accY
local accZ
local accelerationSmoothingX
local accelerationSmoothingY
local accelerationSmoothingZ
local accXSmoother
local accYSmoother
local accZSmoother

local function ternary(cond, T, F, ...)
    if cond then
        return T(...)
    else
        return F(...)
    end
end

local function sternary(cond, T, F)
    if cond then
        return T
    else
        return F
    end
end

local function declareOutgaugeExtraStruct()
    ffi.cdef [[
  typedef struct simhub_telemetry  {
     
     // Just a magic string allowing to throw away invalid packets
      char magic[8];
      float timeStamp;

      char vehicleName[256];
      char vehicleModel[256];
      char vehicleConfig[256];

      float idle_rpm;
      float max_rpm;
      int gear;
      int max_gears;      
      int gearboxGrinding;      

      float speed;           // M/S
      float rpm;             // RPM
      float turbo;           // BAR
      float turboMax;           // BAR
      float engTemp;         // C
      float fuel;            // 0 to 1
      float oilTemp;         // C
      float waterTemp;
      float fuelCapacity;
      float fuelVolume;
      float engineLoad;
      int ignitionOn;

      // lights
      int light_HighBeam;
      int light_Parkingbrake;
      int light_Signal_L;
      int light_Signal_R;
      int light_Abs;
      int light_Oil;
      int light_EngineRunning;
      int light_TC;
      int light_Shift;
      int light_HazardEnabled;

      int light_LowHighBeam;
      int light_LowBeam;
      int light_Fog;

      // Inputs
      float input_throttle;     
      float input_brake;        
      float input_clutch;       
      float input_parkingBrake;    
      float input_steeringPercent;

      // Motion
      //World position of the car
      float          posX;
      float          posY;
      float          posZ;

      //Velocity of the car
      float          velX;
      float          velY;
      float          velZ;

      //Acceleration of the car, gravity not included
      float          accX;
      float          accY;
      float          accZ;

      //Vector components of a vector pointing "up" relative to the car
      float          upVecX;
      float          upVecY;
      float          upVecZ;

      //Vector components of a vector pointing "forward" relative to the car
      float          forwardVecX;
      float          forwardVecY;
      float          forwardVecZ;

      //Roll, pitch and yaw positions of the car
      float          rollPos;
      float          pitchPos;
      float          yawPos;

      //Roll, pitch and yaw "velocities" of the car
      float          rollRate;
      float          pitchRate;
      float          yawRate;

      //Roll, pitch and yaw "accelerations" of the car
      float          rollAcc;
      float          pitchAcc;
      float          yawAcc;

      // Suspensions
      float suspension_position_fl;
      float suspension_position_fr;
      float suspension_position_rl;
      float suspension_position_rr;

      float suspension_velocity_fl;
      float suspension_velocity_fr;
      float suspension_velocity_rl;
      float suspension_velocity_rr;

      float suspension_acceleration_fl;
      float suspension_acceleration_fr;
      float suspension_acceleration_rl;
      float suspension_acceleration_rr;

      float wheel_slip_fl;       
      float wheel_slip_fr;     
      float wheel_slip_rl;        
      float wheel_slip_rr;       
    
      float wheel_RPS_fl;
      float wheel_RPS_fr;
      float wheel_RPS_rl;
      float wheel_RPS_rr;

      float wheel_speed_fl;
      float wheel_speed_fr;
      float wheel_speed_rl;
      float wheel_speed_rr;

      int wheel_isRubberTire_fl;
      int wheel_isRubberTire_fr;
      int wheel_isRubberTire_rl;
      int wheel_isRubberTire_rr;

      int wheel_contactMaterial_fl;
      int wheel_contactMaterial_fr;
      int wheel_contactMaterial_rl;
      int wheel_contactMaterial_rr;
	  
      int wheel_isDeflated_fl;
      int wheel_isDeflated_fr;
      int wheel_isDeflated_rl;
      int wheel_isDeflated_rr;

      float wheel_contactDepth_fl;
      float wheel_contactDepth_fr;
      float wheel_contactDepth_rl;
      float wheel_contactDepth_rr;

      float brake_surfaceTemperature_fl;
      float brake_surfaceTemperature_fr;
      float brake_surfaceTemperature_rl;
      float brake_surfaceTemperature_rr;

      float brake_coreTemperature_fl;
      float brake_coreTemperature_fr;
      float brake_coreTemperature_rl;
      float brake_coreTemperature_rr;
	  
	  int check;

      float reserved_1;
      float reserved_2;
      float reserved_3;
      float reserved_4;
  } simhub_telemetry;
  ]]
end
pcall(declareOutgaugeExtraStruct)

local function resetState()
    lastSimHubFrameData = {
        suspension_position_fl = 0,
        suspension_position_fr = 0,
        suspension_position_rl = 0,
        suspension_position_rr = 0,
        suspension_velocity_fl = 0,
        suspension_velocity_fr = 0,
        suspension_velocity_rl = 0,
        suspension_velocity_rr = 0,
        rollRate = 0,
        pitchRate = 0,
        yawRate = 0
    }

    accXSmoother:reset()
    accYSmoother:reset()
    accZSmoother:reset()
end

local function fitWheelName(wheelName)
    if wheelName == 'RL1' then
        return 'RL'
    end
    if wheelName == 'RR1' then
        return 'RR'
    end
    return wheelName;
end

local function toSuspensionPosition(invQuat, wheel)
    if wheel ~= nil then
        local wheelPos = vec3(0, 0, 0)
        wheelPos = invQuat * vec3(obj:getNodePosition(wheel.node1))
        return wheelPos.z * 10
    end
    return 0
end

local function getLastSlip(wheelRotator)
    if (wheelRotator ~= nil) then
        return wheelRotator.lastSlip
    end
    return 0
end

local function getIsDeflated(wheelRotator)
    if (wheelRotator ~= nil) then
        return wheelRotator.isTireDeflated
    end
    return 0
end

local function getWheelSpeed(wheel)
    if(wheel~= nil) then
        return abs(wheel.angularVelocity * wheel.radius)
    end
    return 0
end

local function getWheelRPS(wheel)
    if(wheel~= nil) then
        return abs(wheel.angularVelocity / 6.28318530718)
    end
    return 0
end

local function getWheelsByName()
    local wheelsByName = {}
    for _, wheel in pairs(wheels.wheels) do
        wheelsByName[fitWheelName(wheel.name)] = wheel
    end
    return wheelsByName
end

local function getWheelsRotatorByName()
    local wheelRotatorsByName = {}
    for i = 0, wheels.wheelRotatorCount - 1 do
        local wheel = wheels.wheelRotators[i]
        wheelRotatorsByName[fitWheelName(wheel.name)] = wheel
    end
    return wheelRotatorsByName
end

local function getContactMaterial(contactData)
    if (contactData ~= nil) then
        return contactData.contactMaterial
    end
    return 0
end

local function getContactDepth(contactData)
    if (contactData ~= nil) then
        return contactData.contactDepth
    end
    return -1
end

local function getIsRubberTire(contactData)
    if (contactData ~= nil) then
        if (contactData.isRubberTire) then
            return 1
        end
    end
    return 0
end

local function getWheelsContactDataByName()
    local wheelsContactByName = {}
    for wi, wd in pairs(wheels.wheels) do
        -- log("I", "",wd.name)
        wheelsContactByName[fitWheelName(wd.name)] = {}
        local mat, mat2 = wd.contactMaterialID1, wd.contactMaterialID2
        if mat == 4 then
            mat, mat2 = mat2, mat
        end
        local isRubberTire = mat2 == 4 and wd.hasTire -- if the tire is rubber
        wheelsContactByName[fitWheelName(wd.name)] = {
            isRubberTire = isRubberTire,
            contactMaterial = mat,
            contactDepth = wd.contactDepth
        }
    end
    return wheelsContactByName
end

local function isVehicleReady()
    if not electrics.values.watertemp then
        -- vehicle not completly initialized, skip sending package
        return false
    end
    return true
end

local function getThermal(wheelName, valueName)
    local wheel = electrics.values.wheelThermals[wheelName]
    if (wheel ~= nil) then
        return wheel[valueName] or -1
    end
    return -1
end

local printed = false
local function printOnce(val)
  if not printed then
    print(val)
  end
  printed = true
end

local function sendData(dt, ip, port)
    if not isVehicleReady() then
        return
    end

    --printOnce(jsonEncode(v.data))

    local o = ffi.new("simhub_telemetry")
    o.magic = "magic"
    o.timeStamp = timeStamp
	o.check = 128
    
    o.vehicleName = v.data.information.name or ""
    o.vehicleModel = v.config.model or ""
    o.vehicleConfig = v.config.partConfigFilename or ""

    -- Engine data
    local engine = powertrain.getDevice("mainEngine")
    if engine ~= nil then
        o.idle_rpm = engine.idleRPM or 0
        o.max_rpm = engine.maxRPM or 0
    end

    -- Gearbox data
    local gearbox = powertrain.getDevice("gearbox")
    if gearbox ~= nil then
        o.max_gears = gearbox and gearbox.maxGearIndex or 0
        o.gearboxGrinding =  gearbox and gearbox.isGrindingShift or 0
    end

    -- Lights data
    o.light_HighBeam = electrics.values.highbeam or 0
    o.light_Parkingbrake = electrics.values.parkingbrake or 0
    o.light_Signal_L = electrics.values.signal_L or 0
    o.light_Signal_R = electrics.values.signal_R or 0
    
    local hasABS = electrics.values.hasABS or false
    if hasABS then
        o.light_Abs = electrics.values.abs or 0
    end

    o.light_Oil = damageTracker.getDamage("engine", "blockMelted") or
   damageTracker.getDamage("engine", "catastrophicOverTorqueDamage") or
   damageTracker.getDamage("engine", "catastrophicOverrevDamage") or
   damageTracker.getDamage("engine", "coolantHot") or
   damageTracker.getDamage("engine", "cylinderWallsMelted") or
   damageTracker.getDamage("engine", "engineDisabled") or
   damageTracker.getDamage("engine", "engineHydrolocked") or
   damageTracker.getDamage("engine", "engineIsHydrolocking"	) or
   damageTracker.getDamage("engine", "engineLockedUp") or
   damageTracker.getDamage("engine", "engineReducedTorque") or
   damageTracker.getDamage("engine", "exhaustBroken") or
   damageTracker.getDamage("engine", "headGasketDamaged") or
   damageTracker.getDamage("engine", "impactDamage") or
   damageTracker.getDamage("engine", "mildOverTorqueDamage") or
   damageTracker.getDamage("engine", "mildOverrevDamage") or
   damageTracker.getDamage("engine", "oilHot") or
   damageTracker.getDamage("engine", "oilStarvation") or
   damageTracker.getDamage("engine", "overRevDanger") or
   damageTracker.getDamage("engine", "overTorqueDanger") or
   damageTracker.getDamage("engine", "pistonRingsDamaged") or
   damageTracker.getDamage("engine", "radiatorLeak") or
   damageTracker.getDamage("engine", "rodBearingsDamaged") or
   damageTracker.getDamage("engine", "turbochargerDamaged") or
   damageTracker.getDamage("engine", "turbochargerHot") or 0

    o.light_EngineRunning = electrics.values.engineRunning or 0
    
    local escVal = 0; 
    local tcVal  = 0;
    if electrics.values.hasESC or false then
        if (electrics.values.escActive and 1 or 0) > 0 then
            escVal = electrics.values.esc
        end
        if (electrics.values.tcsActive and 1 or 0) > 0 then
            escVal = electrics.values.tcs
        end
    end

    o.light_TC = sternary((escVal + tcVal) > 0, 1, 0)    
    
    o.light_Shift = sternary(electrics.values.shouldShift, 1, 0)
    o.light_HazardEnabled = electrics.values.hazard_enabled or 0


    

    o.light_LowHighBeam = electrics.values.lowhighbeam or 0;
    o.light_LowBeam = electrics.values.lowbeam or 0;
    o.light_Fog = electrics.values.fog or 0;

    -- Engine
    o.gear = (electrics.values.gearIndex or 0) + 1 -- reverse = 0 here
    o.speed = electrics.values.wheelspeed or electrics.values.airspeed or 0
    o.rpm = electrics.values.rpm or 0
    o.turbo = electrics.values.cruiseControlActive or 0
    o.turboMax = electrics.values.checkengine or 0

    o.engTemp = electrics.values.watertemp or 0
    o.fuel = electrics.values.fuel or 0
    o.fuelCapacity = electrics.values.fuelCapacity or 0
    o.fuelVolume = electrics.values.fuelVolume or 0
    o.oilTemp = electrics.values.oiltemp or 0
    o.waterTemp = electrics.values.watertemp or 0
    o.engineLoad = electrics.values.engineLoad or 0
    o.ignitionOn = electrics.values.ignition or 0

    -- Input data
    o.input_throttle = electrics.values.throttle or 0
    o.input_brake = electrics.values.brake or 0
    o.input_clutch = electrics.values.clutch or 0
    o.input_parkingBrake = electrics.values.parkingbrake or 0
    o.input_steeringPercent = electrics.values.cruiseControlTarget or 0

    -- Motion data
    o.posX, o.posY, o.posZ = obj:getPositionXYZ()

    local velocity = obj:getVelocity()
    o.velX = electrics.values.doorFLCoupler_notAttached or electrics.values.doorLCoupler_notAttached or 0
    o.velY = electrics.values.doorFRCoupler_notAttached or electrics.values.doorRCoupler_notAttached or 0
    o.velZ = electrics.values.doorRLCoupler_notAttached or 0

    o.accX = electrics.values.doorRRCoupler_notAttached or 0
    o.accY = electrics.values.hoodLatchCoupler_notAttached or 0
    o.accZ = electrics.values.tailgateCoupler_notAttached or 0

    local upVector = obj:getDirectionVectorUp()
    local vectorForward = obj:getDirectionVector()

    local quat = quatFromDir(vectorForward, upVector)
    local euler = quat:toEulerYXZ()

    o.upVecX = upVector.x
    o.upVecY = upVector.y
    o.upVecZ = upVector.z

    o.forwardVecX = vectorForward.x
    o.forwardVecY = vectorForward.y
    o.forwardVecZ = vectorForward.z

    local rollRate = obj:getRollAngularVelocity()
    local pitchRate = obj:getPitchAngularVelocity()
    local yawRate = obj:getYawAngularVelocity()

    o.rollPos = -euler.z --negated angle here, seems like that is the "standard" for motion sims here
    o.pitchPos = -euler.y --negated angle here, seems like that is the "standard" for motion sims here
    o.yawPos = euler.x

    o.rollRate = rollRate
    o.pitchRate = pitchRate
    o.yawRate = yawRate

    o.rollAcc = (rollRate - lastSimHubFrameData.rollRate) / dt
    o.pitchAcc = (pitchRate - lastSimHubFrameData.pitchRate) / dt
    o.yawAcc = (yawRate - lastSimHubFrameData.yawRate) / dt

    -- Wheels data
    local upVector = obj:getDirectionVectorUp()
    local vectorForward = obj:getDirectionVector()
    local quat = quatFromDir(vectorForward, upVector)
    local euler = quat:toEulerYXZ()
    local invQuat = quat:inversed()
    local wheelPos = vec3(0, 0, 0)

    -- Extract rotators by name
    local wheelRotatorsByName = getWheelsRotatorByName()

    -- Extract wheels by name
    local wheelsByName = getWheelsByName()

    -- Extract wheels contact infos by name
    local wheelsContactByName = getWheelsContactDataByName()

    o.suspension_position_fl = toSuspensionPosition(invQuat, wheelsByName["FL"])
    o.suspension_position_fr = toSuspensionPosition(invQuat, wheelsByName["FR"])
    o.suspension_position_rl = toSuspensionPosition(invQuat, wheelsByName["RL"])
    o.suspension_position_rr = toSuspensionPosition(invQuat, wheelsByName["RR"])

    o.suspension_velocity_fl = (o.suspension_position_fl - lastSimHubFrameData.suspension_position_fl) / dt
    o.suspension_velocity_fr = (o.suspension_position_fr - lastSimHubFrameData.suspension_position_fr) / dt
    o.suspension_velocity_rl = (o.suspension_position_rl - lastSimHubFrameData.suspension_position_rl) / dt
    o.suspension_velocity_rr = (o.suspension_position_rr - lastSimHubFrameData.suspension_position_rr) / dt

    o.suspension_acceleration_fl = (o.suspension_velocity_fl - lastSimHubFrameData.suspension_velocity_fl) / dt
    o.suspension_acceleration_fr = (o.suspension_velocity_fr - lastSimHubFrameData.suspension_velocity_fr) / dt
    o.suspension_acceleration_rl = (o.suspension_velocity_rl - lastSimHubFrameData.suspension_velocity_rl) / dt
    o.suspension_acceleration_rr = (o.suspension_velocity_rr - lastSimHubFrameData.suspension_velocity_rr) / dt

    o.wheel_speed_fl = getWheelSpeed(wheelsByName["FL"])
    o.wheel_speed_fr = getWheelSpeed(wheelsByName["FR"])
    o.wheel_speed_rl = getWheelSpeed(wheelsByName["RL"])
    o.wheel_speed_rr = getWheelSpeed(wheelsByName["RR"])

    o.wheel_RPS_fl = getWheelRPS(wheelsByName["FL"])
    o.wheel_RPS_fr = getWheelRPS(wheelsByName["FR"])
    o.wheel_RPS_rl = getWheelRPS(wheelsByName["RL"])
    o.wheel_RPS_rr = getWheelRPS(wheelsByName["RR"])

    o.wheel_slip_fl = getLastSlip(wheelRotatorsByName["FL"])
    o.wheel_slip_fr = getLastSlip(wheelRotatorsByName["FR"])
    o.wheel_slip_rl = getLastSlip(wheelRotatorsByName["RL"])
    o.wheel_slip_rr = getLastSlip(wheelRotatorsByName["RR"])

    o.wheel_contactMaterial_fl = getContactMaterial(wheelsContactByName["FL"])
    o.wheel_contactMaterial_fr = getContactMaterial(wheelsContactByName["FR"])
    o.wheel_contactMaterial_rl = getContactMaterial(wheelsContactByName["RL"])
    o.wheel_contactMaterial_rr = getContactMaterial(wheelsContactByName["RR"])

    o.wheel_contactDepth_fl = getContactDepth(wheelsContactByName["FL"])
    o.wheel_contactDepth_fr = getContactDepth(wheelsContactByName["FR"])
    o.wheel_contactDepth_rl = getContactDepth(wheelsContactByName["RL"])
    o.wheel_contactDepth_rr = getContactDepth(wheelsContactByName["RR"])

    o.wheel_isRubberTire_fl = getIsRubberTire(wheelsContactByName["FL"])
    o.wheel_isRubberTire_fr = getIsRubberTire(wheelsContactByName["FR"])
    o.wheel_isRubberTire_rl = getIsRubberTire(wheelsContactByName["RL"])
    o.wheel_isRubberTire_rr = getIsRubberTire(wheelsContactByName["RR"])

    o.wheel_isDeflated_fl = getIsDeflated(wheelRotatorsByName["FL"])
    o.wheel_isDeflated_fr = getIsDeflated(wheelRotatorsByName["FR"])
    o.wheel_isDeflated_rl = getIsDeflated(wheelRotatorsByName["RL"])
    o.wheel_isDeflated_rr = getIsDeflated(wheelRotatorsByName["RR"])

    o.brake_coreTemperature_fl = getThermal("FL", "brakeCoreTemperature")
    o.brake_coreTemperature_fr = getThermal("FR", "brakeCoreTemperature")
    o.brake_coreTemperature_rl = getThermal("RL", "brakeCoreTemperature")
    o.brake_coreTemperature_rr = getThermal("RR", "brakeCoreTemperature")

    o.brake_surfaceTemperature_fl = getThermal("FL", "brakeSurfaceTemperature")
    o.brake_surfaceTemperature_fr = getThermal("FR", "brakeSurfaceTemperature")
    o.brake_surfaceTemperature_rl = getThermal("RL", "brakeSurfaceTemperature")
    o.brake_surfaceTemperature_rr = getThermal("RR", "brakeSurfaceTemperature")

    -- Keep state for deltas
    lastSimHubFrameData.suspension_position_fl = o.suspension_position_fl
    lastSimHubFrameData.suspension_position_fr = o.suspension_position_fr
    lastSimHubFrameData.suspension_position_rl = o.suspension_position_rl
    lastSimHubFrameData.suspension_position_rr = o.suspension_position_rr
    lastSimHubFrameData.rollRate = rollRate
    lastSimHubFrameData.pitchRate = pitchRate
    lastSimHubFrameData.yawRate = yawRate

    local packet = ffi.string(o, ffi.sizeof(o)) --convert the struct into a string
    udpSocket:sendto(packet, ip, port)
end

local function updateGFX(dt)
    if not playerInfo.firstPlayerSeated then
        return
    end
    timeStamp = timeStamp + dt
    updateTimer = updateTimer + dt
    local accXRaw = -obj:getSensorX()
    local accYRaw = -obj:getSensorY()
    local accZRaw = -obj:getSensorZ()
    accX = accelerationSmoothingX > 0 and accXSmoother:get(accXRaw) or accXRaw
    accY = accelerationSmoothingY > 0 and accYSmoother:get(accYRaw) or accYRaw
    accZ = accelerationSmoothingZ > 0 and accZSmoother:get(accZRaw) or accZRaw

    if updateTimer >= updateTime then
        sendData(dt, ip, port)
        updateTimer = 0
    end
end

local function onExtensionLoaded()
    if not ffi then
        log("E", "outgauge", "Unable to load SimHub extras module: Lua FFI required")
        return false
    end

    port = settings.getValue("simhubPort") or 9999

    if not udpSocket then
        udpSocket = socket.udp()
    end

    updateTime = 1 / updateRate
    accelerationSmoothingX = settings.getValue("motionSimAccelerationSmoothingX") or 30
    accelerationSmoothingY = settings.getValue("motionSimAccelerationSmoothingY") or 30
    accelerationSmoothingZ = settings.getValue("motionSimAccelerationSmoothingZ") or 30
    accXSmoother = newExponentialSmoothing(accelerationSmoothingX)
    accYSmoother = newExponentialSmoothing(accelerationSmoothingY)
    accZSmoother = newExponentialSmoothing(accelerationSmoothingZ)

    resetState()

    log("I", "", "SimHub extras initialized for: " .. tostring(ip) .. ":" .. tostring(port))

    return true
end

local function onExtensionUnloaded()
    if udpSocket then
        udpSocket:close()
    end
    udpSocket = nil
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.updateGFX = updateGFX
M.resetState = resetState
M.sendData = sendData

return M
