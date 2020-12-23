--[[
Control de un Rover de Hands-On para entrenamiento 
en técnicas básicas de visión usando CoppeliaSim

Ivan Jiménez, y otros.
2021 - Ingeniería Mecatrónica
Instituto Tecnológico de Santo Domingo

--]]


function sysCall_init()
  
  MotorL = sim.getObjectHandle('Motor1')
  MotorR = sim.getObjectHandle('Motor2')
  -- Sensores de línea (visión)
  lDelantero = sim.getObjectHandle('Delantero')
  lIzquierda = sim.getObjectHandle('Izquierda')
  lDerecha = sim.getObjectHandle('Derecha')

  -- Sesores de objetos delanteros (vision)
  Objeto = sim.getObjectHandle('Objeto')
  Objeto0 = sim.getObjectHandle('Objeto0')
  Objeto1 = sim.getObjectHandle('Objeto1')

  -- Motor para subir el brazo
  Base = sim.getObjectHandle('Motor5')
  gripperName='BaxterGripper'

  -- Sensor de presencia delantero (laser)
  Presencia = sim.getObjectHandle('Pres')
 
  
  -- Memorias de velocidad asignada
  wL = 0
  wR = 0

  -- Memorias de lectura de sensores
  Del = {}
  Del2 = {}

  -- Posición inicial del brazo
  abrirBrazo()
  subirBrazo()
end

--[[
cerrarBrazo(): cierra el gripper del brazo por medio de una señal
--]]
function cerrarBrazo()
  sim.setIntegerSignal('_close',1)
  -- cerrado = true
end

--[[
abrirBrazo(): abre el gripper del brazo por medio de una señal
--]]
function abrirBrazo()
  sim.setIntegerSignal('_close',0)
  -- cerrado = false
end

--[[
subirBrazo(pos): sube el brazo a la posición de levantar
(opcional) pos: posición angular (rad) [default: -50 grados]
--]]
function subirBrazo(pos)
  local p = pos and pos or -50*math.pi/180
  sim.setJointTargetPosition(Base,p)
  -- brazo_arriba = true
end

--[[
bajarBrazo(pos): baja el brazo a la posición de recoger
(opcional) pos: posición angular (rad) [default: +5 grados]
--]]
function bajarBrazo(pos)
  local p = pos and pos or 5*math.pi/180 -- Hace que pos sea opcional
  sim.setJointTargetPosition(Base,p)
end

--[[
mover(wL, wR): asigna velocidades a las ruedas del vehículo
w_L: velocidad angular del motor izquierdo (rad/s)
w_R: velocidad angular del motor derecho (rad/s)
--]]
function mover(w_L, w_R)
  sim.setJointTargetVelocity(MotorL,w_L)
  sim.setJointTargetVelocity(MotorR,w_R)
  -- Guardar
  wL = w_L
  wR = w_R
end

--[[
leerSensorVis(sensor): lee un sensor y devuelve los datos obtenidos
sensor: handle al sensor a leer
--]]
function leerSensorVis(sensor)
  r, data = sim.getVisionSensorImage(sensor)
  return data
end


function sysCall_actuation()
  lectura()
  print(Z)
  print(acabo)
  -- print(sim.getSimulationTime()-obvio)
  if vuelta==false and STOP==false then
    if Centro==true then
      Z=Rojo+Verde+Azul
      if(Z==2) then
        acabo=true
      end
      final=false
      if  backup==true and aux==false then
        if acabo==true then
          camino=4
        elseif posicion==2 then
          camino=1
        elseif posicion==1 then
          camino=3
        elseif posicion==3 then
          camino=2
        elseif posicion==4 then
          camino=4
        end
        aux=false
        Centro=false
        encontrado =false
        iniciando=false
        quedate=false
        organizando=true
        agarrando =false
        vuelta=false
        -- tiene=false
        final=false
      elseif tiene==true  then
        guarda()
        
        
      end
    end
    if iniciando==true then
      inicio()
    end
    if organizando==true then
      organizar()
    end
  end
  if vuelta==true then
    giro()
  end
end

function sysCall_sensing()
  -- Actualizar las lecturas de los sensores
  Del = leerSensorVis(Delantero)
  Del2 = leerSensorVis(Delantero+sim.handleflag_greyscale)

  -- etcetera
end