--[[
Control de un Rover de Hands-On para entrenamiento 
en técnicas básicas de visión usando CoppeliaSim.

Robot original: Valkyria (J. Almonte y N. Vivieca, 2020)

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

  -- Motor para subir el dedos
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

  -- Posición inicial del dedos
  abrirDedos()
  subirDedos()
end


---------- Movimiento ----------
--[[
cerrarDedos(): cierra el gripper del dedos por medio de una señal
--]]
function cerrarDedos()
  sim.setIntegerSignal('_close',1)
  -- cerrado = true
end

--[[
abrirDedos(): abre el gripper del dedos por medio de una señal
--]]
function abrirDedos()
  sim.setIntegerSignal('_close',0)
  -- cerrado = false
end

--[[
subirDedos(pos): sube el dedos a la posición de levantar
(opcional) pos: posición angular (rad) [default: -50 grados]
--]]
function subirDedos(pos)
  local p = pos and pos or -50*math.pi/180
  sim.setJointTargetPosition(Base,p)
  -- dedos_arriba = true
end

--[[
bajarDedos(pos): baja el dedos a la posición de recoger
(opcional) pos: posición angular (rad) [default: +5 grados]
--]]
function bajarDedos(pos)
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



---------- Detección ----------

--[[
leerSensorVis(sensor): lee un sensor y devuelve los datos de color
sensor: handle al sensor a leer
--]]
function leerSensorVis(sensor)
  r, data = sim.readVisionSensor(sensor)
  return data
end

--[[
leerImagen(sensor): lee un sensor y devuelve la imagen que presenta
sensor: handle al sensor a leer
--]]
function leerImagen(sensor)
  local img = sim.getVisionSensorImage(sensor)
  return img
end

--[[
extraerImagenVIS(sensor): Con simVision, lee un sensor y extrae la imagen al buffer de trabajo.
sensor: handle al sensor a leer
--]]
function extraerImagenVIS(sensor)
  simVision.sensorImgToWorkImg(sensor)
end

--[[
extraerImagenIM(sensor, img=-1): Con simIM, lee un sensor y guarda la imagen en un buffer imagen.
                                 Devuelve el handle al buffer con la imagen.
sensor: handle al sensor a leer
img: handle al buffer de la imagen (o ninguno para crear uno nuevo)
--]]
function extraerImagenIM(sensor, img)
  local img = img or -1
  return simIM.readFromVisionSensor(sensor,img)
end





-----------  Funciones de Simulación  ---------

function sysCall_actuation()
  -- Actualizar los comandos de actuación
  mover(0,0)
end

function sysCall_sensing()
  -- Actualizar las lecturas de los sensores
  
  -- Lee data de color
  Del = leerSensorVis(lDelantero)
  
  -- Lee data de imagen (en escala de grises)
  Del2 = leerImagen(lDelantero+sim.handleflag_greyscale) 

  -- etcetera (¿toma de decisiones y generar comandos de actuación?)
end

function sysCall_cleanup()

end