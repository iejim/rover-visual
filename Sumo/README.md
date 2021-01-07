# Sumo - Extra

Actividad para P1

## Pendientes

1. "Traducir" el código en el archivo _sumo-base.py_ a su versión en Lua, usando el archivo _sumo-base.lua_. 

2. Implementar **en una 2do archivo** un Sumo funcional (según los retos de Hands-On vigentes) usando las funciones traducidas, y expandiéndo el programa y el robot para que este pueda detectar el límite de la pista.

    La escena de coppeliaSim (el archivo _Sumo-Base-Escena.ttt_) ya tiene una copia del robot (incluído en el archivo _Sumo-base-robot.ttm_). Este está conigurado para leer un archivo de nombre _sumo-base.lua_ que contiene el código a ejecutar al correr la simulación. Si desean usar un archivo diferente, deben modificar el script que tiene el robot en coppeliaSim para ajustar el nombre del archivo a correr.

### Explicación

El Código fue tomado de un programa de Python que se conecta
 de manera externa a CoppeliaSim por medio de un cliente,
 (de ahí que la clase Robot sea una subclase de Cliente)

 Todo lo que tenga el prefijo "self." se refiere a un 
 interno de la clase. En la versión de LUA se puede asumir
 que el el robot mismo (el script  que lo controla) es lo
 representado aquí por la clase. Por tanto cualquier 
 variable o función que sea parte de la clase, es decir, 
 que empieza con "self.") es una variable o función parte
 del script del robot en CoppeliaSim.

 De manera similar, se puede asumir que cualquier varible 
 que NO tenga el prefijo "self." es una variable LOCAL a
 la función a la que pertenece. Por tanto, en LUA, esta
 se puede declarar como "local". Por ejemplo
    local a,b,c
    a = 8

 Una observación importante es que, como este programa se
 conecta de forma externa a CoppeliaSim, la funciones que
 representan acciones que se llevan a cabo en el simulador
 (setear una velocidad, por ejemplo), tendran nombres que
 son SIMILARES a los que se usan en LUA. Por ejemplo, lo 
 que en CoppeliaSim se escribiera como
    sim.getObjectHandle(nombre)

 en este código se escribe como
    self.client.simxGetObjectHande(nombre)

 Por su naturaleza de comandos remotos, generalmente estos
 comandos devuelven, además de lo que se les pide, un resul-
 tado que indica si se pudo ejecutar el comando dentro del 
 simulador. Por tanto, habrá casos donde se use una función 
 extra que haga el trabajo del comando,y revise los resultados.
 Tal es el caso de funciones como 
    self.getHandle(nombre)
 
 Por la forma en que funciona la conexión remota con el 
 simulador, todas las funciones que interactúan con el este 
 tienen un último argumento de forma: self.pub() o self.call()
 que en LUA no sería necesario.