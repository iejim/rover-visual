# -*- coding: utf-8 -*-
import numpy as np
from numpy import array as ar
from spatialmath import SE2, base
from coppeliaSincro import Cliente
import math
from math import pi,cos,sin,atan2

''' 
 Código tomado de un programa de Python que se conecta
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
'''

class Robot(Cliente):
    
    def __init__(self,conectar=True):
        super().__init__(conectar)

        # Velocidades actuales
        self.v_act = 0
        self.w_act = 0
        # Velocidades de ruedas
        self.wL = 0
        self.wR = 0

        # Velocidad máxima
        self.v_max = 1

        # Dimensiones del robot
        self.d = 1
        self.R = 1

        # Handles de los joints
        self.ML = 0
        self.MR = 0

        # Lista de Sonares
        self.sonares = []
        self.sonar_max = 0.66

        # Odometría
        self.postura = array([[0],[0],[0]])

        self.h_posturas = []
        
        self.last_time = 0

        # Handle del Centro cinemático (eje) del robot
        self.centro = 0
        # Handle de Goal para visualización
        self.goal_dummy_handle = 0
        self.goal_dummy_actualizado = False
        #Atributos para implementar funciones fuera de la clase
        self.actuar_func = None
        self.detectar_func = None
        self.init_func = None

    # Reemplaza función de coppeliaSim
    def getHandle(self, nombre):
        r, h = self.client.simxGetObjectHandle(nombre, self.call()) # llamadas Bloquean la Sim
        if r:
            return h 
        return -1
    
    # Reemplaza función de coppeliaSim
    def getObjectPosition(self, obj, rel_obj=-1):
        r = self.client.simxGetObjectPosition(obj, rel_obj, self.call())
        if r[0]: # success
            return r[1]
    
    # Reemplaza función de coppeliaSim
    def setObjectPosition(self, obj, pos, rel_obj=-1):
        self.client.simxSetObjectPosition(obj, rel_obj, pos, self.pub())
        
    # Hace el papel de sysCall_init()
    # Inicializa las variables internas del robot
    def init(self):
        ''' Inicializar los valores internos a los del robot.'''
        # Dimensiones del Robot
        self.d = 0.092 # 9.2 cm
        self.R = 0.032224 #3.2 cm

        self.v_max= 0.30 #m/s
        
        self.ML = self.getHandle("ML")
        self.MR = self.getHandle("MR") 
        self.centro = self.getHandle("centro_cine")
        self.goal_dummy_handle = self.getHandle("Goal")

        self.pista = self.getHandle("Pista_Sumo")
        
        for i in range(1,4):
            self.sonares.append(self.getHandle("Sonar%d"%i))

        # Ganancias controladores
        self.go2goal_gains = {'K_w': 0.5, 'K_v':0.2}
        self.go2pose_gains = {'K_w1': 0.5, 'K_w2': 0.3, 'K_v':0.2}
        self.atraccion_gains = {'K_w': 0.5, 'K_v':0.2}
        self.deambular_gain = 0.8

        self.postura = array([[0],[0],[0]])
        self.last_time = 0
        self.v_act = 0
        self.w_act = 0
        self.wL = 0
        self.wR = 0
        self.goal_dummy_actualizado = False
        
            
    # Hace el papel de sysCall_actuation()
    # No necesita traducirse
    def actuation(self):
        pass

    # Hace el papel de sysCall_actuation()
    # No necesita traducirse
    def sensing(self):
        pass    


     #### Movimiento

    # Mueve en línea recta
    def avanzar(self, vel):
        self.desplazar(vel,0)
    
    # Le pasa las velocidades a las ruedas
    def mover(self, wR=0, wL=0):
        # Guardar
        self.wL = wL
        self.wR = wR
        # Enviar
        self.client.simxSetJointTargetVelocity(self.MR, wR, self.pub())
        self.client.simxSetJointTargetVelocity(self.ML, wL, self.pub())
    
    # Calcula las velocidades que necesita cada rueda
    def desplazar(self, v, w):
        wR = (2*v+self.d*w)/(2*self.R) 
        wL = (2*v-self.d*w)/(2*self.R)
        # Guardar
        self.v_act = v
        self.w_act = w

        self.mover(wR=wR, wL=wL)

    # Mueve siguendo un arco
    def arco(self, v, radio):
        w = v/radio
        self.desplazar(v,w)

    # Convierte velocidades de m/s a rad/s
    def ms_a_rads(self, vel):
        return vel/self.R
    
    #### Detección
    def leerSonar(self, disp):
        # Devuelve la distancia desde el sonar 
        r = self.client.simxReadProximitySensor(disp, self.call())
        #print(r)
        if r[0] and r[1]>0:
            return r[2] # Distancia reportada por el sonar
        return self.sonar_max 
    
    def leer_distancias(self):
        '''Lee todos los sonares y devuelve un array con los valores.'''
        d = []
        for sonar in self.sonares:
            d.append(self.leerSonar(sonar))
        return np.array(d)

    def actualizar_postura(self):
        p = self.getObjectPosition(self.centro, rel_obj=self.pista)
        o = self.getObjectOrientation(self.centro, rel_obj=self.pista)
        self.postura = array([p[0], p[1], o[3]]) # x,y,g_ang

    def actualizar_goal_pos(self, pos):
        p0 = self.getObjectPosition(self.goal_dummy_handle, rel_obj=self.centro)
        p0[0:2] = pos[0:2]
        self.setObjectPosition(self.goal_dummy_handle,p0, rel_obj=self.centro)



    #### Controladores
    def go2goal(self,goal, v=None):
        '''Controla para llegar a goal (x,y) controlando el arco.'''
        completo  = False
        if not self.goal_dummy_actualizado:
          self.actualizar_goal_pos(goal.flatten().tolist())
          self.goal_dummy_actualizado = True

        
        K_w = self.go2goal_gains['K_w']
        K_v = self.go2goal_gains['K_v']

        d_min = 0.01 # metros

        pos = self.postura
        p = pos[0:2]
        th = pos[2].item()
        #print(p, goal)
        # TODO: revisar que goal sea array()

        e_pos = goal - p 
        e_dist = np.linalg.norm(e_pos)

        # Error de orientación
        angle = atan2(e_pos[1], e_pos[0])  

        e_ang = angle - th
        e_ang = atan2(sin(e_ang), cos(e_ang)) # Correr asegurar el ángulo adecuao

        # Ley de control
        w = K_w*e_ang
        #if v == None:
        #  v = self.v_act
        #print("G2G D: ", e_dist)
        v = K_v*e_dist
        if e_dist < d_min:
          v = 0
          w = 0
          completo  = True
          self.goal_dummy_actualizado = False # Débil aquí, pero blah

        self.desplazar(v,w)
        return completo
        
    def go2pose(self, pose_g, v=None):
        '''Controla para llegar a goal (x,y) controlando el arco.'''

        completo  = False

        if not self.goal_dummy_actualizado:
          self.actualizar_goal_pos(pose_g.flatten().tolist())
          self.goal_dummy_actualizado = True

        K_w1 = self.go2pose_gains['K_w1']
        K_w2 = self.go2pose_gains['K_w2']
        K_v = self.go2pose_gains['K_v']

        d_min = 0.01 # metros

        pos = self.postura
        p_robot = pos[0:2]
        th_robot = pos[2].item()
        #print(p, goal)
        # TODO: revisar que goal sea array()
        goal = pose_g[0:2]
        th_goal = pose_g[2].item()
        
        # Errores: Ecuación 3.65 de Correll (con ajustes)
        e_pos = goal - p_robot
        e_dist = np.linalg.norm(e_pos) #Rho
        
        # Dirección del punto: 
        angle = atan2(e_pos[1], e_pos[0])  

        # Error de dirección (al goal)
        alpha = angle - th_robot
        alpha = atan2(sin(alpha), cos(alpha)) # Correr para asegurar el ángulo adecuado
        
        # Error de orientación (a postura)
        eta = th_goal - th_robot
        eta = atan2(sin(eta), cos(eta))

        # Ley de control
        # Implementar ecuaciones 3.66 y 3.67 de Correll
        #print("angle: %0.2f, a: %0.2f, n: %0.2f" %(angle, alpha, eta))
        w = K_w1*alpha + K_w2*eta
        #if v == None:
        #  v = self.v_act

        v = K_v*e_dist
        if e_dist < d_min:
          v = 0
          w = 0
          completo  = True
          self.goal_dummy_actualizado = False # Débil aquí, pero blah

        self.desplazar(v,w)
        return completo
    # Comportamientos Pasivos (no mueven el robot)
    def atraccion(self, goal, v=0):
        '''Controla para llegar a goal (x,y) controlando el arco.'''

        
        K_w = self.atraccion_gains['K_w']
        K_v = self.atraccion_gains['K_v']

        d_min = 0.01 # metros

        pos = self.postura
        p = pos[0:2]
        th = pos[2].item()
        #print(p, goal)
        # TODO: revisar que goal sea array()

        e_pos = goal - p 
        e_dist = np.linalg.norm(e_pos)

        # Error de orientación
        angle = atan2(e_pos[1], e_pos[0])  # atan va de [-pi/2, pi]

        e_ang = angle - th
        e_ang = atan2(sin(e_ang), cos(e_ang)) # Correr asegurar el ángulo adecuao

        # Ley de control
        w = K_w*e_ang
        #if v == None:
        #  v = self.v_act
        #print("G2G D: ", e_dist)
        v = K_v*e_dist
        if e_dist < d_min:
          v = 0
          w = 0

        return (v,w)
    
    def evasion(self, v=0):
        '''Cambia su direccion ante cualquier objeto.'''
        # Leer sensores
        # Cambiar la dirección lejos de la detección
        dis = self.leer_distancias()

        #lados = self.sonar_max - array([ dis[0], dis[1], dis[3], dis[4] ]) #no incluir el frente
        lados = self.sonar_max - dis #usar el frente para lograr girar si es solo este.
        
        frente = self.sonar_max - dis[2]
        
        # Vector (tipo MATLAB) de ganancias
        ganancias_lados = array([ 1, 0.9, -1 ]) * 0.5 #darle poco peso al frente.
        ganancia_frente = 0.2

        # Para combinarlos con otros, deberían ser solo el delta (sin suma)
        w = np.dot(lados, ganancias_lados)
        v = v

        return (w,v)
    
    # Comportamientos Activos (sí mueven el robot)
    def miedo(self, v=0):
        '''Se aleja de cualquier objeto.'''
        # Leer sensores
        # Cambiar la dirección lejos de la detección
        w, v = self.evasion()

        # Alejarse si es necesario.
        if v == 0:
            v = self.v_max*0.3

        v = v - ganancia_frente*frente 
        
        self.desplazar(v,w)

    def curioso(self, v=0):
        '''Se acerca a todo lo que ve.'''
        # Leer sensores
        # Cambiar la dirección hacia la detección
        if v == 0:
            v = self.v_max*0.3

        dis = self.leer_distancias()

        lados = self.sonar_max - dis
        frente =  lados[1]

        # Vector (tipo MATLAB) de ganancias
        ganancias_lados = array([ -2, 0.5, 2 ]) * 0.5
        ganancia_frente = -0.4

        #Producto punto entre dos vectores
        w = np.dot(lados, ganancias_lados)
        v = v + ganancia_frente*frente 

        self.desplazar(v,w)

    def interes(self, dir_g, v=0):
        '''Intenta ir en una dirección.'''
        if v == 0:
            v = self.v_max*0.3
        
        pos = self.postura ## Idealmente se haría con algún tipo de faro
        #p_robot = pos[0:2]
        th_robot = pos[2].item()
        
        e = dir_g - th_robot
        e = atan2(sin(e), cos(e))
        
        Ke = self.deambular_gain
        w = Ke*e

        self.desplazar(v,w)
        
    def deambular(self, v=0):
        '''Anda sin rumbo específico.'''
        if v == 0:
            v = self.v_max*0.3
        # Cambiar la direccion paulatinamente.
        
        alpha = self.deambular_gain
        a = 6
        b = a/2
        dth = random()*a - b
        # Moving average
        w = alpha*self.w_act + (1-alpha)*dth

        self.desplazar(v,w)
