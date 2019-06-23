#!/usr/bin/python

print ('### Script:', __file__)

import sys
import time
import vrep
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from typing import List

# --------------------------------------------------------------------------

def get_motor_handles(clientID):
    opmode = vrep.simx_opmode_blocking
    err, lmh = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', opmode)
    # print ('### Left motor handle:', err, lmh)

    err, rmh = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', opmode)
    # print ('### Right motor handle:', err, rmh)

    return lmh, rmh

# --------------------------------------------------------------------------

def read_all_sonar(clientID):
    # simxGetObjectGroupData(clientID, objectType, dataType, operationMode)
    s = vrep.simxGetObjectGroupData(clientID,
		vrep.sim_object_proximitysensor_type, 13,
		vrep.simx_opmode_blocking)

    r = [1.0] * 16
    if s[0] == 0:
        for i in range(16):
            if s[2][2*i] == 1:
                r[i] = s[3][6*i+2]
			
    #print ('### Sonar readings:', r)
    #print ('### Sonar readings:', r[1], r[3], r[4], r[6])
    return r

# --------------------------------------------------------------------------

def set_speed(clientID, lmh, rmh, lspeed, rspeed):
    opmode = vrep.simx_opmode_oneshot
    vrep.simxSetJointTargetVelocity(clientID, lmh, lspeed, opmode)
    vrep.simxSetJointTargetVelocity(clientID, rmh, rspeed, opmode)

# --------------------------------------------------------------------------

def avoid(sonar):

    #0 - sensor 8
    #1 - sensor 7
    #2 - sensor 6
    #3 - sensor 5
    #4 - sensor 4
    #5 - sensor 3
    #6 - sensor 2
    #7 - sensor 1
    sonarTotal = sonar[0:8]
	
    linearCsq = velocidadLineal()
    angularCsq = velocidadAngular()

    sensors = []
    for n in range(0,8):
        sensors.append(proximidadSensor('sensor'+str(n)))
    
    sensoresFrontales = [sensors[3], sensors[4]]
    sensoresDerechos = [sensors[0], sensors[1], sensors[2]]
    sensoresIzquierdos = [sensors[5], sensors[6], sensors[7]]

    #print("lineal", linearCsq)
    #print("Angular", angularCsq)
    #print("Angular", angularCsq['recto'])
    #print("sensoresFrontales1", sensoresFrontales[0])
    #print("sensoresFrontales1", sensoresFrontales[0]['lejos'])
    #print("sensoresFrontales2", sensoresFrontales[1])
    #print("sensoresFrontales2", sensoresFrontales[1]['lejos'])
    #print("sensoresIzquierdos1", sensoresIzquierdos[0])
    #print("sensoresIzquierdos2", sensoresIzquierdos[1])
    #print("sensoresIzquierdos3", sensoresIzquierdos[2])
    #print("sensoresDerechos1", sensoresDerechos[0])
    #print("sensoresDerechos2", sensoresDerechos[1])
    #print("sensoresDerechos3", sensoresDerechos[2])
  
    rules = crearReglas(sensoresFrontales, sensoresIzquierdos, sensoresDerechos, linearCsq, angularCsq)

    tipping_ctrl = ctrl.ControlSystem(rules)
    tipping = ctrl.ControlSystemSimulation(tipping_ctrl)

    for i in range(0,8):
        tipping.input['sensor'+str(i)] = sonar[i]

    tipping.compute()

    print("Velocidad Lineal", tipping.output['velocidad'])
    print("Velocidad Angular", tipping.output['angularVel'])

    if(tipping.output['angularVel'] < -0.05):
        lspeed, rspeed = tipping.output['velocidad'], tipping.output['angularVel']
    elif(tipping.output['angularVel'] >0.05):
        lspeed, rspeed = tipping.output['angularVel'], tipping.output['velocidad']
    else:
        lspeed, rspeed = tipping.output['velocidad'], tipping.output['velocidad']
        
    return lspeed, rspeed

# --------------------------------------------------------------------------
############################ Metodos Auxiliares ############################
# --------------------------------------------------------------------------

# Variables y estados
def proximidadSensor(name: str):
    proximidad = ctrl.Antecedent(np.arange(0, 1, 0.01), name)
    proximidad['muyCerca'] = fuzz.trapmf(proximidad.universe, [0, 0, 0.2, 0.3])
    proximidad['cerca'] = fuzz.trimf(proximidad.universe, [0.2, 0.3, 0.5])
    proximidad['medio'] = fuzz.trimf(proximidad.universe, [0.3, 0.6, 0.9])
    proximidad['lejos'] = fuzz.trapmf(proximidad.universe, [0.8, 0.9, 1, 1])
    return proximidad

def velocidadLineal():
    velocidad = ctrl.Consequent(np.arange(-0.5, 1.5, 0.01), 'velocidad')
    velocidad['atras'] = fuzz.trimf(velocidad.universe, [-0.5, -0.01, 0])
    velocidad['stop'] = fuzz.trimf(velocidad.universe, [-0.01, 0, 0.01])
    velocidad['lento'] = fuzz.trimf(velocidad.universe, [0.01, 0.3, 0.7])
    velocidad['rapido'] = fuzz.trapmf(velocidad.universe, [0.5, 1.1, 1.5, 1.5])
    velocidad.defuzzify_method = 'centroid'
    return velocidad

def velocidadAngular():
    angularVel = ctrl.Consequent(np.arange(-0.2, 0.21, 0.01), 'angularVel')
    angularVel.automf(7, "quant", ["muyDerecha", "derechaLigero", "derecha", "recto", "izquierda", "izquierdaLigero", "muyIzquierda"])
    angularVel.defuzzify_method = 'mom'
    return angularVel

#Reglas

def crearReglas(sensoresFrontales: List[ctrl.Antecedent], sensoresIzquierdos: List[ctrl.Antecedent], sensoresDerechos: List[ctrl.Antecedent], velocidad: ctrl.Consequent, angularVel: ctrl.Consequent):

    rules = [

        #SensoresLejos
        ctrl.Rule(sensoresFrontales[0]['lejos'] & sensoresFrontales[1]['lejos'], angularVel['recto']),
        ctrl.Rule(sensoresFrontales[0]['lejos'] & sensoresFrontales[1]['lejos'], velocidad['rapido']),
        ctrl.Rule(sensoresFrontales[0]['lejos'], velocidad['rapido']),
        ctrl.Rule(sensoresFrontales[1]['lejos'], velocidad['rapido']),

        #SensoresMedio
        ctrl.Rule(sensoresFrontales[0]['medio'] & sensoresFrontales[1]['medio'], angularVel['recto']),
        ctrl.Rule(sensoresFrontales[0]['medio'] & sensoresFrontales[1]['medio'], velocidad['lento']),
        
        #sensoresCerca
        ctrl.Rule(sensoresFrontales[0]['cerca'] & sensoresFrontales[1]['cerca'], angularVel['derecha']),
        ctrl.Rule(sensoresFrontales[0]['cerca'] & sensoresFrontales[1]['cerca'], velocidad['stop']),
        ctrl.Rule(sensoresFrontales[0]['cerca'], velocidad['atras']),
        ctrl.Rule(sensoresFrontales[1]['cerca'], velocidad['stop']),

        #sensoresMuyCerca
        ctrl.Rule(sensoresFrontales[0]['muyCerca'] & sensoresFrontales[1]['muyCerca'], angularVel['derechaLigero']),
        ctrl.Rule(sensoresFrontales[0]['muyCerca'] & sensoresFrontales[1]['muyCerca'], velocidad['atras']),
        ctrl.Rule(sensoresFrontales[0]['muyCerca'], velocidad['atras']),
        ctrl.Rule(sensoresFrontales[1]['muyCerca'], velocidad['atras']),
    ]

    #sensoresIzquierdos
    for proximidad in sensoresIzquierdos:
        rules.append(ctrl.Rule(proximidad['muyCerca'], angularVel['muyDerecha']))
        rules.append(ctrl.Rule(proximidad['cerca'], angularVel['derechaLigero']))
        rules.append(ctrl.Rule(proximidad['lejos'], angularVel['izquierda']))

    #sensoresDerechos
    for proximidad in sensoresDerechos:
        rules.append(ctrl.Rule(proximidad['muyCerca'], angularVel['muyIzquierda']))
        rules.append(ctrl.Rule(proximidad['cerca'], angularVel['izquierdaLigero']))
        rules.append(ctrl.Rule(proximidad['lejos'], angularVel['derecha']))

    return rules

# --------------------------------------------------------------------------

print('### Program started')

# print ('### Number of arguments:', len(sys.argv), 'arguments.')
# print ('### Argument List:', str(sys.argv))

vrep.simxFinish(-1) # just in case, close all opened connections

port = int(sys.argv[1])
clientID = vrep.simxStart('127.0.0.2', port, True, True, 2000, 5)

if clientID == -1:
    print ('### Failed connecting to remote API server')

else:
    print ('### Connected to remote API server')
    lmh, rmh = get_motor_handles(clientID)

    while vrep.simxGetConnectionId(clientID) != -1:
        # Perception
        sonar = read_all_sonar(clientID)

        # Planning
        lspeed, rspeed = avoid(sonar)

        # Action
        set_speed(clientID, lmh, rmh, lspeed, rspeed)
        time.sleep(0.1)

    vrep.simxFinish(clientID)

print ('### Program ended')
