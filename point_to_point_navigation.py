#!/usr/bin/env python3
#script: point-to-point navigation - by Diego Rosa, the Pontifical Catholic University of Rio de Janeiro - PUC-Rio
#code used at IronCup 2020 on PUMA+ Robot (Robotics Laboratory & RioBotz - PUC-Rio)
#based on conde used at Winter Challenge Concordia 2018, IronCup 2019, and COBEM 2019
#this code considers an initial process of survey-in, i.e., GNSS initial and final coordinates previously known
#this code also considers a previous calibration of a Pixhawk or similar inertial/magnetic sensor to alignment with trajectories
#this code considers that the robot (differencial or skid-steering drive) is using a velocity control for trajectory accomplishment 
#this code considers that two ldr sensors are activated when the robot reaches a final point with ground truth measurement

yawoffset = 0 # compass offset - difference in alignment for each programmed trajectory [degrees]

e = 1 # admissible error threshold around the final point of the assumed trajectory [m]

g_ang = 1.4 # ganho angular, calibravel, relacionado a velocidade angular (maior valor = correcao mais rapida)
# este ganho g_ang age apenas dentro de uma area interna a um patamar definido a seguir pela variavel 'pat'
g_spi = 2 # ganho espiral, calibravel, obtido experimentalmente
# este ganho espiral deve ser usado apenas em competicoes de robotica e em casos de necessidade
# este ganho deve ser ajustado para quando um erro menor que 'e' seja encontrado mas nenhum 'ldr' ativado
# a variavel 'ldr' sera descrita mais para a frente e refere-se ao acionamento de sensores externos ao controle do robo

lim_perc = 110 # limite percentual (valor deve ser maior que 100)
# esta variavel define um limiar a partir do qual o robo pode ultrapassar o ponto final da trajetoria
# esta variavel deve ser usada quando os sensores ldr estao ativos - no contrario, o valor deve ser fixado em 100 (posicao exata)

pat = 2*e # patamar de distancia ao redor do alvo no qual a velocidade do robo deve reduzir consideravelmente (a half) [m]
# cuidado com o valor de pat quando ldr ativos:
# se alto, 2 ldrs podem indicar um falso positivo; se baixo, o robo pode nao reconhecer ponto final mesmo passando por ele
# na pratica, esse valor dependera da qualidade do processo de survey-in realizado antes da execucao das trajetorias
# este valor considera que os sensores ldr ativam leds externos quando o robo chega a um alvo

vel_lin = 2 # linear velocity in a linear trajectory [m/s]
# this value is previously calibrated from experiments considering wheels with 6in diameter
# it should be changed for other diameters

# initial latitude and longitude in a linear trajectory
Lat0 = -22.258617243
Lon0 = -45.695450688

# final latitude and longitude in a linear trajectory
Lat1 =  -22.258283962
Lon1 = -45.695284840

# the trajectories are considered linear,
# but the robot is able in this code to return to a desired goal with pertubations act on the system
# perturbations are external forces and terrain changes
# if ultrassound sensors, lidars, or cameras are inserted, the robot can avoid obstacles

ws = 6 # wheel size (related to maximum rover velocity) [in, pol]
# if the wheels size 'ws' is changed, o valor 'lv' abaixo se altera de forma inversamente proporcional
# contudo, estes valores nao possuem relacao com a velocidade linear descrita mais acima

lv = 1000/(2*ws) # fator que relaciona wheel size com maxima potencia enviada aos motores
# IMPORTANTE: o valor 1000 refere-se a pontencia maxima - os motores sao de 18V
# ou seja, para baterias acima de 18V, o valor 1000 implica em over voltage!!!
# o valor 1000 nao pode ser mantido por mais que 2 segundos ou o motor pode ser danificado

kh = 1 # gain of controller related to term h (unused - older fuzzy control)
ky = 1 # gain of controller related to term yaw (unused - older fuzzy control)

# import library for python3
import math
import serial
import os
import sys
import time
import threading
import numpy as np
from time import sleep
from gps3 import gps3
from time import *
from skfuzzy import control as ctrl # unused - older fuzzy control
from dronekit import connect, VehicleMode
import Jetson.GPIO as GPIO

# this code considers the use of one roboteq controller with two channels
# right and left wheels are connected to each channel, respectively

# start with null velocity
roboteqch1 = 0 # zero linear velocity in motor driver
roboteqch2 = 0 # zero angular velociry in motor driver

# Estimativa de correcao visual baseada no Google Maps: unused
# -22.977747, -43.231785

# this code considers the use of the NVidia Jetson TX2 development kit
# raspberry has similar pin out, but different names should be used

# setting GPIO Jetson
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(35, GPIO.IN) #ldr_d - sensor de luz no fundo do robo do lado direito
GPIO.setup(37, GPIO.IN) #ldr_e - sensor de luz no fundo do robo do lado esquerdo
GPIO.setup(11, GPIO.OUT) #led - este led serve apenas como indicador de que o robo chegou ao ponto final da trajetoria
# novamente, este codigo foi utilizado em competicao e considera um ground truth especifico da competicao
# quando nao houver ground truth, sugere-se comentar todas as linhas relacionadas a sensores ldr ou leds neste codigo

# led - turn off before start the path
GPIO.output(11, GPIO.LOW)

# Starting GNSS
gpsd = None #setting the global variable
print('>>> Starting GNSS <<<')
connection_string = "/dev/ttyACM1" # this value can be automatically changed by Jetson - possible names: ACM0 or ACM1
baud_rate=115200 # previously set 
# note that the string name is changed when manual connections are done on the system - example: new USB device installed

# Starting Roboteq
ser2 = serial.Serial('/dev/ttyS0', 115200) # previously set
ser2.isOpen()
sleep(0.1)
R1 = str(roboteqch1)
R2 = str(roboteqch2)
my_str1 = str.encode('!G 1 '+R1+'_')
my_str2 = str.encode('!G 2 '+R2+'_')
sleep(0.1)
ser2.write(my_str1)
ser2.write(my_str2)
print('>>> Setting zero velocity <<<')
sleep(0.1)

# Starting Pixhawk IMU
print(">>> Connecting with the UAV <<<")
vehicle = connect(connection_string, baud = baud_rate, wait_ready=True) 

# Reading GNSS
gps_socket = gps3.GPSDSocket()
data_stream = gps3.DataStream()
gps_socket.connect()
gps_socket.watch()

print(">>> Turn motor controller on <<<")
sleep(0.1)
GPIO.output(11, GPIO.HIGH)
sleep(0.6)
GPIO.output(11, GPIO.LOW)
sleep(1)

# prints here: usados quando uma tela esta conectada ao hardware do robo
# sleeps here: uteis para evitar problemas na transmissao de dados entre componentes do robo
# para um codigo continuo, sensores devem ser iniciados apenas uma vez e print/sleep devem ser retirados

try: # this code runs in a infinite loop until the robot comes to its final desired position
    for new_data in gps_socket:
        if new_data:
            data_stream.unpack(new_data)
#           sleep(0) # unused
            Lat = data_stream.TPV['lat']
            if Lat == 'n/a':
                Lat = 0
            Lon = data_stream.TPV['lon']
            if Lon == 'n/a':
                Lon = 0
            # se falha na leitura de latitude ou longitude, robot reads null - o mesmo para mode, abaixo
            DLat = float(Lat) - Lat0
            DLon = float(Lon) - Lon0
            TIME =  data_stream.TPV['time']
            MODE = data_stream.TPV['mode']
            #print(MODE) # unused
            if MODE == 'n/a':
                MODE = 0
            MODE = float(MODE)
            yaw = vehicle.attitude.yaw # orientacao do robo sobre a superficie da terra
            Yaw = np.degrees(yaw) #yaw minusculo dado em rad
            Pitch = np.degrees(vehicle.attitude.pitch)
            Roll = np.degrees(vehicle.attitude.roll)
            Roll = -Roll +90 # ajuste por diferenca entre calibracao e posicao atual da imu no rover
            Yaw = Yaw + yawoffset # offset inserido apenas para alinhar yaw com norte magnetico da terra
            print('__________________')
            print('Yaw', Yaw)
            #print('Pitch', Pitch) # unused
            #print('Roll', Roll) # unused
            # para navegacao baseada em controle de velocidade, apenas o valor de yaw se faz necessario
            # para navegacao usando controle de torque (outro codigo), pitch and roll necessarios

            # geometrical and control variables (from winter challenge 2018) - some of them are not used anymore
            # IMPORTANTE: a matematica por detras do que segue foi usada em codigos para controle fuzzy
            # unsued values permanecem pois as variaveis sao indicadores do comportamento do robo na nova estrategia adotada
            XA = DLon*math.cos(Lat0*math.pi*(1/180))*111320 # X_atual: relacionando posicao x local com longitude global
            YA = DLat*111320 # Y_atual: relacionando posicao y local com latitude global
            d = math.sqrt(XA*XA+YA*YA) # distance between initial point and actual position in local reference frame
            XA1 = (Lon1-Lon0)*math.cos(Lat0*math.pi*(1/180))*111320
            YA1 = (Lat1-Lat0)*111320
            D1 = math.sqrt(XA1*XA1+YA1*YA1) # distance between initial point and final point in local reference frame 
            alpha = math.atan2(YA1,XA1) # angle of vector that conects final position and origin of ref system with relation to local x-axis
            alpha = math.degrees(alpha)
            beta = math.atan2(YA,XA) # angle of vector that conects actual position and origin of ref system with relation to x-axis
            beta = math.degrees(beta)
            delta = (alpha)-(beta) # signal of delta is important (erro na Winter Challenge Concordia, provavelmente)
            delta = float(delta)
            delta = math.radians(delta)
            h = d*math.sin(delta) # perpendicular distance between rover and nearest point in desired trajectory
            D1a = d*math.cos(delta)
            D1b = math.fabs(D1-D1a)
            theta = math.atan(h/D1b)
            phi = (math.pi/2)-theta
            psi = (math.pi/2)-delta
            c = math.sqrt(D1b*D1b+h*h)
#           dist_to_end = math.fabs(math.fabs(D1)-math.fabs(d)) # approximated distance (actual and final points)
#           dist_to_end aproximado pois sempre se conhece a pos inicial e o erro relacionado mantem-se pequeno ao longo da trajetoria
            dist_to_end = math.sqrt(D1b*D1b+h*h)
            mode = float(MODE) # GNSS mode (adjustment)
            RD = d/D1 # ratio of accomplished trajectory

            # read light sensors - os valores sao alternados dependendo do tipo de sensor usado
            if (GPIO.input(35) == GPIO.LOW):
                ldr_right = 1
            else:
                ldr_right = 0
            if (GPIO.input(37) == GPIO.LOW): 
                ldr_left = 1
            else:
                ldr_left = 0
            
            LDR_soma = ldr_right + ldr_left # redundance
            print('LDR soma: ', LDR_soma)
            print('LA: ', Lat)
            print('LO: ', Lon)
            print('Mode: ', mode)
            print('Ratio of accomplished trajectory(%): ', 100*RD)
            print('Pseudo-distance to final point: ', dist_to_end)
            print('time: ', TIME)
            
            # controller (positive values to turn left and negative to right - defined in Roboteq)
            # estes valores podem ser mudado tambem caso os fios dos canais esq e dir sejam alternados na montagem do robo

            if (Lat != 0) and (Lon !=0) and (math.fabs(h) < dist_to_end): 
                var_ctr = math.asin(h/dist_to_end) # variavel de controle (first implemented - cobem2019) em rad
            else:
                if h > 0:
                    var_ctr = 1; # it assumes value zero only in case of GNSS error during code execution
                else:
                    var_ctr = -1

# if delta > 0: #poderia usar o valor delta, mas espero maior precisao da IMU em relacao ao GPS (change in case of drifting)

            var_ang = Yaw + math.degrees(var_ctr)
#           else:
#               var_ang = Yaw - math.degrees(var_ctr)

            # var_ang = Yaw*ky + h*kh  # variavel angular: zero to +- 90 (180 degrees in 1.5 meter of radius) # unused (old)

            print('alpha (related to final position): ', alpha)
            print('beta (related to actual position): ', beta)
            print('h (distance between rover and desired trajectory): ',h)
            #print('var_ang (controller channel 2): ', var_ang)
            
            # o conjunto de condicionais a seguir refere-se ao que pode ocorrer em competicoes trekking: 

            if (mode==3 or mode==2) and RD>=(lim_perc*0.01): # passou do alvo e nao o encontrou - retroceder (re) em direcao preferencial
                roboteqch1 = round(-vel_lin*lv)
                roboteqch2 = round(g_ang*var_ang)
                R1 = str(roboteqch1)
                R2 = str(roboteqch2)
                my_str1 = str.encode('!G 1 '+R1+'_')
                my_str2 = str.encode('!G 2 '+R2+'_')
                ser2.write(my_str1)
                print(my_str1)
                ser2.write(my_str2)
                print(my_str2)
                print('Out of range')
                sleep(0.1)

            elif (mode==3 or mode==2) and RD<(lim_perc*0.01) and dist_to_end>pat: # ir em direcao ao alvo
                roboteqch1 = round(vel_lin*lv)
                roboteqch2 = round(g_ang*var_ang)
                R1 = str(roboteqch1)
                R2 = str(roboteqch2)
                my_str1 = str.encode('!G 1 '+R1+'_')
                my_str2 = str.encode('!G 2 '+R2+'_')
                ser2.write(my_str1)
                print(my_str1)
                ser2.write(my_str2)
                print(my_str2)
                print('Full velocity - go to the goal')
                sleep(0.1)

            elif (mode==3 or mode==2) and RD<(lim_perc*0.01) and dist_to_end<=pat and dist_to_end>e and LDR_soma<1: # go to final point, search at least 2 marks
                roboteqch1 = round(0.7*vel_lin*lv)
                roboteqch2 = round(g_ang*var_ang)
                R1 = str(roboteqch1)
                R2 = str(roboteqch2)
                my_str1 = str.encode('!G 1 '+R1+'_')
                my_str2 = str.encode('!G 2 '+R2+'_')
                ser2.write(my_str1)
                print(my_str1)
                ser2.write(my_str2)
                print(my_str2)
                print('Reduced velocity - searching the goal')
                sleep(0.1)

            elif (mode==3 or mode==2) and RD<(lim_perc*0.01) and dist_to_end<=e and LDR_soma<1: # search final point with 'spiral' movement, search at least 1 mark
                roboteqch1 = round(vel_lin*lv)
                roboteqch2 = round(g_spi*g_ang*var_ang)
                R1 = str(roboteqch1)
                R2 = str(roboteqch2)
                my_str1 = str.encode('!G 1 '+R1+'_')
                my_str2 = str.encode('!G 2 '+R2+'_')
                ser2.write(my_str1)
                print(my_str1)
                ser2.write(my_str2)
                print(my_str2)
                print('Very near & low speed- searching the goal - like spiral movement')
                sleep(1.5)
                # espiral pode ser inserida em forma de estrutura (not fair) - implementado iron 2019 - codigo em e-mail privado

            elif (mode==3 or mode==2) and RD<(lim_perc*0.01) and dist_to_end<=pat and LDR_soma>=1: # alvo atingido
                print ('Alvo atingido')
                roboteqch1 = 0
                roboteqch2 = 0
                R1 = str(roboteqch1)
                R2 = str(roboteqch2)
                my_str1 = str.encode('!G 1 '+R1+'_')
                my_str2 = str.encode('!G 2 '+R2+'_')
                ser2.write(my_str1)
                print(my_str1)
                ser2.write(my_str2)
                print(my_str2)
                GPIO.output(11, GPIO.HIGH)
                sleep(0.8)
                sys.exit()
                # alvo atingido - esta condicional deve ser mantida mesmo que o robo nao seja usado em situacao de competicao
                # o mesmo para o caso de nao correspondencia - erro desconhecido

            else:
                print ('No correspondence - erro desconhecido!') # caso o robo tenha um comportamento nao registrado ate agora
                roboteqch1 = 0
                roboteqch2 = 0
                R1 = str(roboteqch1)
                R2 = str(roboteqch2)
                my_str1 = str.encode('!G 1 '+R1+'_')
                my_str2 = str.encode('!G 2 '+R2+'_')
                ser2.write(my_str1)
                print(my_str1)
                ser2.write(my_str2)
                print(my_str2)
                sleep(0.1)

            data1 = Lat
            data2 = Lon
            data3 = dist_to_end
            data4 = TIME
            data5 = my_str1
            data6 = my_str2  
            data7 = Yaw

            with open('test.csv','a') as log:
                output = "{},{},{},{},{},{},{}\n".format(data1, data2, data3, data4, data5, data6,data7)
                log.write(output)
                # print variaveis principais para analise posterior
                # mais variaveis devem ser usadas em caso de controle de torque em terrenos nao estruturados
                          
except (KeyboardInterrupt,SystemExit): # interrupcao no sistema via teclado ou erros no sistema na Jeton/raspberry pi
    
            print ('INTERROMPIDO')
            roboteqch1 = 0
            roboteqch2 = 0
            R1 = str(roboteqch1)
            R2 = str(roboteqch2)
            my_str1 = str.encode('!G 1 '+R1+'_')
            my_str2 = str.encode('!G 2 '+R2+'_')
            ser2.write(my_str1)
            print(my_str1)
            ser2.write(my_str2)
            print(my_str2)
            vehicle.close()
            print ('DESLIGAR CONTROLADOR')
            ser2.close()
            GPIO.output(11, GPIO.HIGH)
            sleep(0.3)
            GPIO.output(11, GPIO.LOW)
            sleep(0.2)
            GPIO.output(11, GPIO.HIGH)
            sleep(0.3)
            GPIO.output(11, GPIO.LOW)
            sleep(0.2)
            GPIO.cleanup()
            
print ('Fim deste trajeto!')

# codigo continuo - o mesmo pode ser usado em forma de threading, para melhor performance