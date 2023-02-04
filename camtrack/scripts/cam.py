#!/usr/bin/python3

import rospy
import cv2 #lib opencv
import numpy as np #lib numpy
import mediapipe as mp
import os




from openni import openni2 #lib openni
from openni import _openni2 as c_api
from geometry_msgs.msg import Twist


hands = mp.solutions.hands
Hands = hands.Hands(max_num_hands=1)
mpDraw = mp.solutions.drawing_utils

C = "\033[0m"     # clear (end)
R = "\033[0;31m"  # red (error)
G = "\033[0;32m"  # green (process)
B = "\033[0;36m"  # blue (choice)
Y = "\033[0;33m"  # yellow (info)

rospy.init_node("camtrack")

os.system = 'source ~/Documentos/OpenNI-Linux-x64-2.3/OpenNIDevEnvironment'
rospy.logwarn("source ~/Documentos/OpenNI-Linux-x64-2.3/OpenNIDevEnvironment")



openni2.initialize() #Inicialização do openni2
dev = openni2.Device.open_any() #abrindo dispositivo 
print (dev.get_device_info())  #obtendo informaçoes da porta do dispositivo


VideoColor = dev.create_color_stream() # Ativação de imagem colorida da Astra
VideoColor.start() # Iniciando Video RGB
VideoColor.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, 
resolutionX = 640, 
resolutionY = 480, 
fps = 30)) # Set. modo e resolução para opencv


def banner():
    print(B)
    print(r"""  
    
 ________ __  ____   __ _____ _____ _____ ____  
 |__   __| |  | \ \ / // ____/ ____|_   _/ __ \ 
    | |  | |__| |\ V /| (___| (___   | || |  | |
    | |  |  __  | > <  \___ \\___ \  | || |  | |
    | |  | |  | |/ . \ ____) |___) |_| || |__| |
    |_|  |_|  |_/_/ \_\_____/_____/|_____\____/
    
    """,end="")
    print(f"{C} CamTrack using mediaPipe\n")
    print(f"     Written by {B}Thxssio{C} (DEX)")

pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
move = Twist()

def movimentacao(dist):
      rospy.logwarn("1")                  
      move.linear.x = dist
      pub.publish(move)

def rastreamento():
      print(cx)
      proporcional = cx
      integral = proporcional/2
      derivada = 0.3
      pid = (proporcional - integral)*derivada
      #print(pid/100) 

      rospy.logwarn(pid/100)                  
      move.linear.x = pid/100
      pub.publish(move)
      if pid > 300 and pid < 500:
            rospy.logwarn((pid/100))
            move.linear.x = pid/100
            pub.publish(move)


banner()
while not rospy.is_shutdown(): #inicio do loop
            
            frame = VideoColor.read_frame() # Abrindo Frame de video, baseado na resolução
            DataFrame = frame.get_buffer_as_uint8() # obtendo um Buffer de 8 bits
            colorArray = np.ndarray((frame.height, frame.width, 3),
                                        dtype=np.uint8,buffer=DataFrame) #Matriz
            colorArray = cv2.cvtColor(colorArray, cv2.COLOR_BGR2RGB) #OpenCV
            results = Hands.process(colorArray)
            handspoints = results.multi_hand_landmarks
            h, w, _ = colorArray.shape
            pontos = []

            if handspoints:
                  for points in handspoints:
                        #print(points)
                        mpDraw.draw_landmarks(colorArray, points, hands.HAND_CONNECTIONS)
                        for id, cord in enumerate(points.landmark):
                              cx, cy = int(cord.x*w), int(cord.y*h)
                             #cv2.putText(colorArray, str(id), (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,0),2)
                              pontos.append((cx,cy))
                              rastreamento()
                             
                        
    
            
            dedos = [8, 12, 16, 20]
            contador = 0
            
            if handspoints:
                  if pontos[4][0] < pontos [2][0]:
                        contador +=1
                  for x in dedos:
                        if pontos[x][1] < pontos[x-2][1]:
                              contador +=1

            #print(contador)
            #cv2.rectangle(colorArray,(80,20), (200,100),(0,0,0),-1)
            cv2.putText(colorArray, str(contador),(100,100), cv2.FONT_HERSHEY_SIMPLEX, 4,(255,0,0), 5)
            



            cv2.imshow('color', colorArray) #Mostrar janela
            key = cv2.waitKey(1) & 0xFF
            if key == ord("c"):
                                break
                                
            elif contador == 1:
                  movimentacao(1)
            elif contador == 3:
                  movimentacao(1.5)
            elif contador == 4:
                  movimentacao(2.0)
            elif contador == 5:
                  movimentacao(2.5)
            else:
                  rospy.loginfo("Sem imagem para ser indentificada")
                  move.linear.x = 0
                  pub.publish(move)

openni2.unload()
cv2.destroyAllWindows()


