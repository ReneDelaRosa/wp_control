#!/usr/bin/env python
# coding=utf-8
#Segmentación de imagen y localización de masa mayor.
#Cálculo de centroide de masa y radio de círculo.

#------LIBRERÍAS---------"""
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32


#-----DATOS A PUBLICAR EN ROS--------
pub_x = rospy.Publisher('c_x', Int32, queue_size=10)  #---(nombre de variable,clase de mensjae,tamaño frecuencia)
pub_y = rospy.Publisher('c_y', Int32, queue_size=10)
pub_a = rospy.Publisher('c_a', Int32, queue_size=10)

br=CvBridge()

#-----FUNCIONES-----
def nothing(x):   #Función para trackbars de segmentación
	pass

def callback(data): 
	try:
		#----Caprura de imagen en BGR
		frame = br.imgmsg_to_cv2(data, "bgr8")
		#----Conversión de imagen de BGR a HSV
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		#----Creación de ventana de trackbars
		cv2.namedWindow('image') #---Nombre de imagen
		#----Nombre y rangos de trackbars
		cv2.createTrackbar('Matiz Min','image',10,255,nothing)  
		cv2.createTrackbar('Matiz Max','image',0,255,nothing)
		cv2.createTrackbar('Saturacion Min','image',10,255,nothing)
		cv2.createTrackbar('Saturacion Max','image',10,255,nothing)
		cv2.createTrackbar('Valor Min','image',10,255,nothing)
		cv2.createTrackbar('Valor Max','image',10,255,nothing)
		#----Trackbar para filtro de imagen 
		cv2.createTrackbar('Closing','image',0,20,nothing)		

		#----Obtener valores de trackbars
		hMin =0#cv2.getTrackbarPos('Matiz Min','image')# 84
		hMax = 255#cv2.getTrackbarPos('Matiz Max','image')#255
		sMin =120#cv2.getTrackbarPos('Saturacion Min','image')#134
		sMax =255#cv2.getTrackbarPos('Saturacion Max','image')#255
		vMin =127#cv2.getTrackbarPos('Valor Min','image')#83
		vMax =255#cv2.getTrackbarPos('Valor Max','image')#255
		#----Crear una rreglo para la máscara
		lower=np.array([hMin,sMin,vMin])
		upper=np.array([hMax,sMax,vMax])
	 
		#----Aplicar máscara a imagen HSV con los valores obtenidos
		mask = cv2.inRange(hsv, lower, upper)

		#----Aplicar filtro a la imagen segemntada
		Clos = cv2.getTrackbarPos('Closing','image')
		kernel=np.ones((Clos,Clos), np.uint8)
		closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

		#----Obtener el área de la masa
		moments = cv2.moments(mask)	
		area = moments['m00'] 	

		#---Encontrar contornos en la imagen
		(_, contornos,_) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		if len(contornos)!=0:  #---Si hay un contorno o más 
			grande=max(contornos, key=cv2.contourArea) #-----Mostrar solamentente el contorno más grande
			cv2.drawContours(frame,grande,-1,(0,0,255), 2) #---Dibujar contorno en imagen original
			(x,y),radius = cv2.minEnclosingCircle(grande) #---Obtener radio y coordenadas de masa
			center = (int(x),int(y)) #---Declarar centroide como el conjunto de coordenadas x, y
			texto = 'x='+str (center[0]) + 'y='+str (center[1])
			radius = int(radius)     #---Declarar radio como radio
			cv2.rectangle(frame, center, (center[0]+2, center[1]+2),(0,0,255), 2) #---Dibujar rectángulo al centro de la masa
			#cv2.circle(frame,center,radius,(0,255,0),2) #---Dibujar un circulo que contnga la masa
			moments = cv2.moments(grande)	#---Obtener momentos de la masa
			area = moments['m00']           #---Obtener área de la masa
			#print "area=",area              #---Imprimir área
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(frame,texto,(20,30), font, 1,(255,0,255),2,cv2.LINE_AA)
			cv2.imshow('Camara',frame)      #---Mostrar imagen original
			cv2.imshow('mask',closing)	#---Mostrar imagen segemntada
			cv2.waitKey(10)			#---Esperar 
			pub_x.publish(center[0])	#---Publicar coordenada x en ROS
			pub_y.publish(center[1])	#---Publicar coordenada y en ROS
			pub_a.publish(radius)		#---Publicar radio en ROS
			print "x = ",(center[0])
			print "y = ",(center[1])
			
		else:
			texto = 'no detectado'
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(frame,texto,(20,30), font, 1,(255,0,255),2,cv2.LINE_AA)
			cv2.imshow('Camara',frame)      #---Mostrar imagen original
			cv2.imshow('mask',closing)	#---Mostrar imagen segemntada
			cv2.waitKey(10)			#---Esperar 		
			x=-1
			y=-1			
			print "No detectado"
			pub_x.publish(x)	#---Publicar coordenada x en ROS
			pub_y.publish(y)	#---Publicar coordenada y en ROS
			#pub_a.publish(0)		#---Publicar radio en ROS
	except CvBridgeError, e:
		print e
		

def main():
	rospy.init_node('Visualizador', anonymous=False)  #---Iniciar nodo
	rospy.Subscriber("/bebop/image_raw", Image, callback) #---Suscribirse al nodo de cámara 
	#----(nodo,tipo,funcion)
	rospy.spin()

if __name__=='__main__':
	main()
