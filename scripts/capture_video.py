#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def video_publisher():
    # Inicializa un nodo ROS
    rospy.init_node('video_publisher', anonymous=True)

    # TODO Crea un publicador en el tópico /operator/image

    # TODO Configura la captura de video desde la webcam (o desde un vídeo)
    cap = None

    # Crea una instancia de CvBridge para convertir las imágenes de OpenCV a mensajes de ROS
    bridge = CvBridge()

    # Define la tasa de publicación (e.g., 10 Hz)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # TODO Captura un frame de la webcam
                
        # TODO Convierte el frame de OpenCV a un mensaje ROS

        # TODO Publica el mensaje en el tópico

        # Espera para cumplir con la tasa de publicación
        rate.sleep()

    # Cuando termines, libera la captura
    cap.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
