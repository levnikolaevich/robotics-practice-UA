#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# TODO Declarar el detector de pose de mediapipe a utilizar

# Publicador de mensajes de control
ackermann_command_publisher = None

#Procesar la imagen del operador
def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convertir la imagen de ROS a una imagen de OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # TODO Procesar la imagen con MediaPipe

    # TODO Reconocer el gesto mediante alguna clasificación a partir de los landmarks
    
    # TODO Dibujar los landsmarks sobre la imagen

    # Mostrar la imagen con los landmarks/gestos detectados
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)

    # TODO Interpretar el gesto obtenido y enviar la orden de control ackermann

def main():
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
