#!/usr/bin/env python3

# Importar librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Función callback para cuando se recibe una imagen
def image_callback(msg):
    try:
        # Convertir el mensaje de ROS a una imagen de OpenCV
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Mostrar la imagen en una ventana de OpenCV y esperar 1 ms para que OpenCV procese los eventos de la GUI
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)  # Espere 1 ms entre cada marco para procesar eventos de GUI

def main():
    rospy.init_node('image_viewer', anonymous=True)
    # Suscribir al tópico de ROS que tiene las imágenes
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    # Prevenir que Python cierre hasta que el nodo es detenido
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()  # Asegurarse de cerrar todas las ventanas de OpenCV

if __name__ == '__main__':
    main()
