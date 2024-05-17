#!/usr/bin/env python3

import rospy
import cv2, os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def video_publisher():
    # Inicializa un nodo ROS
    rospy.init_node('video_publisher', anonymous=True)
    

    # Crea un publicador en el tópico /operator/image
    image_pub = rospy.Publisher('/operator/image', Image, queue_size=10)

    # Configura la captura de video desde la webcam (o desde un vídeo)
    # cap = cv2.VideoCapture(0)
    video_path = '/home/docker/catkin_ws/src/robotica_inteligente/scripts/gestos.mp4'
    print(os.path.exists(video_path)) 

    def open_video(video_path):
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            rospy.logerr(f"Unable to open video source: {video_path}")
            return None
        return cap
    
    cap = open_video(video_path)

    # Crea una instancia de CvBridge para convertir las imágenes de OpenCV a mensajes de ROS
    bridge = CvBridge()

    # Define la tasa de publicación (e.g., 30 Hz)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Captura un frame de la webcam
        print("Opened:", cap.isOpened()) 
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("Reached end of the video file, restarting...")
            cap.release()
            cap = open_video(video_path)
            continue

        # Convierte el frame de OpenCV a un mensaje ROS
        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publica el mensaje en el tópico
        image_pub.publish(image_msg)

        # Espera para cumplir con la tasa de publicación
        rate.sleep()

    # Cuando termines, libera la captura
    cap.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
