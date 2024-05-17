#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# Inicializar MediaPipe Hand pose model
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)

# Publicador de mensajes de control
ackermann_command_publisher = None

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convertir la imagen de ROS a una imagen de OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Procesar la imagen con MediaPipe
    results = hands.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

    # Verificar si se detectan manos
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Dibujar los landmarks sobre la imagen
            mp_drawing.draw_landmarks(cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # TODO: Reconocer el gesto mediante alguna clasificación a partir de los landmarks

            # TODO: Interpretar el gesto obtenido y enviar la orden de control ackermann
            # Ejemplo de envío de un mensaje
            ackermann_cmd = ackermann_msgs.msg.AckermannDrive()
            ackermann_cmd.steering_angle = 0.5  # ejemplo: girar a la derecha
            ackermann_cmd.speed = 1.0           # ejemplo: moverse hacia delante
            ackermann_command_publisher.publish(ackermann_cmd)

    # Mostrar la imagen con los landmarks/gestos detectados
    cv2.imshow("Hand Pose Estimation", cv_image)
    cv2.waitKey(1)

def main():
    global ackermann_command_publisher
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
