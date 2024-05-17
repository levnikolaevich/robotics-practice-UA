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
ackermann_preorder_command_publisher = None
last_cmd = None

def classify_gesture(hand_landmarks):
    thumb_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y
    index_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y
    middle_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].y
    ring_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].y
    pinky_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].y

    # print(f"Thumb is open: {thumb_is_open}")
    # print(f"Index is open: {index_is_open}")
    # print(f"Middle is open: {middle_is_open}")
    # print(f"Ring is open: {ring_is_open}")
    # print(f"Pinky is open: {pinky_is_open}")

    if index_is_open and not middle_is_open and not ring_is_open and not pinky_is_open and not thumb_is_open:
        return 1
    elif index_is_open and middle_is_open and not ring_is_open and not pinky_is_open and not thumb_is_open:
        return 2
    elif thumb_is_open and index_is_open and middle_is_open:
        return 3
    else:
        return 0

def image_callback(msg):
    global last_cmd
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

            # Reconocer el gesto mediante alguna clasificación a partir de los landmarks
            gesture = classify_gesture(hand_landmarks)
            rospy.loginfo(f"Detected gesture: {gesture}")

            # Interpretar el gesto obtenido y enviar la orden de control ackermann            

            print(f'gesture: {gesture}')

            ackermann_cmd = ackermann_msgs.msg.AckermannDrive()
            if gesture == 1:
                ackermann_cmd.steering_angle = 0.0  # seguir recto
                ackermann_cmd.speed = 0.5           # moverse hacia delante
            elif gesture == 2:
                ackermann_cmd.steering_angle = 0.5 # girar a la izquierda
                ackermann_cmd.speed = 0.8           # moverse hacia delante
            elif gesture == 3:
                ackermann_cmd.steering_angle = -0.5  # girar a la derecha
                ackermann_cmd.speed = 0.8           # moverse más lento
            else:
                return
            
            if last_cmd != ackermann_cmd:
                ackermann_preorder_command_publisher.publish(ackermann_cmd)
                last_cmd = ackermann_cmd

    # Mostrar la imagen con los landmarks/gestos detectados
    cv2.imshow("Hand Pose Estimation", cv_image)
    cv2.waitKey(1)

def main():
    global ackermann_preorder_command_publisher
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    # Publisher definition
    ackermann_preorder_command_publisher = rospy.Publisher(
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
