#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('obstacle_avoidance')

        # Suscriptor para la nube de puntos de los obstáculos captados por el lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Suscriptor para comandos de control Ackermann
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # Publicador para comandos Ackermann modificados
        self.ackermann_command_publisher = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)

        # Almacenar el último mensaje Ackermann recibido
        self.last_ackermann_cmd = AckermannDrive()
        self.obstacle_detected = False

    def obstacle_callback(self, msg):
        # Procesar la nube de puntos con los obstáculos teniendo en cuenta el último mensaje de movimiento recibido para evitar colisiones
        points = list(pc2.read_points(msg, skip_nans=True))
        self.obstacle_detected = self.check_for_obstacles(points)
        
        cmd = self.last_ackermann_cmd

        # Modificar mensaje de ackermann si es necesario
        # self.obstacle_detected = False
        if self.obstacle_detected:
            rospy.loginfo("Obstacle detected! Stopping the robot.")
            cmd.speed = 0.0 
            cmd.steering_angle = 0.0

        self.ackermann_command_publisher.publish(cmd)

    def check_for_obstacles(self, points):

        for point in points:
            x, y, z = point[:3]

            if -0.5 < y < 0.5 and 0 < x < 1.0:
                return True
        return False

    def ackermann_callback(self, msg):
        # Almacena el último comando recibido
        self.last_ackermann_cmd = msg

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
