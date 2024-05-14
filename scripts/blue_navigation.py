#!/usr/bin/env python
#
# Nodo de ROS para calcular la trayectoria hacia el objetivo evitando los obstáculos.

import sys
import copy
import rospy
import std_msgs.msg
import ackermann_msgs.msg
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import sensor_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
import tf_conversions
import numpy as np
from math import pi, dist, cos, sin, fabs, sqrt, atan2, tan

#Características del vehículo
MAX_STEER_ANGLE = 24.0*pi/180.0 # Radianes
MAX_SPEED = 1.3    # m/s
MIN_SPEED = 0.6
VEHICLE_LENGHT = 1.05 

class BlueTrajectoryPlanner(object):

    def __init__(self):
        super(BlueTrajectoryPlanner, self).__init__()

        ## Inicialización del nodo de ROS:
        rospy.init_node("blue_planner_node", anonymous=True)

        #Inicializar parámetros
        self.delta_angle=6.0*pi/180.0
        self.delta_sample=0.2
        self.max_sample=1.0
        self.reached_distance=1.5
        self.slow_down_distance=2.0
        self.rate=5

        #TODO Declarar otros parámetros necesarios para el planificador.

        #Inicializar variables
        self.position = None
        self.theta = 0 
        self.obstacles = []
        self.limits = None
        self.goal_reached = False #Para esperar que el UR5 nos comunique que nos acerquemos.
        #TODO Declarar otros variables que se consideren necesarias.
        self.local_path=[]

        # TODO inicializar posición objetivo. También se pueden considerar varios puntos objetivos para maniobrar y acercarse al UR5.
        self.goal_id = 0
        self.goals = [geometry_msgs.msg.Point(5.0, 2.0, 0.0), geometry_msgs.msg.Point(0.0, 0.0, 0.0)]

        # TODO considerar más subscribers/publishers necesarios
        # Subscribers definition
        self.position_subscriber = rospy.Subscriber("/blue/ground_truth",
             Odometry, self.position_callback, queue_size=1)
        self.obstacles_subscriber = rospy.Subscriber("/obstacles",
            sensor_msgs.msg.PointCloud2, self.obstacles_callback, queue_size=1)
        self.obstacles_subscriber = rospy.Subscriber("/free_zone",
            sensor_msgs.msg.PointCloud2, self.limits_callback, queue_size=1)
        
        # self.position_camera_subscriber = rospy.Subscriber("/pose_array",
        #     geometry_msgs.msg.PoseArray, self.detection_camera_callback, queue_size=1)

        ## Publishers definition
        self.ackermann_command_publisher = rospy.Publisher(
            "/blue/ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

        self.marker_publisher = rospy.Publisher(
            "/local_path",
            MarkerArray,
            queue_size=10,
        )

    def position_callback(self, data: Odometry):
        self.position = data.pose.pose.position
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]
        
    #Callbacks
    #Adaptar este callback para no depender de la posición aportada por el simulador Gazebo.    
    def detection_camera_callback(self, pose_array:geometry_msgs.msg.PoseArray):
        #print(pose_array)
        if len(pose_array.poses) > 1:  # Проверка, что массив поз не пуст и содержит как минимум две позы
                    self.position = pose_array.poses[1].position
                    euler = tf_conversions.transformations.euler_from_quaternion([
                        pose_array.poses[1].orientation.x,
                        pose_array.poses[1].orientation.y,
                        pose_array.poses[1].orientation.z,
                        pose_array.poses[1].orientation.w
                    ])
                    self.theta = euler[2]
        else:
            rospy.logwarn("PoseArray does not contain enough poses to determine robot position.")


    def obstacles_callback(self, obstacles: sensor_msgs.msg.PointCloud2):
        self.obstacles = [geometry_msgs.msg.Point(x=p[0], y=p[1], z=p[2])
                          for p in pc2.read_points(obstacles, field_names=("x", "y", "z"), skip_nans=True)]

    def limits_callback(self, limits: sensor_msgs.msg.PointCloud2):
        self.limits = [geometry_msgs.msg.Point(x=p[0], y=p[1], z=p[2])
                       for p in pc2.read_points(limits, field_names=("x", "y", "z"), skip_nans=True)]


    # Planificar una trayectoria hacia el objetivo evitando obstáculos, se ejecutada a una frecuencia más baja.
    ''' Código implementado a partir del artículo de investigación:
            OpenStreetMap-Based Autonomous Navigation With LiDAR Naive-Valley-Path Obstacle Avoidance
            Miguel Ángel Muñoz Bañón, Edison Velasco Sánchez, Francisco A. Candelas, Fernando Torres
            IEEE Transactions on Intelligent Transportation Systems, 2022
    '''
    def localGoalCalculation(self):
        print('BLUE: localGoalCalculation started')
        if self.position is  None or self.goal_reached or self.limits is None: return
        #Definición de parámetros
        current_goal = self.goals[self.goal_id]
        wa, wr, ar, aa = 100.0, 1.0, 0.8, 0.3
        min_force = wr / pow(0.1, ar) - wa / pow(100.0, aa) + 1000

        local_goal=geometry_msgs.msg.Point()
        self.local_path=[]
        min_distance = 10000.0
        force_r = 0
        goal_in_local_axis = self.global2local(current_goal)
        # 1) Puntos de la trayectoria en el radio límite
        for limit_point in self.limits:
            distance_a = self.distance(limit_point, goal_in_local_axis)
            if distance_a<0.0: distance_a=0.1

            force_a = wa / pow(distance_a,aa)

            #Calculo de la distancia a obstáculo mínima y su peso
            min_distance=10000.0

            for obstacle_point in self.obstacles:
                distance_r = self.distance(limit_point,obstacle_point)
                if distance_r<0.1: distance_r=0.1
                if distance_r<min_distance:
                    min_distance=distance_r
                    force_r = wr / pow(distance_r, ar)

            force = force_r - force_a

            if force < min_force:
                min_force=force
                local_goal=limit_point
        self.local_path.append(local_goal)
        # 2) Puntos de la trayectoria en los anillos internos
        # Definición de parámetros
        wa2 = 3.0
        wr2 = 1.0
        ar2 = 0.5
        aa2 = 0.3
        radious = 4.0
        delta_rad = radious / 3.0

        for rad in np.arange(radious, delta_rad-0.1, -delta_rad):
            min_force = wr / pow(0.1, ar) - wa / pow(100.0, aa) + 1000
            #Calculo de la distancia a obstáculo mínima y su peso
            min_distance = 10000.0
            for limit_point in self.limits:
                depth, azimuth=self.cartesian2Spherical(limit_point.x, limit_point.y)
                p_in  = self. spherical2Cartesian(rad, azimuth)

                distance_a=self.distance(p_in, self.local_path[0])
                if distance_a<0.0: distance_a=0.1
                force_a = wa2 /pow(distance_a,aa2)

                min_distance = 10000.0
                for obstacle_point in self.obstacles:
                    distance_r = self.distance(p_in, obstacle_point)
                    if distance_r<0.1: distance_r=0.1
                    if distance_r<min_distance:
                        min_distance=distance_r
                        force_r = wr2 / pow(distance_r, ar2)

                force = force_r-force_a
                if force < min_force:
                    min_force=force
                    local_goal=p_in
            self.local_path.append(local_goal)

        #Publicar la visualiazación dek objetivo
        marker_msg = MarkerArray()
        for id, point in enumerate(self.local_path):
            marker = Marker()
            marker.header.frame_id = "blue/velodyne"
            marker.header.stamp = rospy.Time()
            marker.id = id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker_msg.markers.append(marker)
        self.marker_publisher.publish(marker_msg)

    def controlActionCalculation(self):
        print('BLUE: controlActionCalculation started')
        # Detectar si ha terminado la trayectoria
        if self.position is None or self.goal_reached:
            return
        
        ackermann_control=ackermann_msgs.msg.AckermannDrive()
        ackermann_control.speed, ackermann_control.steering_angle = 0.0, 0.0
        current_goal = self.goals[self.goal_id]
        # Detectar si ha llegado al siguiente punto objetivo. 
        if self.distance(self.position, current_goal) < self.reached_distance:
            print("BLUE: Goal reached")
            self.goal_reached=True
            self.ackermann_command_publisher.publish(ackermann_control)
            return
        
        self.margin = 0.3
        # Reducer la velocidad cuando se acerca al objetivo
        goal_distance = self.distance(self.position, current_goal)
        if goal_distance - self.reached_distance < self.slow_down_distance:
            speed = MIN_SPEED
            # Establecer el objetivo como objetivo local
            if len(self.local_path) >= 2:
                self.local_path[-2] = self.global2local(current_goal)
            else:
                self.local_path.append(self.global2local(current_goal))
            print('BLUE: next stop - finish')
        else:
            self.margin = 0.3
            speed = MAX_SPEED
            print('BLUE: continue moving...')

        #Inicializar variables
        min_error = 1000

        # Comprobar posibles acciones de control
        for steer in np.arange(-MAX_STEER_ANGLE, MAX_STEER_ANGLE + 0.01, self.delta_angle):
            if abs(steer) < 0.01:
                steer = 0.0  # Игнорировать очень маленькие углы
            k_sp = (MAX_STEER_ANGLE - abs(steer)) / MAX_STEER_ANGLE
            speed2 = max(speed * k_sp, MIN_SPEED)
            
            # Comprobar movimiento hacia adelante y hacia atrás.
            directions = [-speed2, speed2]
            for dir in directions:
                flag_collision_risk = False

                #TODO Calcular radio y velocidad de giro mediante la cinemática del robot.
                turning_radius = VEHICLE_LENGHT / tan(steer) if tan(steer) != 0 else float('inf')
                angular_velocity = dir / turning_radius if turning_radius != float('inf') else 0

                #Calcular la trayectoria mediante el ángulo de giro
                for sample in np.arange(self.delta_sample, self.max_sample+0.01, self.delta_sample):
                    local_point=geometry_msgs.msg.Point()

                    #TODO calcular los puntos de la trayectoria mediante la cinemática del robot.
                    local_point.x = self.position.x + dir * cos(self.theta + steer) * sample
                    local_point.y = self.position.y + dir * sin(self.theta + steer) * sample
                    local_point.z = 0

                    #TODO Detectar riesgo de colisiones
                    for obstacle in self.obstacles:
                        if self.distance(local_point, obstacle) < self.margin:
                            flag_collision_risk = True
                            print('BLUE: collision_risk DETECTED')
                            break
                    
                    if flag_collision_risk: break

                # Si no hay colisiones, calcular su puntuación. 
                if not flag_collision_risk:
                    #TODO calcular la puntuación de la trayectoria en base al error en distancia y ángulo al objetivo (self.local_path[-2]).
                    if len(self.local_path) >= 2:
                        target_point = self.local_path[-2]                        
                    else:
                        # Если local_path пуст или недостаточно элементов, используем текущую целевую точку
                        target_point = self.global2local(current_goal)

                    print(f'BLUE: next goal X:{target_point.x} Y:{target_point.y}')
                    distance_error = self.distance(local_point, target_point)
                    angle_error = fabs(self.angle(local_point, target_point) - steer)

                    # Взвешенная сумма ошибок
                    error = distance_error + angle_error
                    if error < min_error:
                        min_error=error
                        ackermann_control.steering_angle=steer
                        ackermann_control.speed = dir * speed2
        
        #Publicar el mensaje
        self.ackermann_command_publisher.publish(ackermann_control)
        print(f'BLUE: ackermann_control.steering_angle - {ackermann_control.steering_angle}, ackermann_control.speed - {ackermann_control.speed}')
    
    
    def distance(self, p1:geometry_msgs.msg.Point, p2:geometry_msgs.msg.Point):
        return sqrt((p1.x-p2.x)**2+(p1.y-p2.y)**2)
    
    # Ángulo del punto p2 al p1
    def angle(self, p1:geometry_msgs.msg.Point, p2:geometry_msgs.msg.Point):
        return atan2((p1.y-p2.y),p1.x-p2.x)
    
    #Transformación de posición global a local
    def global2local(self, p:geometry_msgs.msg.Point):
        result = geometry_msgs.msg.Point()
        #Traslación
        x = (p.x-self.position.x) 
        y = (p.y-self.position.y)
        #Rotación
        result.x = x * cos(-self.theta) - y * sin(-self.theta)
        result.y = x * sin(-self.theta) + y * cos(-self.theta)
        return result
    
    #Coordenadas esféricas a cartesianas
    def spherical2Cartesian(self, depth, azimuth):
        sin_azimuth = sin(azimuth)
        cos_azimuth = cos(azimuth)
        p=geometry_msgs.msg.Point()
        p.x = depth * cos_azimuth
        p.y = depth * sin_azimuth
        return p
    
    #Coordenadas cartesianas a esféricas
    def cartesian2Spherical(self, x,  y):
        depth = sqrt((x * x) + (y * y))

        azimuth = atan2(y, x)

        if (azimuth < 0): azimuth += 2*pi
        if (azimuth >= 2*pi): azimuth -= 2*pi

        return depth, azimuth
    
    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.controlActionCalculation()
            if self.goal_reached:
                self.goal_reached = False
                self.goal_id = (self.goal_id + 1) % len(self.goals)
                ackermann_control = ackermann_msgs.msg.AckermannDrive()
                ackermann_control.speed, ackermann_control.steering_angle = 0.0, 0.0
                self.ackermann_command_publisher.publish(ackermann_control)
            
            self.localGoalCalculation()
            rate.sleep()

def main():
    try:
        print("Init blue_planner_node")
        node = BlueTrajectoryPlanner()
        node.run()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()    
    