#!/usr/bin/env python

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
MAX_STEER_ANGLE = 24.0 * pi / 180.0  # Radianes
MAX_SPEED = 1.3  # m/s
MIN_SPEED = 0.6
VEHICLE_LENGHT = 1.05

class BlueTrajectoryPlanner(object):

    def __init__(self):
        super(BlueTrajectoryPlanner, self).__init__()

        ## Inicialización del nodo de ROS:
        rospy.init_node("blue_planner_node", anonymous=True)

        # Inicializar parámetros
        self.delta_angle = 3.0 * pi / 180.0  # Уменьшить для более плавных поворотов
        self.delta_sample = 0.5  # Увеличить для уменьшения частоты пересчета траектории
        self.max_sample = 2.0  # Увеличить для более дальнего прогноза траектории
        self.reached_distance = 1.5
        self.slow_down_distance = 2.0
        self.rate = 5

        # Inicializar variables
        self.position = None
        self.theta = 0
        self.obstacles = []
        self.limits = None
        self.goal_reached = False  # Para esperar que el UR5 nos comunique que nos acerquemos.
        self.goal_id = 0
        self.local_path = [] 
        self.state = None
        
        # Definir точку назначения. También можно определить несколько точек для маневрирования.
        self.goals = [geometry_msgs.msg.Point(5.0, 3.5, 0.0)]  # Точка назначения около UR5
        print("Goals: ", [(goal.x, goal.y) for goal in self.goals])

        # Subscribers definition
        self.position_subscriber = rospy.Subscriber("/blue/ground_truth", Odometry, self.position_callback, queue_size=1)
        self.obstacles_subscriber = rospy.Subscriber("/obstacles", sensor_msgs.msg.PointCloud2, self.obstacles_callback, queue_size=1)
        self.limits_subscriber = rospy.Subscriber("/free_zone", sensor_msgs.msg.PointCloud2, self.limits_callback, queue_size=1)

        # Publishers definition
        self.ackermann_command_publisher = rospy.Publisher("/blue/ackermann_cmd", ackermann_msgs.msg.AckermannDrive, queue_size=10)
        self.marker_publisher = rospy.Publisher("/local_path", MarkerArray, queue_size=10)

        self.ur5_status_subscriber = rospy.Subscriber("/ur5/status", std_msgs.msg.String, self.ur5_status_callback)
        self.blue_status_publisher = rospy.Publisher("/blue/status", std_msgs.msg.String, queue_size=1)


    def ur5_status_callback(self, msg):
        if msg.data == "COME_ON":
            self.state = "UR5_READY"
        elif msg.data == "LET_S_GO":
            self.state = "UR5_DONE"

    # Callbacks
    # Adaptar este callback para no depender de la posición aportada por el simulador Gazebo.
    def position_callback(self, ground_truth: Odometry):
        self.position = ground_truth.pose.pose.position
        euler = tf_conversions.transformations.euler_from_quaternion([
            ground_truth.pose.pose.orientation.x,
            ground_truth.pose.pose.orientation.y,
            ground_truth.pose.pose.orientation.z,
            ground_truth.pose.pose.orientation.w
        ])
        self.theta = euler[2]

    def obstacles_callback(self, obstacles: sensor_msgs.msg.PointCloud2):
        pc_obstacles = pc2.read_points(obstacles, field_names=("x", "y", "z"), skip_nans=True)
        # Guardar como geometry_msg.Point
        self.obstacles = []
        num_points = max(int(obstacles.width / 30), 0)
        num_points = 1
        count = 0
        for point in pc_obstacles:
            if count == 0:
                x, y, z = point
                new_point = geometry_msgs.msg.Point(x, y, z)
                self.obstacles.append(new_point)
                count = num_points
            else:
                count -= 1

    def limits_callback(self, limits: sensor_msgs.msg.PointCloud2):
        pc_limits = pc2.read_points(limits, field_names=("x", "y", "z"), skip_nans=True)
        # Guardar como geometry_msg.Point
        self.limits = []

        # Reducir la nube de puntos para acelerar la búsqueda
        num_points = max(int(limits.width / 18), 0)
        count = 0
        for point in pc_limits:
            if count == 0:
                x, y, z = point
                new_point = geometry_msgs.msg.Point(x, y, z)
                self.limits.append(new_point)
                count = num_points
            else:
                count -= 1

    # Planificar una trayectoria hacia el objetivo evitando obstáculos, se ejecutada a una frecuencia más baja.
    def localGoalCalculation(self):
        print("localGoalCalculation started")
        if self.position is None or self.goal_reached or self.limits is None:
            print("self.position is None or self.goal_reached or self.limits is None")
            return
        # Definición de parámetros
        wa, wr, ar, aa = 100.0, 1.0, 0.8, 0.3
        min_force = wr / pow(0.1, ar) - wa / pow(100.0, aa) + 1000

        local_goal = geometry_msgs.msg.Point()
        self.local_path = []
        min_distance = 10000.0
        force_r = 0
        goal_in_local_axis = self.global2local(self.goals[self.goal_id])
        # 1) Puntos de la trayectoria en el radio límite
        for limit_point in self.limits:
            distance_a = self.distance(limit_point, goal_in_local_axis)
            if distance_a < 0.0:
                distance_a = 0.1

            force_a = wa / pow(distance_a, aa)

            # Calculo de la distancia a obstáculo mínima y su peso
            min_distance = 10000.0
            for obstacle_point in self.obstacles:
                distance_r = self.distance(limit_point, obstacle_point)
                if distance_r < 0.1:
                    distance_r = 0.1
                if distance_r < min_distance:
                    min_distance = distance_r
                    force_r = wr / pow(distance_r, ar)

            force = force_r - force_a

            if force < min_force:
                min_force = force
                local_goal = limit_point
        self.local_path.append(local_goal)
        # 2) Puntos de la trayectoria en los anillos internos
        # Definición de parámetros
        wa2 = 3.0
        wr2 = 1.0
        ar2 = 0.5
        aa2 = 0.3
        radious = 4.0
        delta_rad = radious / 3.0

        for rad in np.arange(radious, delta_rad - 0.1, -delta_rad):
            min_force = wr / pow(0.1, ar) - wa / pow(100.0, aa) + 1000
            # Calculo de la distancia a obstáculo mínima y su peso
            min_distance = 10000.0
            for limit_point in self.limits:
                depth, azimuth = self.cartesian2Spherical(limit_point.x, limit_point.y)
                p_in = self.spherical2Cartesian(rad, azimuth)

                distance_a = self.distance(p_in, self.local_path[0])
                if distance_a < 0.0:
                    distance_a = 0.1
                force_a = wa2 / pow(distance_a, aa2)

                min_distance = 10000.0
                for obstacle_point in self.obstacles:
                    distance_r = self.distance(p_in, obstacle_point)
                    if distance_r < 0.1:
                        distance_r = 0.1
                    if distance_r < min_distance:
                        min_distance = distance_r
                        force_r = wr2 / pow(distance_r, ar2)

                force = force_r - force_a
                if force < min_force:
                    min_force = force
                    local_goal = p_in
            self.local_path.append(local_goal)

        # Publicar la visualización del objetivo
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
        # Detectar si ha terminado la trayectoria
        if self.position is None or self.goal_reached:
            return

        ackermann_control = ackermann_msgs.msg.AckermannDrive()
        ackermann_control.speed, ackermann_control.steering_angle = 0.0, 0.0
        # Detectar si ha llegado al siguiente punto objetivo. 
        if self.distance(self.position, self.goals[self.goal_id]) < self.reached_distance:
            print("Goal reached")
            self.blue_status_publisher.publish(std_msgs.msg.String(data="I_AM_HERE"))
            self.goal_reached = True
            self.ackermann_command_publisher.publish(ackermann_control)
            return

        # Reducir la velocidad cuando se acerca al objetivo
        goal_distance = self.distance(self.position, self.goals[self.goal_id])
        if goal_distance - self.reached_distance < self.slow_down_distance:
            speed = MIN_SPEED
            # Establecer el objetivo como objetivo local
            if len(self.local_path) > 1:
                self.local_path[-2] = self.global2local(self.goals[self.goal_id])
        else:
            self.margin = 1.0
            speed = MAX_SPEED

        # Inicializar variables
        min_error = 1000

        # Comprobar posibles acciones de control
        for steer in np.arange(-MAX_STEER_ANGLE, MAX_STEER_ANGLE + 0.01, self.delta_angle):
            if abs(steer) < 0.01:
                steer = 0.0
            # Reducir la velocidad cuando el giro es mayor
            k_sp = (MAX_STEER_ANGLE - abs(steer)) / MAX_STEER_ANGLE
            speed2 = max(speed * k_sp, MIN_SPEED)

            # Comprobar movimiento hacia adelante и назад.
            directions = [speed2, -speed2]  # Добавляем движение назад
            for dir in directions:
                flag_collision_risk = False

                # Calcular radio и velocidad de giro mediante la cinemática del робот.
                if steer == 0:
                    radius = float('inf')  # Прямолинейное движение
                    angular_velocity = 0
                else:
                    radius = VEHICLE_LENGHT / tan(steer)
                    angular_velocity = dir / radius

                rospy.loginfo(f"Steering: {steer}, Speed: {dir}, Radius: {radius}, Angular Velocity: {angular_velocity}")

                # Вычисление траектории с помощью угла поворота
                for sample in np.arange(self.delta_sample, self.max_sample + 0.01, self.delta_sample):
                    local_point = geometry_msgs.msg.Point()
                    local_point.x = self.position.x + sample * cos(self.theta + angular_velocity * sample)
                    local_point.y = self.position.y + sample * sin(self.theta + angular_velocity * sample)

                    # Обнаружение риска столкновений
                    for obstacle in self.obstacles:
                        if self.distance(local_point, obstacle) < 0.5:
                            flag_collision_risk = True
                            rospy.logwarn("Collision risk detected")
                            break

                    if flag_collision_risk:
                        break

                # Если нет столкновений, вычислить его оценку.
                if not flag_collision_risk:
                    # Вычисление оценки траектории на основе ошибки в расстоянии и угле к цели (self.local_path[-2]).
                    if len(self.local_path) > 1:
                        error = self.distance(local_point, self.local_path[-2])
                        if error < min_error:
                            min_error = error
                            ackermann_control.steering_angle = steer
                            ackermann_control.speed = dir

        rospy.loginfo(f"Published ackermann control: speed={ackermann_control.speed}, steering_angle={ackermann_control.steering_angle}")
        self.ackermann_command_publisher.publish(ackermann_control)


    def distance(self, p1: geometry_msgs.msg.Point, p2: geometry_msgs.msg.Point):
        return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    # Ángulo del punto p2 al p1
    def angle(self, p1: geometry_msgs.msg.Point, p2: geometry_msgs.msg.Point):
        return atan2((p1.y - p2.y), p1.x - p2.x)

    # Transformación de posición global a local
    def global2local(self, p: geometry_msgs.msg.Point):
        result = geometry_msgs.msg.Point()
        # Traslación
        x = (p.x - self.position.x)
        y = (p.y - self.position.y)
        # Rotación
        result.x = x * cos(-self.theta) - y * sin(-self.theta)
        result.y = x * sin(-self.theta) + y * cos(-self.theta)
        return result

    # Coordenadas esféricas a cartesianas
    def spherical2Cartesian(self, depth, azimuth):
        sin_azimuth = sin(azimuth)
        cos_azimuth = cos(azimuth)
        p = geometry_msgs.msg.Point()
        p.x = depth * cos_azimuth
        p.y = depth * sin_azimuth
        return p

    # Coordenadas cartesianas a esféricas
    def cartesian2Spherical(self, x, y):
        depth = sqrt((x * x) + (y * y))

        azimuth = atan2(y, x)

        if azimuth < 0:
            azimuth += 2 * pi
        if azimuth >= 2 * pi:
            azimuth -= 2 * pi

        return depth, azimuth

    def run(self):
        # Bucle de control
        rate = rospy.Rate(self.rate)
        count = 3
        while not rospy.is_shutdown():
            if self.state == "UR5_READY" or True:
                print("Blue started their path a the UR5")
                print(f"BLUE: current goal is {self.goals}")
                self.controlActionCalculation()
                if count == 0:
                    self.localGoalCalculation()
                    count = 3
                else:
                    count -= 1
            elif self.state == "UR5_DONE":
                print("Blue started their path at home")
                self.goals = [geometry_msgs.msg.Point(0.0, 0.0, 0.0)] 
                print(f"BLUE: current goal is {self.goals}")
                self.controlActionCalculation()
                if count == 0:
                    self.localGoalCalculation()
                    count = 3
                else:
                    count -= 1

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
