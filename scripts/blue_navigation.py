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

MAX_STEER_ANGLE = 24.0 * pi / 180.0  # Radianes
MAX_SPEED = 1.3  # m/s
MIN_SPEED = 0.6
VEHICLE_LENGTH = 1.05  # m

class BlueTrajectoryPlanner(object):
    def __init__(self):
        super(BlueTrajectoryPlanner, self).__init__()
        rospy.init_node("blue_planner_node", anonymous=True)

        # Inicializar parámetros adicionales
        self.goals = []  # Ejemplos de metas
        self.current_goal_index = 0

        #Inicializar parámetros
        self.delta_angle=3.0*pi/180.0
        self.delta_sample=0.2
        self.max_sample=3.0
        self.reached_distance=1.0
        self.slow_down_distance=1.0
        self.rate=5

        #TODO Declarar otros parámetros necesarios para el planificador.

        #Inicializar variables
        self.position = None
        self.theta = 0 
        self.obstacles = []
        self.limits = None
        self.goal_reached = False #Para esperar que el UR5 nos comunique que nos acerquemos.
        #TODO Declarar otros variables que se consideren necesarias.
        self.state = None

        # Subscribers y Publishers
        self.position_subscriber = rospy.Subscriber("/blue/ground_truth",
            Odometry, self.position_callback, queue_size=1)
        self.obstacles_subscriber = rospy.Subscriber("/obstacles",
            sensor_msgs.msg.PointCloud2, self.obstacles_callback, queue_size=1)
        self.obstacles_subscriber = rospy.Subscriber("/free_zone",
            sensor_msgs.msg.PointCloud2, self.limits_callback, queue_size=1)

        self.ur5_status_subscriber = rospy.Subscriber("/ur5/status", std_msgs.msg.String, self.ur5_status_callback)
        self.blue_status_publisher = rospy.Publisher("/blue/status", std_msgs.msg.String, queue_size=1)

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

    def ur5_status_callback(self, msg):
        if msg.data == "COME_ON":
            self.state = "UR5_READY"
        elif msg.data == "LET_S_GO":
            self.state = "UR5_DONE"

    def position_callback(self, data: Odometry):
        self.position = data.pose.pose.position
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                      data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def obstacles_callback(self, obstacles: sensor_msgs.msg.PointCloud2):
        self.obstacles = [geometry_msgs.msg.Point(x=p[0], y=p[1], z=p[2])
                          for p in pc2.read_points(obstacles, field_names=("x", "y", "z"), skip_nans=True)]

    def limits_callback(self, limits: sensor_msgs.msg.PointCloud2):
        self.limits = [geometry_msgs.msg.Point(x=p[0], y=p[1], z=p[2])
                       for p in pc2.read_points(limits, field_names=("x", "y", "z"), skip_nans=True)]

    def localGoalCalculation(self):
        if self.position is None or self.goal_reached or self.limits is None:
            return

        # Utilizar la meta actual de la lista de metas
        current_goal = self.goals[self.current_goal_index]

        # Definición de parámetros
        wa, wr, ar, aa = 100.0, 1.0, 0.8, 0.3
        min_force = wr / pow(0.1, ar) - wa / pow(100.0, aa) + 1000

        local_goal = geometry_msgs.msg.Point()
        self.local_path = []
        min_distance = 10000.0
        force_r = 0
        goal_in_local_axis = self.global2local(current_goal)

        # 1) Puntos de la trayectoria en el radio límite
        for limit_point in self.limits:
            distance_a = self.distance(limit_point, goal_in_local_axis)
            if distance_a < 0.0:
                distance_a = 0.1

            force_a = wa / pow(distance_a, aa)

            # Cálculo de la distancia a obstáculo mínima y su peso
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
        # Continuar con el resto del cálculo y la publicación de los marcadores
        
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
        if self.position is None or self.goal_reached:
            return

        ackermann_control = ackermann_msgs.msg.AckermannDrive()
        ackermann_control.speed, ackermann_control.steering_angle = 0.0, 0.0

        current_goal = self.goals[self.current_goal_index]
        #print(f'self.distance to current_goal {self.distance(self.position, current_goal)}')
        if self.distance(self.position, current_goal) < self.reached_distance:
            if not self.goal_reached:
                print("Goal reached")

            self.blue_status_publisher.publish(std_msgs.msg.String(data="I_AM_HERE"))
            self.goal_reached = True
            self.ackermann_command_publisher.publish(ackermann_control)
            return

        goal_distance = self.distance(self.position, current_goal)
        speed = MIN_SPEED if goal_distance - self.reached_distance < self.slow_down_distance else MAX_SPEED

        min_error = float('inf')
        best_direction = None

        for steer in np.arange(-MAX_STEER_ANGLE, MAX_STEER_ANGLE + 0.01, self.delta_angle):
            speed2 = max(speed * ((MAX_STEER_ANGLE - abs(steer)) / MAX_STEER_ANGLE), MIN_SPEED)
            turning_radius = VEHICLE_LENGTH / tan(steer) if steer != 0 else float('inf')
            angular_velocity = speed2 / turning_radius if turning_radius != float('inf') else 0

            for dir in [-speed2, speed2]:
                flag_collision_risk = False
                collision_free_distance = float('inf')  # Track the free distance before collision

                for sample in np.arange(self.delta_sample, self.max_sample + 0.01, self.delta_sample):
                    delta_theta = angular_velocity * sample
                    new_theta = self.theta + delta_theta
                    local_point = geometry_msgs.msg.Point(
                        x = self.position.x + dir * sample * cos(new_theta),
                        y = self.position.y + dir * sample * sin(new_theta)
                    )

                    for obstacle in self.obstacles:
                        obstacle_distance = self.distance(local_point, obstacle)
                        if obstacle_distance < 0.2:
                            flag_collision_risk = True
                            collision_free_distance = min(collision_free_distance, obstacle_distance)
                            break

                    if flag_collision_risk:
                        break

                if not flag_collision_risk or (flag_collision_risk and collision_free_distance > 1.0):  # Acceptable buffer distance to the nearest obstacle
                    error = self.distance(current_goal, local_point)
                    if error < min_error:
                        min_error = error
                        ackermann_control.steering_angle = steer
                        ackermann_control.speed = dir * speed2
                        best_direction = (steer, dir * speed2)

        if best_direction:
            print(f"Choosed direction: Steering={best_direction[0]}, Speed={best_direction[1]}")
            self.ackermann_command_publisher.publish(ackermann_control)
        else:
            print("No viable path found, stopping.")
            ackermann_control.speed = 0
            self.ackermann_command_publisher.publish(ackermann_control)



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
            if self.state == "UR5_READY":
                self.goals = [geometry_msgs.msg.Point(x=4.5, y=3.5)]  
                self.controlActionCalculation()
                if self.goal_reached:
                    self.goal_reached = False
                    self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
                    ackermann_control = ackermann_msgs.msg.AckermannDrive()
                    ackermann_control.speed, ackermann_control.steering_angle = 0.0, 0.0
                    self.ackermann_command_publisher.publish(ackermann_control)
                
                self.localGoalCalculation()

            elif self.state == "UR5_DONE":
                self.goals = [geometry_msgs.msg.Point(x=0.0, y=6.0)]  
                self.controlActionCalculation()
                if self.goal_reached:
                    self.goal_reached = False
                    self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
                    ackermann_control = ackermann_msgs.msg.AckermannDrive()
                    ackermann_control.speed, ackermann_control.steering_angle = 0.0, 0.0
                    self.ackermann_command_publisher.publish(ackermann_control)
                
                self.localGoalCalculation()

            rate.sleep()

def main():
    try:
        node = BlueTrajectoryPlanner()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
