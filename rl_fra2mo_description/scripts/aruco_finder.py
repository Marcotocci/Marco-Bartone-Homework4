#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
import os
import math
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
import tf_transformations as tft
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage




class ArucoFinder(Node):

    aruco_pose_mf_= PoseStamped() #aruco pose in map_frame
    aruco_pose_cf_ = PoseStamped() #aruco pose in camera_frame
    aruco_pose_basefootprint_ = PoseStamped() #aruco pose in base_footprint frame
    robot_pose_odom_ = Odometry() # robot pose in odometry_frame (built as Odometry type)

    target_frame = 'fra2mo/odom' #id of the requested frame that will be read in /tf topic
    

    odom_transformation_matrix_mf_ = tft.identity_matrix() #Transformation matrix from map frame to odometry frame
    aruco_transformation_matrix_of_ = tft.identity_matrix() #Transformation matrix from odometry frame to aruco frame

    aruco_detected_ = False #Bool: true if the aruco has been detected once 
    aruco_correctly_computed = False #Bool:true if the aruco has been computed 

    aruco_position_mf_ = np.array([])

    #aruco_position_norm_ = 0

    waypoint_order_ = [9, 10] 

    def __init__(self): #Constructor
        super().__init__('aruco_finder')

        navigator = BasicNavigator()

        # Ottieni il percorso del pacchetto rl_fra2mo_description
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        # Carica i waypoint
        waypoints = ArucoFinder.load_waypoints(package_path)

        if not waypoints:
            print("Nessun waypoint caricato. Uscita.")
            return

        goal_poses = [ArucoFinder.create_pose(waypoints[i], navigator) for i in ArucoFinder.waypoint_order_]

        # Wait for navigation to fully activate, since autostarting nav2
        navigator.waitUntilNav2Active(localizer="smoother_server")

        nav_start = navigator.get_clock().now()

        self.subscription = self.create_subscription( #subscriber to the aruco_pose in sensor frame, computes aruco matrix in base_footprint frame
            PoseStamped,
            '/aruco_single/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.subscription_2 = self.create_subscription( #subscriber to robot pose in odometry frame, computes transformation matrix
            Odometry,
            '/model/fra2mo/odometry',
            self.odom_listener_callback,
            10)
        self.subscription_2  # prevent unused variable warning

        self.subscription_3 = self.create_subscription( #subcriber to odometry frame in map frame, computes the transformation matrix
            TFMessage,
            '/tf',
            self.tf_listener_callback,
            10)
        self.subscription_3  # prevent unused variable warning
        
        navigator.goToPose(goal_poses[0])


        # Creazione del timer che richiama timer_callback ogni 10 millisecondi
        self.timer = self.create_timer(0.01, lambda: self.timer_callback(navigator))


    
    def timer_callback(self, navigator): 

            # Computation of the global transformation matrix (aruco in map frame)

        if ArucoFinder.aruco_detected_ is True:

            aruco_transformation_matrix_mf = tft.concatenate_matrices(ArucoFinder.odom_transformation_matrix_mf_, ArucoFinder.aruco_transformation_matrix_of_)

            #ex_aruco_position_norm = ArucoFinder.aruco_position_norm_

            #ArucoFinder.aruco_position_norm_ = np.linalg.norm(tft.translation_from_matrix(aruco_transformation_matrix_mf))

            ex_aruco_position_mf = ArucoFinder.aruco_position_mf_

            ArucoFinder.aruco_position_mf_ = tft.translation_from_matrix(aruco_transformation_matrix_mf)


            if ArucoFinder.aruco_position_mf_.all() == ex_aruco_position_mf.all():
                
                if ArucoFinder.aruco_correctly_computed is False:
                    navigator.cancelTask()
                    print(tft.translation_from_matrix(aruco_transformation_matrix_mf))  

                ArucoFinder.aruco_correctly_computed = True

                

                              




            #print(tft.translation_from_matrix(aruco_transformation_matrix_mf))


            #navigator.cancelTask()
    

    def odom_listener_callback(self, msg):
        ArucoFinder.robot_pose_odom_ = msg

         # Computation of the transformation matrix (robot in odometry frame)

        robot_transformation_matrix_of = tft.concatenate_matrices(tft.translation_matrix((ArucoFinder.robot_pose_odom_.pose.pose.position.x,
                                                                                          ArucoFinder.robot_pose_odom_.pose.pose.position.y,
                                                                                          ArucoFinder.robot_pose_odom_.pose.pose.position.z)), tft.quaternion_matrix((ArucoFinder.robot_pose_odom_.pose.pose.orientation.x,
                                                                                                                                                                    ArucoFinder.robot_pose_odom_.pose.pose.orientation.y,
                                                                                                                                                                    ArucoFinder.robot_pose_odom_.pose.pose.orientation.z,
                                                                                                                                                                    ArucoFinder.robot_pose_odom_.pose.pose.orientation.w)))

        # Computation of the transformation matrix (aruco in base_footprint)

        aruco_transformation_matrix_basefootprint = tft.concatenate_matrices(tft.translation_matrix((ArucoFinder.aruco_pose_basefootprint_.pose.position.x,
                                                                                                     ArucoFinder.aruco_pose_basefootprint_.pose.position.y,
                                                                                                     ArucoFinder.aruco_pose_basefootprint_.pose.position.z)), tft.quaternion_matrix((ArucoFinder.aruco_pose_basefootprint_.pose.orientation.x,
                                                                                                                                                                                   ArucoFinder.aruco_pose_basefootprint_.pose.orientation.y,
                                                                                                                                                                                   ArucoFinder.aruco_pose_basefootprint_.pose.orientation.z,
                                                                                                                                                                                   ArucoFinder.aruco_pose_basefootprint_.pose.orientation.w)))

        ArucoFinder.aruco_transformation_matrix_of_ = tft.concatenate_matrices(robot_transformation_matrix_of, aruco_transformation_matrix_basefootprint)

        #aruco_translation_of = tft.translation_from_matrix(ArucoFinder.aruco_transformation_matrix_of_)

    
    
        
    def tf_listener_callback(self, msg):
        #odom_trans_mf = TransformStamped()
        for transform in msg.transforms:
            if transform.child_frame_id == ArucoFinder.target_frame:
                odom_trans_mf = msg.transforms[0]

                # Computation of the transformation matrix (odometry in map frame)

                ArucoFinder.odom_transformation_matrix_mf_ = tft.concatenate_matrices(tft.translation_matrix((odom_trans_mf.transform.translation.x,
                                                                                                              odom_trans_mf.transform.translation.y,
                                                                                                              odom_trans_mf.transform.translation.z)), tft.quaternion_matrix((odom_trans_mf.transform.rotation.x,
                                                                                                                                                                              odom_trans_mf.transform.rotation.y,
                                                                                                                                                                              odom_trans_mf.transform.rotation.z,
                                                                                                                                                                              odom_trans_mf.transform.rotation.w)))
        


    def listener_callback(self, msg):
        if ArucoFinder.aruco_correctly_computed is False:
            aruco_pose_sf = msg

            rx = tft.rotation_matrix(-np.pi/2, (1,0,0))

            rz = tft.rotation_matrix(-np.pi/2, (0,0,1))


            R_sf_to_cf = tft.concatenate_matrices(rz,rx)


            aruco_quaternion_sf = (aruco_pose_sf.pose.orientation.x,
                                aruco_pose_sf.pose.orientation.y,
                                aruco_pose_sf.pose.orientation.z,
                                aruco_pose_sf.pose.orientation.w )

            Aruco_rotation_sf = tft.quaternion_matrix(aruco_quaternion_sf)

            aruco_position_sf = (aruco_pose_sf.pose.position.x,
                                aruco_pose_sf.pose.position.y,
                                aruco_pose_sf.pose.position.z)

            Aruco_translation_sf = tft.translation_matrix(aruco_position_sf)

            Aruco_transformation_matrix_sf = tft.concatenate_matrices(Aruco_translation_sf,Aruco_rotation_sf)

            Aruco_transformation_matrix_cf = tft.concatenate_matrices(R_sf_to_cf, Aruco_transformation_matrix_sf)

            aruco_temp_position = tft.translation_from_matrix(Aruco_transformation_matrix_cf)

            (ArucoFinder.aruco_pose_cf_.pose.position.x, 
            ArucoFinder.aruco_pose_cf_.pose.position.y, 
            ArucoFinder.aruco_pose_cf_.pose.position.z) = aruco_temp_position

            aruco_temp_orientation = tft.quaternion_from_matrix(Aruco_transformation_matrix_cf)

            (ArucoFinder.aruco_pose_cf_.pose.orientation.x, 
            ArucoFinder.aruco_pose_cf_.pose.orientation.y, 
            ArucoFinder.aruco_pose_cf_.pose.orientation.z,
            ArucoFinder.aruco_pose_cf_.pose.orientation.w) = aruco_temp_orientation

            
            #Passaggio da camera_frame a base_footprint_frame

            ArucoFinder.aruco_pose_basefootprint_.pose.position.x = ArucoFinder.aruco_pose_cf_.pose.position.x
            ArucoFinder.aruco_pose_basefootprint_.pose.position.y = ArucoFinder.aruco_pose_cf_.pose.position.y 
            ArucoFinder.aruco_pose_basefootprint_.pose.position.z = ArucoFinder.aruco_pose_cf_.pose.position.z + 0.237 + 0.1 #0.237 is camera height, 0.1 robot height in reference to the floor

            ArucoFinder.aruco_pose_basefootprint_.pose.orientation = ArucoFinder.aruco_pose_cf_.pose.orientation

            ArucoFinder.aruco_detected_ = True
            #print(ArucoFinder.aruco_pose_cf_.pose.position)
            #print(ArucoFinder.aruco_pose_basefootprint_.pose.position)
            
            



    def rpy_to_quaternion(roll, pitch, yaw):
        """
        Converte angoli di Roll, Pitch, Yaw in un quaternione.
        
        :param roll: Rotazione intorno all'asse X (in radianti)
        :param pitch: Rotazione intorno all'asse Y (in radianti)
        :param yaw: Rotazione intorno all'asse Z (in radianti)
        :return: Dizionario con componenti del quaternione (x, y, z, w)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return {
            'x': qx,
            'y': qy,
            'z': qz,
            'w': qw
        }


    def load_waypoints(package_path):
        """
        Carica i waypoint dal file goal.yaml.
        
        :param package_path: Percorso base del pacchetto rl_fra2mo_description
        :return: Lista di waypoint
        """
        config_path = '/home/user/ros2_ws/src/rl_fra2mo_description/config/goal.yaml'
        
        try:
            with open(config_path, 'r') as file:
                waypoints_data = yaml.safe_load(file)
            return waypoints_data.get('waypoints', [])
        except FileNotFoundError:
            print(f"Errore: File {config_path} non trovato!")
            return []
        except Exception as e:
            print(f"Errore durante il caricamento dei waypoint: {e}")
            return []

    def create_pose(transform, navigator):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        
        # Imposta posizione
        pose.pose.position.x = -(transform["position"]["y"]-3.5)
        pose.pose.position.y = (transform["position"]["x"]+3)
        pose.pose.position.z = transform["position"]["z"]
        
        # Converti RPY in quaternione
        rpy = transform.get("orientation", {})
        roll = rpy.get("roll", 0)
        pitch = rpy.get("pitch", 0)
        yaw = rpy.get("yaw", 0)+1.57
        
        # Converti in quaternione (gli angoli sono già in radianti)
        quaternion = ArucoFinder.rpy_to_quaternion(roll, pitch, yaw)
        
        # Imposta orientamento del quaternione
        pose.pose.orientation.x = quaternion['x']
        pose.pose.orientation.y = quaternion['y']
        pose.pose.orientation.z = quaternion['z']
        pose.pose.orientation.w = quaternion['w']
        
        return pose
    
    



def main(args=None):
    rclpy.init(args=args)

    aruco_finder = ArucoFinder()

    rclpy.spin(aruco_finder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_finder.destroy_node()
    rclpy.shutdown()

    
    exit(0)


if __name__ == '__main__':
    main()