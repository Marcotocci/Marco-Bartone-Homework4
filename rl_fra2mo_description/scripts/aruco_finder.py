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


from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
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
    aruco_transformation_matrix_basefootprint_ = tft.identity_matrix() #Transformation matrix from basefootprint to aruco frame

    aruco_detected_ = False #Bool: true if the aruco has been detected once 
    print_check_ = False #Bool:true if the aruco has been computed 

    aruco_position_mf_ = np.zeros(3)
    aruco_orientation_quaternion_mf_ = np.zeros(4)
    aruco_orientation_rpy_mf_ = np.zeros(3)
    waypoint_order_ = [14, 0]
    counter_ = 0
    aruco_correctly_computed_ = False
    goal_poses_ = np.zeros(2)
    aruco_position_wf = np.zeros(3)
    aruco_orientation_quaternion_wf = np.zeros(4) 
    transform_broadcasted_ = TransformStamped()

    def __init__(self): #Constructor
        super().__init__('aruco_finder')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        

        navigator = BasicNavigator()

        # Ottieni il percorso del pacchetto rl_fra2mo_description
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        # Carica i waypoint
        waypoints = ArucoFinder.load_waypoints(package_path)

        if not waypoints:
            print("Nessun waypoint caricato. Uscita.")
            return

        ArucoFinder.goal_poses_ = [ArucoFinder.create_pose(waypoints[i], navigator) for i in ArucoFinder.waypoint_order_]

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
        
        navigator.goToPose(ArucoFinder.goal_poses_[0])


        # Creazione del timer che richiama timer_callback ogni 10 millisecondi
        self.timer = self.create_timer(0.01, lambda: self.timer_callback(navigator))
    #END constructor


    
    def timer_callback(self, navigator): 

        # Computation of the global transformation matrix (aruco in map frame)
        if ArucoFinder.aruco_detected_ is True:

            aruco_transformation_matrix_mf = tft.concatenate_matrices(ArucoFinder.odom_transformation_matrix_mf_, ArucoFinder.aruco_transformation_matrix_of_)

            ArucoFinder.aruco_position_mf_ = tft.translation_from_matrix(aruco_transformation_matrix_mf)
            ArucoFinder.aruco_orientation_quaternion_mf_  = tft.quaternion_from_matrix(aruco_transformation_matrix_mf)
            ArucoFinder.aruco_orientation_rpy_mf_ = tft.euler_from_quaternion(ArucoFinder.aruco_orientation_quaternion_mf_,axes = 'sxyz')
            
            if navigator.isTaskComplete():
                ArucoFinder.counter_+=1
                if ArucoFinder.counter_== 20:
                    print("Posizione dell'aruco nel frame della mappa:",ArucoFinder.aruco_position_mf_)
                    print("Orientamento dell'aruco nel frame della mappa:",ArucoFinder.aruco_orientation_rpy_mf_)

                    ArucoFinder.aruco_position_wf = [ArucoFinder.aruco_position_mf_[1]-3, 3.5-ArucoFinder.aruco_position_mf_[0], ArucoFinder.aruco_position_mf_[2]]
                    aruco_orientation_rpy_wf = [ArucoFinder.aruco_orientation_rpy_mf_[0],ArucoFinder.aruco_orientation_rpy_mf_[1],ArucoFinder.aruco_orientation_rpy_mf_[2]-np.pi/2]
                    ArucoFinder.aruco_orientation_quaternion_wf = tft.quaternion_from_euler(aruco_orientation_rpy_wf[0],aruco_orientation_rpy_wf[1],aruco_orientation_rpy_wf[2],axes='sxyz')

                    print("Posizione dell'aruco nel frame del mondo:",ArucoFinder.aruco_position_wf)
                    print("Orientamento dell'aruco nel frame del mondo:",aruco_orientation_rpy_wf)

                    ArucoFinder.aruco_correctly_computed_ = True 
                    navigator.goToPose(ArucoFinder.goal_poses_[1])
            
            if ArucoFinder.aruco_correctly_computed_ is True:

                
                ArucoFinder.transform_broadcasted_.transform.translation = Vector3(x=ArucoFinder.aruco_position_wf[0],y=ArucoFinder.aruco_position_wf[1],z=ArucoFinder.aruco_position_wf[2])
                ArucoFinder.transform_broadcasted_.transform.rotation = Quaternion(x=ArucoFinder.aruco_orientation_quaternion_wf[0],y=ArucoFinder.aruco_orientation_quaternion_wf[1],z=ArucoFinder.aruco_orientation_quaternion_wf[2],w=ArucoFinder.aruco_orientation_quaternion_wf[3])
                ArucoFinder.transform_broadcasted_.header.stamp = self.get_clock().now().to_msg()
                ArucoFinder.transform_broadcasted_.header.frame_id = 'world'
                ArucoFinder.transform_broadcasted_.child_frame_id = 'aruco_tag'
                self.tf_static_broadcaster.sendTransform(ArucoFinder.transform_broadcasted_)
    #END timer_callback


    def odom_listener_callback(self, msg):
        ArucoFinder.robot_pose_odom_ = msg

         # Computation of the transformation matrix (robot in odometry frame)
        robot_transformation_matrix_of = ArucoFinder.pose_to_matrix(ArucoFinder.robot_pose_odom_.pose.pose)
        
        # Computation of the trasnformation matrix (aruco in odometry frame)
        ArucoFinder.aruco_transformation_matrix_of_ = tft.concatenate_matrices(robot_transformation_matrix_of,ArucoFinder.aruco_transformation_matrix_basefootprint_)

    #END odom_listener_callback
    
    
        
    def tf_listener_callback(self, msg):
        #odom_trans_mf = TransformStamped()
        for transform in msg.transforms:
            if transform.child_frame_id == ArucoFinder.target_frame:
                odom_trans_mf = msg.transforms[0]

                # Computation of the transformation matrix (odometry in map frame)
                
                ArucoFinder.odom_transformation_matrix_mf_ =ArucoFinder.transform_to_matrix(odom_trans_mf.transform)
    #END tf_listener_callback


    def listener_callback(self, msg):
        aruco_pose_sf = msg

        #rotations required to transform sensor frame in camera frame reference
        rx = tft.rotation_matrix(-np.pi/2, (1,0,0))
        rz = tft.rotation_matrix(-np.pi/2, (0,0,1))

        #transformation matrix from camera frame to basefootprint frame. It's a z-traslation
        camera_transformation_matrix_basefootprint = tft.translation_matrix((0,0, 0.237 + 0.1))

        #concatenation of previously defined matrices
        R_sf_to_cf = tft.concatenate_matrices(rz,rx)

        #build the transformation matrix from the aruco pose obtained from /aruco_single
        Aruco_transformation_matrix_sf = ArucoFinder.pose_to_matrix(aruco_pose_sf.pose)

        #transform aruco transf matrix from sensor frame to camera frame
        Aruco_transformation_matrix_cf = tft.concatenate_matrices(R_sf_to_cf, Aruco_transformation_matrix_sf)

        ArucoFinder.aruco_transformation_matrix_basefootprint_=tft.concatenate_matrices(camera_transformation_matrix_basefootprint,
                                                                                        Aruco_transformation_matrix_cf)

        ArucoFinder.aruco_detected_ = True
    #END listener_callback          


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
    #END rpy_to_quaternion

    
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
    #END load_waypoints

    
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
    #END create_pose
    
    
    def pose_to_matrix(pose):

        rotMat = tft.concatenate_matrices(
            tft.translation_matrix((pose.position.x,
                                    pose.position.y,
                                    pose.position.z)), 
            tft.quaternion_matrix((pose.orientation.x,
                                   pose.orientation.y,
                                   pose.orientation.z,
                                   pose.orientation.w)))
        
        return rotMat
    #END pose_to_matrix


    def transform_to_matrix(transform):

        rotMat = tft.concatenate_matrices(
            tft.translation_matrix((transform.translation.x,
                                    transform.translation.y,
                                    transform.translation.z)), 
            tft.quaternion_matrix((transform.rotation.x,
                                   transform.rotation.y,
                                   transform.rotation.z,
                                   transform.rotation.w)))

        return rotMat
    #END transform_to_matrix


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