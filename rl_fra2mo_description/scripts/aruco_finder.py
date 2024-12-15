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


from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
import os
import math
from rclpy.node import Node
from std_msgs.msg import String
import time


class ArucoFinder(Node):

    aruco_pose_mf_= PoseStamped()
    aruco_pose_cf_ = PoseStamped()
    aruco_detected_ = False
    waypoint_order_ = [9, 10]

    def __init__(self):
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

        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        navigator.goToPose(goal_poses[0])

        # Creazione del timer che richiama cmd_publisher ogni 10 millisecondi
        self.timer = self.create_timer(0.01, lambda: self.wait_for_aruco(navigator))


        
    def wait_for_aruco(self, navigator):
         if ArucoFinder.aruco_detected_ is True:
                print("Cancel task")
                #navigator.cancelTask()




    def listener_callback(self, msg):
        #if ArucoFinder.aruco_detected_ is False:
            aruco_pose_sf = msg
            ArucoFinder.aruco_pose_cf_.pose.position.x = aruco_pose_sf.pose.position.z
            ArucoFinder.aruco_pose_cf_.pose.position.y = -aruco_pose_sf.pose.position.x
            ArucoFinder.aruco_pose_cf_.pose.position.z = -aruco_pose_sf.pose.position.y
            print(ArucoFinder.aruco_pose_cf_.pose.position)
            ArucoFinder.aruco_detected_ = True
            

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