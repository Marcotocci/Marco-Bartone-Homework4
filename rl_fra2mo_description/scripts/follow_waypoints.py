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
import string


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


def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    path_selector = ' '

    while path_selector != 'map_explore' and path_selector != 'goals': 
        print("Insert path (map_explore/goals):")
        path_selector = input()

    # Ottieni il percorso del pacchetto rl_fra2mo_description
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Carica i waypoint
    waypoints = load_waypoints(package_path)

    if not waypoints:
        print("Nessun waypoint caricato. Uscita.")
        return

    def create_pose(transform):
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
        quaternion = rpy_to_quaternion(roll, pitch, yaw)
        
        # Imposta orientamento del quaternione
        pose.pose.orientation.x = quaternion['x']
        pose.pose.orientation.y = quaternion['y']
        pose.pose.orientation.z = quaternion['z']
        pose.pose.orientation.w = quaternion['w']
        
        return pose

    # Definisce l'ordine specifico dei waypoint

    if path_selector == 'map_explore':
        waypoint_order = [5, 6, 7, 8, 9, 10, 11, 12, 13]

    else:    
        waypoint_order = [3, 4, 2, 1]  # Obiettivo 3 → Obiettivo 4 → Obiettivo 2 → Obiettivo 1
    

    goal_poses = [create_pose(waypoints[i]) for i in waypoint_order]

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    nav_start = navigator.get_clock().now()

    # Ciclo per navigare attraverso i waypoint uno per uno
    for i, goal_pose in enumerate(goal_poses):
        print(f'Navigating to waypoint {waypoint_order[i]}')
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            feedback = navigator.getFeedback()

            if feedback:
                now = navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600):
                    navigator.cancelTask()

        # Controlla il risultato del singolo waypoint
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Goal {waypoint_order[i]} succeeded!')
        elif result == TaskResult.CANCELED:
            print(f'Goal {waypoint_order[i]} was canceled!')
        elif result == TaskResult.FAILED:
            print(f'Goal {waypoint_order[i]} failed!')
        else:
            print(f'Goal {waypoint_order[i]} has an invalid return status!')

    exit(0)


if __name__ == '__main__':
    main()