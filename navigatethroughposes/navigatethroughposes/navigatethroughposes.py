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
import rclpy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import transforms3d
from tf2_geometry_msgs import do_transform_pose

class NavigateToARUco(rclpy.node.Node):
    def __init__(self, nav):
        super().__init__("NavigateToARUco")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            lambda msg: self.aruco_pose_callback(msg, nav),
            10
        )

    def aruco_pose_callback(self, msg: PoseArray, navigator):
        if msg.poses:
            aruco_pose = msg.poses[0]
            try:
                max_attempts = 5
                attempts = 0
                while attempts < max_attempts:
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = navigator.get_clock().now().to_msg()
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.pose.position.x = aruco_pose.position.x
                    pose_stamped.pose.position.y = aruco_pose.position.y
                    pose_stamped.pose.position.z = 0.0
                    pose_stamped.pose.orientation.x = 0.0
                    pose_stamped.pose.orientation.y = 0.0
                    pose_stamped.pose.orientation.z = 1.0
                    pose_stamped.pose.orientation.w = 0.0
                    navigator.goToPose(pose_stamped)
                    i = 0
                    while not navigator.isTaskComplete():
                        i = i + 1
                        feedback = navigator.getFeedback()
                        if feedback and i % 5 == 0:
                            print(
                                'Estimated time of arrival: '
                                + '{0:.0f}'.format(
                                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                                / 1e9
                                )
                                + ' seconds.'
                            )
                            # Some navigation timeout to demo cancellation
                            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                                navigator.cancelTask()

                            # Some navigation request change to demo preemption
                            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                                pose_stamped.pose.position.x = 0.0
                                pose_stamped.pose.position.y = 0.0
                                pose_stamped.pose.orientation.w = 1.0
                                pose_stamped.pose.orientation.z = 0.0
                                navigator.goToPose(pose_stamped)

                        # Do something depending on the return code
                        result = navigator.getResult()
                        if result == TaskResult.SUCCEEDED:
                            print('ARUCO Goal succeeded!')
                        elif result == TaskResult.CANCELED:
                            print('ARUCO Goal was canceled!')
                        elif result == TaskResult.FAILED:
                            print('ARUCO Goal failed!')
                        else:
                            print('ARUCO Goal has an invalid return status!')
                        break
            except Exception as e:
                self.get_logger().error(f"Failed to transform aruco pose to map frame: {str(e)}")

class InitialPosePublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__("InitialPosePublisher")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_initial_pose(self):
        try:
            self.get_logger().info('lookingup')
            try:
                trans = self.tf_buffer.lookup_transform(target_frame='map', source_frame='base_link', time=rclpy.time.Time())

                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp = self.get_clock().now().to_msg()

                pose_msg.pose.position.x = trans.transform.translation.x
                pose_msg.pose.position.y = trans.transform.translation.y
                pose_msg.pose.position.z = trans.transform.translation.z
                orientation = trans.transform.rotation
                pose_msg.pose.orientation = orientation
                self.get_logger().info('Got initial pose from RTAB-Map')
                return pose_msg
            
            except Exception as e:
                self.get_logger().info(f'{str(e)}')

        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {str(e)}')
            return None

def main():
    rclpy.init()

    # Initialize the navigator and node
    navigator = BasicNavigator()

    # Create InitialPosePublisher to get the pose from RTAB-Map
    initial_pose_publisher = InitialPosePublisher()
    # Set the initial pose from RTAB-Map
    initial_pose = initial_pose_publisher.get_initial_pose()
    if initial_pose:
        navigator.setInitialPose(initial_pose)
    else:
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        navigator.setInitialPose(initial_pose)

    # Activate navigation, wait until Nav2 is active
    navigator.waitUntilNav2Active()

    # Set the waypoints to follow with the new configuration
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 27.0
    goal_pose1.pose.position.y = 0.0
    goal_pose1.pose.orientation.w = 0.0
    goal_pose1.pose.orientation.z = 1.0

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = -7.0
    goal_pose2.pose.position.y = 0.0
    goal_pose2.pose.orientation.w = 0.0
    goal_pose2.pose.orientation.z = 1.0

    # Append the poses as per your request
    goal_poses.append(goal_pose1)
    goal_poses.append(goal_pose2)
    goal_poses.append(goal_pose1)
    goal_poses.append(goal_pose2)
    goal_poses.append(goal_pose1)
    goal_poses.append(goal_pose2)

    # Start following the waypoints
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + '/'
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()

            # Cancel task if it takes too long
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

            # Preempt task with new goal if it takes too long
            if now - nav_start > Duration(seconds=35.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = now.to_msg()
                goal_pose4.pose.position.x = 0.0
                goal_pose4.pose.position.y = 0.0
                goal_pose4.pose.orientation.w = 0.0
                goal_pose4.pose.orientation.z = 1.0
                nav_start = now
                navigator.goToPose(goal_pose4)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')      
    # Check result of navigation
    aruco_navigator = NavigateToARUco(navigator)
    # 關閉導航與節點
    aruco_navigator.destroy_node()
    initial_pose_publisher.destroy_node()
    navigator.lifecycleShutdown()

    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()