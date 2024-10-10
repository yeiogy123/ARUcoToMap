"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import tf2_ros
import subprocess  # Import subprocess module
import json 
class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()
        self.first_image_processed = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
        if self.first_image_processed:
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = "map"
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp
        # self.get_logger().info(f"frame id is = {img_msg.header.frame_id}")
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )
        if marker_ids is not None:
            if cv2.__version__ > "4.0.0":
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            self.get_logger().info(f"self.intrinsic_mat:{self.intrinsic_mat}, :self.distortion{ self.distortion}")
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                self.get_logger().info(f"{tvecs[i][0][0]}, {tvecs[i][0][1]}, {tvecs[i][0][2]}")
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.info_msg.header.frame_id
                pose_stamped.header.stamp = img_msg.header.stamp
                pose_stamped.pose = pose
                if pose_stamped.header.frame_id == "":
                    self.get_logger().error("PoseStamped frame_id is empty!")
                    continue

                try:
                    transform = self.tf_buffer.lookup_transform(target_frame='map', source_frame='camera_link', time=rclpy.time.Time())
                    self.get_logger().info(f"Transform: {transform}")

                    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
                    pose = pose_transformed
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().error(f"TF Error: {e}")
                    continue
                self.get_logger().info(f"{pose}")
                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])
            

            self.get_logger().info(f"{pose_array}")
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            self.first_image_processed = True
            # if pose_array.poses:
            #     self.send_goal_to_navigation(pose_array.poses[0])
            # else:
            #     self.get_logger().warn("No valid poses found to send to navigation")


    # def send_goal_to_navigation(self, pose):
    #     # Construct the goal message in the required format
    #     # goal = { pose: {header: {frame_id: 'map'},pose:{position: {x: pose.position.x,y: pose.position.y,z: pose.position.z},orientation: {x: pose.orientation.x,y: pose.orientation.y,z: pose.orientation.z,w: pose.orientation.w}}}}
    #     x = pose.position.x
    #     y = pose.position.y
    #     z = pose.position.z
    #     ox = pose.orientation.x
    #     oy = pose.orientation.y
    #     oz = pose.orientation.z
    #     ow = pose.orientation.w
    #     goal_str = f"\"{{ pose: {{header: {{frame_id: \'map\'}},pose: {{position: {{x: {x}, y: {y}, z: {z}}},orientation: {{x: {ox}, y: {oy}, z: {oz}, w: {ow}}}}}}}}}\""
    #     ## command = [
    #     #     'ros2', 'action', 'send_goal', '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose', goal_str
    #     # ]
    #     command = 'ros2' + ' action' + ' send_goal'+' /navigate_to_pose' +' nav2_msgs/action/NavigateToPose' + ' '+goal_str
    #         # 將目標資料轉換為JSON格式

    #     # # Convert the goal message to JSON
    #     # goal_json = json.dumps(goal, indent=4)
    #     # goal_json = f'"{goal_json}"'
    #     # self.get_logger().info(f"{goal_json}")
    #     # Call the `ros2 action send_goal` command
    #     try:
    #         self.get_logger().info(f"{command}")

    #         result = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,shell=True)
    #         for line in iter(result.stdout.readline, ' '):
    #             self.get_logger().info(line)
    #         self.get_logger().info(f"Send goal result: {result.stdout}")
    #     except subprocess.CalledProcessError as e:
    #         self.get_logger().error(f"Failed to send goal: {e.stderr}")

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
