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
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException

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
        self.marker_ids = None
        self.pose_stamped = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()
        self.first_image_processed = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.pose_stamped = None
        self.pose_array = PoseArray()
        self.markers = ArucoMarkers()


    def timer_callback(self):
        if self.marker_ids is not None and self.pose_stamped is not None:
            try:
                map_to_odom = self.tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame='odom',
                    time=Time(),  
                    timeout=Duration(seconds=2.0)
                )
                odom_to_base_link = self.tf_buffer.lookup_transform(
                    target_frame='odom',
                    source_frame='base_link',
                    time=Time(),  
                    timeout=Duration(seconds=2.0)
                )
                base_link_to_camera_link = self.tf_buffer.lookup_transform(
                    target_frame='base_link',
                    source_frame='camera_link',
                    time=Time(),  
                    timeout=Duration(seconds=2.0)
                )  
                combined_transform1 = self.combine_transforms(map_to_odom, odom_to_base_link)
                transform = self.combine_transforms(combined_transform1, base_link_to_camera_link)
                pose = tf2_geometry_msgs.do_transform_pose(self.pose_stamped.pose, transform)
                if pose is not None and self.first_image_processed is not True:
                    self.get_logger().info(f"Transformed Pose: {pose}")
                    self.print_transform(transform)
                    self.pose_array.poses.append(pose)
                    self.markers.poses.append(pose)
                    self.markers.marker_ids.append(int(self.marker_ids[0]))
                    self.get_logger().info(f"PoseArray: {self.pose_array}")
                    self.poses_pub.publish(self.pose_array)
                    self.markers_pub.publish(self.markers)
                    self.first_image_processed = True
                if pose is not None and self.first_image_processed is True:
                    self.poses_pub.publish(self.pose_array)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"TF Error: {e}")

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
        if self.first_image_processed:
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        if self.camera_frame == "":
            self.markers.header.frame_id = self.info_msg.header.frame_id
            self.pose_array.header.frame_id = "map"
        else:
            self.markers.header.frame_id = self.camera_frame
            self.pose_array.header.frame_id = self.camera_frame

        self.markers.header.stamp = img_msg.header.stamp
        self.pose_array.header.stamp = img_msg.header.stamp
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )
        if marker_ids is not None:
            self.marker_ids = marker_ids
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
                pose.position.x = tvecs[i][0][2]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][0]
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
                self.pose_stamped = pose_stamped

    def combine_transforms(self, transform1, transform2):
        t1_translation = transform1.transform.translation
        t1_rotation = transform1.transform.rotation
        t2_translation = transform2.transform.translation
        t2_rotation = transform2.transform.rotation

        t1_matrix = tf_transformations.translation_matrix(
            [t1_translation.x, t1_translation.y, t1_translation.z])
        t1_rotation_matrix = tf_transformations.quaternion_matrix(
            [t1_rotation.x, t1_rotation.y, t1_rotation.z, t1_rotation.w])
        t1_matrix = tf_transformations.concatenate_matrices(t1_matrix, t1_rotation_matrix)

        t2_matrix = tf_transformations.translation_matrix(
            [t2_translation.x, t2_translation.y, t2_translation.z])
        t2_rotation_matrix = tf_transformations.quaternion_matrix(
            [t2_rotation.x, t2_rotation.y, t2_rotation.z, t2_rotation.w])
        t2_matrix = tf_transformations.concatenate_matrices(t2_matrix, t2_rotation_matrix)

        combined_matrix = tf_transformations.concatenate_matrices(t1_matrix, t2_matrix)

        combined_transform = transform1
        combined_transform.transform.translation.x, combined_transform.transform.translation.y, combined_transform.transform.translation.z = tf_transformations.translation_from_matrix(
            combined_matrix)
        combined_transform.transform.rotation.x, combined_transform.transform.rotation.y, combined_transform.transform.rotation.z, combined_transform.transform.rotation.w = tf_transformations.quaternion_from_matrix(
            combined_matrix)

        return combined_transform

    def print_transform(self, trans):
        translation = trans.transform.translation
        rotation = trans.transform.rotation
        rot_matrix = tf_transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

        self.get_logger().info(f'At time {trans.header.stamp.sec}.{trans.header.stamp.nanosec}')
        self.get_logger().info(f'- Translation: [{translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f}]')
        self.get_logger().info(f'- Rotation: in Quaternion [{rotation.x:.3f}, {rotation.y:.3f}, {rotation.z:.3f}, {rotation.w:.3f}]')
        rpy_rad = tf_transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        rpy_deg = [angle * 180.0 / 3.141592653589793 for angle in rpy_rad]
        self.get_logger().info(f'- Rotation: in RPY (radian) [{rpy_rad[0]:.3f}, {rpy_rad[1]:.3f}, {rpy_rad[2]:.3f}]')
        self.get_logger().info(f'- Rotation: in RPY (degree) [{rpy_deg[0]:.3f}, {rpy_deg[1]:.3f}, {rpy_deg[2]:.3f}]')
        self.get_logger().info('- Matrix:')
        for row in rot_matrix:
            self.get_logger().info(f' {row[0]:.3f} {row[1]:.3f} {row[2]:.3f} {row[3]:.3f}')

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
