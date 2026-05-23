#!/usr/bin/env python3
# main.py
import os
import rclpy
import numpy as np
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import (
    translation_matrix, quaternion_matrix,
    translation_from_matrix, quaternion_from_matrix
)

from detection2025.math_module import (
    extract_detections, organize_detections, process_all_detections
)
from detection2025.overlay import overlay_frame, resize_frame


class TomatoDetectionNode(Node):
    def __init__(self):
        super().__init__('tomato_detection_node')
        
        # Publishers
        self.processed_image_pub = self.create_publisher(
            Image, '/tomato/processed_image', 10
        )
        self.optical_pose_pub = self.create_publisher(
            PoseStamped, '/tomato/pose_from_camera_color_optical_frame', 10
        )
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/tomato/target_pose', 10
        )
        
        # Camera intrinsics
        self.camera_matrix = None
        self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )

        # Synchronized image subscribers
        # for d435i
        # color_sub = message_filters.Subscriber(
        #     self, Image, '/camera/camera/color/image_raw'
        # )

        # for d405
        color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_rect_raw'
        )
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], 10, 0.1
        )
        self.sync.registerCallback(self.image_callback)

        # Initialize components
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Configuration
        self.config = {
            "elements": {
                "good tomato": [0],
                "bad tomato": [0],
                "green tomato": [0],
                "stem": [0]
            },
            "math_processes": {
                "detection": True,
                "name": True,
                "contour": True,
                "centroid": True,
                "ellipse": True
            },
            "overlay": True,
            "ros2": True,
            "resize": True,
            "max_width": 800,
            "confidence_threshold": 0.6,
            "max_distance": 1.0  # Maximum distance from link_base in meters
        }
        
        # Load YOLO model
        self._load_model()
        self.get_logger().info("Tomato detection node initialized")

    def _load_model(self):
        """Load YOLO model and send it to GPU if available"""
        try:
            import torch
            model_path = os.path.expanduser(
                '~/tomato2025_ws/detection_ws/src/detection2025/models/best.pt'
            )
            self.model = YOLO(model_path)

            # --- ✅ Move model to GPU if available ---
            if torch.cuda.is_available():
                self.model.to('cuda')
                self.get_logger().info("🚀 YOLO model moved to GPU (CUDA)")
            else:
                self.get_logger().warn("⚠️ CUDA not available — using CPU")

            self.get_logger().info(f"YOLO model loaded from: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise


    def camera_info_callback(self, msg):
        """Store camera intrinsic matrix"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("Camera intrinsic matrix updated")

    def pixel_to_3d(self, x, y, depth_meters):
        """Convert pixel coordinates to 3D coordinates in camera optical frame"""
        if self.camera_matrix is None:
            self.get_logger().warn("Using approximate camera parameters")
            fx, fy, cx, cy = 500.0, 500.0, 320.0, 240.0
        else:
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]

        x_cam = (x - cx) * depth_meters / fx
        y_cam = (y - cy) * depth_meters / fy
        z_cam = depth_meters

        return x_cam, y_cam, z_cam

    def get_robust_depth(self, depth_frame, x, y, radius=5):
        """Get robust depth value using median filtering"""
        h, w = depth_frame.shape
        x, y = int(x), int(y)
        
        # Clamp coordinates
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        
        # Extract ROI
        roi = depth_frame[
            max(0, y - radius):min(h, y + radius + 1),
            max(0, x - radius):min(w, x + radius + 1)
        ]
        
        # Get median of valid depths
        valid_depths = roi[roi > 0]
        if len(valid_depths) > 0:
            return np.median(valid_depths)
        return depth_frame[y, x]

    def transform_pose(self, input_pose, source_frame, target_frame):
        """Transform pose from source frame to target frame using tf2"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )

            # Extract source pose
            pos_src = np.array([
                input_pose.pose.position.x,
                input_pose.pose.position.y,
                input_pose.pose.position.z,
                1.0
            ])
            quat_src = [
                input_pose.pose.orientation.x,
                input_pose.pose.orientation.y,
                input_pose.pose.orientation.z,
                input_pose.pose.orientation.w,
            ]

            # Extract transform
            trans_tf = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            quat_tf = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]

            # Build transformation matrices
            tf_mat = np.dot(
                translation_matrix(trans_tf),
                quaternion_matrix(quat_tf)
            )
            pose_mat = np.dot(
                translation_matrix(pos_src[:3]),
                quaternion_matrix(quat_src)
            )

            # Apply transformation
            result_mat = np.dot(tf_mat, pose_mat)
            translation = translation_from_matrix(result_mat)
            quaternion = quaternion_from_matrix(result_mat)

            # Build output pose
            transformed_pose = PoseStamped()
            transformed_pose.header.frame_id = target_frame
            transformed_pose.header.stamp = input_pose.header.stamp
            transformed_pose.pose.position.x = float(translation[0])
            transformed_pose.pose.position.y = float(translation[1])
            transformed_pose.pose.position.z = float(translation[2])
            transformed_pose.pose.orientation.x = float(quaternion[0])
            transformed_pose.pose.orientation.y = float(quaternion[1])
            transformed_pose.pose.orientation.z = float(quaternion[2])
            transformed_pose.pose.orientation.w = float(quaternion[3])

            self.get_logger().debug(
                f"Transform {source_frame} → {target_frame}: "
                f"({pos_src[0]:.3f}, {pos_src[1]:.3f}, {pos_src[2]:.3f}) → "
                f"({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f})"
            )

            return transformed_pose, True

        except TransformException as ex:
            self.get_logger().error(
                f"Transform failed {source_frame} → {target_frame}: {ex}"
            )
            return None, False

    def is_within_distance_limit(self, pose_msg, max_distance):
        """Check if pose is within distance limit from origin"""
        x = abs(pose_msg.pose.position.x)
        y = abs(pose_msg.pose.position.y)
        z = abs(pose_msg.pose.position.z)
        
        return x <= max_distance and y <= max_distance and z <= max_distance

    def process_good_tomato(self, item, depth_frame, color_msg, 
                           resize_ratio_x, resize_ratio_y):
        """Process a single good tomato detection"""
        if item.get('centroid') is None:
            return

        # Get centroid and scale to original coordinates
        x_center_resized, y_center_resized = item['centroid']
        x_center = x_center_resized * resize_ratio_x
        y_center = y_center_resized * resize_ratio_y

        # Get robust depth value
        depth_value = self.get_robust_depth(depth_frame, x_center, y_center)
        
        # Validate depth
        if depth_value == 0 or np.isnan(depth_value):
            return
        
        depth_meters = depth_value / 1000.0
        if depth_meters <= 0.1 or depth_meters > 5.0:
            return

        # Convert to 3D coordinates
        x_cam, y_cam, z_cam = self.pixel_to_3d(x_center, y_center, depth_meters)

        # Create optical frame pose
        optical_pose = PoseStamped()
        optical_pose.header.stamp = self.get_clock().now().to_msg()
        optical_pose.header.frame_id = "camera_color_optical_frame"
        optical_pose.pose.position.x = x_cam
        optical_pose.pose.position.y = y_cam
        optical_pose.pose.position.z = z_cam
        optical_pose.pose.orientation.w = 1.0

        # Publish optical frame pose
        self.optical_pose_pub.publish(optical_pose)

        # Transform to link_base
        target_pose, success = self.transform_pose(
            optical_pose,
            "camera_color_optical_frame",
            "link_base"
        )

        if not success:
            self.get_logger().warn("Failed to transform pose to link_base")
            return

        # Filter by distance from link_base
        max_dist = self.config["max_distance"]
        if not self.is_within_distance_limit(target_pose, max_dist):
            self.get_logger().info(
                f"Tomato filtered: distance from link_base exceeds {max_dist}m "
                f"(X:{abs(target_pose.pose.position.x):.3f}, "
                f"Y:{abs(target_pose.pose.position.y):.3f}, "
                f"Z:{abs(target_pose.pose.position.z):.3f})"
            )
            return

        # Publish target pose
        target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_pub.publish(target_pose)

        # Log detection
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"GOOD TOMATO DETECTED\n"
            f"Confidence: {item['detection'].get('confidence', 0):.2f}\n"
            f"Pixel: ({x_center:.1f}, {y_center:.1f})\n"
            f"Depth: {depth_meters:.3f} m\n"
            f"Optical frame: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f})\n"
            f"Link_base: ({target_pose.pose.position.x:.3f}, "
            f"{target_pose.pose.position.y:.3f}, "
            f"{target_pose.pose.position.z:.3f})\n"
            f"{'='*50}"
        )

    def image_callback(self, color_msg, depth_msg):
        """Process incoming synchronized images"""
        try:
            # Convert messages to OpenCV format

            # # with No GPU version
            # color_frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            # depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

            # # Store original frame
            # original_h, original_w = color_frame.shape[:2]

            # # Resize for YOLO if configured
            # if self.config.get("resize", False):
            #     color_frame = resize_frame(
            #         color_frame, 
            #         max_width=self.config.get("max_width", 800)
            #     )

            # --- Convert ROS images to OpenCV (CPU) ---
            color_cpu = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

            original_h, original_w = color_cpu.shape[:2]

            # --- ✅ Try GPU resize (CUDA) ---
            try:
                gpu_color = cv2.cuda_GpuMat()
                gpu_color.upload(color_cpu)

                # Resize on GPU — match your target YOLO width (e.g., 800 px)
                target_width = self.config.get("max_width", 800)
                scale = target_width / original_w
                target_height = int(original_h * scale)
                gpu_resized = cv2.cuda.resize(gpu_color, (target_width, target_height))

                color_frame = gpu_resized.download()   # YOLO expects numpy array
                self.get_logger().debug("Using GPU resize ✅")

            except cv2.error as e:
                # --- fallback if CUDA not available ---
                self.get_logger().warn(f"⚠️ CUDA resize failed, fallback to CPU: {e}")
                color_frame = resize_frame(
                    color_cpu, 
                    max_width=self.config.get("max_width", 800)
                )


            # Calculate resize ratios
            resized_h, resized_w = color_frame.shape[:2]
            resize_ratio_x = original_w / resized_w
            resize_ratio_y = original_h / resized_h

            # Run YOLO detection
            results = self.model.predict(color_frame)

            # Process detections
            detections = extract_detections(results)
            organized = organize_detections(detections)
            processed_data = process_all_detections(
                organized,
                color_frame,
                confidence_threshold=self.config["confidence_threshold"]
            )

            # Publish processed image with overlay
            if self.config["overlay"]:
                overlay_result = overlay_frame(color_frame, processed_data, self.config)
                output_msg = self.bridge.cv2_to_imgmsg(overlay_result, 'bgr8')
                output_msg.header = color_msg.header
                self.processed_image_pub.publish(output_msg)

            # Process good tomato detections
            if "good tomato" in processed_data:
                for item in processed_data["good tomato"]:
                    try:
                        self.process_good_tomato(
                            item, depth_frame, color_msg,
                            resize_ratio_x, resize_ratio_y
                        )
                    except Exception as e:
                        self.get_logger().warn(
                            f"Error processing good tomato: {e}"
                        )

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TomatoDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()