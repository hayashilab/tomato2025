# overlay.py
import cv2
import os
import numpy as np

def process_input(source):
    """
    Determines the type of input:
      - "camera": opens the default camera.
      - Video file (with extensions .mp4, .avi, .mov): opens as video.
      - Otherwise, assumes an image file.
    """
    if source == "camera":
        return cv2.VideoCapture(0)
    ext = os.path.splitext(source)[1].lower()
    if ext in [".mp4", ".avi", ".mov"]:
        return cv2.VideoCapture(source)
    image = cv2.imread(source)
    return image

def resize_frame(frame, max_width=800):
    """
    Resizes the frame if its width exceeds max_width.
    """
    height, width = frame.shape[:2]
    if width > max_width:
        scaling_factor = max_width / width
        frame = cv2.resize(frame, (max_width, int(height * scaling_factor)))
    return frame

def overlay_frame(frame, processed_data, config):
    """
    Draws overlays on the frame based on processed detection data and configuration.
    
    Supported overlays (based on config["math_processes"]):
      - "detection": draws the bounding box.
      - "name": draws the element name, detection number (ranked by confidence), and confidence.
      - "contour": draws the detection contour.
      - "centroid": draws a small circle at the centroid.
      - "ellipse": draws the recreated ellipse (if available).
      
    Each element type uses a distinct, moderate color.
    
    Logic:
      • If config["elements"][element] is [0] (show all):
            For each detection in order, label = array index + 1.
      • Otherwise, for a custom list (e.g. [1, 3]), we use each number minus one as the array index,
            and display the number as the label.
    """
    # Define moderate colors for each element (BGR format):
    color_mapping = {
        "good tomato": (0, 200, 0),    # moderate green
        "bad tomato": (200, 0, 0),       # moderate blue-ish (avoid red)
        "green tomato": (0, 200, 200),   # moderate yellow/cyan blend
        "stem": (200, 200, 0)            # moderate cyan-like
    }
    
    for element, detections in processed_data.items():
        color = color_mapping.get(element, (255, 255, 255))
        config_indices = config["elements"].get(element, [])
        if config_indices == [0]:
            # Show all detections: label as array index + 1.
            for i, det in enumerate(detections):
                label_num = i + 1
                d = det.get("detection", {})
                if config["math_processes"].get("detection") and "img_box" in d:
                    x_min, x_max, y_min, y_max = d["img_box"]
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
                if config["math_processes"].get("name"):
                    name = d.get("element", "unknown")
                    confidence = d.get("confidence", 0)
                    text = f"{name} {label_num} conf: {confidence:.2f}"
                    if "img_box" in d:
                        x_min, _, y_min, _ = d["img_box"]
                        cv2.putText(frame, text, (x_min, y_min - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                if config["math_processes"].get("contour") and d.get("contour") is not None:
                    cnt = np.array(d["contour"]).reshape((-1, 1, 2))
                    cv2.polylines(frame, [cnt], isClosed=True, color=color, thickness=2)
                if config["math_processes"].get("centroid") and det.get("centroid") is not None:
                    cx, cy = det["centroid"]
                    cv2.circle(frame, (cx, cy), 4, color, -1)
                if config["math_processes"].get("ellipse") and det.get("ellipse") is not None:
                    center, axes, angle = det["ellipse"]
                    center = (int(center[0]), int(center[1]))
                    axes = (int(axes[0] / 2), int(axes[1] / 2))
                    cv2.ellipse(frame, center, axes, angle, 0, 360, color, 2)
        else:
            # Custom config: treat each value as human-readable label;
            # subtract one to get the array index.
            for custom_label in config_indices:
                idx = custom_label - 1
                if idx < 0 or idx >= len(detections):
                    continue
                # Display the custom label (as given in config) as the detection number.
                label_num = custom_label
                det = detections[idx]
                d = det.get("detection", {})
                if config["math_processes"].get("detection") and "img_box" in d:
                    x_min, x_max, y_min, y_max = d["img_box"]
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
                if config["math_processes"].get("name"):
                    name = d.get("element", "unknown")
                    confidence = d.get("confidence", 0)
                    text = f"{name} {label_num} conf: {confidence:.2f}"
                    if "img_box" in d:
                        x_min, _, y_min, _ = d["img_box"]
                        cv2.putText(frame, text, (x_min, y_min - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                if config["math_processes"].get("contour") and d.get("contour") is not None:
                    cnt = np.array(d["contour"]).reshape((-1, 1, 2))
                    cv2.polylines(frame, [cnt], isClosed=True, color=color, thickness=2)
                if config["math_processes"].get("centroid") and det.get("centroid") is not None:
                    cx, cy = det["centroid"]
                    cv2.circle(frame, (cx, cy), 4, color, -1)
                if config["math_processes"].get("ellipse") and det.get("ellipse") is not None:
                    center, axes, angle = det["ellipse"]
                    center = (int(center[0]), int(center[1]))
                    axes = (int(axes[0] / 2), int(axes[1] / 2))
                    cv2.ellipse(frame, center, axes, angle, 0, 360, color, 2)
    return frame


def simplify_contour(contour, epsilon_ratio=0.01):
    """
    Simplifies a contour using polygonal approximation.
    :param contour: A numpy array or list of (x, y) points.
    :param epsilon_ratio: Fraction of arc length to use as approximation accuracy.
    :return: A list of (x, y) tuples representing the simplified contour.
    """
    if contour is None or len(contour) == 0:
        return None
    arr = np.array(contour, dtype=np.float32)
    if arr.ndim == 2 and arr.shape[1] == 2:
        arr = arr.reshape((-1, 1, 2))
    arc_len = cv2.arcLength(arr, True)
    epsilon = epsilon_ratio * arc_len
    approx = cv2.approxPolyDP(arr, epsilon, True)
    approx = approx.reshape(-1, 2)
    return [tuple(map(int, pt)) for pt in approx]

def publish_data(processed_data, config, node=None):
    """
    Publishes ROS2 data with modular field selection.
    
    The configuration includes:
      - "elements": a dict mapping element names to a list of human-readable detection numbers ([0] means all).
      - "ros2_fields": a dict mapping element names to a list of field names to include.
        For example, you might want to send "contour" only for "good tomato" but not for "bad tomato".
    
    In this function, the "detection" field is always cleansed of the "contour" key.
    
    Args:
        processed_data: The processed detection data.
        config: Configuration dictionary.
        node: Optional ROS2 node instance. If not provided, just prints the data.
    """
    # Import ROS2 libraries only when needed
    if node:
        try:
            from rclpy.node import Node
            from geometry_msgs.msg import Point, PointStamped, Polygon, Point32
            from std_msgs.msg import Float32, String, Float32MultiArray
            from visualization_msgs.msg import Marker
            from cv_bridge import CvBridge
            import numpy as np
        except ImportError:
            print("ROS2 libraries not available. Falling back to console output.")
            node = None
    
    ros2_fields_config = config.get("ros2_fields", {})
    data_to_send = {}
    
    # Initialize publishers if using ROS2 and publishers don't exist yet
    if node and not hasattr(node, 'publishers'):
        node.publishers = {}
        node.bridge = CvBridge()
        
        # Create a publisher for each element type and data field
        for element, fields in ros2_fields_config.items():
            element_key = element.replace(" ", "_")
            for field in fields:
                topic_name = f'/tomato/{element_key}/{field}'
                
                # Choose message type based on field
                if field == "detection":
                    msg_type = String  # Simple string serialization for now
                elif field == "contour":
                    msg_type = Polygon  # Polygon for contour points
                elif field == "centroid":
                    msg_type = PointStamped  # PointStamped for centroid
                elif field == "ellipse":
                    msg_type = Marker  # Visualization marker for ellipse
                else:
                    continue  # Skip unknown fields
                
                publisher = node.create_publisher(msg_type, topic_name, 10)
                node.publishers[(element, field)] = publisher
    
    # Process data for each element
    for element, detections in processed_data.items():
        allowed_fields = ros2_fields_config.get(element, [])
        if not allowed_fields:
            continue  # Skip if no fields are enabled for this element.
        
        config_indices = config.get("elements", {}).get(element, [0])
        if config_indices == [0]:
            selected = detections
        else:
            selected = []
            for human_label in config_indices:
                idx = human_label - 1  # convert human-readable number (1-indexed) to 0-indexed array index
                if 0 <= idx < len(detections):
                    selected.append(detections[idx])
        
        element_msgs = []
        for det in selected:
            msg = {}
            if "detection" in allowed_fields:
                detection_data = det.get("detection", {}).copy()
                # Always remove the "contour" key from detection_data to avoid duplication.
                detection_data.pop("contour", None)
                msg["detection"] = detection_data
            if "contour" in allowed_fields:
                raw_contour = det.get("contour")
                from overlay import simplify_contour
                msg["contour"] = simplify_contour(raw_contour, epsilon_ratio=0.01)
            if "centroid" in allowed_fields:
                msg["centroid"] = det.get("centroid")
            if "ellipse" in allowed_fields:
                msg["ellipse"] = det.get("ellipse")
            filtered_msg = {k: v for k, v in msg.items() if v is not None}
            if filtered_msg:
                element_msgs.append(filtered_msg)
        if element_msgs:
            data_to_send[element] = element_msgs
    
    # Publish to ROS2 if a node is provided
    if node:
        try:
            for element, element_msgs in data_to_send.items():
                for det_idx, msg_data in enumerate(element_msgs):
                    # Get current ROS time for timestamp
                    now = node.get_clock().now().to_msg()
                    
                    for field, data in msg_data.items():
                        publisher = node.publishers.get((element, field))
                        if not publisher:
                            continue
                        
                        if field == "detection":
                            # Serialize detection data as JSON string
                            import json
                            str_msg = String()
                            str_msg.data = json.dumps(data)
                            publisher.publish(str_msg)
                        
                        elif field == "contour" and data:
                            # Convert contour points to ROS2 Polygon message
                            polygon_msg = Polygon()
                            for point in data:
                                p = Point32()
                                p.x = float(point[0])
                                p.y = float(point[1])
                                p.z = 0.0
                                polygon_msg.points.append(p)
                            publisher.publish(polygon_msg)
                        
                        elif field == "centroid" and data:
                            # Convert centroid to ROS2 PointStamped message
                            point_msg = PointStamped()
                            point_msg.header.stamp = now
                            point_msg.header.frame_id = "camera_frame"  # Adjust as needed
                            point_msg.point.x = float(data[0])
                            point_msg.point.y = float(data[1])
                            point_msg.point.z = 0.0
                            publisher.publish(point_msg)
                        
                        elif field == "ellipse" and data:
                            # Create visualization marker for ellipse
                            marker = Marker()
                            marker.header.stamp = now
                            marker.header.frame_id = "camera_frame"  # Adjust as needed
                            marker.ns = f"{element}_{det_idx}"
                            marker.id = det_idx
                            marker.type = Marker.SPHERE
                            marker.action = Marker.ADD
                            
                            # Ellipse center and dimensions
                            center, axes, angle = data
                            marker.pose.position.x = float(center[0])
                            marker.pose.position.y = float(center[1])
                            marker.pose.position.z = 0.0
                            
                            # Orientation from angle (simplified)
                            import math
                            marker.pose.orientation.w = math.cos(angle * math.pi / 360.0)
                            marker.pose.orientation.z = math.sin(angle * math.pi / 360.0)
                            
                            # Scale
                            marker.scale.x = float(axes[0])
                            marker.scale.y = float(axes[1])
                            marker.scale.z = 0.1  # Thin in Z dimension
                            
                            # Color based on element type
                            if element == "good tomato":
                                marker.color.r = 0.0
                                marker.color.g = 1.0
                                marker.color.b = 0.0
                            elif element == "bad tomato":
                                marker.color.r = 1.0
                                marker.color.g = 0.0
                                marker.color.b = 0.0
                            elif element == "green tomato":
                                marker.color.r = 0.0
                                marker.color.g = 1.0
                                marker.color.b = 1.0
                            else:
                                marker.color.r = 0.0
                                marker.color.g = 0.0
                                marker.color.b = 1.0
                            
                            marker.color.a = 0.5  # Semi-transparent
                            publisher.publish(marker)
            
            node.get_logger().info(f"Published tomato detection data for {len(data_to_send)} element types")
        
        except Exception as e:
            if node:
                node.get_logger().error(f"Error publishing ROS2 data: {str(e)}")
            else:
                print(f"Error: {str(e)}")
    else:
        # Fallback: print the data that would be published
        print("Publishing data via ROS2:")
        for element, msgs in data_to_send.items():
            print(f"  {element}: {len(msgs)} detections")

