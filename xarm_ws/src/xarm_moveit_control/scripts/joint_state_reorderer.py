#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

class JointStateReorderer(Node):
    def __init__(self):
        super().__init__('joint_state_reorderer')
        
        # Correct joint order for xArm7
        self.correct_order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        # Subscribe to the messy joint states from fake hardware
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Match the original joint_states QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
                
        # Publisher for corrected joint states
        self.publisher = self.create_publisher(JointState, '/xarm/joint_states', qos_profile)
        
        self.get_logger().info('Joint State Reorderer started')
        self.get_logger().info(f'Expected order: {self.correct_order}')
    
    def joint_state_callback(self, msg):
        # Log the received order (for debugging - only first time)
        if not hasattr(self, '_first_msg'):
            self.get_logger().info(f'Received order: {msg.name}')
            self._first_msg = True
        
        # Create reordered message
        corrected_msg = JointState()
        corrected_msg.header = msg.header
        corrected_msg.name = self.correct_order.copy()
        
        # Initialize arrays with correct size
        corrected_msg.position = [0.0] * 7
        corrected_msg.velocity = [0.0] * 7 if msg.velocity else []
        corrected_msg.effort = [0.0] * 7 if msg.effort else []
        
        # Reorder the data arrays based on joint names
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.correct_order:
                correct_idx = self.correct_order.index(joint_name)
                corrected_msg.position[correct_idx] = msg.position[i]
                if msg.velocity:
                    corrected_msg.velocity[correct_idx] = msg.velocity[i]
                if msg.effort:
                    corrected_msg.effort[correct_idx] = msg.effort[i]
            else:
                self.get_logger().warn(f'Unknown joint: {joint_name}')
        
        # Publish the corrected joint states
        self.publisher.publish(corrected_msg)

def main(args=None):
    rclpy.init(args=args)
    reorderer = JointStateReorderer()
    
    try:
        rclpy.spin(reorderer)
    except KeyboardInterrupt:
        pass
    finally:
        reorderer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
