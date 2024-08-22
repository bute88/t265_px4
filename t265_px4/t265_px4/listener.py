import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.time import Time
import numpy as np

class VisualInertialOdometryPublisher(Node):
    def __init__(self):
        super().__init__('VIO_pub')
        self.odometry_publisher_ = self.create_publisher(
            VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', 
            10
        )
        self.odometry_subscription_ = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.odometry_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.timer = self.create_timer(1/30, self.timer_callback)  # set rate publish for 30 Hz

        self.latest_msg = None  # Store the latest message

    def odometry_callback(self, msg):
        # Update the latest message with each callback
        self.latest_msg = msg

    def timer_callback(self):
        # Check if there is a new message to publish
        if self.latest_msg is None:
            return
        
        # Create VehicleOdometry message
        vehicle_odometry_msg = VehicleOdometry()
        vehicle_odometry_msg.timestamp = Time.from_msg(self.latest_msg.header.stamp).nanoseconds

        vehicle_odometry_msg.pose_frame = 2
        vehicle_odometry_msg.velocity_frame = 2

        # Rotate position and velocity vectors
        vehicle_odometry_msg.position[0] = self.latest_msg.pose.pose.position.x
        vehicle_odometry_msg.position[1] = -self.latest_msg.pose.pose.position.y
        vehicle_odometry_msg.position[2] = -self.latest_msg.pose.pose.position.z

        vehicle_odometry_msg.velocity[0] = self.latest_msg.twist.twist.linear.x
        vehicle_odometry_msg.velocity[1] = -self.latest_msg.twist.twist.linear.y
        vehicle_odometry_msg.velocity[2] = -self.latest_msg.twist.twist.linear.z

        vehicle_odometry_msg.angular_velocity[0] = self.latest_msg.twist.twist.angular.x
        vehicle_odometry_msg.angular_velocity[1] = -self.latest_msg.twist.twist.angular.y
        vehicle_odometry_msg.angular_velocity[2] = -self.latest_msg.twist.twist.angular.z

        # Set variances
        self.pose_covariance_matrix = np.array(self.latest_msg.pose.covariance).reshape((6, 6)) 
        self.twist_covariance_matrix = np.array(self.latest_msg.twist.covariance).reshape((6, 6))   
        
        vehicle_odometry_msg.position_variance = [self.pose_covariance_matrix[0, 0],
                                                  self.pose_covariance_matrix[1, 1],
                                                  self.pose_covariance_matrix[2, 2]]
        vehicle_odometry_msg.orientation_variance = [self.pose_covariance_matrix[3, 3],
                                                     self.pose_covariance_matrix[4, 4],
                                                     self.pose_covariance_matrix[5, 5]]
        vehicle_odometry_msg.velocity_variance = [self.twist_covariance_matrix[0, 0],
                                                  self.twist_covariance_matrix[1, 1],
                                                  self.twist_covariance_matrix[2, 2]]

        # Rotate the quaternion
        orientation = self.latest_msg.pose.pose.orientation
        euler_angles = self.quaternion_to_euler(orientation)
        euler_angles[0] += math.pi  # Rotate by 180 degrees around the X-axis
        rotated_quaternion = self.euler_to_quaternion(euler_angles)

        vehicle_odometry_msg.q[0] = rotated_quaternion[0]
        vehicle_odometry_msg.q[1] = rotated_quaternion[1]
        vehicle_odometry_msg.q[2] = rotated_quaternion[2]
        vehicle_odometry_msg.q[3] = rotated_quaternion[3]

        # Publish the vehicle odometry message
        self.odometry_publisher_.publish(vehicle_odometry_msg)
        

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return [roll, pitch, yaw]

    def euler_to_quaternion(self, euler_angles):
        # Convert Euler angles (roll, pitch, yaw) to quaternion
        roll = euler_angles[0]
        pitch = euler_angles[1]
        yaw = euler_angles[2]

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return [x, y, z, w]
        
def main(args=None):
    rclpy.init(args=args)
    publisher = VisualInertialOdometryPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
