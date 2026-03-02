#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class DisparityExtender(Node):
    def __init__(self):
        super().__init__('gap_finder')

        self.max_speed = 12.0 
        self.max_steering = 0.40  
        self.car_width = 0.50     

        self.prev_steering = 0.0

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def get_disparities(self, ranges, angle_increment):
        proc_ranges = ranges.copy()
        for i in range(1, len(ranges)):
            if abs(ranges[i] - ranges[i-1]) > 0.5:
                closer_idx = i if ranges[i] < ranges[i-1] else i-1
                further_idx = i-1 if ranges[i] < ranges[i-1] else i
                
                dist = ranges[closer_idx]
                if dist <= 0: continue
                
                angle = np.arcsin(min(1.0, self.car_width / dist))
                idx_span = int(angle / angle_increment)
                
                if closer_idx == i:
                    start = max(0, further_idx - idx_span)
                    proc_ranges[start:further_idx+1] = dist
                else:
                    end = min(len(ranges)-1, further_idx + idx_span)
                    proc_ranges[further_idx:end+1] = dist
        return proc_ranges

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges)] = 0
        ranges[np.isinf(ranges)] = 0
        
        center = len(ranges) // 2
        fov = int(75 / (msg.angle_increment * 180 / np.pi)) 
        start, end = center - fov, center + fov
        forward_ranges = ranges[start:end].copy()

        safe_ranges = self.get_disparities(forward_ranges, msg.angle_increment)

        masked = np.where(safe_ranges > 1.2, 1, 0)
        slices = np.split(np.arange(len(masked)), np.where(np.diff(masked) != 0)[0] + 1)
        gaps = [s for s in slices if len(s) > 0 and masked[s[0]] == 1]
        
        if not gaps:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 1.0
            self.pub.publish(drive_msg)
            return
            
        largest_gap = max(gaps, key=len)
        best_idx = (largest_gap[0] + largest_gap[-1]) // 2
        
        steering = msg.angle_min + (best_idx + start) * msg.angle_increment
        steering = np.clip(steering, -self.max_steering, self.max_steering)
        self.prev_steering = 0.6 * self.prev_steering + 0.4 * steering 
        
        steer_ratio = abs(self.prev_steering) / self.max_steering
        path_dist = safe_ranges[best_idx]
        
        if np.min(safe_ranges) < 1.1:
            speed = 2.0
        elif path_dist > 8.0 and steer_ratio < 0.1:
            speed = self.max_speed
        elif path_dist < 4.5:
            speed = self.max_speed * (1.0 - (steer_ratio ** 1.2))
            speed = max(2.5, speed)
        else:
            speed = self.max_speed * (1.0 - (steer_ratio ** 1.8))
            speed = max(3.5, speed) 

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = self.prev_steering
        drive_msg.drive.speed = speed
        self.pub.publish(drive_msg)

def main():
    rclpy.init()
    node = DisparityExtender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()