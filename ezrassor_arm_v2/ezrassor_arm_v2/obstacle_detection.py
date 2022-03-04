from ezrassor_arm_v2.arms_driver import QUEUE_SIZE
import rclpy
from sensor_msgs.msg import LaserScan
import numpy as np
from ezrassor_arm_v2.pointcloud_processor import PointCloudProcessor
import sys

class ObstacleDetector(PointCloudProcessor):

    def __init__(
        self,
        max_angle,
        max_obstacle_dist,
        min_hole_diameter,
        scan_time,
        range_min,
        range_max
    ):
        super(ObstacleDetector, self).__init__('obstacle_detection')

        self.max_slope = np.tan(max_angle * np.pi / 180.0)
        self.max_obstacle_dist = max_obstacle_dist
        self.min_hole_diameter = min_hole_diameter
        self.scan_time = scan_time
        self.range_min = range_min
        self.range_max = range_max

        self.topic_prefix = 'obstacle_detection/'

        self.hike_pub = self.create_publisher(
            LaserScan, self.topic_prefix + 'hike', QUEUE_SIZE
        )

        self.slope_pub = self.create_publisher(
            LaserScan, self.topic_prefix + 'slope', QUEUE_SIZE
        )

        self.combined_pub = self.create_publisher(
            LaserScan, self.topic_prefix + 'combined', QUEUE_SIZE
        )

        self.get_logger().info(
            'Obstacle Detection Initalized'
        )

    def init_pc_info(self, camera_info):

        super(ObstacleDetector, self).init_pc_info(camera_info)

        self.ranges_size = camera_info.width
        self.frame_id = camera_info.header.frame_id
        self.angle_increment = (
            (self.angle_max - self.angle_min) / (self.ranges_size - 1)
        )

    def create_laser_scan(self, ranges):

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = []
        return scan

    def on_pc_update(self, pc):
        super(ObstacleDetector, self).on_pc_update(pc)
        self.point_cloud_to_laser_scan()
    
    def point_cloud_to_laser_scan(self):

        hike_ranges = [float('nan')] * self.ranges_size
        slope_ranges = [float('nan')] * self.ranges_size
        min_ranges = [float('nan')] * self.ranges_size

        pc = self.get_points()

        if pc is not None:

            forward = pc[:, PointCloudProcessor.XYZ['FORWARD']]
            right = pc[:, PointCloudProcessor.XYZ['RIGHT']]
            down = pc[:, PointCloudProcessor.XYZ['DOWN']]
            steps, dists = self.to_laser_scan_data(forward, right)

            directions = np.column_stack((steps, dists, down))

            directions = directions[directions[:, 0].argsort()]

            directions = np.split(
                directions,
                np.unique(directions[:, 0], return_index=True)[1][1:],
                axis=0
            )

            for direction in directions:

                direction = direction[direction[:, 1].argsort()]

                step = int(direction[0,0])

                down1 = direction[:-1, 2]
                down2 = direction[1:, 2]
                dist1 = direction[:-1, 1]
                dist2 = direction[1:, 1]

                drop = np.subtract(down2, down1)
                hike = np.subtract(dist2, dist1)
                slope = np.abs(np.divide(drop, hike))

                cond_hike = hike > self.min_hole_diameter
                cond_slope = slope > self.max_slope
                index_hike = cond_hike.argmax() if cond_hike.any() else None
                index_slope = cond_slope.argmax() if cond_slope.any() else None

                if (
                    index_hike is not None
                    and direction[index_hike, 1] <= self.max_obstacle_dist
                ):
                    hike_ranges[step] = direction[index_hike, 1]
                
                if index_slope is not None:
                    slope_ranges[step] = direction[index_slope, 1]

                min_ranges[step] = np.nanmin(
                    (hike_ranges[step], slope_ranges[step])
                )
            
            self.hike_pub.publish(self.create_laser_scan(hike_ranges))
            self.slope_pub.publish(self.create_laser_scan(slope_ranges))
            self.combined_pub.publish(self.create_laser_scan(min_ranges))
        
    def to_laser_scan_data(self, forward, right):

        angles = np.negative(np.arctan2(right, forward))

        steps = np.divide(
            np.subtract(angles, self.angle_min), self.angle_increment
        ).astype(int)

        dists = np.linalg.norm(forward, right)

        return steps, dists


def main(args=None):
    try:
        rclpy.init(args=args)
        
        max_angle = float(sys.argv[1])
        max_obstacle_dist = float(sys.argv[2])
        min_hole_diameter = float(sys.argv[3])
        scan_time = 1.0 / 30,
        range_min=0.105,
        range_max=10.0,

        detector = ObstacleDetector(
            max_angle,
            max_obstacle_dist,
            min_hole_diameter,
            scan_time,
            range_min,
            range_max
        )

        rclpy.spin(detector)
        
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
