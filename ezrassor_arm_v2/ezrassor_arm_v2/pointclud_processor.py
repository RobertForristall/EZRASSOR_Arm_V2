from email.mime import image
import rclpy
from sensor_msgs.msg import (PointCloud2, CameraInfo)
import numpy as np
import image_geometry

class PointCloudProcessor(Node):

    XYZ = {'RIGHT': 0, 'DOWN': 1, 'FORWARD': 2}

    def __init__(self, node_name):

        super().__init__(node_name)
        self.point_cloud = None
        self.angle_max = None
        self.angle_min = None

        self.cam_info_sub = self.create_subscription (
            CameraInfo,
            'depth/camera_info',
            self.init_pc_info,
            10
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            'depth/points',
            self.on_pc_update,
            10
        )

    def on_pc_update(self, data):
        self.point_cloud = data
    
    def get_points(self):
        if self.point_cloud is None:
            return None
        
        pc = np.frombuffer(self.point_cloud.data, np.float32)

        pc = np.reshape(pc, (-1, 8))[:, :3]

        pc = pc[~np.isnan(pc).any(axis=1)]

        return pc if pc.size > 0 else None
    
    @staticmethod
    def angle_between_rays(ray1, ray2):
        dp = np.dot(ray1, ray2)
        mag1 = np.linalg.norm(ray1)
        mag2 = np.linalg.norm(ray2)
        return np.arccos(dp / (mag1 * mag2))
    
    def init_pc_info(self, camera_info):

        if (self.angle_max is None) or (self.angle_min is None):

            cam_model = image_geometry.PinholeCameraModel()
            cam_model.fromCameraInfo(camera_info)
            width = camera_info.width

            raw_pixel_left = (0, cam_model.cy())
            rect_pixel_left = cam_model.rectifyPoint(raw_pixel_left)
            left_ray = cam_model.projectPixelTo3dRay(rect_pixel_left)

            raw_pixel_right = (width - 1, cam_model.cy())
            rect_pixel_right = cam_model.rectifyPoint(raw_pixel_right)
            right_ray = cam_model.projectPixelTo3dRay(rect_pixel_right)

            raw_pixel_center = (cam_model.cx(), cam_model.cy())
            rect_pixel_center = cam_model.rectifyPoint(raw_pixel_center)
            center_ray = cam_model.projectPixelTo3dRay(rect_pixel_center)

            self.angle_max = PointCloudProcessor.angle_between_rays(
                left_ray, center_ray
            )

            self.angle_min = PointCloudProcessor.angle_between_rays(
                center_ray, right_ray
            )
