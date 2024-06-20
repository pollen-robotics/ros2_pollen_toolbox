import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import numpy.typing as npt

from pollen_vision.camera_wrappers.depthai import SDKWrapper
from pollen_vision.camera_wrappers.depthai.utils import get_config_file_path

import FramesViewer.utils as fv_utils
import open3d as o3d
import struct


T_world_cam = fv_utils.make_pose([0.049597, 0.009989, 0.038089], [0, 0, 0])
T_world_cam[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
T_world_cam = fv_utils.rotateInSelf(T_world_cam, [-48, 0, 0])




from ctypes import * # convert float to uint32


from std_msgs.msg import Header




class PointCloudPublisher(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_publisher_node')
        self._logger = self.get_logger()

        T_world_cam = fv_utils.make_pose([0.052644, 0.01, 0.034150], [0, 0, 0])
        T_world_cam[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        self.T_world_cam = fv_utils.rotateInSelf(T_world_cam, [-48, 0, 0])

        self.publisher_ = self.create_publisher(PointCloud2, 'pointcloud', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pointcloud)
        self.cam = SDKWrapper(get_config_file_path("CONFIG_SR"), compute_depth=True)
        self.K = self.cam.get_K()
        self.pcd = o3d.geometry.PointCloud()

        self.get_pcl_ros_msg()




    def get_point_cloud_from_rgbd(
        self, rgb_image: npt.NDArray[np.uint8], depth_image: npt.NDArray[np.float32]
    ) -> o3d.geometry.PointCloud:
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rgb_image),
            o3d.geometry.Image(depth_image),
            convert_rgb_to_intensity=False,
        )
        height, width, _ = rgb_image.shape
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, self.K)

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
        return pcd

    def get_pcl_ros_msg(self) -> PointCloud2:
        data, _, _ = self.cam.get_data()
        rgb, depth = data['left'], data['depth']
        pcl = self.get_point_cloud_from_rgbd(rgb_image=rgb, depth_image=depth)
        points = np.asarray(pcl.points, dtype=np.float32)
        one_column = np.ones((points.shape[0], 1))
        points = np.column_stack((points, one_column))
        # points = points @ self.T_world_cam.T
        points = points[:, :3]
        colors = np.asarray(pcl.colors, dtype=np.float64)
        colors = colors * 255
        colors = colors.astype(np.uint8)
        # self._logger.info(f"Color of the first point: {colors[0]}")
        # self._logger.info(f"Point cloud has {np.asarray(pcl.points).shape[0]} points")
        # self._logger.info(f"Pcl colors shape: {colors.shape}")

        pc2_msg = PointCloud2()
        pc2_msg.header.stamp = self.get_clock().now().to_msg()
        # pc2_msg.header.frame_id = 'torso'
        pc2_msg.header.frame_id = 'sr_cam_l_optical'

        pc2_msg.height = 1
        pc2_msg.width = len(pcl.points)

        data = [
            struct.pack('fffBBBB', pt[0], pt[1], pt[2], cl[0], cl[1], cl[2],255) for (pt, cl) in zip(points, colors)
        ]
        # for i in range(len(pcl.points)):
        #     x, y, z = points[i]
        #     r, g, b = colors[i]
        #     data.append(struct.pack('fffBBB', x, y, z, r, g, b))

        # pc2_msg.fields = [
        #     PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        #     PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
        #     PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
        #     PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
        # ]


        pc2_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),

        ]


        pc2_msg.is_bigendian = False
        # pc2_msg.point_step = 12
        pc2_msg.point_step = len(pc2_msg.fields)*4
        pc2_msg.row_step = pc2_msg.point_step * points.shape[0]
        pc2_msg.is_dense = True
        pc2_msg.data = b''.join(data)
        # pc2_msg.data = np.asarray(points, np.float32).tobytes()

        # self._logger.info(f"Publishing pointcloud ros msg {pc2_msg}")
        return pc2_msg

    def publish_pointcloud(self):
        pc2_msg = self.get_pcl_ros_msg()

        self.publisher_.publish(pc2_msg)
        # self.get_logger().info(f'Publishing pointcloud with {pointcloud.width} points')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
