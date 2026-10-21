from commlink import Subscriber
import time

POSE_TOPIC = "zed/pose"
IMAGE_TOPIC = "zed/image"
DEPTH_TOPIC = "zed/depth"
QUAT_XYZ_TOPIC = "zed/quat_xyz"
ROI_MASK_TOPIC = "zed/roi_mask"
ZED_PUB_PORT = 6000

CONTROLLER_RPC_HOST = "0.0.0.0"
CONTROLLER_RPC_PORT = 7755

BASE_RPC_HOST = "192.168.1.10"
BASE_RPC_PORT = 5557


class ZedSub:
    def __init__(self, host: str = "127.0.0.1", port: int = ZED_PUB_PORT, pose_only: bool = False):
        # pose_only skips the image/depth topics, so a pose consumer (e.g. the Pi
        # over Wi-Fi) isn't sent full frames it never reads.
        topics = [POSE_TOPIC, QUAT_XYZ_TOPIC] if pose_only else [POSE_TOPIC, IMAGE_TOPIC, DEPTH_TOPIC, QUAT_XYZ_TOPIC]
        self._sub = Subscriber(
            host=host,
            port=port,
            topics=topics,
            buffer=False,
        )

    def stop(self):
        self._sub.stop()
    
    def ready(self):
        return True if self._sub[POSE_TOPIC] is not None else False
    
    def get_pose(self):
        start_time = time.time()
        pose_msg = self._sub[POSE_TOPIC]
        print(f"zed pose retrieval cost {1.0/(time.time()-start_time):.1f} Hz", end='\n')
        
        return pose_msg["base_pose_6DOF"], pose_msg["timestamp"]
    
    def get_camera_pose(self):
        start_time = time.time()
        pose_msg = self._sub[POSE_TOPIC]
        print(f"zed camera pose retrieval cost {1.0/(time.time()-start_time):.1f} Hz", end='\n')
        
        return pose_msg["camera_pose"], pose_msg["timestamp"]
    
    def get_quat_pose(self):
        """(base_wxyz_xyz, cam_wxyz_xyz, zed_timestamp), or (None, None, None) if no pose yet."""
        quat_xyz_pose_msg = self._sub[QUAT_XYZ_TOPIC]
        if quat_xyz_pose_msg is None:
            return None, None, None
        base_pose_wxyz_xyz = [float(x) for x in quat_xyz_pose_msg[0:7]]
        cam_pose_wxyz_xyz = [float(x) for x in quat_xyz_pose_msg[7:14]]
        timestamp = float(quat_xyz_pose_msg[14])
        return base_pose_wxyz_xyz, cam_pose_wxyz_xyz, timestamp
    
    def get_image(self):
        img_msg = self._sub[IMAGE_TOPIC]
        
        return img_msg["image"], img_msg["timestamp"]
    
    def get_depth(self):
        depth_msg = self._sub[DEPTH_TOPIC]
        
        return depth_msg["depth"], depth_msg["timestamp"]

