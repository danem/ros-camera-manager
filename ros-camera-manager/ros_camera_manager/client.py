import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import Parameter
import ros_camera_manager_msgs.msg as cmm
import ros_camera_manager_msgs.srv as cms
from typing import List, Awaitable


class Client (Node):
    def __init__(self, topic_name = None) -> None:
        super(Client, self).__init__("camera_client")

        if not topic_name:
            topic_name = "/camera_manager"

        self._getStatus = self.create_client(cms.GetStatus, topic_name + "/get_status")
        self._queryCamera = self.create_client(cms.QueryCamera, topic_name + "/query_camera")
        self._registerCamera = self.create_client(cms.RegisterCamera, topic_name + "/register_camera")

        res = self._wait_for_services([
            self._getStatus,
            self._queryCamera,
            self._registerCamera,
        ])

        if not res:
            raise Exception("Unable to connect to services")
    
    def _wait_for_services (self, srvs, timeout = 1.0):
        for s in srvs:
            if not s.wait_for_service(timeout_sec = timeout):
                self.get_logger().info('Service not available')
                return False
        return True
    
    def _call_sync (self, fn, *args):
        future = fn(*args)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def statusAsync (self) -> Awaitable[List[cmm.CameraInfo]]:
        req = cms.GetStatus.Request()
        future = self._getStatus.call_async(req)
        return future

    def status (self) -> List[cmm.CameraInfo]:
        return self._call_sync(self.statusAsync)
    
    def queryAsync (self, name: str, type: int) -> Awaitable[List[cmm.CameraInfo]]:
        req = cms.QueryCamera.Request()
        req.query_type = type
        req.query = name
        future = self._queryCamera.call_async(req)
        return future
    
    def querySync (self, name: str, type: int) -> List[cmm.CameraInfo]:
        return self._call_sync(self.queryAsync, name, type)

    def queryByNameAsync (self, name: str) -> Awaitable[List[cmm.CameraInfo]]:
        return self.queryAsync(name, cms.QueryCamera.Request.NAME_QUERY)

    def queryByName (self, name: str) -> List[cmm.CameraInfo]:
        return self.querySync(name, cms.QueryCamera.Request.NAME_QUERY)

    def queryByDriverAsync (self, name: str) -> Awaitable[List[cmm.CameraInfo]]:
        return self.queryAsync(name, cms.QueryCamera.Request.DRIVER_QUERY)

    def queryByDriver (self, name: str) -> List[cmm.CameraInfo]:
        return self.querySync(name, cms.QueryCamera.Request.DRIVER_QUERY)
    
    def queryById (self, id: str) -> List[cmm.CameraInfo]:
        return self.querySync(id, cms.QueryCamera.Request.ID_QUERY)
    
    def queryByIdAsync (self, id: str) -> Awaitable[List[cmm.CameraInfo]]:
        return self.queryAsync(id, cms.QueryCamera.Request.ID_QUERY)

    def registerCameraRaw (self, info: cmm.CameraInfo):
        req = cms.RegisterCamera.Request()
        req.camera = info
        future = self._registerCamera.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def _stream (topic, info_topic = None):
    if not info_topic:
        info_topic = topic + "/camera_info"
    res = cmm.CameraStream()
    res.topic = topic
    res.info_topic = info_topic
    return res

def DepthStream (topic, info_topic = None):
    res = _stream(topic, info_topic)
    res.stream_type = cmm.CameraStream.DEPTH
    return res

def ColorStream (topic, info_topic = None):
    res = _stream(topic, info_topic)
    res.stream_type = cmm.CameraStream.COLOR
    return res

def RealsenseInfo (id, topic):
    cam = cmm.CameraInfo()
    color = ColorStream(topic + "/color")
    depth = DepthStream(topic + "/depth")
    cam.streams.append(color)
    cam.streams.append(depth)
    cam.driver_type = "REALSENSE"
    cam.id = id

    return cam

