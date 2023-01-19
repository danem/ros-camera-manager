#!/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import Parameter
import ros_camera_manager_msgs.msg as cmm
import ros_camera_manager_msgs.srv as cms

import database as db


class CameraServer (Node):
    def __init__ (self):
        super(CameraServer, self).__init__("camera_manager")
        #db_path = self.declare_parameter("database_path", "")
        #self._db = db.Database(db_path= db_path.get_parameter_value().str_value)
        self._registeredCameras = []

        self.create_service(cms.QueryCamera, '~/query_camera', self._queryCamera)
        self.create_service(cms.GetStatus, '~/get_status', self._getStatus)
        self.create_service(cms.RegisterCamera, '~/register_camera', self._registerCamera)
        self.get_logger().info("Camera Manager Started")
    
    def _queryCamera (self, request, response):
        self.get_logger().info("Querying Cameras")
        if request.query_type == cms.QueryCamera.Request.NAME_QUERY:
            for cam in self._registeredCameras:
                if request.name in cam.names:
                    response.cameras.append(cam)

        elif request.query_type == cms.QueryCamera.Request.DRIVER_QUERY:
            for cam in self._registeredCameras:
                if request.name == cam.driver_type:
                    response.cameras.append(cam)

        elif request.query_type == cms.QueryCamera.Request.ID_QUERY:
            for cam in self._registeredCameras:
                if request.name == cam.id:
                    response.cameras.append(cam)

        return response

    def _getStatus (self, request, response):
        self.get_logger().info("Getting status")
        response.cameras = self._registeredCameras
        return response

    def _registerCamera (self, request, resposne):
        self.get_logger().info("Registering camera")
        self._registeredCameras.append(request.camera)
        resposne.result = True
        return resposne


def main(args=None):
    rclpy.init(args=args)

    node = CameraServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()