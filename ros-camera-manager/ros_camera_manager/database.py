import yaml
from typing import List
import ros_camera_manager_msgs.msg as cmm

class Database (object):
    def __init__(self, db_path: str) -> None:
        pass

    def queryCamerasByName (name: str) -> List[cmm.CameraInfo]:
        pass

    def queryCamerasByDriverType (tipe: str) -> List[cmm.CameraInfo]:
        pass


