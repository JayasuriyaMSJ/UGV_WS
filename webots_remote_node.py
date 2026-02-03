import json
import time
from ws4py.client.threadedclient import WebSocketClient

class RosBridgeRemote(WebSocketClient):
    def __init__(self, url, protocols=None, extensions=None, heartbeat_freq=None, ssl_options=None, headers=None, exclude_headers=None):
        super().__init__(url, protocols, extensions, heartbeat_freq, ssl_options, headers, exclude_headers)
    
    def cmd_vel_publisher(self, id, topic, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        publish_msg = {
            "op": "publish",
            "id": id,
            "topic": topic,
            "type": "geometry_msgs/msg/Twist",
            "msg": {
                "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
                "angular": {"x": angular_x, "y": angular_y, "z": angular_z}
            }
        }
        self.send(json.dumps(publish_msg))
        print("Published cmd_vel")



if __name__ == "__main__":
    ws = RosBridgeRemote('ws://172.20.0.20:9090')