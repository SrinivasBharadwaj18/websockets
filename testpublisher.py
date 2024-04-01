import asyncio
import websockets
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
from rclpy.node import Node
import rclpy


class RosWebsocketBridge(Node):
    def __init__(self, websocket):
        super().__init__('ros_websocket_bridge')
        self.websocket = websocket
        self.subscription = self.create_subscription(
            Image, 'raw_image', self.image_callback, 30)
        self.bridge = CvBridge()

    async def image_callback(self, msg):
        print("!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            _, image_data = cv2.imencode('.jpg', cv_image)
            payload = base64.b64encode(image_data)
            await self.websocket.send(payload)
            print("sending video frames")
        except Exception as e:
            print(f"Error processing image: {e}")
            await asyncio.sleep(3)

async def connect_to_websocket(websocket_url):
    try:
        async with websockets.connect(websocket_url) as websocket:
            try:
                ask = await websocket.recv()
                print(ask)
                await websocket.send("streamer")
                greeting = await websocket.recv()
                print(greeting)
                message = await websocket.recv()
                print(message)
                if message == "start stream":
                    rclpy.init(args=None)
                    node = RosWebsocketBridge(websocket)
                    rclpy.spin(node)
                    node.destroy_node()
                    rclpy.shutdown()
                    print("stopped spinning the node")
            except KeyboardInterrupt:
                print("blah")
            
    except Exception as e:
        print(f"WebSocket connection error: {e}")

async def main(websocket_url):
    while True:
        await connect_to_websocket(websocket_url)
        print("Reconnecting to WebSocket server...")
        await asyncio.sleep(5)  # Reconnect after 5 seconds if connection is lost

if __name__ == '__main__':
    # Modify the WebSocket server URL accordingly
    websocket_url = "ws://192.168.1.123:5050"
    asyncio.run(main(websocket_url))
