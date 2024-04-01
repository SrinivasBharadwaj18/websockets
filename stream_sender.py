import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import asyncio
import websockets
import base64
from websockets import exceptions

class RosWebsocketBridge(Node):
    def __init__(self, websocket, url):
        super().__init__('ros_websocket_bridge')
        self.url = url
        self.websocket = websocket
        self.subscription = self.create_subscription(Image, 'raw_image', self.start_stream, 10)
        self.bridge = CvBridge()

    async def start_stream(self, message):
        try:
            if message is not None:
                cv_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
                _, image_data = cv2.imencode('.jpg', cv_image)
                payload = base64.b64encode(image_data)
                if payload is not None:
                    print("have the image")
                    try:
                        # await asyncio.sleep(1)
                        await self.websocket.send(payload)
                        print("publishing video frames")
                    except exceptions.ConnectionClosedError as e:

                        print("unable to send the stream for unknown reasons",e)
                    except:
                        await asyncio.sleep(1)

            else: print("no message from the publisher")

        except exceptions.ProtocolError as p:
            print(f"protocol error: {p}")

        except exceptions.PayloadTooBig as e:
            print(f"connection closed with error {e}")

        except exceptions.InvalidHeaderValue as e:
            print(f"could not publish :{e}")
        
        except websockets.exceptions.ConnectionClosedError:
            print("the connection is closed")

        except exceptions.WebSocketException as e:
            print(f"could not publish : {e}")
        except:
            print(f"not getting the message from the publisher")




async def main(args=None):
    PORT = 5050
    ADDR = "192.168.1.123"
    url = f"ws://{ADDR}:{PORT}"
    try:
        async with websockets.connect(url) as websocket:
            try:
                ask = await websocket.recv()
                print(ask)
                await websocket.send("streamer")
                greeting = await websocket.recv()
                print(greeting)
                message = await websocket.recv()
                print(message)
                if message == "start stream":
                    rclpy.init(args=args)
                    node = RosWebsocketBridge(websocket, url)
                    try:
                        rclpy.spin(node)
                    except KeyboardInterrupt:
                        pass
                    node.destroy_node()
                    rclpy.shutdown()
            except Exception as e:
                print(str(e))
                await asyncio.sleep(2)
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    asyncio.run(main())
