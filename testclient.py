import cv2
import asyncio
import websockets
import base64

async def capture_and_publish(websocket):
    # Open the webcam
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Convert frame to base64 string
        _, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer)

        # Send frame as base64 string over WebSocket
        try:
            await websocket.send(jpg_as_text)
            print(f"publishing video frames: {type(jpg_as_text)}")
            await asyncio.sleep(0.1)
        except websockets.exceptions.ConnectionClosedError:
            print("connection closed due to unexpected termination of the connection")
        except websockets.exceptions.InvalidStatusCode:
            print("invalid status code sent by the sever")
        except websockets.exceptions.ConnectionClosed:
            print("connection closed. Stopping the stream")
            await asyncio.sleep(1)
            await websocket.send(jpg_as_text)
            break
        except:
            await asyncio.sleep(1)


async def main():
    PORT = 5050
    ADDR = "192.168.1.123"
    url = f"ws://{ADDR}:{PORT}"
    try:
        async with websockets.connect(url) as websocket:
            ask = await websocket.recv()
            print(ask)
            await websocket.send("streamer")
            greeting = await websocket.recv()
            print(greeting)
            message = await websocket.recv()
            print(message)
            if message == "start stream":
                try:
                    await capture_and_publish(websocket)
                except Exception as e:
                    print(e)
    except:
        print("unable to connect to the server")
# Run the main function
asyncio.run(main())











# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import asyncio
# import websockets
# import base64
# from websockets import exceptions
# from datetime import timedelta

# class RosWebsocketBridge(Node):
#     def __init__(self, websocket, url):
#         super().__init__('ros_websocket_bridge')
#         self.url = url
#         self.websocket = websocket
#         self.subscription = self.create_subscription(Image, 'raw_image', self.start_stream, 10)
#         self.bridge = CvBridge()

#     async def start_stream(self, message):
#         while True:
#             try:
#                 if message is not None:
#                     cv_image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
#                     _, image_data = cv2.imencode('.jpg', cv_image)
#                     payload = base64.b64encode(image_data)
#                     if payload is not None:
#                         print("have the image")
#                         try:
#                             # await asyncio.sleep(1)
#                             await self.websocket.send(payload)
#                             print("publishing video frames")
#                         except exceptions.ConnectionClosedError as e:

#                             print("unable to send the stream for unknown reasons",e)
#                         except:
#                             await asyncio.sleep(1)
#                     else:
#                         await asyncio.sleep(1)
#                         continue


#                 else: print("no message from the publisher")

#             except exceptions.ProtocolError as p:
#                 print(f"protocol error: {p}")

#             except exceptions.PayloadTooBig as e:
#                 print(f"connection closed with error {e}")

#             except exceptions.InvalidHeaderValue as e:
#                 print(f"could not publish :{e}")

#             except exceptions.WebSocketException as e:
#                 print(f"could not publish : {e}")




# async def main(args=None):
#     PORT = 5050
#     ADDR = "192.168.1.123"
#     url = f"ws://{ADDR}:{PORT}"
#     try:
#         async with websockets.connect(url) as websocket:
#             try:
#                 ask = await websocket.recv()
#                 print(ask)
#                 await websocket.send("streamer")
#                 greeting = await websocket.recv()
#                 print(greeting)
#                 message = await websocket.recv()
#                 print(message)
#                 if message == "start stream":
#                     rclpy.init(args=args)
#                     node = RosWebsocketBridge(websocket, url)
#                     try:
#                         rclpy.spin(node)
#                     except KeyboardInterrupt:
#                         pass
#                     node.destroy_node()
#                     rclpy.shutdown()
#             except Exception as e:
#                 print(str(e))
#                 await asyncio.sleep(2)
#     except Exception as e:
#         print(f"An error occurred: {e}")

# if __name__ == '__main__':
#     asyncio.run(main())
