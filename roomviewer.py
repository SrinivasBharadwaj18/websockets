import asyncio
import websockets
import cv2
import base64
import numpy as np

async def connect_to_room(room_name):
    async with websockets.connect(f"ws://192.168.1.123:5050/{room_name}") as websocket:

            greet = await websocket.recv()
            print(greet)

            async for message in websocket:
                image = base64.b64decode(message)
                image_data = np.frombuffer(image, dtype=np.uint8)
                cv_image = cv2.imdecode(image_data, 1) 
                if cv_image is not None:
                    print("Image present")
                    height, width, channels = cv_image.shape
                    print("Image dimensions (height, width, channels):", height, width, channels)
                    cv2.imshow("subscriber", cv_image)
                    cv2.waitKey(1)
                else:
                    print("Failed to decode image data")
                    await asyncio.sleep(1)

# Specify the room name
async def main():
    room_name = "srinivas"
    await connect_to_room(room_name)

asyncio.run(main())