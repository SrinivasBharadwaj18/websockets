import websockets
import asyncio
import numpy as np
import base64
import cv2
from websockets import exceptions

PORT = 5050
ADDR = "192.168.1.123"
url = f"ws://{ADDR}:{PORT}"

async def receive_stream(message):
    image = base64.b64decode(message)
    image_data = np.frombuffer(image, dtype=np.uint8)
    cv_image = cv2.imdecode(image_data, 1)  # Decode image data to OpenCV format
  # Decode image data to OpenCV format
    if cv_image is not None:
        print("Image present")
        height, width, channels = cv_image.shape
        print("Image dimensions (height, width, channels):", height, width, channels)
        cv2.imshow("subscriber", cv_image)
        cv2.waitKey(1)
    else:
        print("Failed to decode image data")
        await asyncio.sleep(1)


async def main():
    async with websockets.connect(url) as websocket:
        ask = await websocket.recv()
        print(ask)
        await websocket.send("viewer")
        print("Client : viewer")
        greeting = await websocket.recv()
        print(greeting)
        try:
            while True:
                try:
                    message = await websocket.recv()
                    await receive_stream(message)
                except exceptions.PayloadTooBig as e:
                    print(f"connection closed with error {e}")

                except exceptions.InvalidHeaderValue as e:
                    print(f"could not publish invalid header :{e}")

                except exceptions.WebSocketException as e:
                    print(f"could not receive websocket exception : {e}")
                    await asyncio.sleep(2)
                    continue
        except:
            print("could not receive any frames")


if __name__ == "__main__":
    asyncio.run(main())      