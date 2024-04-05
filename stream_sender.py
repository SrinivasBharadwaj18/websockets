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
            await asyncio.sleep(0.5)
        except websockets.exceptions.ConnectionClosedError:
            print("connection closed due to unexpected termination of the connection")
            # await capture_and_publish(websocket)
            break
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
    name = "srinivas"
    url = f"ws://{ADDR}:{PORT}/{name}"
    try:
        while True:
            async with websockets.connect(url) as websocket:
                    try:
                        
                        await capture_and_publish(websocket)
                    except:
                        await asyncio.sleep(0.3)
    except:
        print("unable to connect to the server")
# Run the main function
asyncio.run(main())
