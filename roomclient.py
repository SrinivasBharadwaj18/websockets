import asyncio
import websockets

async def connect_to_room(room_name):
    async with websockets.connect(f"ws://192.168.1.123:5050/{room_name}") as websocket:
            async for _ in websocket:
                 print("receing stream")

# Specify the room name
async def main():
    room_name = "srinivas"
    await connect_to_room(room_name)

asyncio.run(main())