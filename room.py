import asyncio
import websockets


rooms = {}

clients = set()


async def broadcast(message):
    # Send the message to all connected clients in the room
    clients_copy = clients.copy()
    while True:
            try:
                for websocket in clients_copy:
                    print(f"brodacast: {websocket}")
                    await websocket.send(message)
                    await asyncio.sleep(0.2)
                break
            except RuntimeError:
                await asyncio.sleep(1)

async def sender(room_name,**kwargs):

    try:
        while clients:  # Continue as long as there are clients
            for websocket in clients.copy():  # Iterate over a copy to avoid RuntimeError
                try:
                    if 'wait' in kwargs:
                        await asyncio.sleep(1)
                        async for message in websocket:
                            print("receiving stream")
                            await broadcast(message)

                    else:
                        async for message in websocket:
                            print("receiving stream")
                            await broadcast(message)

                except websockets.exceptions.ConnectionClosedError:
                    await on_disconnect(websocket, room_name)

    except asyncio.CancelledError:
        print("Sender coroutine cancelled")

    finally:
        print("sender stopped")



async def on_disconnect(websocket, name):
    clients.remove(websocket)
    print(f"{websocket.remote_address} disconnected from room: {name}")


async def handler(websocket, path):

    room_name = path.strip("/")
    chat_room = rooms.get(room_name)
    if chat_room is None:
        print(f"Creating new room: {room_name}")
        rooms[room_name] = websocket

    global clients
    print("client connected")
    clients.add(websocket)
    print(f"{websocket.remote_address} connected to room:{room_name}")
    print(len(clients))
    

    await sender(room_name=room_name,wait = 1)


async def main():
    server_address = "192.168.1.123"
    server_port = 5050
    async with websockets.serve(handler, server_address, server_port):
        print(f"Server started at {server_address}:{server_port}")
        await asyncio.Future()  # Keep the main coroutine running

if __name__ == "__main__":
    asyncio.run(main(),debug=True)