import asyncio
import websockets
from websockets import exceptions


connected_clients = []

async def server(websocket, path):
    print("Client Connected")
    try:
        connected_clients.append(websocket)
        await asyncio.sleep(5)
        
        
        if len(connected_clients) == 2:
            # Send a message to each client asking for their role
            await asyncio.gather(*[client.send("Send your role ('streamer' or 'viewer'):") for client in connected_clients])
        
            # Collect roles from clients
            roles = {}
            for client in connected_clients:
                role = await client.recv()
                roles[role] = client
            
            # Assign roles
            streamer = roles.get('streamer')
            viewer = roles.get('viewer')
            await stream_controller(streamer = streamer,viewer = viewer)
            async for message in handle_streamer(streamer):
                await handle_viewer(viewer,message)
                
        
    except websockets.exceptions.ConnectionClosedError:
        print("Client Disconnected")
        connected_clients.remove(websocket)


async def stream_controller(**kwargs):
    for name , client in kwargs.items():
        print(f"{name} got connected")
        await client.send("hey")

        


async def handle_viewer(websocket,message):
    viewer = websocket
    try:
        await viewer.send(message)
    except exceptions.ConnectionClosedError as e:
        print(f"could not send the frames to the viewer: {e}")
    except exceptions.ConnectionClosedOK:
        print("connection getting closed")
        await asyncio.sleep(1)


async def handle_streamer(websocket):
    try:
        streamer = websocket
        await streamer.send("start stream")
        try:
            async for message in streamer:
                print("receing stream from the streamer")
                yield message
        except exceptions.ProtocolError as p:
            print(f"protocol error {p}")

        except exceptions.PayloadTooBig as f:
            print(f" Payload too big: {f}")
        except exceptions.WebSocketException as e:
            print(e)

    except:
        print("streamer not connected")
        await websocket.close()



async def main():
    server_address = "192.168.1.123"
    server_port = 5050
    async with websockets.serve(server, server_address, server_port):
        print(f"Server started at ws://{server_address}:{server_port}")
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())
