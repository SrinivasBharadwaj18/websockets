Below is the description for the files.

room.py >>>> This file is responsible to act as a websocket server which supports rooms, where the clients can join a room using the uri with an addition of room_name.
roomviewer.py  >>> This file is responsible to act as a websocket client which connects to the server and joins a room and receives the stream and shows it.
roomclient.py  >>> This file is responsible to act as a websocket client which connects to the server to receive the stream by joining a room using uri but doesnt do anything with the stream, simply prints receiving stream
streamsender.py  >>> This file is responsible to act as a websocket client which connects to the server joins a room using the uri and sends the stream to the server.
