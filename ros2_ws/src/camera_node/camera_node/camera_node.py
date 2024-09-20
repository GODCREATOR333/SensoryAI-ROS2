#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaBlackhole, MediaPlayer
import asyncio
import json
import cv2 
import numpy as np
from aiortc import RTCIceCandidate 


class WebRTCServer(Node):
    def __init__(self):
        super().__init__('webrtc_server')
        self.peer_connection = None
        self.data_channel = None

    async def handle_offer(self, offer):
        self.peer_connection = RTCPeerConnection()
        self.peer_connection.on('datachannel', self.on_datachannel)

        @self.peer_connection.on('track')
        async def on_track(track):
            if track.kind == 'video':
                self.get_logger().info('Received video track')
                while True:
                    frame = await track.recv()
                    self.process_frame(frame)

        await self.peer_connection.setRemoteDescription(RTCSessionDescription(sdp=offer, type='offer'))
        answer = await self.peer_connection.createAnswer()
        await self.peer_connection.setLocalDescription(answer)

        return {'sdp': self.peer_connection.localDescription.sdp, 'type': self.peer_connection.localDescription.type}

    async def add_ice_candidate(self, candidate):
        if self.peer_connection:
            try:
                self.get_logger().info(f'Received ICE candidate: {json.dumps(candidate)}')
                ice_candidate = RTCIceCandidate(
                    candidate=candidate['candidate'],
                    sdpMid=candidate['sdpMid'],
                    sdpMLineIndex=candidate['sdpMLineIndex']
                )
                await self.peer_connection.addIceCandidate(ice_candidate)
                self.get_logger().info('ICE candidate added successfully')
            except Exception as e:
                self.get_logger().error(f'Error adding ICE candidate: {str(e)}')
        else:
            self.get_logger().warning('Peer connection not established; unable to add ICE candidate')
    

    def on_datachannel(self, channel):
        self.data_channel = channel
        self.data_channel.on('message', self.on_message)

    def on_message(self, message):
        self.get_logger().info(f'Received message: {message}')
        # Handle incoming messages (e.g., commands)

    def process_frame(self, frame):
        # Convert frame to numpy array
        frame_data = frame.to_ndarray()

        # Log the shape and type of the incoming data to the terminal
        self.get_logger().info(f'Incoming frame shape: {frame_data.shape}, dtype: {frame_data.dtype}')
        print(f'Incoming frame shape: {frame_data.shape}, dtype: {frame_data.dtype}')
        print(f'First 10 bytes of frame data: {frame_data.flatten()[:10]}')

        # Check if the frame is grayscale (2D array)
        if len(frame_data.shape) == 2:  # Grayscale image
            img = cv2.cvtColor(frame_data, cv2.COLOR_GRAY2BGR)  # Convert to BGR
        else:
            img = frame_data  # If it's already in the correct format

        # Display the image using OpenCV
        cv2.imshow('Received Frame', img)
        self.get_logger().info(f'Processing frame of shape: {img.shape}')

        # Wait for a short period to make the window responsive
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to close
            cv2.destroyAllWindows()
            return  # Exit the processing if window is closed

        # Here you would run your YOLO and SLAM algorithms
        # ...

        # Example control data (you would calculate this based on your algorithms)
        if self.data_channel and self.data_channel.readyState == 'open':
            control_data = json.dumps({'x': 0.5, 'y': 0.3, 'z': 0.1})
            self.data_channel.send(control_data)



class SignalingServer(Node):
    def __init__(self):
        super().__init__('signaling_server')
        self.webrtc_server = WebRTCServer()

    async def handle_websocket(self, websocket, path):
        self.get_logger().info("New client connected")
        try:
            async for message in websocket:
                data = json.loads(message)
                if data['type'] == 'offer':
                    self.get_logger().info("Received offer")
                    answer = await self.webrtc_server.handle_offer(data['sdp'])
                    await websocket.send(json.dumps({
                        'type': 'answer',
                        'sdp': answer['sdp']
                    }))
                elif data['type'] == 'ice-candidate':
                    self.get_logger().info("Received ICE candidate")
                    await self.webrtc_server.add_ice_candidate(data['candidate'])
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("Client disconnected")

async def main():
    rclpy.init()
    signaling_server = SignalingServer()

    server = await websockets.serve(
        signaling_server.handle_websocket, 
        "0.0.0.0",
        8080,
        ping_interval=None,
        ping_timeout=None
    )

    signaling_server.get_logger().info("Signaling server started")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())