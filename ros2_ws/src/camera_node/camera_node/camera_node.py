#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription
import asyncio
import json
from aiortc import RTCIceCandidate
import numpy as np
import cv2 
import os

class WebRTCServer(Node):
    def __init__(self):
        super().__init__('webrtc_server')
        self.peer_connection = None
        self.data_channel = None
        self.video_writer = None
        self.output_file = 'output_video.avi'
        self.frame_width = None
        self.frame_height = None
        self.fps = 30  # Set the frames per second to a fixed value

    async def handle_offer(self, offer):
        self.get_logger().info('Handling WebRTC offer...')
        self.peer_connection = RTCPeerConnection()

        # Set up datachannel event
        self.peer_connection.on('datachannel', self.on_datachannel)

        # Set up track event for video/audio streams
        @self.peer_connection.on('track')
        async def on_track(track):
            if track.kind == 'video':
                self.get_logger().info('Received video track, starting frame processing loop...')
                try:
                    while True:
                        frame = await track.recv()
                        if frame is None:
                            break  # Exit the loop if there are no more frames
                        await self.process_frame(frame)
                except Exception as e:
                    self.get_logger().error(f'Error receiving video track: {str(e)}')

        await self.peer_connection.setRemoteDescription(RTCSessionDescription(sdp=offer, type='offer'))
        self.get_logger().info('Remote description set. Creating and setting local answer...')
        answer = await self.peer_connection.createAnswer()
        await self.peer_connection.setLocalDescription(answer)
        self.get_logger().info('Local description set.')

        # Add connection state change event listeners
        self.add_connection_state_listeners()

        return {'sdp': self.peer_connection.localDescription.sdp, 'type': self.peer_connection.localDescription.type}

    async def add_ice_candidate(self, candidate):
        if self.peer_connection:
            try:
                self.get_logger().info(f'Received ICE candidate: {json.dumps(candidate)}')

                parts = candidate['candidate'].split()
                if len(parts) >= 8:
                    foundation = parts[0].split(':')[1]
                    component = int(parts[1])
                    protocol = parts[2].lower()
                    priority = int(parts[3])
                    ip = parts[4]
                    port = int(parts[5])
                    type = parts[7]

                    ice_candidate = RTCIceCandidate(
                        foundation=foundation,
                        component=component,
                        protocol=protocol,
                        priority=priority,
                        ip=ip,
                        port=port,
                        type=type,
                        sdpMid=candidate['sdpMid'],
                        sdpMLineIndex=candidate['sdpMLineIndex']
                    )
                    await self.peer_connection.addIceCandidate(ice_candidate)
                    self.get_logger().info('ICE candidate added successfully')
                else:
                    self.get_logger().warning(f'Invalid ICE candidate format: {candidate["candidate"]}')
            except Exception as e:
                self.get_logger().error(f'Error adding ICE candidate: {str(e)}')
        else:
            self.get_logger().warning('Peer connection not established; unable to add ICE candidate')
    
    async def process_frame(self, frame):
        try:
            if self.video_writer is None:
                # Initialize video writer once we receive the first frame
                self.frame_width = frame.width
                self.frame_height = frame.height
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writer = cv2.VideoWriter(self.output_file, fourcc, self.fps,
                                                    (self.frame_width, self.frame_height))
                self.get_logger().info(f'VideoWriter initialized: {self.output_file}')

            # Convert YUV to RGB
            img_rgb = frame.to_ndarray(format='rgb24')

            # Convert RGB to BGR for OpenCV (OpenCV uses BGR format)
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

            # Write the frame to disk
            self.video_writer.write(img_bgr)

            self.get_logger().info(f'Frame written to disk: {self.output_file}')
        
        except Exception as e:
            self.get_logger().error(f'Error processing video frame: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def close_video_writer(self):
        """Release the video writer to finalize the video file."""
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f'Video file written to: {os.path.abspath(self.output_file)}')

    def on_shutdown(self):
        self.close_video_writer()  # Ensure the video writer is closed when shutting down


    def on_datachannel(self, channel):
        self.get_logger().info('Data channel established')
        self.data_channel = channel
        self.data_channel.on('message', self.on_message)

    def on_message(self, message):
        self.get_logger().info(f'Received message: {message}')
        # Handle incoming messages (e.g., commands)

    def add_connection_state_listeners(self):
        @self.peer_connection.on('iceconnectionstatechange')
        async def on_iceconnectionstatechange():
            self.get_logger().info(f'ICE connection state changed: {self.peer_connection.iceConnectionState}')
            if self.peer_connection.iceConnectionState == 'disconnected':
                self.get_logger().warning("ICE connection disconnected")
            elif self.peer_connection.iceConnectionState == 'failed':
                self.get_logger().error("ICE connection failed")

        @self.peer_connection.on('connectionstatechange')
        async def on_connectionstatechange():
            self.get_logger().info(f'Connection state changed: {self.peer_connection.connectionState}')
            if self.peer_connection.connectionState == 'closed':
                self.get_logger().warning("Connection closed")
            elif self.peer_connection.connectionState == 'failed':
                self.get_logger().error("Connection failed")


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