import React, { useEffect, useRef, useState } from 'react';
import { Button, Alert } from 'react-bootstrap';

const WebRTCClient = () => {
  const [isConnected, setIsConnected] = useState(false);
  const localVideoRef = useRef(null);
  const peerConnection = useRef(null);
  const dataChannel = useRef(null);
  const websocket = useRef(null);
  const SERVER_IP = '192.168.0.107'; // Replace with your PC's IP address
  const SERVER_PORT = 8080;

  useEffect(() => {
    console.log('Component rendered');
    
    // Set up WebSocket connection
    websocket.current = new WebSocket(`ws://${SERVER_IP}:${SERVER_PORT}`);
    
    websocket.current.onopen = () => {
      console.log('WebSocket connection established');
    };
    
    
    websocket.current.onmessage = async (event) => {
      const message = JSON.parse(event.data);
      console.log('Message received from server:', message);
      await handleServerMessage(message);
    };
    
    websocket.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };
    
    websocket.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    // Clean up when the component unmounts
    return () => {
      disconnect();
    };
  }, []);


  const startStream = async () => {
    try {
      console.log('Starting video stream...');
      const stream = await navigator.mediaDevices.getUserMedia({
        video: {
          frameRate: { ideal: 60, max: 60 },
          width: { ideal: 1280 },
          height: { ideal: 720 },
        }
      });
      
  
      if (localVideoRef.current) {
        localVideoRef.current.srcObject = stream;
        console.log('Video stream started successfully');
      }
  
      // Log video track details such as frame rate, resolution, etc.
      const videoTrack = stream.getVideoTracks()[0];
      const settings = videoTrack.getSettings();
      console.log(`Video Track Settings:`);
      console.log(`- Frame Rate: ${settings.frameRate} FPS`);
      console.log(`- Width: ${settings.width}px`);
      console.log(`- Height: ${settings.height}px`);
      console.log(`- Aspect Ratio: ${settings.aspectRatio}`);
      
      // You can add more logs for other video-related settings
      return stream;
    } catch (error) {
      console.error('Error accessing camera:', error);
    }
  };
  
  const createPeerConnection = () => {
    console.log('Creating peer connection...');
    const pc = new RTCPeerConnection({
      iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
    });
  
    pc.onicecandidate = (event) => {
      if (event.candidate) {
        console.log('ICE candidate generated:', event.candidate);
        sendToServer({
          type: 'ice-candidate',
          candidate: {
            candidate: event.candidate.candidate,
            sdpMid: event.candidate.sdpMid,
            sdpMLineIndex: event.candidate.sdpMLineIndex,
          },
        });
      }
    };
  
    pc.onconnectionstatechange = async (event) => {
      console.log('Peer connection state changed:', pc.connectionState);
      if (pc.connectionState === 'connected') {
        console.log('WebRTC connection established');
        setIsConnected(true);
  
        // Log codec information after connection is established
        const senders = pc.getSenders();
        for (const sender of senders) {
          if (sender.track.kind === 'video') {
            const parameters = sender.getParameters();
            const codec = parameters.codecs[0]; // The first codec is typically the one used
            console.log('Video Codec Information:');
            console.log(`- Codec: ${codec.mimeType}`);
            console.log(`- Payload Type: ${codec.payloadType}`);
            console.log(`- Clock Rate: ${codec.clockRate}`);
          }
        }
      } else if (pc.connectionState === 'disconnected') {
        console.log('WebRTC connection disconnected');
        setIsConnected(false);
      } else if (pc.connectionState === 'failed') {
        console.error('WebRTC connection failed');
        setIsConnected(false);
      }
    };
  
    dataChannel.current = pc.createDataChannel('controlChannel');
    dataChannel.current.onmessage = (event) => {
      console.log('Received control data:', event.data);
      // Handle control data here
    };
  
    return pc;
  };
  

  const connectToServer = async () => {
    try {
      console.log('Connecting to server...');
      const stream = await startStream();
      peerConnection.current = createPeerConnection();

      stream.getTracks().forEach(track => {
        peerConnection.current.addTrack(track, stream);
        console.log('Track added to peer connection:', track);
      });

      const offer = await peerConnection.current.createOffer();
      await peerConnection.current.setLocalDescription(offer);
      console.log('Offer created and set as local description:', offer);

      sendToServer({ type: 'offer', sdp: offer.sdp });
    } catch (err) {
      console.error('Error connecting to server:', err);
      setError(`Failed to connect: ${err.message}`);
    }
  };

  const disconnect = () => {
    console.log('Disconnecting...');
    if (peerConnection.current) {
      peerConnection.current.close();
      peerConnection.current = null;
      console.log('Peer connection closed');
    }
    if (websocket.current) {
      websocket.current.close();
      websocket.current = null;
      console.log('WebSocket connection closed');
    }
    setIsConnected(false);
    
    // Stop video tracks
    const stream = localVideoRef.current.srcObject;
    if (stream) {
      stream.getTracks().forEach(track => {
        track.stop();
        console.log('Video track stopped:', track);
      });
      localVideoRef.current.srcObject = null; // Clear the video source
      console.log('Video source cleared');
    }
  };

  const sendToServer = (message) => {
    if (websocket.current && websocket.current.readyState === WebSocket.OPEN) {
      websocket.current.send(JSON.stringify(message));
      console.log('Message sent to server:', message);
    } else {
      console.error('WebSocket is not connected');
    }
  };

  const handleServerMessage = async (message) => {
    console.log('Handling server message:', message);
    switch (message.type) {
      case 'answer':
        console.log('Answer received from server:', message);
        await peerConnection.current.setRemoteDescription(new RTCSessionDescription(message));
        break;
      case 'ice-candidate':
        if (message.candidate) {
          console.log('ICE candidate received from server:', message.candidate);
          await peerConnection.current.addIceCandidate(new RTCIceCandidate(message.candidate));
        } else {
          console.warn('Received invalid ICE candidate:', message);
        }
        break;
      default:
        console.log('Unknown message type:', message.type);
    }
  };
  

  return (
    <div className="d-flex flex-column align-items-center justify-content-center vh-100">
      <video ref={localVideoRef} autoPlay playsInline muted className="w-100 mb-4" style={{ maxWidth: '500px' }} />
      <Button onClick={connectToServer} disabled={isConnected} variant={isConnected ? "success" : "primary"}>
        {isConnected ? 'Connected' : 'Connect to Server'}
      </Button>
      <Button onClick={disconnect} disabled={!isConnected} variant="danger" className="mt-2">
        Disconnect
      </Button>
    </div>
  );
};

export default WebRTCClient;
