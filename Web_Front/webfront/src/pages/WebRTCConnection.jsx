import React from 'react'

function WebRTCConnection() {
    const localconnection = new RTCPeerConnection
    const datachannel = localconnection.createDataChannel("Channel");
    datachannel.onmessage = e => console.log("Msg Received : ");
    datachannel.onopen = e => console.log("Connection established")
    localconnection.onicecandidate = e => console.log("New Ice Candidate ! reprinting SDP :: " 
        + JSON.strinfy(localconnection.localDescription)
)

  return (
    <div>WebRTCConnection</div>

    
  )
}

export default WebRTCConnection