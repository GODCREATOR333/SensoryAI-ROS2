import { useState } from 'react'
import './App.css'
import WebRTCConnection from './pages/WebRTCConnection';

function App() {
  const [connect,isconnected] = useState(0)

  return (
    <>
        <h1>WebRTC Test V1</h1>
        <WebRTCConnection />
    </>
  )
}

export default App
