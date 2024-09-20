import { useState } from 'react'
import './App.css'
import 'bootstrap/dist/css/bootstrap.min.css';
import WebRTCClient from './pages/WebRTCClient';
import ErrorBoundary from './pages/ErrorBoundary';

function App() {
  const [connect,isconnected] = useState(0)

  return (
    <>
        <h1>WebRTC Test V1</h1>
        <ErrorBoundary>
      <WebRTCClient />
    </ErrorBoundary>
    </>
  )
}

export default App
