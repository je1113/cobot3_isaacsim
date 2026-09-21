import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import { BrowserRouter } from 'react-router-dom'
import './index.css'
import App from './App.jsx'
import { MetaProvider } from './contexts/MetaContext'

createRoot(document.getElementById('root')).render(
  <StrictMode>
    <BrowserRouter>
      {/* 로봇 목록·열거값을 받아 오기 전에는 화면을 그리지 않는다.
          폴백을 두면 하드코딩을 폴백으로 옮겨 적는 것일 뿐이다. */}
      <MetaProvider>
        <App />
      </MetaProvider>
    </BrowserRouter>
  </StrictMode>,
)
