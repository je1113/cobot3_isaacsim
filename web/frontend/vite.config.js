import process from 'node:process'

import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// dev 서버(5173)가 /api 와 /ws 를 백엔드(8000)로 넘긴다.
//
// ★ 프록시를 쓰면 프론트는 항상 자기 출처로만 요청하므로
//   VITE_API_BASE_URL 을 비워 둘 수 있고, CORS 도 타지 않는다.
//   배포 때는 백엔드가 web/frontend/dist 를 같이 서빙하므로 역시 같은 출처다.
//   즉 개발과 배포에서 프론트 코드가 달라지지 않는다.
//
// 백엔드 주소가 다르면 COBOT3_BACKEND 로 덮는다.
const BACKEND = process.env.COBOT3_BACKEND ?? 'http://localhost:8000'

export default defineConfig({
  plugins: [react()],
  server: {
    // ★ 기본값(host 미지정)은 Vite 가 localhost 에만 bind 한다 — 스위치로
    //   연결된 다른 PC 에서는 이 머신 IP 로 아예 접속이 안 된다(연결 거부).
    //   true 로 두면 0.0.0.0 에 bind 해서 같은 네트워크의 다른 PC 도
    //   http://<이 머신 IP>:5173 으로 들어올 수 있다.
    host: true,
    proxy: {
      '/api': { target: BACKEND, changeOrigin: true },
      '/ws': { target: BACKEND, ws: true, changeOrigin: true },
    },
  },
})
