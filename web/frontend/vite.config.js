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
    proxy: {
      '/api': { target: BACKEND, changeOrigin: true },
      '/ws': { target: BACKEND, ws: true, changeOrigin: true },
    },
  },
})
