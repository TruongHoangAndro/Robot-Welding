/// <reference types="vitest/config" />
import react from '@vitejs/plugin-react';
import { defineConfig, type Plugin } from 'vite';
import { viewDataMiddleware } from './server/viewData';

/** Serves $AWM_VIEW_DIR at /data/ on the dev and preview servers (read-only, see server/viewData.ts). */
function awmViewData(): Plugin {
  const dir = process.env.AWM_VIEW_DIR;
  return {
    name: 'awm-view-data',
    configureServer(server) {
      server.middlewares.use(viewDataMiddleware(dir));
    },
    configurePreviewServer(server) {
      server.middlewares.use(viewDataMiddleware(dir));
    },
  };
}

/** Dev / preview servers forward /api (REST + WebSocket) to awm_api; default the local backend on port 8000. */
const apiTarget = process.env.AWM_API_URL ?? 'http://127.0.0.1:8000';
const apiProxy = { '/api': { target: apiTarget, ws: true, changeOrigin: false } };

export default defineConfig({
  plugins: [react(), awmViewData()],
  server: { proxy: apiProxy },
  preview: { proxy: apiProxy },
  build: { chunkSizeWarningLimit: 2000 },
  test: { environment: 'node', include: ['tests/**/*.test.ts'] },
});
