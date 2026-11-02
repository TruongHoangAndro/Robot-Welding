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

export default defineConfig({
  plugins: [react(), awmViewData()],
  build: { chunkSizeWarningLimit: 2000 },
  test: { environment: 'node', include: ['tests/**/*.test.ts'] },
});
