import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Dev server proxies /ws and /artifact to the real dashboard_node (aiohttp) process
// so `npm run dev` can run against a live robot/sim without a separate build step.
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      "/ws": { target: "ws://127.0.0.1:8765", ws: true },
      "/artifact": { target: "http://127.0.0.1:8765" },
    },
  },
  build: {
    outDir: "dist",
    emptyOutDir: true,
  },
});
