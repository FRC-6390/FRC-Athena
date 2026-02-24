import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig({
  plugins: [svelte()],
  clearScreen: false,
  optimizeDeps: {
    // Svelte component packages should be processed by the Svelte plugin,
    // not prebundled by esbuild.
    exclude: ['@fortawesome/svelte-fontawesome']
  },
  server: {
    port: 1420,
    strictPort: true,
    hmr: {
      protocol: 'ws',
      host: 'localhost',
      port: 1421
    }
  }
});
