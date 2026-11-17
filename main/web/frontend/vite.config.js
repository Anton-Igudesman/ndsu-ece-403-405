import { svelte } from '@sveltejs/vite-plugin-svelte'
import tailwindcss from '@tailwindcss/vite'
import { defineConfig } from 'vite'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'

// Read the firmware brightness limit during the frontend build
const ledCubeHeader = readFileSync(
  fileURLToPath(new URL('../../led/led_cube.h', import.meta.url)),
  'utf8'
);

const brightnessMatch = ledCubeHeader.match(
  /^\s*#define\s+LED_CUBE_MAX_BRIGHTNESS\s+(\d+)U?\b/m
);

if (!brightnessMatch) {
  throw new Error('LED_CUBE_MAX_BRIGHTNESS not found in led_cube.h');
}

const maxBrightness = Number(brightnessMatch[1]);

// https://vite.dev/config/
export default defineConfig({
  plugins: [
    svelte(),
    tailwindcss()
  ],
  define: {
    __LED_MAX_BRIGHTNESS__: JSON.stringify(maxBrightness)
  },

  build: {
    rollupOptions: {
      output: {
        entryFileNames: 'assets/app.js',
        chunkFileNames: 'assets/[name].js',
        assetFileNames: (assetInfo) => {
          if (assetInfo.name?.endsWith('.css')) {
            return 'assets/app.css'
          }

          return 'assets/[name][extname]'
        }
      }
    }
  }
})
