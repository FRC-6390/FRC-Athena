#!/usr/bin/env node
// Builds the release bundles for the host OS only (no cross-OS or cross-arch targets).
// Linux: deb + AppImage. Windows: NSIS + MSI. macOS: .app + DMG for the host architecture.
// Outputs land in src-tauri/target/release/bundle/ (see README.md).

import { spawn } from 'node:child_process';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const scriptDir = path.dirname(fileURLToPath(import.meta.url));

const bundlesByPlatform = {
  linux: ['deb', 'appimage'],
  win32: ['nsis', 'msi'],
  darwin: ['app', 'dmg'],
};

const bundles = bundlesByPlatform[process.platform];
if (!bundles) {
  console.error(`[package] Unsupported host platform: ${process.platform}`);
  process.exit(1);
}

const args = ['build', '--bundles', bundles.join(',')];
console.log(`[package] Host ${process.platform}/${process.arch}: tauri build --bundles ${bundles.join(',')}`);

const env = { ...process.env };
if (process.platform === 'linux') {
  // linuxdeploy (AppImage) runs without FUSE: extract-and-run avoids needing libfuse2 at bundle time.
  env.APPIMAGE_EXTRACT_AND_RUN ??= '1';
  // linuxdeploy's bundled strip (old binutils) cannot read the .relr.dyn sections of newer system libraries
  // (seen on Fedora 44). Skipping strip only makes the AppImage larger. Unset NO_STRIP to strip anyway.
  env.NO_STRIP ??= '1';
}

const child = spawn(process.execPath, [path.join(scriptDir, 'tauri-launch.mjs'), ...args], {
  stdio: 'inherit',
  env,
});

child.once('error', (error) => {
  console.error(`[package] Failed to start the Tauri CLI: ${error.message}`);
  process.exit(1);
});
child.once('exit', (code, signal) => {
  if (signal) {
    process.kill(process.pid, signal);
    return;
  }
  process.exit(code ?? 0);
});
