#!/usr/bin/env node

import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const rootDir = path.resolve(scriptDir, '..');

function run(command, args, options = {}) {
  return new Promise((resolve, reject) => {
    const child = spawn(command, args, {
      cwd: rootDir,
      stdio: 'inherit',
      ...options,
    });

    child.once('error', (error) => {
      reject(error);
    });

    child.once('exit', (code, signal) => {
      if (signal) {
        resolve(1);
        return;
      }
      resolve(code ?? 0);
    });
  });
}

const launcher = path.join(rootDir, 'scripts', 'tauri-launch.mjs');
const buildCode = await run(process.execPath, [launcher, 'build', '--no-bundle']);
if (buildCode !== 0) {
  process.exit(buildCode);
}

let binPath = null;
if (process.platform === 'linux' || process.platform === 'darwin') {
  binPath = path.join(rootDir, 'src-tauri', 'target', 'release', 'arcp-host-dashboard');
} else if (process.platform === 'win32') {
  binPath = path.join(rootDir, 'src-tauri', 'target', 'release', 'arcp-host-dashboard.exe');
} else {
  console.log('Unsupported OS for auto-run. Build finished; run binary from src-tauri/target/release.');
  process.exit(0);
}

if (!existsSync(binPath)) {
  console.error(`Release binary not found at ${binPath}`);
  process.exit(1);
}

const runCode = await run(binPath, []);
process.exit(runCode);
