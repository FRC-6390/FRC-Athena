#!/usr/bin/env node

import { spawn } from 'node:child_process';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const rootDir = path.resolve(scriptDir, '..');
const tauriArgs = process.argv.slice(2);

const env = { ...process.env };
if (process.platform === 'linux') {
  const sessionType = (env.XDG_SESSION_TYPE || '').toLowerCase();
  const forceWayland = env.ARCP_TAURI_WAYLAND || '0';

  if (sessionType === 'wayland' && forceWayland !== '1') {
    env.GDK_BACKEND = 'x11';
    env.WINIT_UNIX_BACKEND = 'x11';
    env.WEBKIT_DISABLE_DMABUF_RENDERER = '1';
    console.log('[arcp-host-dashboard] Wayland session detected; launching Tauri via X11 compatibility path.');
    console.log('[arcp-host-dashboard] Set ARCP_TAURI_WAYLAND=1 to bypass this fallback.');
  }
}

const isWindows = process.platform === 'win32';
const localTauri = path.join(rootDir, 'node_modules', '.bin', isWindows ? 'tauri.cmd' : 'tauri');
const candidates = [
  { command: isWindows ? 'tauri.cmd' : 'tauri', args: tauriArgs },
  { command: localTauri, args: tauriArgs },
  { command: isWindows ? 'bunx.cmd' : 'bunx', args: ['tauri', ...tauriArgs] },
];

function spawnCandidate(index) {
  if (index >= candidates.length) {
    console.error('[arcp-host-dashboard] Unable to find a Tauri CLI command (tauri or bunx tauri).');
    process.exit(1);
  }

  const candidate = candidates[index];
  const child = spawn(candidate.command, candidate.args, {
    cwd: rootDir,
    env,
    stdio: 'inherit',
  });

  let finished = false;
  child.once('error', (error) => {
    if (finished) return;
    finished = true;
    if (error && error.code === 'ENOENT') {
      spawnCandidate(index + 1);
      return;
    }
    console.error(`[arcp-host-dashboard] Failed to launch ${candidate.command}: ${error.message}`);
    process.exit(1);
  });

  child.once('exit', (code, signal) => {
    if (finished) return;
    finished = true;
    if (signal) {
      process.kill(process.pid, signal);
      return;
    }
    process.exit(code ?? 0);
  });
}

spawnCandidate(0);
