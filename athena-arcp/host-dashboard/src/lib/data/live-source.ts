/**
 * Live data source: the robot (or simulator) through the Tauri backend (`live.rs`). Records arrive as
 * E1-W bytes in batches and go through the same decoder and store as the mock. Commands are invoked
 * on the backend and return the robot's replies unchanged (E5). Only works inside the Tauri webview.
 */
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { createDecodeContext, decodeRecords, type DecodeContext } from './decoder';
import type { CommandReply, RobotCommands } from './commands';
import type { DataSource, SourceSink } from './source';
import { fromWireValue, toWireValue, type WireValue } from './live-convert';
import type { SignalDescriptor, SignalValue } from './types';
import { ARCP_CONTROL_PORT, DEFAULT_SUBSCRIPTION, DEFAULT_SUBSCRIPTION_HZ } from './arcp-ports';

export type LiveServerStatus = {
  clients: number;
  bytesPerSecond: number;
  budgetBytesPerSecond: number;
  throttled: boolean;
  throttledPatterns: string[];
  droppedFrames: number;
};

export type LiveStatus = {
  connected: boolean;
  sessionId: number;
  rttMicros: number | null;
  offsetMicros: number | null;
  frames: number;
  duplicates: number;
  gaps: number;
  lostCycles: number;
  droppedIncomplete: number;
  backfills: number;
  restarts: number;
  malformedDatagrams: number;
  server: LiveServerStatus | null;
};

export type LiveEvent =
  | { kind: 'gap'; from: number; to: number; lost: number }
  | { kind: 'backfill'; from: number; to: number; records?: number; error?: string }
  | { kind: 'disconnected'; reason: string }
  | { kind: 'recordingFailed' };

export type ConnectOptions = { host: string; controlPort?: number; replay?: boolean };

export type ConnectInfo = { sessionId: number; udpPort: number; udpFallback?: boolean; controlAddr: string };

type WriteReplyWire = { accepted: boolean; reason: string; actualValue: WireValue };
type ActionReplyWire = { accepted: boolean; reason: string };

/** True inside the Tauri webview (the live bridge exists only there). */
export function isTauriRuntime(): boolean {
  return typeof window !== 'undefined' && '__TAURI_INTERNALS__' in window;
}

/** Team number to robot address: 10.TE.AM.2 (team 6390 is 10.63.90.2). */
export function teamHost(team: number): string | null {
  if (!Number.isInteger(team) || team < 1 || team > 9999) return null;
  return `10.${Math.floor(team / 100)}.${team % 100}.2`;
}

function base64ToBytes(data: string): Uint8Array {
  const binary = atob(data);
  const out = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) out[i] = binary.charCodeAt(i);
  return out;
}

export class LiveSource implements DataSource {
  readonly name = 'live';
  readonly commands: RobotCommands;
  private sink: SourceSink | null = null;
  private unlisteners: UnlistenFn[] = [];
  private chain: Promise<void> = Promise.resolve();
  private ctx: DecodeContext = createDecodeContext();
  private readonly descriptors = new Map<number, SignalDescriptor>();
  private readonly statusListeners = new Set<(status: LiveStatus) => void>();
  private readonly eventListeners = new Set<(event: LiveEvent) => void>();

  constructor() {
    this.commands = {
      run: (requestId, signalId) => this.runAction(requestId, signalId, 'run'),
      cancel: (requestId, signalId) => this.runAction(requestId, signalId, 'cancel'),
      write: async (requestId, signalId, value: SignalValue) => {
        const descriptor = this.descriptors.get(signalId);
        if (!descriptor) throw new Error(`unknown signal ${signalId}`);
        const reply = await invoke<WriteReplyWire>('live_write', {
          signalId,
          value: toWireValue(value, descriptor.typeTag)
        });
        return {
          requestId,
          op: 'write',
          signalId,
          accepted: reply.accepted,
          reason: reply.reason,
          actualValue: fromWireValue(reply.actualValue),
          receivedAtMs: Date.now()
        } satisfies CommandReply;
      }
    };
  }

  private async runAction(requestId: number, signalId: number, op: 'run' | 'cancel'): Promise<CommandReply> {
    const reply = await invoke<ActionReplyWire>('live_action', { signalId, run: op === 'run' });
    return {
      requestId,
      op,
      signalId,
      accepted: reply.accepted,
      reason: reply.reason,
      actualValue: null,
      receivedAtMs: Date.now()
    };
  }

  start(sink: SourceSink): void {
    this.sink = sink;
    void Promise.all([
      listen<{ data: string; frames: number }>('live://records', (e) => this.enqueue(e.payload.data)),
      listen<LiveStatus>('live://status', (e) => this.statusListeners.forEach((fn) => fn(e.payload))),
      listen<LiveEvent>('live://event', (e) => this.eventListeners.forEach((fn) => fn(e.payload)))
    ]).then((unlisteners) => {
      this.unlisteners = unlisteners;
    });
  }

  stop(): void {
    this.unlisteners.forEach((off) => off());
    this.unlisteners = [];
    this.sink = null;
  }

  /** Decodes batches in arrival order and hands them to the store. */
  private enqueue(data: string): void {
    this.chain = this.chain
      .then(async () => {
        const records = await decodeRecords(base64ToBytes(data), this.ctx);
        for (const record of records) {
          if (record.kind === 'dictionary') {
            for (const entry of record.entries) this.descriptors.set(entry.signalId, entry);
          }
        }
        this.sink?.records(records);
      })
      .catch((error: unknown) => console.error('live batch dropped', error));
  }

  onStatus(listener: (status: LiveStatus) => void): () => void {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }

  onEvent(listener: (event: LiveEvent) => void): () => void {
    this.eventListeners.add(listener);
    return () => this.eventListeners.delete(listener);
  }

  async connect(options: ConnectOptions): Promise<ConnectInfo> {
    this.ctx = createDecodeContext();
    this.descriptors.clear();
    const info = await invoke<ConnectInfo>('live_connect', {
      options: { ...options, controlPort: options.controlPort ?? ARCP_CONTROL_PORT }
    });
    await this.subscribe(DEFAULT_SUBSCRIPTION, DEFAULT_SUBSCRIPTION_HZ);
    return info;
  }

  disconnect(): Promise<void> {
    return invoke('live_disconnect');
  }

  subscribe(pattern: string, hz?: number): Promise<number> {
    return invoke<number>('live_subscribe', { pattern, hz: hz ?? null });
  }

  timeSync(samples = 8): Promise<{ offsetMicros: number; rttMicros: number; samples: number }> {
    return invoke('live_time_sync', { samples });
  }

  fileList(): Promise<{ name: string; size: number; modifiedUnixSecs: number }[]> {
    return invoke('live_file_list');
  }

  async fileDownload(name: string): Promise<Uint8Array> {
    return base64ToBytes(await invoke<string>('live_file_download', { name }));
  }

  fileDelete(name: string): Promise<void> {
    return invoke('live_file_delete', { name });
  }

  pullTuning(projectDir: string): Promise<string> {
    return invoke<string>('live_pull_tuning', { projectDir });
  }

  tuningDiff(projectDir: string): Promise<{ robot: string | null; project: string | null }> {
    return invoke('live_tuning_diff', { projectDir });
  }

  recordStart(): Promise<string> {
    return invoke<string>('live_record_start');
  }

  recordStop(): Promise<string | null> {
    return invoke<string | null>('live_record_stop');
  }
}
