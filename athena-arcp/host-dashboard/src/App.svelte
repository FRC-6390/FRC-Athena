<script lang="ts">
  import { listen, type UnlistenFn } from '@tauri-apps/api/event';
  import { onMount } from 'svelte';
  import type {
    DashboardSnapshot,
    RemoteLogEntry,
    RobotLinkProbe,
    SignalRow
  } from './lib/arcp';
  import {
    connectArcp,
    downloadRemoteLog,
    deltaArcp,
    deleteServerLayout,
    disconnectArcp,
    fireAction,
    listRemoteLogs,
    listServerLayouts,
    loadServerLayout,
    probeRobotLink,
    readRemoteLogPreview,
    saveServerLayout,
    setDriverstationDockMode,
    setPresentationMode,
    setSignal,
    startRemoteLogStream,
    stopRemoteLogStream,
    windowModeSnapshot
  } from './lib/arcp';
  import { DEFAULT_GRID_COLUMNS, applyFilters, createDefaultLayout, defaultWidgetKind, formatCpu, formatMemory, formatUptime, isLayoutWidgetKind, leafPath, loadLayout, parseLayoutImport, saveLayout, serializeLayoutExport, widgetKindLabel, type DashboardLayoutState, type DashboardWidget } from './lib/dashboard';


  import TopBar from './components/TopBar.svelte';
  import GlobalRail from './components/GlobalRail.svelte';
  import SignalBrowser from './components/SignalBrowser.svelte';
  import { classicKindsFor } from './lib/widgets/registry';
  import InspectorPanel from './components/InspectorPanel.svelte';
  import SettingsScreen from './components/SettingsScreen.svelte';
  import DiagnosticsScreen from './components/DiagnosticsScreen.svelte';
  import LogsScreen from './components/LogsScreen.svelte';
  import MechanismsScreen from './components/MechanismsScreen.svelte';
  import ControlTunerScreen from './components/ControlTunerScreen.svelte';
  import CanvasDashboards from './screens/canvas/CanvasDashboards.svelte';
  import type { CanvasDashboardsApi } from './screens/canvas/types';
  import type { CanvasCommands } from './lib/canvas-bridge';
  import { isIgnoredPublishSignal, isNtCompatPath } from './lib/canvas-bridge';
  import { advanceHistory, collectConfiguredSignalIds } from './lib/canvas/history';
  import { downloadBinaryFile, downloadJsonFile, pickJsonFileText } from './lib/browser-io';

  const REFRESH_MS_DASHBOARDS = 120;
  const REFRESH_MS_DASHBOARDS_HEAVY = 180;
  const REFRESH_MS_DASHBOARDS_DENSE = 240;
  const REFRESH_MS_OTHER = 180;
  const REFRESH_MS_DISCONNECTED = 500;
  const SERVER_LAYOUT_POLL_MS = 2000;
  const DEFAULT_SERVER_LAYOUT_PROFILE = 'atheana-generated';
  const MAX_QUEUED_ACTIONS = 256;
  const CONNECT_RETRY_INTERVAL_MS = 1500;
  const GRID_PREFERENCES_STORAGE_KEY = 'arcp.host.grid.preferences.v1';
  const RECORDING_ACCEPT_REQUESTS_STORAGE_KEY = 'arcp.host.recording.accept-requests.v1';
  const RECORDING_SCHEMA = 'arcp-recording-v1';
  const RECORDING_CLIENT_HEARTBEAT_INTERVAL_MS = 1_000;
  const RECORDING_REPLAY_TICK_MS = 33;
  const REMOTE_LOG_LIVE_POLL_MS = 1_000;
  const REMOTE_LOG_MAX_LINES = 6000;
  const ATHENA_MECHANISM_LOG_PATH_PREFIX = 'athena://mechanisms/log/';

  type GridDensity = 'compact' | 'balanced' | 'comfortable';

  type RailSection =
    | 'dashboards'
    | 'signals'
    | 'mechanisms'
    | 'control_tuner'
    | 'diagnostics'
    | 'logs'
    | 'settings';
  type RecordedSignalDescriptor = Omit<SignalRow, 'value'> & {
    initialValue: string;
  };
  type RecordingFrame = {
    offsetMs: number;
    updates: [number, string][];
  };
  type TelemetryRecording = {
    id: string;
    name: string;
    source: string;
    startedAtEpochMs: number;
    durationMs: number;
    signals: RecordedSignalDescriptor[];
    frames: RecordingFrame[];
  };
  type RecordingSummary = {
    id: string;
    name: string;
    source: string;
    startedAtEpochMs: number;
    durationMs: number;
    signalCount: number;
    frameCount: number;
  };
  type RecordingCaptureState = {
    id: string;
    name: string;
    source: string;
    startedAtEpochMs: number;
    startedAtPerfMs: number;
    signalsById: Map<number, RecordedSignalDescriptor>;
    lastValuesById: Map<number, string>;
    frames: RecordingFrame[];
  };
  type ReplayState = {
    recordingId: string;
    startedAtPerfMs: number;
    cursor: number;
    valueById: Map<number, string>;
    timerId: number | null;
  };
  type PendingRecordingAck = {
    sequence: number;
    status: string;
  };
  type RemoteLogStreamLinePayload = {
    host: string;
    path: string;
    line: string;
  };
  type RemoteLogStreamStatusPayload = {
    host: string;
    path: string;
    state: string;
    message: string;
  };

  function parseGridDensity(raw: unknown): GridDensity {
    return raw === 'compact' || raw === 'comfortable' || raw === 'balanced' ? raw : 'balanced';
  }

  function loadGridPreferences(): { columns: number; density: GridDensity } {
    if (typeof window === 'undefined') {
      return { columns: DEFAULT_GRID_COLUMNS, density: 'balanced' };
    }
    try {
      const raw = window.localStorage.getItem(GRID_PREFERENCES_STORAGE_KEY);
      if (!raw) {
        return { columns: DEFAULT_GRID_COLUMNS, density: 'balanced' };
      }
      const parsed = JSON.parse(raw) as { columns?: unknown; density?: unknown };
      const columnsRaw = Number(parsed.columns);
      const columns = Number.isFinite(columnsRaw)
        ? Math.max(8, Math.min(96, Math.round(columnsRaw)))
        : DEFAULT_GRID_COLUMNS;
      return {
        columns,
        density: parseGridDensity(parsed.density)
      };
    } catch {
      return { columns: DEFAULT_GRID_COLUMNS, density: 'balanced' };
    }
  }

  function saveGridPreferences(columns: number, density: GridDensity) {
    if (typeof window === 'undefined') return;
    try {
      window.localStorage.setItem(
        GRID_PREFERENCES_STORAGE_KEY,
        JSON.stringify({ columns, density })
      );
    } catch {
      // no-op; preferences are best-effort
    }
  }

  function loadAcceptRobotRecordingRequests(): boolean {
    if (typeof window === 'undefined') return true;
    try {
      const raw = window.localStorage.getItem(RECORDING_ACCEPT_REQUESTS_STORAGE_KEY);
      if (raw === null) return true;
      return raw === 'true';
    } catch {
      return true;
    }
  }

  function saveAcceptRobotRecordingRequests(enabled: boolean) {
    if (typeof window === 'undefined') return;
    try {
      window.localStorage.setItem(RECORDING_ACCEPT_REQUESTS_STORAGE_KEY, enabled ? 'true' : 'false');
    } catch {
      // best-effort only
    }
  }

  function widgetHasResolvableBinding(widget: DashboardWidget, validIds: Set<number>): boolean {
    const config = widget.config;
    if (!config || typeof config !== 'object' || Array.isArray(config)) {
      return false;
    }
    const ids = new Set<number>();
    collectConfiguredSignalIds(config, ids);
    for (const id of ids) {
      if (validIds.has(id)) {
        return true;
      }
    }
    const topicPath = (config as Record<string, unknown>).topicPath;
    return typeof topicPath === 'string' && topicPath.trim().length > 0;
  }

  let host = $state('ds');
  let controlPort = $state('5805');
  let serverLayoutName = $state(DEFAULT_SERVER_LAYOUT_PROFILE);
  let availableServerLayouts = $state<string[]>([]);
  let defaultGridColumns = $state(DEFAULT_GRID_COLUMNS);
  let defaultGridDensity = $state<GridDensity>('balanced');
  let connected = $state(false);
  let status = $state('ready');
  let lastError = $state('');

  let snapshot = $state<DashboardSnapshot | null>(null);
  let liveSignals = $state<SignalRow[]>([]);
  let selectedId = $state<number | null>(null);
  let selectedWidgetId = $state<string | null>(null);

  let layoutState = $state<DashboardLayoutState>(createDefaultLayout());
  let editMode = $state(true);
  let canvas: CanvasDashboardsApi | undefined = $state();
  let showInspector = $state(false);
  let railSection = $state<RailSection>('dashboards');
  let gridColumns = $state(DEFAULT_GRID_COLUMNS);
  let presentationMode = $state(false);
  let driverstationDockMode = $state(false);

  let historyBySignal = $state(new Map<number, number[]>());
  let historyTimeBySignal = $state(new Map<number, number[]>());

  let query = $state('');
  let roleFilter = $state<string>('all');
  let typeFilter = $state<string>('all');
  let refreshInFlight = false;
  let layoutListRefreshInFlight = false;
  let layoutDirtyWhileStale = $state(false);
  let queuedSetWrites = $state(new Map<number, string>());
  let queuedActions = $state<number[]>([]);
  let reconnectSyncInFlight = false;
  let connectInFlight = $state(false);
  let autoReconnectArmed = $state(false);
  let nextReconnectAttemptAt = $state(0);
  let needsSessionReconnect = $state(false);
  let acceptRobotRecordingRequests = $state(loadAcceptRobotRecordingRequests());
  let recordingActive = $state(false);
  let replayActive = $state(false);
  let recordingStatus = $state('idle');
  let recordings = $state<TelemetryRecording[]>([]);
  let selectedRecordingId = $state('');
  let lastRecordingRequestSequence = 0;
  let lastRecordingHeartbeatSentAt = 0;

  let recordingCaptureState: RecordingCaptureState | null = null;
  let replayState: ReplayState | null = null;
  let pendingRecordingAck: PendingRecordingAck | null = null;
  let lastProcessedUpdateCount = -1;
  let lastManifestRevision = -1;
  let remoteLogs = $state<RemoteLogEntry[]>([]);
  let remoteLogsResolvedHost = $state('');
  let remoteLogsRefreshInFlight = $state(false);
  let remoteLogPreviewInFlight = $state(false);
  let remoteLogDownloadInFlightPath = $state<string | null>(null);
  let remoteLogError = $state('');
  let selectedRemoteLogPath = $state('');
  let selectedRemoteLogLines = $state<string[]>([]);
  let selectedRemoteLogPreviewTruncated = $state(false);
  let remoteLogLiveFollow = $state(true);
  let remoteLogStreamConnected = $state(false);
  let remoteLogStreamState = $state('idle');
  let remoteLogStreamMessage = $state('');
  let robotLinkProbe = $state<RobotLinkProbe | null>(null);
  let robotLinkProbeInFlight = $state(false);
  let controlTunerTrackedSignalIds = $state<number[]>([]);

  const signalRows = $derived(liveSignals.filter((signal) => !isIgnoredPublishSignal(signal)));
  const ntCompatSignalRows = $derived(signalRows.filter((signal) => isNtCompatPath(signal.path)));
  const arcpSignalRows = $derived(signalRows.filter((signal) => !isNtCompatPath(signal.path)));
  const signalById = $derived(new Map(signalRows.map((signal) => [signal.signal_id, signal])));
  const trimmedQuery = $derived(query.trim());
  const filteredSignals = $derived.by(() => {
    if (!trimmedQuery && roleFilter === 'all' && typeFilter === 'all') {
      return arcpSignalRows;
    }
    return applyFilters(arcpSignalRows, query, roleFilter, typeFilter);
  });
  const filteredNtSignals = $derived.by(() => {
    if (!trimmedQuery) {
      return ntCompatSignalRows;
    }
    return applyFilters(ntCompatSignalRows, query, 'all', 'all');
  });

  const activeTab = $derived(
    layoutState.tabs.find((tab) => tab.id === layoutState.activeTabId) ?? layoutState.tabs[0] ?? null
  );

  const widgets = $derived(activeTab?.widgets ?? []);

  const selectedSignal = $derived(
    selectedId === null ? null : signalById.get(selectedId) ?? null
  );

  const selectedWidget = $derived(
    selectedWidgetId ? widgets.find((widget) => widget.id === selectedWidgetId) ?? null : null
  );


  const dashboardStates = $derived([
    { key: 'signals', label: 'signals', value: String(arcpSignalRows.length), valueWidthCh: 5 },
    { key: 'nt4', label: 'nt4', value: String(ntCompatSignalRows.length), valueWidthCh: 5 },
    { key: 'updates', label: 'updates', value: String(snapshot?.update_count ?? 0), valueWidthCh: 7 },
    { key: 'uptime', label: 'uptime', value: snapshot ? formatUptime(snapshot.uptime_ms) : '0m 0s', valueWidthCh: 9 },
    { key: 'server-cpu', label: 'server cpu', value: snapshot ? formatCpu(snapshot.server_cpu_percent) : 'n/a', valueWidthCh: 6 },
    { key: 'server-rss', label: 'server rss', value: snapshot ? formatMemory(snapshot.server_rss_bytes) : 'n/a', valueWidthCh: 10 }
  ]);
  const staleData = $derived(!connected && snapshot !== null);
  const reconnecting = $derived(!connected && connectInFlight);
  const recordingSummaries = $derived<RecordingSummary[]>(
    recordings.map((recording) => ({
      id: recording.id,
      name: recording.name,
      source: recording.source,
      startedAtEpochMs: recording.startedAtEpochMs,
      durationMs: recording.durationMs,
      signalCount: recording.signals.length,
      frameCount: recording.frames.length
    }))
  );

  // Only the control tuner keeps history in the host; widget history lives in the canvas.
  const historySignalIds = $derived.by(() => {
    const ids = new Set<number>();
    for (const signalId of controlTunerTrackedSignalIds) {
      if (signalId > 0) {
        ids.add(signalId);
      }
    }
    return ids;
  });

  function isConnectionError(err: unknown): boolean {
    const token = String(err).toLowerCase();
    return (
      token.includes('not connected') ||
      token.includes('connection refused') ||
      token.includes('connection reset') ||
      token.includes('broken pipe') ||
      token.includes('timed out') ||
      token.includes('transport') ||
      token.includes('channel closed') ||
      token.includes('os error 111') ||
      token.includes('os error 104')
    );
  }

  function queueSetWrite(signalId: number, valueRaw: string) {
    const next = new Map(queuedSetWrites);
    next.set(signalId, valueRaw);
    queuedSetWrites = next;
  }

  function queueAction(signalId: number) {
    const next = [...queuedActions, signalId];
    if (next.length > MAX_QUEUED_ACTIONS) {
      next.splice(0, next.length - MAX_QUEUED_ACTIONS);
    }
    queuedActions = next;
  }

  function markStaleStatus(message = 'connection lost (stale data)') {
    connected = false;
    if (snapshot) {
      status = message;
      return;
    }
    status = autoReconnectArmed ? 'offline (retrying...)' : 'disconnected';
  }

  function scheduleReconnect(delayMs = CONNECT_RETRY_INTERVAL_MS) {
    nextReconnectAttemptAt = Date.now() + Math.max(0, delayMs);
  }

  function parseControlPort(raw: string): number | null {
    const value = Number(raw);
    if (!Number.isInteger(value) || value < 1 || value > 65535) {
      return null;
    }
    return value;
  }

  function isTauriRuntime(): boolean {
    if (typeof window === 'undefined') return false;
    return '__TAURI_INTERNALS__' in window;
  }

  function splitLogContent(content: string): string[] {
    const normalized = content.replace(/\r\n/g, '\n').replace(/\r/g, '\n');
    const lines = normalized.split('\n');
    if (lines.length > 0 && lines[lines.length - 1] === '') {
      lines.pop();
    }
    return lines;
  }

  function trimLogLines(lines: string[]): string[] {
    if (lines.length <= REMOTE_LOG_MAX_LINES) return lines;
    return lines.slice(lines.length - REMOTE_LOG_MAX_LINES);
  }

  function isStreamableRemoteLogPath(path: string): boolean {
    return !path.trim().startsWith(ATHENA_MECHANISM_LOG_PATH_PREFIX);
  }

  function nowPerfMs(): number {
    if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
      return performance.now();
    }
    return Date.now();
  }

  function makeRecordingId(): string {
    if (typeof crypto !== 'undefined' && typeof crypto.randomUUID === 'function') {
      return crypto.randomUUID();
    }
    return `rec-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
  }

  function makeRecordingName(prefix: string): string {
    const stamp = new Date().toISOString().replace(/[:]/g, '-').replace(/\..+$/, '');
    return `${prefix}-${stamp}`;
  }

  function findSignalByPath(signals: SignalRow[], path: string): SignalRow | null {
    return signals.find((signal) => signal.path === path) ?? null;
  }

  function parseSignalI64(signal: SignalRow | null): number | null {
    if (!signal) return null;
    const parsed = Number(signal.value);
    if (!Number.isFinite(parsed)) return null;
    return Math.floor(parsed);
  }

  function buildRecordedSignalDescriptor(signal: SignalRow): RecordedSignalDescriptor {
    return {
      signal_id: signal.signal_id,
      signal_type: signal.signal_type,
      kind: signal.kind,
      access: signal.access,
      policy: signal.policy,
      durability: signal.durability,
      path: signal.path,
      initialValue: signal.value
    };
  }

  function queueRecordingAck(sequence: number, statusValue: string) {
    pendingRecordingAck = {
      sequence,
      status: statusValue
    };
  }

  function beginRecording(source: string): boolean {
    if (recordingActive) {
      recordingStatus = 'recording already active';
      return false;
    }
    if (replayActive) {
      recordingStatus = 'stop replay before starting recording';
      return false;
    }

    const startedAtEpochMs = Date.now();
    const startedAtPerfMs = nowPerfMs();
    const signalsById = new Map<number, RecordedSignalDescriptor>();
    const lastValuesById = new Map<number, string>();
    for (const signal of signalRows) {
      signalsById.set(signal.signal_id, buildRecordedSignalDescriptor(signal));
      lastValuesById.set(signal.signal_id, signal.value);
    }

    const safeSource = source.trim() || 'manual';
    recordingCaptureState = {
      id: makeRecordingId(),
      name: makeRecordingName(safeSource),
      source: safeSource,
      startedAtEpochMs,
      startedAtPerfMs,
      signalsById,
      lastValuesById,
      frames: []
    };
    recordingActive = true;
    recordingStatus = `recording (${safeSource})`;
    return true;
  }

  function captureRecordingFrame(signals: SignalRow[]) {
    if (!recordingActive || !recordingCaptureState) return;

    const updates: [number, string][] = [];
    for (const signal of signals) {
      if (!recordingCaptureState.signalsById.has(signal.signal_id)) {
        recordingCaptureState.signalsById.set(signal.signal_id, buildRecordedSignalDescriptor(signal));
        recordingCaptureState.lastValuesById.set(signal.signal_id, signal.value);
        continue;
      }
      const previous = recordingCaptureState.lastValuesById.get(signal.signal_id);
      if (previous === signal.value) continue;
      recordingCaptureState.lastValuesById.set(signal.signal_id, signal.value);
      updates.push([signal.signal_id, signal.value]);
    }

    if (updates.length === 0) return;
    const offsetMs = Math.max(0, Math.round(nowPerfMs() - recordingCaptureState.startedAtPerfMs));
    recordingCaptureState.frames.push({ offsetMs, updates });
  }

  function finalizeRecording(completedStatus: string) {
    if (!recordingCaptureState) return;

    const durationMs = Math.max(0, Math.round(nowPerfMs() - recordingCaptureState.startedAtPerfMs));
    const recording: TelemetryRecording = {
      id: recordingCaptureState.id,
      name: recordingCaptureState.name,
      source: recordingCaptureState.source,
      startedAtEpochMs: recordingCaptureState.startedAtEpochMs,
      durationMs,
      signals: [...recordingCaptureState.signalsById.values()].sort((a, b) =>
        a.path.localeCompare(b.path, undefined, { sensitivity: 'base' })
      ),
      frames: [...recordingCaptureState.frames]
    };

    recordings = [recording, ...recordings];
    selectedRecordingId = recording.id;
    recordingCaptureState = null;
    recordingActive = false;
    recordingStatus = completedStatus;
  }

  function stopRecording() {
    if (!recordingActive) return;
    finalizeRecording('recording stopped');
  }

  function stopReplay(statusText = 'replay stopped') {
    if (replayState && replayState.timerId !== null) {
      window.clearInterval(replayState.timerId);
    }
    replayState = null;
    replayActive = false;
    recordingStatus = statusText;
    void refreshSnapshot();
  }

  function buildReplaySignals(
    recording: TelemetryRecording,
    valueById: Map<number, string>
  ): SignalRow[] {
    return recording.signals.map((signal) => ({
      signal_id: signal.signal_id,
      signal_type: signal.signal_type,
      kind: signal.kind,
      access: signal.access,
      policy: signal.policy,
      durability: signal.durability,
      path: signal.path,
      value: valueById.get(signal.signal_id) ?? signal.initialValue
    }));
  }

  function applyReplaySnapshot(
    recording: TelemetryRecording,
    valueById: Map<number, string>,
    elapsedMs: number
  ) {
    const signals = buildReplaySignals(recording, valueById);
    liveSignals = signals;
    const clampedDuration = Math.max(1, recording.durationMs);
    const progress = Math.max(0, Math.min(100, Math.round((elapsedMs / clampedDuration) * 100)));
    const previous = snapshot;
    snapshot = {
      connected,
      status: `replay ${recording.name} (${progress}%)`,
      signal_count: signals.length,
      update_count: previous?.update_count ?? 0,
      uptime_ms: elapsedMs,
      server_cpu_percent: previous?.server_cpu_percent ?? null,
      server_cpu_cores: previous?.server_cpu_cores ?? null,
      server_rss_bytes: previous?.server_rss_bytes ?? null,
      server_ram_total_bytes: previous?.server_ram_total_bytes ?? null,
      server_ram_available_bytes: previous?.server_ram_available_bytes ?? null,
      server_disk_total_bytes: previous?.server_disk_total_bytes ?? null,
      server_disk_available_bytes: previous?.server_disk_available_bytes ?? null,
      host_cpu_percent: previous?.host_cpu_percent ?? null,
      host_rss_bytes: previous?.host_rss_bytes ?? null,
      signals
    };
  }

  function replayRecording() {
    const recording = recordings.find((entry) => entry.id === selectedRecordingId);
    if (!recording) {
      recordingStatus = 'no recording selected';
      return;
    }
    if (recordingActive) {
      recordingStatus = 'stop active recording before replay';
      return;
    }

    if (replayState && replayState.timerId !== null) {
      window.clearInterval(replayState.timerId);
    }

    const valueById = new Map<number, string>();
    for (const signal of recording.signals) {
      valueById.set(signal.signal_id, signal.initialValue);
    }

    replayState = {
      recordingId: recording.id,
      startedAtPerfMs: nowPerfMs(),
      cursor: 0,
      valueById,
      timerId: null
    };
    replayActive = true;
    recordingStatus = `replay running (${recording.name})`;
    applyReplaySnapshot(recording, replayState.valueById, 0);

    const tick = () => {
      if (!replayState || replayState.recordingId !== recording.id) return;
      const elapsedMs = Math.max(0, Math.round(nowPerfMs() - replayState.startedAtPerfMs));
      while (
        replayState.cursor < recording.frames.length &&
        recording.frames[replayState.cursor].offsetMs <= elapsedMs
      ) {
        const frame = recording.frames[replayState.cursor];
        for (const [signalId, value] of frame.updates) {
          replayState.valueById.set(signalId, value);
        }
        replayState.cursor += 1;
      }

      applyReplaySnapshot(recording, replayState.valueById, elapsedMs);
      if (elapsedMs >= recording.durationMs) {
        stopReplay(`replay complete (${recording.name})`);
      }
    };

    const timerId = window.setInterval(tick, RECORDING_REPLAY_TICK_MS);
    replayState.timerId = timerId;
  }

  async function exportSelectedRecording() {
    const recording = recordings.find((entry) => entry.id === selectedRecordingId);
    if (!recording) {
      recordingStatus = 'no recording selected';
      return;
    }
    const payload = {
      schema: RECORDING_SCHEMA,
      exportedAtEpochMs: Date.now(),
      recordings: [recording]
    };
    const filename = `${recording.name}.arcp-recording.json`;
    downloadJsonFile(filename, JSON.stringify(payload, null, 2));
    recordingStatus = `saved recording file (${recording.name})`;
  }

  function sanitizeImportedRecording(input: unknown): TelemetryRecording | null {
    if (!input || typeof input !== 'object') return null;
    const raw = input as Partial<TelemetryRecording>;
    if (typeof raw.id !== 'string' || typeof raw.name !== 'string' || typeof raw.source !== 'string') {
      return null;
    }
    if (!Array.isArray(raw.signals) || !Array.isArray(raw.frames)) {
      return null;
    }

    const signals: RecordedSignalDescriptor[] = raw.signals
      .filter((signal): signal is RecordedSignalDescriptor => {
        if (!signal || typeof signal !== 'object') return false;
        const candidate = signal as RecordedSignalDescriptor;
        return (
          typeof candidate.signal_id === 'number' &&
          typeof candidate.signal_type === 'string' &&
          typeof candidate.kind === 'string' &&
          typeof candidate.access === 'string' &&
          typeof candidate.policy === 'string' &&
          typeof candidate.durability === 'string' &&
          typeof candidate.path === 'string' &&
          typeof candidate.initialValue === 'string'
        );
      })
      .map((signal) => ({ ...signal }));

    const frames: RecordingFrame[] = raw.frames
      .filter((frame): frame is RecordingFrame => {
        if (!frame || typeof frame !== 'object') return false;
        const candidate = frame as RecordingFrame;
        return typeof candidate.offsetMs === 'number' && Array.isArray(candidate.updates);
      })
      .map((frame) => ({
        offsetMs: Math.max(0, Math.round(frame.offsetMs)),
        updates: frame.updates
          .filter(
            (update): update is [number, string] =>
              Array.isArray(update) &&
              update.length === 2 &&
              typeof update[0] === 'number' &&
              typeof update[1] === 'string'
          )
          .map((update) => [Math.floor(update[0]), update[1]])
      }));

    return {
      id: raw.id,
      name: raw.name,
      source: raw.source,
      startedAtEpochMs: Number.isFinite(raw.startedAtEpochMs) ? Math.floor(raw.startedAtEpochMs ?? 0) : Date.now(),
      durationMs: Number.isFinite(raw.durationMs) ? Math.max(0, Math.round(raw.durationMs ?? 0)) : 0,
      signals,
      frames
    };
  }

  async function importRecordingFromFile() {
    try {
      const raw = await pickJsonFileText();
      if (!raw) return;
      const parsed = JSON.parse(raw) as unknown;

      const importedRecordings: TelemetryRecording[] = [];
      if (Array.isArray(parsed)) {
        for (const entry of parsed) {
          const sanitized = sanitizeImportedRecording(entry);
          if (sanitized) importedRecordings.push(sanitized);
        }
      } else if (parsed && typeof parsed === 'object') {
        const payload = parsed as { schema?: string; recordings?: unknown[] };
        if (Array.isArray(payload.recordings)) {
          for (const entry of payload.recordings) {
            const sanitized = sanitizeImportedRecording(entry);
            if (sanitized) importedRecordings.push(sanitized);
          }
        } else {
          const single = sanitizeImportedRecording(parsed);
          if (single) importedRecordings.push(single);
        }
      }

      if (importedRecordings.length === 0) {
        recordingStatus = 'load failed (no recordings found in file)';
        return;
      }

      const deduped = new Map<string, TelemetryRecording>();
      for (const recording of [...importedRecordings, ...recordings]) {
        deduped.set(recording.id, recording);
      }
      recordings = [...deduped.values()];
      selectedRecordingId = importedRecordings[0].id;
      recordingStatus = `loaded ${importedRecordings.length} recording(s)`;
    } catch (err) {
      recordingStatus = `load failed: ${String(err)}`;
    }
  }

  function deleteSelectedRecording() {
    if (!selectedRecordingId) return;
    const target = recordings.find((entry) => entry.id === selectedRecordingId);
    if (!target) return;
    const confirmed = window.confirm(`Delete recording '${target.name}'?`);
    if (!confirmed) return;

    const next = recordings.filter((entry) => entry.id !== selectedRecordingId);
    recordings = next;
    selectedRecordingId = next[0]?.id ?? '';
    recordingStatus = `deleted ${target.name}`;
  }

  function setAcceptRobotRecordingRequests(enabled: boolean) {
    acceptRobotRecordingRequests = enabled;
    saveAcceptRobotRecordingRequests(enabled);
    recordingStatus = enabled ? 'robot recording requests enabled' : 'robot recording requests ignored';
  }

  function handleRobotRecordingRequests(signals: SignalRow[]) {
    const sequenceSignal = findSignalByPath(signals, 'Athena/Recording/Request/sequence');
    const sequence = parseSignalI64(sequenceSignal);
    if (sequence === null || sequence <= 0) return;
    if (sequence <= lastRecordingRequestSequence) return;

    lastRecordingRequestSequence = sequence;
    const mode = findSignalByPath(signals, 'Athena/Recording/Request/mode')?.value ?? 'unknown';
    const phase = findSignalByPath(signals, 'Athena/Recording/Request/phase')?.value ?? 'unknown';
    const source = `robot-${mode}-${phase}`;

    if (!acceptRobotRecordingRequests) {
      queueRecordingAck(sequence, 'ignored_by_client');
      recordingStatus = `ignored robot record request (${mode}/${phase})`;
      return;
    }
    if (replayActive) {
      queueRecordingAck(sequence, 'rejected_replay_active');
      recordingStatus = 'robot record request rejected (replay active)';
      return;
    }
    if (recordingActive) {
      queueRecordingAck(sequence, 'accepted_already_recording');
      recordingStatus = 'robot record request accepted (already recording)';
      return;
    }

    const started = beginRecording(source);
    queueRecordingAck(sequence, started ? 'accepted_started' : 'rejected');
  }

  async function syncRecordingClientSignals(signals: SignalRow[]) {
    if (!connected) return;

    const heartbeatSignal = findSignalByPath(signals, 'Athena/Recording/Client/heartbeatMs');
    const ackSequenceSignal = findSignalByPath(signals, 'Athena/Recording/Client/ackSequence');
    const ackStatusSignal = findSignalByPath(signals, 'Athena/Recording/Client/ackStatus');

    const now = Date.now();
    try {
      if (
        heartbeatSignal &&
        now - lastRecordingHeartbeatSentAt >= RECORDING_CLIENT_HEARTBEAT_INTERVAL_MS
      ) {
        await setSignal(heartbeatSignal.signal_id, String(now));
        lastRecordingHeartbeatSentAt = now;
      }

      if (pendingRecordingAck && ackSequenceSignal && ackStatusSignal) {
        await setSignal(ackSequenceSignal.signal_id, String(pendingRecordingAck.sequence));
        await setSignal(ackStatusSignal.signal_id, pendingRecordingAck.status);
        pendingRecordingAck = null;
      }
    } catch (err) {
      if (isConnectionError(err)) {
        markStaleStatus('connection lost while sending recording client state');
      } else {
        lastError = `recording client sync failed: ${String(err)}`;
      }
    }
  }

  async function connectWithCurrentSettings(source: 'manual' | 'auto' | 'badge'): Promise<boolean> {
    if (connectInFlight) return false;

    const normalizedHost = host.trim();
    const port = parseControlPort(controlPort);
    if (port === null) {
      connected = false;
      needsSessionReconnect = true;
      if (source === 'auto') {
        status = 'offline (retrying...)';
      } else {
        lastError = 'connection failed: port must be 1-65535';
        status = 'connection failed';
      }
      scheduleReconnect();
      return false;
    }

    connectInFlight = true;
    availableServerLayouts = [];

    try {
      const info = await connectArcp(normalizedHost, port);
      if (!info.connected) {
        connected = false;
        status = 'connection failed';
        scheduleReconnect();
        return false;
      }

      autoReconnectArmed = true;
      nextReconnectAttemptAt = 0;
      needsSessionReconnect = false;
      lastProcessedUpdateCount = -1;
      lastManifestRevision = -1;
      lastError = '';
      lastRecordingHeartbeatSentAt = 0;
      status = `connected to ${info.host}:${info.control_port} (udp ${info.udp_port})`;
      await refreshSnapshot();
      await refreshServerLayoutList();
      return true;
    } catch (err) {
      connected = false;
      needsSessionReconnect = true;
      scheduleReconnect();
      void refreshRobotLinkProbe({ silent: true });
      if (source === 'auto') {
        status = 'offline (retrying...)';
      } else {
        status = 'connection failed';
        lastError = String(err);
      }
      return false;
    } finally {
      connectInFlight = false;
    }
  }

  async function maybeAutoReconnect(force = false) {
    if (connected || connectInFlight) return;
    if (!force && !autoReconnectArmed) return;
    if (!force && !needsSessionReconnect) return;
    if (!force && Date.now() < nextReconnectAttemptAt) return;
    await connectWithCurrentSettings(force ? 'badge' : 'auto');
  }

  function noteLayoutMutation() {
    if (!connected) {
      layoutDirtyWhileStale = true;
    }
  }

  function handleCanvasLayoutChange(next: DashboardLayoutState) {
    noteLayoutMutation();
    saveLayout(next);
  }

  const canvasCommands: CanvasCommands = {
    setValue: async (signalId, raw) => {
      await sendSet(signalId, raw);
      return null;
    },
    triggerAction: async (signalId) => {
      await sendAction(signalId);
      return null;
    }
  };

  function activateRail(section: RailSection) {
    if (railSection === 'logs' && section !== 'logs') {
      void stopRemoteLogStreaming({ silent: true });
    }
    if (section !== 'control_tuner') {
      controlTunerTrackedSignalIds = [];
    }
    railSection = section;
    switch (section) {
      case 'dashboards':
        roleFilter = 'all';
        typeFilter = 'all';
        break;
      case 'signals':
        showInspector = true;
        roleFilter = 'all';
        typeFilter = 'all';
        selectedWidgetId = null;
        break;
      case 'mechanisms':
        roleFilter = 'all';
        typeFilter = 'all';
        query = '';
        showInspector = false;
        selectedWidgetId = null;
        break;
      case 'control_tuner':
        roleFilter = 'all';
        typeFilter = 'all';
        query = '';
        showInspector = false;
        selectedWidgetId = null;
        break;
      case 'diagnostics':
        roleFilter = 'all';
        typeFilter = 'all';
        query = '';
        showInspector = false;
        selectedWidgetId = null;
        break;
      case 'logs':
        roleFilter = 'all';
        typeFilter = 'all';
        query = '';
        showInspector = false;
        selectedWidgetId = null;
        void refreshRobotLinkProbe({ silent: true });
        void refreshRemoteLogs(true);
        break;
      case 'settings':
        roleFilter = 'all';
        typeFilter = 'all';
        void refreshServerLayoutList();
        break;
      default:
        break;
    }
  }

  async function refreshWindowModes() {
    try {
      const modes = await windowModeSnapshot();
      presentationMode = modes.presentationMode;
      driverstationDockMode = modes.dockMode;
    } catch {
      // ignore if desktop window APIs are unavailable
    }
  }

  async function togglePresentationMode() {
    const enabled = !presentationMode;
    try {
      await setPresentationMode(enabled);
      presentationMode = enabled;
      if (enabled) {
        driverstationDockMode = false;
      }
      status = enabled ? 'presentation mode enabled' : 'presentation mode disabled';
      lastError = '';
    } catch (err) {
      lastError = String(err);
    }
  }

  async function toggleDriverstationDockMode() {
    const enabled = !driverstationDockMode;
    try {
      await setDriverstationDockMode(enabled);
      driverstationDockMode = enabled;
      if (enabled) {
        presentationMode = false;
      }
      status = enabled ? 'driver station dock mode enabled' : 'driver station dock mode disabled';
      lastError = '';
    } catch (err) {
      lastError = String(err);
    }
  }

  function updateHistory(signals: SignalRow[], trackedSignalIds: Set<number>) {
    const next = advanceHistory({ values: historyBySignal, times: historyTimeBySignal }, signals, trackedSignalIds, Date.now());
    historyBySignal = next.values;
    historyTimeBySignal = next.times;
  }


  function normalizeServerLayoutName(raw: string): string {
    return raw.trim().replace(/[^a-zA-Z0-9._-]/g, '').slice(0, 64);
  }

  function preferredServerLayoutName(layouts: string[]): string {
    if (layouts.length === 0) return '';
    const preferred = layouts.find((name) => name === DEFAULT_SERVER_LAYOUT_PROFILE);
    return preferred ?? layouts[0];
  }

  async function refreshServerLayoutList() {
    if (!connected) {
      availableServerLayouts = [];
      serverLayoutName = '';
      return;
    }
    if (layoutListRefreshInFlight) return;
    layoutListRefreshInFlight = true;
    try {
      const layouts = await listServerLayouts();
      const normalizedLayouts = [...layouts].sort((a, b) =>
        a.localeCompare(b, undefined, { sensitivity: 'base', numeric: true })
      );
      availableServerLayouts = normalizedLayouts;
      if (normalizedLayouts.length === 0) {
        serverLayoutName = DEFAULT_SERVER_LAYOUT_PROFILE;
      } else if (!normalizedLayouts.includes(serverLayoutName)) {
        serverLayoutName = preferredServerLayoutName(normalizedLayouts);
      }
      lastError = '';
    } catch (err) {
      availableServerLayouts = [];
      serverLayoutName = '';
      lastError = `server layout list failed: ${String(err)}`;
    } finally {
      layoutListRefreshInFlight = false;
    }
  }

  function clearRemoteLogSelection() {
    void stopRemoteLogStreaming({ silent: true });
    selectedRemoteLogPath = '';
    selectedRemoteLogLines = [];
    selectedRemoteLogPreviewTruncated = false;
    remoteLogDownloadInFlightPath = null;
    remoteLogPreviewInFlight = false;
    remoteLogStreamConnected = false;
    remoteLogStreamState = 'idle';
    remoteLogStreamMessage = '';
  }

  function clearRemoteLogs() {
    remoteLogs = [];
    remoteLogsResolvedHost = '';
    remoteLogError = '';
    robotLinkProbe = null;
    clearRemoteLogSelection();
  }

  function setRemoteLogLiveFollow(enabled: boolean) {
    if (remoteLogLiveFollow === enabled) return;
    remoteLogLiveFollow = enabled;
    if (!enabled) {
      void stopRemoteLogStreaming({ silent: true });
      return;
    }

    if (!selectedRemoteLogPath) return;
    if (isTauriRuntime()) {
      if (isStreamableRemoteLogPath(selectedRemoteLogPath)) {
        void startRemoteLogStreaming(selectedRemoteLogPath, { silent: false });
      } else {
        void stopRemoteLogStreaming({ silent: true });
      }
    } else {
      void loadRemoteLogPreview(selectedRemoteLogPath, { silent: true });
    }
  }

  async function stopRemoteLogStreaming(options?: { silent?: boolean }) {
    if (!isTauriRuntime()) {
      remoteLogStreamConnected = false;
      remoteLogStreamState = 'idle';
      remoteLogStreamMessage = '';
      return;
    }

    try {
      await stopRemoteLogStream();
      remoteLogStreamConnected = false;
      remoteLogStreamState = 'stopped';
      remoteLogStreamMessage = 'stream stopped';
      if (!options?.silent) {
        remoteLogError = '';
      }
    } catch (err) {
      if (!options?.silent) {
        remoteLogError = String(err);
      }
    }
  }

  async function startRemoteLogStreaming(
    path: string,
    options?: {
      silent?: boolean;
    }
  ) {
    const normalizedPath = path.trim();
    if (!normalizedPath) return;
    if (!remoteLogLiveFollow) return;
    if (!isTauriRuntime()) return;
    if (!isStreamableRemoteLogPath(normalizedPath)) {
      remoteLogStreamConnected = false;
      remoteLogStreamState = 'unsupported';
      remoteLogStreamMessage = 'live stream not supported for Athena mechanism logs';
      return;
    }

    try {
      await startRemoteLogStream(host.trim(), normalizedPath);
      remoteLogStreamConnected = false;
      remoteLogStreamState = 'starting';
      remoteLogStreamMessage = 'starting stream';
      if (!options?.silent) {
        remoteLogError = '';
      }
    } catch (err) {
      remoteLogStreamConnected = false;
      remoteLogStreamState = 'error';
      remoteLogStreamMessage = String(err);
      if (!options?.silent) {
        remoteLogError = String(err);
      }
    }
  }

  function setRemoteLogPreviewContent(content: string, truncated: boolean) {
    const lines = trimLogLines(splitLogContent(content));
    selectedRemoteLogLines = lines;
    selectedRemoteLogPreviewTruncated = truncated || lines.length >= REMOTE_LOG_MAX_LINES;
  }

  function appendRemoteLogLine(line: string) {
    const normalized = line.replace(/\r$/, '');
    const next = [...selectedRemoteLogLines, normalized];
    const trimmed = trimLogLines(next);
    selectedRemoteLogLines = trimmed;
    if (trimmed.length < next.length) {
      selectedRemoteLogPreviewTruncated = true;
    }
  }

  async function refreshRobotLinkProbe(
    options?: {
      silent?: boolean;
    }
  ) {
    const port = parseControlPort(controlPort);
    if (port === null) {
      robotLinkProbe = null;
      return;
    }
    if (robotLinkProbeInFlight) return;

    robotLinkProbeInFlight = true;
    try {
      const probe = await probeRobotLink(host.trim(), port);
      robotLinkProbe = probe;
      if (!remoteLogsResolvedHost && probe.host) {
        remoteLogsResolvedHost = probe.host;
      }
      if (!options?.silent) {
        remoteLogError = '';
      }
    } catch (err) {
      if (!options?.silent) {
        remoteLogError = String(err);
      }
    } finally {
      robotLinkProbeInFlight = false;
    }
  }

  async function refreshRemoteLogs(force = false) {
    if (remoteLogsRefreshInFlight) return;
    if (!force && remoteLogs.length > 0) return;

    remoteLogsRefreshInFlight = true;
    remoteLogError = '';
    try {
      await refreshRobotLinkProbe({ silent: true });
      const listing = await listRemoteLogs(host.trim());
      remoteLogs = listing.entries;
      remoteLogsResolvedHost = listing.host;
      const hasSelection = listing.entries.some((entry) => entry.path === selectedRemoteLogPath);
      if (!hasSelection) {
        selectedRemoteLogPath = listing.entries[0]?.path ?? '';
        selectedRemoteLogLines = [];
        selectedRemoteLogPreviewTruncated = false;
      }
      if (selectedRemoteLogPath) {
        await loadRemoteLogPreview(selectedRemoteLogPath, { silent: true });
        if (remoteLogLiveFollow && isStreamableRemoteLogPath(selectedRemoteLogPath)) {
          await startRemoteLogStreaming(selectedRemoteLogPath, { silent: true });
        } else {
          await stopRemoteLogStreaming({ silent: true });
        }
      } else {
        await stopRemoteLogStreaming({ silent: true });
      }
      status = `log index refreshed (${listing.entries.length} files)`;
      lastError = '';
    } catch (err) {
      remoteLogError = String(err);
    } finally {
      remoteLogsRefreshInFlight = false;
    }
  }

  async function loadRemoteLogPreview(
    path: string,
    options?: {
      silent?: boolean;
    }
  ) {
    const normalizedPath = path.trim();
    if (!normalizedPath) {
      clearRemoteLogSelection();
      return;
    }

    selectedRemoteLogPath = normalizedPath;
    remoteLogPreviewInFlight = true;
    remoteLogError = '';
    try {
      const preview = await readRemoteLogPreview(host.trim(), normalizedPath, 64 * 1024);
      remoteLogsResolvedHost = preview.host;
      setRemoteLogPreviewContent(preview.content, preview.truncated);
      if (!options?.silent) {
        lastError = '';
      }
    } catch (err) {
      selectedRemoteLogLines = [];
      selectedRemoteLogPreviewTruncated = false;
      remoteLogError = String(err);
      if (!options?.silent) {
        lastError = String(err);
      }
    } finally {
      remoteLogPreviewInFlight = false;
    }
  }

  async function downloadSelectedRemoteLog(path: string) {
    const normalizedPath = path.trim();
    if (!normalizedPath || remoteLogDownloadInFlightPath) return;

    remoteLogDownloadInFlightPath = normalizedPath;
    remoteLogError = '';
    try {
      const payload = await downloadRemoteLog(host.trim(), normalizedPath, 20 * 1024 * 1024);
      remoteLogsResolvedHost = payload.host;
      const fileName = payload.fileName || 'remote-log.bin';
      downloadBinaryFile(fileName, new Uint8Array(payload.bytes));
      status = `downloaded ${fileName}`;
      lastError = '';
    } catch (err) {
      remoteLogError = String(err);
    } finally {
      remoteLogDownloadInFlightPath = null;
    }
  }

  async function deleteServerLayoutProfile(layoutName: string) {
    const profile = normalizeServerLayoutName(layoutName);
    if (!profile) {
      lastError = 'delete from server failed: layout name must be [a-zA-Z0-9._-]';
      return;
    }
    if (!connected) {
      lastError = 'delete from server failed: not connected';
      return;
    }

    const confirmed = window.confirm(`Delete server UI profile '${profile}'?`);
    if (!confirmed) return;

    try {
      await deleteServerLayout(profile);
      status = `ui profile '${profile}' deleted from server`;
      lastError = '';
      await refreshServerLayoutList();
      if (serverLayoutName === profile && availableServerLayouts.length > 0) {
        serverLayoutName = availableServerLayouts[0];
      }
    } catch (err) {
      lastError = `delete from server failed: ${String(err)}`;
    }
  }

  async function saveLayoutToServerProfile() {
    if (!connected) {
      lastError = 'save to server failed: not connected';
      return;
    }
    const profile = normalizeServerLayoutName(serverLayoutName);
    if (!profile) {
      lastError = 'save to server failed: layout name must be [a-zA-Z0-9._-]';
      return;
    }

    try {
      const serialized = serializeLayoutExport(layoutState);
      await saveServerLayout(profile, serialized);
      serverLayoutName = profile;
      layoutDirtyWhileStale = false;
      status = `ui saved to server profile '${profile}'`;
      lastError = '';
      await refreshServerLayoutList();
    } catch (err) {
      lastError = `save to server failed: ${String(err)}`;
    }
  }

  async function loadLayoutFromServerProfile() {
    if (!connected) {
      lastError = 'load from server failed: not connected';
      return;
    }
    const profile = normalizeServerLayoutName(serverLayoutName);
    if (!profile) {
      lastError = 'load from server failed: layout name must be [a-zA-Z0-9._-]';
      return;
    }

    try {
      const raw = await loadServerLayout(profile);
      const imported = parseLayoutImport(raw);
      const hasExistingWidgets = layoutState.tabs.some((tab) => tab.widgets.length > 0);
      if (hasExistingWidgets) {
        const confirmed = window.confirm(
          `Load server UI profile '${profile}' and replace current layout? This action can be undone with Ctrl+Z.`
        );
        if (!confirmed) return;
      }

      canvas?.commitLayout(imported);
      selectedWidgetId = null;
      selectedId = null;
      status = `ui loaded from server profile '${profile}' (${imported.tabs.length} tabs)`;
      lastError = '';
      serverLayoutName = profile;
    } catch (err) {
      lastError = `load from server failed: ${String(err)}`;
    }
  }

  function updateDefaultGridColumns(value: string) {
    const columnsRaw = Number(value);
    if (!Number.isFinite(columnsRaw)) return;
    const columns = Math.max(8, Math.min(96, Math.round(columnsRaw)));
    defaultGridColumns = columns;
    saveGridPreferences(defaultGridColumns, defaultGridDensity);
    canvas?.setGridColumns(columns);
  }

  function updateDefaultGridDensity(value: string) {
    const density = parseGridDensity(value);
    defaultGridDensity = density;
    saveGridPreferences(defaultGridColumns, defaultGridDensity);
  }


  async function flushQueuedMutations() {
    if (!connected || reconnectSyncInFlight) return;
    if (queuedSetWrites.size === 0 && queuedActions.length === 0 && !layoutDirtyWhileStale) return;

    reconnectSyncInFlight = true;
    const hadDirtyLayout = layoutDirtyWhileStale;
    let flushedSetCount = 0;
    let flushedActionCount = 0;

    try {
      const pendingSetEntries = [...queuedSetWrites.entries()];
      const remainingSetEntries = new Map(queuedSetWrites);
      for (const [signalId, valueRaw] of pendingSetEntries) {
        await setSignal(signalId, valueRaw);
        remainingSetEntries.delete(signalId);
        queuedSetWrites = new Map(remainingSetEntries);
        flushedSetCount += 1;
      }

      const pendingActions = [...queuedActions];
      const remainingActions = [...queuedActions];
      for (const signalId of pendingActions) {
        await fireAction(signalId);
        remainingActions.shift();
        queuedActions = [...remainingActions];
        flushedActionCount += 1;
      }

      if (layoutDirtyWhileStale) {
        const profile = normalizeServerLayoutName(serverLayoutName);
        if (!profile) {
          lastError = 'reconnect sync skipped: layout name must be [a-zA-Z0-9._-]';
        } else {
          const serialized = serializeLayoutExport(layoutState);
          await saveServerLayout(profile, serialized);
          serverLayoutName = profile;
          layoutDirtyWhileStale = false;
        }
      }

      const layoutSynced = hadDirtyLayout && !layoutDirtyWhileStale;
      if (flushedSetCount > 0 || flushedActionCount > 0 || layoutSynced) {
        status = `reconnected: synced ${flushedSetCount} set(s), ${flushedActionCount} action(s)${
          layoutSynced ? ', layout' : ''
        }`;
      }
    } catch (err) {
      if (isConnectionError(err)) {
        markStaleStatus('connection lost during reconnect sync (stale data)');
        lastError = '';
      } else {
        lastError = `reconnect sync failed: ${String(err)}`;
      }
    } finally {
      reconnectSyncInFlight = false;
    }
  }

  async function refreshSnapshot() {
    if (replayActive) return;
    if (refreshInFlight) return;
    refreshInFlight = true;
    try {
      const wasConnected = connected;
      const next = await deltaArcp(
        lastProcessedUpdateCount >= 0 ? lastProcessedUpdateCount : null,
        lastManifestRevision >= 0 ? lastManifestRevision : null
      );
      const previous = snapshot;
      let resolvedSignals = liveSignals;

      if (!next.connected && next.signals.length === 0 && previous && !next.full_snapshot) {
        resolvedSignals = previous.signals;
        snapshot = {
          ...previous,
          connected: false,
          status: next.status,
          signal_count: next.signal_count,
          update_count: next.update_count,
          uptime_ms: next.uptime_ms,
          server_cpu_percent: next.server_cpu_percent,
          server_cpu_cores: next.server_cpu_cores,
          server_rss_bytes: next.server_rss_bytes,
          server_ram_total_bytes: next.server_ram_total_bytes,
          server_ram_available_bytes: next.server_ram_available_bytes,
          server_disk_total_bytes: next.server_disk_total_bytes,
          server_disk_available_bytes: next.server_disk_available_bytes,
          host_cpu_percent: next.host_cpu_percent,
          host_rss_bytes: next.host_rss_bytes,
          signals: resolvedSignals
        };
      } else {
        let nextSignals: SignalRow[];
        if (next.full_snapshot || !previous) {
          nextSignals = next.signals;
        } else if (next.signals.length === 0) {
          nextSignals = previous.signals;
        } else {
          const indexById = new Map<number, number>();
          previous.signals.forEach((signal, index) => {
            indexById.set(signal.signal_id, index);
          });
          const merged = [...previous.signals];
          let appended = false;
          for (const signal of next.signals) {
            const index = indexById.get(signal.signal_id);
            if (index === undefined) {
              merged.push(signal);
              appended = true;
            } else {
              merged[index] = signal;
            }
          }
          if (appended) {
            merged.sort((left, right) => left.signal_id - right.signal_id);
          }
          nextSignals = merged;
        }
        resolvedSignals = nextSignals;

        snapshot = {
          connected: next.connected,
          status: next.status,
          signal_count: next.signal_count,
          update_count: next.update_count,
          uptime_ms: next.uptime_ms,
          server_cpu_percent: next.server_cpu_percent,
          server_cpu_cores: next.server_cpu_cores,
          server_rss_bytes: next.server_rss_bytes,
          server_ram_total_bytes: next.server_ram_total_bytes,
          server_ram_available_bytes: next.server_ram_available_bytes,
          server_disk_total_bytes: next.server_disk_total_bytes,
          server_disk_available_bytes: next.server_disk_available_bytes,
          host_cpu_percent: next.host_cpu_percent,
          host_rss_bytes: next.host_rss_bytes,
          signals: nextSignals
        };
      }
      if (liveSignals !== resolvedSignals) {
        liveSignals = resolvedSignals;
      }

      connected = next.connected;
      status = next.status;
      needsSessionReconnect = false;

      const filteredSignals = resolvedSignals.filter((signal) => !isIgnoredPublishSignal(signal));
      const updateChanged = next.full_snapshot || next.update_count !== lastProcessedUpdateCount;
      if (connected) {
        if (updateChanged) {
          handleRobotRecordingRequests(filteredSignals);
          captureRecordingFrame(filteredSignals);
          updateHistory(filteredSignals, historySignalIds);

          const validIds = new Set(filteredSignals.map((signal) => signal.signal_id));
          const nextTabs = layoutState.tabs.map((tab) => {
            const filtered = tab.widgets.filter(
              (widget) =>
                isLayoutWidgetKind(widget.kind) ||
                validIds.has(widget.signalId) ||
                widgetHasResolvableBinding(widget, validIds)
            );
            return filtered.length === tab.widgets.length ? tab : { ...tab, widgets: filtered };
          });

          const changed = nextTabs.some((tab, index) => tab !== layoutState.tabs[index]);
          if (changed) {
            canvas?.commitLayout({ ...layoutState, tabs: nextTabs }, { recordHistory: false, clearRedo: false });
          }

          if (selectedId !== null && !validIds.has(selectedId)) {
            selectedId = null;
          }

          if (selectedWidgetId && !nextTabs.some((tab) => tab.widgets.some((widget) => widget.id === selectedWidgetId))) {
            selectedWidgetId = null;
          }
        }

        await syncRecordingClientSignals(filteredSignals);
        lastProcessedUpdateCount = next.update_count;
        lastManifestRevision = next.manifest_revision;
      } else {
        lastProcessedUpdateCount = -1;
        lastManifestRevision = -1;
      }

      if (!wasConnected && connected) {
        await flushQueuedMutations();
        await refreshServerLayoutList();
      }
    } catch (err) {
      if (isConnectionError(err)) {
        markStaleStatus();
        lastError = '';
        needsSessionReconnect = true;
        if (autoReconnectArmed) {
          scheduleReconnect();
        }
      } else {
        lastError = String(err);
      }
    } finally {
      refreshInFlight = false;
    }
  }

  async function connect() {
    autoReconnectArmed = true;
    scheduleReconnect(0);
    lastError = '';
    await connectWithCurrentSettings('manual');
  }

  async function disconnect() {
    autoReconnectArmed = false;
    nextReconnectAttemptAt = 0;
    needsSessionReconnect = false;
    pendingRecordingAck = null;
    lastRecordingHeartbeatSentAt = 0;
    if (recordingActive) {
      finalizeRecording('recording stopped (disconnect)');
    }
    if (replayActive) {
      stopReplay('replay stopped (disconnect)');
    }
    try {
      await disconnectArcp();
    } catch (err) {
      lastError = String(err);
    }
    connected = false;
    lastProcessedUpdateCount = -1;
    lastManifestRevision = -1;
    status = snapshot ? 'disconnected (stale data retained)' : 'disconnected';
    availableServerLayouts = [];
  }

  async function retryConnectionFromStatusBadge() {
    if (connected) return;
    autoReconnectArmed = true;
    scheduleReconnect(0);
    await maybeAutoReconnect(true);
  }

  async function sendAction(signalId: number) {
    if (!connected) {
      queueAction(signalId);
      status = `queued action ${signalId} (stale)`;
      lastError = '';
      return;
    }

    try {
      await fireAction(signalId);
      lastError = '';
      status = `action ${signalId}`;
    } catch (err) {
      if (isConnectionError(err)) {
        queueAction(signalId);
        markStaleStatus(`connection lost; queued action ${signalId}`);
        lastError = '';
      } else {
        lastError = String(err);
      }
    }
  }

  async function sendSet(signalId: number, valueRaw: string) {
    if (!valueRaw.trim()) {
      lastError = 'set value cannot be empty';
      return;
    }

    if (!connected) {
      queueSetWrite(signalId, valueRaw);
      status = `queued set ${signalId} (stale)`;
      lastError = '';
      return;
    }

    try {
      await setSignal(signalId, valueRaw);
      lastError = '';
      status = `set ${signalId}`;
    } catch (err) {
      if (isConnectionError(err)) {
        queueSetWrite(signalId, valueRaw);
        markStaleStatus(`connection lost; queued set ${signalId}`);
        lastError = '';
      } else {
        lastError = String(err);
      }
    }
  }

  function handleHostInput(value: string) {
    host = value;
    availableServerLayouts = [];
    clearRemoteLogs();
  }

  function handlePortInput(value: string) {
    controlPort = value;
    availableServerLayouts = [];
    robotLinkProbe = null;
  }



  function onGlobalKeyDown(event: KeyboardEvent) {
    const key = event.key.toLowerCase();
    if (event.key === 'F11') {
      event.preventDefault();
      void togglePresentationMode();
      return;
    }

    const hasAccel = event.ctrlKey || event.metaKey;
    if (hasAccel && event.shiftKey && key === 'd') {
      event.preventDefault();
      void toggleDriverstationDockMode();
      return;
    }

    if (key === 'escape' && (presentationMode || driverstationDockMode)) {
      event.preventDefault();
      if (presentationMode) {
        void setPresentationMode(false).then(
          () => {
            presentationMode = false;
            status = 'presentation mode disabled';
          },
          (err) => {
            lastError = String(err);
          }
        );
      } else {
        void setDriverstationDockMode(false).then(
          () => {
            driverstationDockMode = false;
            status = 'driver station dock mode disabled';
          },
          (err) => {
            lastError = String(err);
          }
        );
      }
      return;
    }

  }

  onMount(() => {
    const gridPreferences = loadGridPreferences();
    defaultGridColumns = gridPreferences.columns;
    defaultGridDensity = gridPreferences.density;
    gridColumns = gridPreferences.columns;
    layoutState = loadLayout();
    void refreshWindowModes();
    if (Number(controlPort) > 0) {
      void connect();
    }
    const onFullscreenChange = () => {
      void refreshWindowModes();
    };
    let refreshCancelled = false;
    let refreshTimer: number | null = null;
    const refreshDelayMs = () => {
      if (!connected) return REFRESH_MS_DISCONNECTED;
      if (railSection !== 'dashboards') return REFRESH_MS_OTHER;
      const panelCount = widgets.length;
      if (panelCount >= 40) return REFRESH_MS_DASHBOARDS_DENSE;
      if (panelCount >= 20) return REFRESH_MS_DASHBOARDS_HEAVY;
      return REFRESH_MS_DASHBOARDS;
    };
    const refreshTick = async () => {
      if (refreshCancelled) return;
      await refreshSnapshot();
      await maybeAutoReconnect();
      if (refreshCancelled) return;
      refreshTimer = window.setTimeout(() => {
        void refreshTick();
      }, refreshDelayMs());
    };
    void refreshTick();
    const serverLayoutTimer = setInterval(() => {
      if (!connected) return;
      void refreshServerLayoutList();
    }, SERVER_LAYOUT_POLL_MS);
    const robotLinkProbeTimer = setInterval(() => {
      if (railSection !== 'logs') return;
      if (robotLinkProbeInFlight) return;
      void refreshRobotLinkProbe({ silent: true });
    }, REMOTE_LOG_LIVE_POLL_MS);
    let unlistenRemoteLogLine: UnlistenFn | null = null;
    let unlistenRemoteLogStatus: UnlistenFn | null = null;
    if (isTauriRuntime()) {
      void listen<RemoteLogStreamLinePayload>('remote-log-stream-line', (event) => {
        const payload = event.payload;
        if (!payload || payload.path !== selectedRemoteLogPath) return;
        if (!remoteLogLiveFollow) return;
        appendRemoteLogLine(payload.line);
      }).then((unlisten) => {
        unlistenRemoteLogLine = unlisten;
      });

      void listen<RemoteLogStreamStatusPayload>('remote-log-stream-status', (event) => {
        const payload = event.payload;
        if (!payload || payload.path !== selectedRemoteLogPath) return;
        remoteLogStreamState = payload.state || 'unknown';
        remoteLogStreamMessage = payload.message || '';
        remoteLogStreamConnected = payload.state === 'connected';
        if (payload.host) {
          remoteLogsResolvedHost = payload.host;
        }
        if (payload.state === 'error') {
          remoteLogError = payload.message || 'stream error';
        }
      }).then((unlisten) => {
        unlistenRemoteLogStatus = unlisten;
      });
    }
    window.addEventListener('keydown', onGlobalKeyDown);
    document.addEventListener('fullscreenchange', onFullscreenChange);
    return () => {
      refreshCancelled = true;
      if (refreshTimer !== null) {
        clearTimeout(refreshTimer);
      }
      clearInterval(serverLayoutTimer);
      clearInterval(robotLinkProbeTimer);
      if (unlistenRemoteLogLine) {
        unlistenRemoteLogLine();
      }
      if (unlistenRemoteLogStatus) {
        unlistenRemoteLogStatus();
      }
      void stopRemoteLogStreaming({ silent: true });
      if (replayState && replayState.timerId !== null) {
        window.clearInterval(replayState.timerId);
      }
      window.removeEventListener('keydown', onGlobalKeyDown);
        document.removeEventListener('fullscreenchange', onFullscreenChange);
    };
  });
</script>

<main class="app-shell">
  <GlobalRail section={railSection} onActivate={activateRail} />

  <section class="work-root">
    <TopBar
      {connected}
      stale={staleData}
      connecting={reconnecting}
      {serverLayoutName}
      availableServerLayouts={availableServerLayouts}
      {recordingActive}
      {replayActive}
      {recordingStatus}
      recordings={recordingSummaries}
      {selectedRecordingId}
      {presentationMode}
      {driverstationDockMode}
      {dashboardStates}
      onLoadLayoutFromHost={() => void canvas?.importLayoutFromFile()}
      onLoadLayoutFromServer={() => void loadLayoutFromServerProfile()}
      onSaveLayoutToHost={() => void canvas?.exportLayoutToFile()}
      onSaveLayoutToServer={() => void saveLayoutToServerProfile()}
      onServerLayoutNameInput={(value) => (serverLayoutName = normalizeServerLayoutName(value))}
      onToggleRecording={() => {
        if (recordingActive) {
          stopRecording();
          return;
        }
        beginRecording('manual');
      }}
      onSelectedRecordingIdInput={(recordingId) => (selectedRecordingId = recordingId)}
      onLoadSelectedRecording={replayRecording}
      onImportRecording={() => void importRecordingFromFile()}
      onStopReplay={stopReplay}
      onRefreshServerLayouts={() => void refreshServerLayoutList()}
      onTogglePresentationMode={() => void togglePresentationMode()}
      onToggleDriverstationDockMode={() => void toggleDriverstationDockMode()}
      onConnectionBadgeClick={() => void retryConnectionFromStatusBadge()}
    />

    <section class="workspace-frame panel">
      {#if lastError}
        <section class="error-banner panel">{lastError}</section>
      {/if}

      {#if railSection === 'settings'}
        <SettingsScreen
          {host}
          {controlPort}
          defaultGridColumns={defaultGridColumns}
          defaultGridDensity={defaultGridDensity}
          {acceptRobotRecordingRequests}
          {recordingStatus}
          recordings={recordingSummaries}
          {selectedRecordingId}
          {serverLayoutName}
          availableServerLayouts={availableServerLayouts}
          {connected}
          {status}
          onHostInput={handleHostInput}
          onPortInput={handlePortInput}
          onServerLayoutNameInput={(value) => (serverLayoutName = normalizeServerLayoutName(value))}
          onDefaultGridColumnsInput={updateDefaultGridColumns}
          onDefaultGridDensityInput={updateDefaultGridDensity}
          onAcceptRobotRecordingRequestsInput={setAcceptRobotRecordingRequests}
          onSelectedRecordingIdInput={(recordingId) => (selectedRecordingId = recordingId)}
          onConnect={connect}
          onDisconnect={disconnect}
          onReplayRecording={replayRecording}
          onExportRecording={() => void exportSelectedRecording()}
          onImportRecording={() => void importRecordingFromFile()}
          onDeleteRecording={deleteSelectedRecording}
          onDeleteServerLayout={(layoutName) => void deleteServerLayoutProfile(layoutName)}
        />
      {:else if railSection === 'signals'}
        <section class={`signals-stage ${showInspector ? '' : 'no-inspector'}`}>
          <div class="signals-layout">
            <SignalBrowser
              signals={filteredSignals}
              ntSignals={filteredNtSignals}
              {selectedId}
              {query}
              onQueryInput={(value) => (query = value)}
              onSelectSignal={(signalId) => {
                selectedId = signalId;
                selectedWidgetId = null;
              }}
              onAddSignal={(signal) => canvas?.createWidgetFromSignal(signal, defaultWidgetKind(signal))}
              {leafPath}
            />
            {#if showInspector}
              <InspectorPanel
                signals={signalRows}
                {signalById}
                {selectedSignal}
                {selectedWidget}
                widgetKinds={classicKindsFor(selectedSignal)}
                {widgetKindLabel}
                onShowAsKind={(kind) => canvas?.showAsWidgetKind(kind)}
                onTriggerAction={sendAction}
                onSelectTunableWidget={() => {
                  if (selectedSignal) canvas?.createWidgetFromSignal(selectedSignal, 'tunable');
                }}
                onRemoveWidget={(widgetId) => canvas?.removeWidget(widgetId)}
                onUpdateWidgetTitle={(widgetId, title) => canvas?.updateWidgetTitle(widgetId, title)}
                onUpdateWidgetConfig={(widgetId, config) => canvas?.updateWidgetConfig(widgetId, config)}
                onRequestSignalMap={(request) => canvas?.requestSignalMap(request)}
              />
            {/if}
          </div>
        </section>
      {:else if railSection === 'mechanisms'}
        <MechanismsScreen
          signals={arcpSignalRows}
          {selectedId}
          onSelectSignal={(signalId) => {
            selectedId = signalId;
            selectedWidgetId = null;
          }}
          onTriggerAction={sendAction}
          onSendSet={sendSet}
        />
      {:else if railSection === 'control_tuner'}
        <ControlTunerScreen
          signals={arcpSignalRows}
          {historyBySignal}
          {selectedId}
          onSelectSignal={(signalId) => {
            selectedId = signalId;
            selectedWidgetId = null;
          }}
          onSendSet={sendSet}
          onTriggerAction={sendAction}
          onTrackSignalIds={(signalIds) => {
            controlTunerTrackedSignalIds = signalIds;
          }}
        />
      {:else if railSection === 'diagnostics'}
        <DiagnosticsScreen
          {connected}
          {status}
          {lastError}
          {snapshot}
          signals={arcpSignalRows}
          onSelectSignal={(signalId) => {
            selectedId = signalId;
            selectedWidgetId = null;
            railSection = 'signals';
            showInspector = true;
          }}
        />
      {:else if railSection === 'logs'}
        <LogsScreen
          {connected}
          {status}
          requestedHost={host}
          resolvedHost={remoteLogsResolvedHost}
          probeHost={robotLinkProbe?.host ?? ''}
          controlPort={robotLinkProbe?.controlPort ?? parseControlPort(controlPort) ?? 0}
          robotReachable={robotLinkProbe?.robotReachable ?? false}
          arcpReachable={robotLinkProbe?.arcpReachable ?? false}
          probeLoading={robotLinkProbeInFlight}
          refreshing={remoteLogsRefreshInFlight}
          previewLoading={remoteLogPreviewInFlight}
          downloadingPath={remoteLogDownloadInFlightPath}
          error={remoteLogError}
          entries={remoteLogs}
          selectedPath={selectedRemoteLogPath}
          previewLines={selectedRemoteLogLines}
          previewTruncated={selectedRemoteLogPreviewTruncated}
          liveFollow={remoteLogLiveFollow}
          streamConnected={remoteLogStreamConnected}
          streamState={remoteLogStreamState}
          streamMessage={remoteLogStreamMessage}
          onRefresh={() => {
            void refreshRobotLinkProbe();
            void refreshRemoteLogs(true);
          }}
          onSelectLog={(path) => {
            setRemoteLogLiveFollow(true);
            void (async () => {
              await loadRemoteLogPreview(path);
              if (isStreamableRemoteLogPath(path)) {
                await startRemoteLogStreaming(path, { silent: true });
              } else {
                await stopRemoteLogStreaming({ silent: true });
              }
            })();
          }}
          onDownloadLog={(path) => void downloadSelectedRemoteLog(path)}
          onToggleLiveFollow={(enabled) => setRemoteLogLiveFollow(enabled)}
        />
      {/if}

      <CanvasDashboards
        bind:this={canvas}
        bind:layout={layoutState}
        bind:editing={editMode}
        bind:gridColumns
        bind:selectedId
        bind:selectedWidgetId
        bind:showInspector
        rows={signalRows}
        commands={canvasCommands}
        active={railSection === 'dashboards'}
        onLayoutChange={handleCanvasLayoutChange}
      />
    </section>
  </section>
</main>

<style>
  .work-root {
    min-width: 0;
    min-height: 0;
    display: grid;
    grid-template-rows: auto 1fr;
    gap: 0.45rem;
    padding: 0.45rem;
  }

  .workspace-frame {
    min-width: 0;
    height: 100%;
    padding: 0.5rem;
    min-height: 0;
    display: flex;
    flex-direction: column;
    gap: 0.45rem;
    overflow: auto;
  }

  .signals-stage {
    flex: 1 1 auto;
    min-height: 0;
    display: grid;
    grid-template-rows: 1fr;
    gap: 0;
  }

  .signals-layout {
    min-height: 0;
    display: grid;
    grid-template-columns: minmax(280px, 420px) minmax(320px, 1fr);
    gap: 0.58rem;
  }

  .signals-stage.no-inspector .signals-layout {
    grid-template-columns: 1fr;
  }

  @media (max-width: 1200px) {
    .signals-layout {
      grid-template-columns: 1fr;
    }
  }

  @media (max-width: 900px) {
    .signals-stage {
      grid-template-rows: auto 1fr;
    }
  }
</style>
