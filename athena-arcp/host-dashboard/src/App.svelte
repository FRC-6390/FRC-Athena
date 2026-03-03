<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faMagnifyingGlass, faXmark } from '@fortawesome/free-solid-svg-icons';
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
    windowModeSnapshot
  } from './lib/arcp';
  import {
    DEFAULT_GRID_COLUMNS,
    applyFilters,
    clampLayout,
    createDefaultLayout,
    createEmptyTab,
    defaultLeafWidgetKind,
    defaultWidgetKind,
    formatCpu,
    formatMemory,
    formatUptime,
    hardwareWidgetKindForPath,
    isActionSignal,
    isLayoutWidgetKind,
    isWritableSignal,
    leafPath,
    loadLayout,
    makeLayoutWidget,
    makeWidget,
    parseLayoutImport,
    parseNumericSignal,
    saveLayout,
    serializeLayoutExport,
    sparklinePath,
    widgetKindsFor,
    widgetKindLabel,
    type DashboardLayoutState,
    type DashboardWidget,
    type WidgetConfigRecord,
    type LayoutToolKind,
    type WidgetKind,
    type WidgetLayout
  } from './lib/dashboard';
  import {
    buildDefaultWidgetConfig,
    cloneWidgetConfig,
    readLayoutAccordionConfig,
    readLayoutGridConfig
  } from './lib/widget-config';

  import TopBar from './components/TopBar.svelte';
  import GlobalRail from './components/GlobalRail.svelte';
  import WorkspaceTabs from './components/WorkspaceTabs.svelte';
  import SignalBrowser from './components/SignalBrowser.svelte';
  import LayoutToolPalette from './components/LayoutToolPalette.svelte';
  import WidgetCanvas from './components/WidgetCanvas.svelte';
  import InspectorPanel from './components/InspectorPanel.svelte';
  import SettingsScreen from './components/SettingsScreen.svelte';
  import ActionsScreen from './components/ActionsScreen.svelte';
  import CameraTuningScreen from './components/CameraTuningScreen.svelte';
  import DiagnosticsScreen from './components/DiagnosticsScreen.svelte';
  import LogsScreen from './components/LogsScreen.svelte';
  import MechanismsScreen from './components/MechanismsScreen.svelte';
  import FloatingDock from './components/FloatingDock.svelte';
  import DashboardContextMenu from './components/DashboardContextMenu.svelte';

  const REFRESH_MS_DASHBOARDS = 120;
  const REFRESH_MS_OTHER = 180;
  const REFRESH_MS_DISCONNECTED = 500;
  const SERVER_LAYOUT_POLL_MS = 2000;
  const HISTORY_LIMIT = 120;
  const ACCORDION_COLLAPSED_ROWS = 1;
  const DEFAULT_ACCORDION_EXPANDED_ROWS = 3;
  const DEFAULT_SERVER_LAYOUT_PROFILE = 'atheana-generated';
  const MAX_QUEUED_ACTIONS = 256;
  const CONNECT_RETRY_INTERVAL_MS = 1500;
  const GRID_PREFERENCES_STORAGE_KEY = 'arcp.host.grid.preferences.v1';
  const RECORDING_ACCEPT_REQUESTS_STORAGE_KEY = 'arcp.host.recording.accept-requests.v1';
  const RECORDING_SCHEMA = 'arcp-recording-v1';
  const RECORDING_CLIENT_HEARTBEAT_INTERVAL_MS = 1_000;
  const RECORDING_REPLAY_TICK_MS = 33;
  const REMOTE_LOG_LIVE_POLL_MS = 1_000;

  type GridDensity = 'compact' | 'balanced' | 'comfortable';

  type RailSection =
    | 'dashboards'
    | 'signals'
    | 'mechanisms'
    | 'actions'
    | 'diagnostics'
    | 'logs'
    | 'camera_tuning'
    | 'settings';
  type CommitLayoutOptions = {
    recordHistory?: boolean;
    clearRedo?: boolean;
  };
  type CopiedWidgetSnapshot = {
    kind: WidgetKind;
    signalId: number;
    title: string;
    layout: WidgetLayout;
    parentLayoutId: string | undefined;
    config: WidgetConfigRecord | undefined;
    inputValue: string | null;
  };
  type DashboardContextMenuState = {
    x: number;
    y: number;
  };
  type SignalMapRequest = {
    title: string;
    candidates: SignalRow[];
    selectedSignalId: number | null;
    allowNone: boolean;
    noneLabel: string;
    onPick: (signalId: number | null) => void;
  };
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

  function isIgnoredPublishSignal(signal: SignalRow): boolean {
    const token = signal.path.toLowerCase().replace(/[^a-z0-9]+/g, '');
    return token.includes('publish');
  }

  function collectConfiguredSignalIds(value: unknown, out: Set<number>): void {
    if (value === null || value === undefined) return;
    if (typeof value === 'number') {
      if (Number.isFinite(value) && value > 0) {
        out.add(Math.floor(value));
      }
      return;
    }
    if (Array.isArray(value)) {
      for (const item of value) {
        collectConfiguredSignalIds(item, out);
      }
      return;
    }
    if (typeof value !== 'object') {
      return;
    }
    const record = value as Record<string, unknown>;
    for (const [key, nested] of Object.entries(record)) {
      if (key.toLowerCase().endsWith('signalid')) {
        collectConfiguredSignalIds(nested, out);
        continue;
      }
      if (
        key === 'series' ||
        key === 'actions' ||
        key === 'columns' ||
        key === 'signals' ||
        key === 'targets'
      ) {
        collectConfiguredSignalIds(nested, out);
      }
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
  let selectedId = $state<number | null>(null);
  let selectedWidgetId = $state<string | null>(null);

  let layoutState = $state<DashboardLayoutState>(createDefaultLayout());
  let editMode = $state(true);
  let showExplorer = $state(false);
  let showLayoutTools = $state(false);
  let showInspector = $state(false);
  let railSection = $state<RailSection>('dashboards');
  let gridColumns = $state(DEFAULT_GRID_COLUMNS);
  let presentationMode = $state(false);
  let driverstationDockMode = $state(false);

  let widgetInputs = $state<Record<string, string>>({});
  let historyBySignal = $state(new Map<number, number[]>());

  let query = $state('');
  let layoutToolQuery = $state('');
  let roleFilter = $state<string>('all');
  let typeFilter = $state<string>('all');
  let refreshInFlight = false;
  let layoutListRefreshInFlight = false;
  let undoStack = $state<DashboardLayoutState[]>([]);
  let redoStack = $state<DashboardLayoutState[]>([]);
  let copiedWidget = $state<CopiedWidgetSnapshot | null>(null);
  let dashboardMenu = $state<DashboardContextMenuState | null>(null);
  let pendingSignalMapRequest = $state<SignalMapRequest | null>(null);
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
  let selectedRemoteLogPreview = $state('');
  let selectedRemoteLogPreviewTruncated = $state(false);
  let remoteLogLiveFollow = $state(true);
  let robotLinkProbe = $state<RobotLinkProbe | null>(null);
  let robotLinkProbeInFlight = $state(false);

  const signalRows = $derived((snapshot?.signals ?? []).filter((signal) => !isIgnoredPublishSignal(signal)));
  const ntCompatSignalRows = $derived(
    signalRows.filter((signal) => signal.path.startsWith('Athena/NT4/'))
  );
  const arcpSignalRows = $derived(
    signalRows.filter((signal) => !signal.path.startsWith('Athena/NT4/'))
  );
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
  const explorerSignals = $derived.by(() =>
    pendingSignalMapRequest
      ? applyFilters(pendingSignalMapRequest.candidates, query, 'all', 'all')
      : filteredSignals
  );
  const actionSignals = $derived(arcpSignalRows.filter((signal) => isActionSignal(signal)));

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

  const tabSummaries = $derived(
    layoutState.tabs.map((tab) => ({
      id: tab.id,
      name: tab.name,
      widgetCount: tab.widgets.length
    }))
  );

  const dashboardStates = $derived([
    { key: 'signals', label: 'signals', value: String(arcpSignalRows.length), valueWidthCh: 5 },
    { key: 'nt4', label: 'nt4', value: String(ntCompatSignalRows.length), valueWidthCh: 5 },
    { key: 'updates', label: 'updates', value: String(snapshot?.update_count ?? 0), valueWidthCh: 7 },
    { key: 'uptime', label: 'uptime', value: snapshot ? formatUptime(snapshot.uptime_ms) : '0m 0s', valueWidthCh: 9 },
    { key: 'server-cpu', label: 'server cpu', value: snapshot ? formatCpu(snapshot.server_cpu_percent) : 'n/a', valueWidthCh: 6 },
    { key: 'server-rss', label: 'server rss', value: snapshot ? formatMemory(snapshot.server_rss_bytes) : 'n/a', valueWidthCh: 10 },
    { key: 'ui-cpu', label: 'ui cpu', value: snapshot ? formatCpu(snapshot.host_cpu_percent) : 'n/a', valueWidthCh: 6 },
    { key: 'ui-rss', label: 'ui rss', value: snapshot ? formatMemory(snapshot.host_rss_bytes) : 'n/a', valueWidthCh: 10 }
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

  const historySignalIds = $derived.by(() => {
    const ids = new Set<number>();
    for (const widget of widgets) {
      if (widget.kind === 'trend' && widget.signalId > 0) {
        ids.add(widget.signalId);
      }
      if (widget.kind === 'graph') {
        if (widget.signalId > 0) {
          ids.add(widget.signalId);
        }
        collectConfiguredSignalIds(widget.config, ids);
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
    if (replayState?.timerId !== null) {
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
      server_rss_bytes: previous?.server_rss_bytes ?? null,
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

    if (replayState?.timerId !== null) {
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
      startedAtEpochMs: Number.isFinite(raw.startedAtEpochMs) ? Math.floor(raw.startedAtEpochMs) : Date.now(),
      durationMs: Number.isFinite(raw.durationMs) ? Math.max(0, Math.round(raw.durationMs)) : 0,
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

  function activateRail(section: RailSection) {
    dashboardMenu = null;
    railSection = section;
    if (section !== 'dashboards') {
      showLayoutTools = false;
    }
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
      case 'actions':
        roleFilter = 'action';
        typeFilter = 'all';
        query = '';
        showInspector = true;
        break;
      case 'mechanisms':
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
        void refreshRemoteLogs();
        break;
      case 'camera_tuning':
        roleFilter = 'all';
        typeFilter = 'all';
        query = '';
        showInspector = false;
        selectedWidgetId = null;
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

  function cloneLayoutState(state: DashboardLayoutState): DashboardLayoutState {
    return {
      activeTabId: state.activeTabId,
      tabs: state.tabs.map((tab) => ({
        id: tab.id,
        name: tab.name,
        widgets: tab.widgets.map((widget) => ({
          id: widget.id,
          signalId: widget.signalId,
          kind: widget.kind,
          title: widget.title,
          layout: { ...widget.layout },
          parentLayoutId: widget.parentLayoutId,
          config: cloneWidgetConfig(widget.config)
        }))
      }))
    };
  }

  function pushHistorySnapshot(stack: DashboardLayoutState[], snapshot: DashboardLayoutState): DashboardLayoutState[] {
    const next = [...stack, cloneLayoutState(snapshot)];
    if (next.length <= HISTORY_LIMIT) return next;
    return next.slice(next.length - HISTORY_LIMIT);
  }

  function layoutHasWidget(layout: DashboardLayoutState, widgetId: string): boolean {
    return layout.tabs.some((tab) => tab.widgets.some((widget) => widget.id === widgetId));
  }

  function commitLayout(next: DashboardLayoutState, options: CommitLayoutOptions = {}) {
    const { recordHistory = true, clearRedo = true } = options;
    if (recordHistory) {
      undoStack = pushHistorySnapshot(undoStack, layoutState);
      if (clearRedo) {
        redoStack = [];
      }
    }

    layoutState = next;
    noteLayoutMutation();
    saveLayout(next);
  }

  function undoLayout() {
    if (undoStack.length === 0) return;

    const previous = undoStack[undoStack.length - 1];
    undoStack = undoStack.slice(0, undoStack.length - 1);
    redoStack = pushHistorySnapshot(redoStack, layoutState);
    layoutState = cloneLayoutState(previous);
    noteLayoutMutation();
    saveLayout(layoutState);

    if (selectedWidgetId && !layoutHasWidget(layoutState, selectedWidgetId)) {
      selectedWidgetId = null;
    }

    status = 'undo';
  }

  function redoLayout() {
    if (redoStack.length === 0) return;

    const next = redoStack[redoStack.length - 1];
    redoStack = redoStack.slice(0, redoStack.length - 1);
    undoStack = pushHistorySnapshot(undoStack, layoutState);
    layoutState = cloneLayoutState(next);
    noteLayoutMutation();
    saveLayout(layoutState);

    if (selectedWidgetId && !layoutHasWidget(layoutState, selectedWidgetId)) {
      selectedWidgetId = null;
    }

    status = 'redo';
  }

  function setActiveWidgets(nextWidgets: DashboardWidget[]) {
    if (!activeTab) return;
    const normalized = normalizeContainerChildren(nextWidgets);
    const nextTabs = layoutState.tabs.map((tab) =>
      tab.id === activeTab.id ? { ...tab, widgets: normalized } : tab
    );
    commitLayout({ ...layoutState, tabs: nextTabs });
  }

  type GridNormalizationSpec = {
    parent: WidgetLayout;
    columns: number;
    rows: number;
    xEdges: number[];
    yEdges: number[];
  };

  function buildAxisEdges(start: number, span: number, count: number): number[] {
    const safeCount = Math.max(1, Math.floor(count));
    const edges: number[] = [];
    for (let index = 0; index <= safeCount; index++) {
      edges.push(start + (index * span) / safeCount);
    }
    return edges;
  }

  function resolveGridNormalizationSpec(parent: DashboardWidget): GridNormalizationSpec | null {
    if (parent.kind !== 'layout_grid') return null;
    const parentLayout = clampLayout(parent.layout, gridColumns);
    const gridConfig = readLayoutGridConfig(parent.config, parentLayout.w, parentLayout.h);
    const columns = gridConfig.autoSize
      ? Math.max(1, Math.min(96, Math.round(parentLayout.w)))
      : Math.max(1, Math.floor(gridConfig.columns));
    const rows = gridConfig.autoSize
      ? Math.max(1, Math.min(96, Math.round(parentLayout.h)))
      : Math.max(1, Math.floor(gridConfig.rows));

    return {
      parent: parentLayout,
      columns,
      rows,
      xEdges: buildAxisEdges(parentLayout.x, parentLayout.w, columns),
      yEdges: buildAxisEdges(parentLayout.y, parentLayout.h, rows)
    };
  }

  function axisCellFromAbsoluteRange(
    rangeStart: number,
    rangeEnd: number,
    edges: number[]
  ): { index: number; span: number } {
    const count = Math.max(1, edges.length - 1);
    const start = Math.min(rangeStart, rangeEnd);
    const end = Math.max(rangeStart, rangeEnd);
    const epsilon = 1e-6;
    let first = -1;
    let last = -1;

    for (let idx = 0; idx < count; idx++) {
      const cellStart = edges[idx];
      const cellEnd = edges[idx + 1];
      if (end <= cellStart + epsilon || start >= cellEnd - epsilon) continue;
      if (first < 0) first = idx + 1;
      last = idx + 1;
    }

    if (first < 0) {
      const center = (start + end) / 2;
      let nearest = 1;
      let nearestDistance = Number.POSITIVE_INFINITY;
      for (let idx = 0; idx < count; idx++) {
        const cellStart = edges[idx];
        const cellEnd = edges[idx + 1];
        const cellCenter = (cellStart + cellEnd) / 2;
        const distance = Math.abs(center - cellCenter);
        if (distance < nearestDistance) {
          nearestDistance = distance;
          nearest = idx + 1;
        }
      }
      first = nearest;
      last = nearest;
    }

    return {
      index: first,
      span: Math.max(1, last - first + 1)
    };
  }

  function axisAbsoluteRangeFromCell(
    index: number,
    span: number,
    edges: number[]
  ): { start: number; span: number } {
    const count = Math.max(1, edges.length - 1);
    const clampedIndex = Math.max(1, Math.min(count, Math.floor(index)));
    const maxSpan = count - clampedIndex + 1;
    const clampedSpan = Math.max(1, Math.min(maxSpan, Math.floor(span)));
    const start = edges[clampedIndex - 1];
    const end = edges[clampedIndex + clampedSpan - 1];
    return {
      start,
      span: Math.max(1e-6, end - start)
    };
  }

  function snapLayoutToGridContainer(parent: DashboardWidget, layout: WidgetLayout): WidgetLayout {
    const spec = resolveGridNormalizationSpec(parent);
    if (!spec) return clampLayout(layout, gridColumns);

    const maxX = spec.parent.x + spec.parent.w;
    const maxY = spec.parent.y + spec.parent.h;
    const xStart = Math.max(spec.parent.x, Math.min(maxX, layout.x));
    const yStart = Math.max(spec.parent.y, Math.min(maxY, layout.y));
    const xEnd = Math.max(
      xStart,
      Math.min(maxX, layout.x + layout.w)
    );
    const yEnd = Math.max(
      yStart,
      Math.min(maxY, layout.y + layout.h)
    );

    const col = axisCellFromAbsoluteRange(xStart, xEnd, spec.xEdges);
    const row = axisCellFromAbsoluteRange(yStart, yEnd, spec.yEdges);

    const xRange = axisAbsoluteRangeFromCell(col.index, col.span, spec.xEdges);
    const yRange = axisAbsoluteRangeFromCell(row.index, row.span, spec.yEdges);

    return clampLayout(
      {
        x: xRange.start,
        y: yRange.start,
        w: xRange.span,
        h: yRange.span
      },
      gridColumns
    );
  }

  function normalizeContainerChildren(nextWidgets: DashboardWidget[]): DashboardWidget[] {
    if (nextWidgets.length === 0) return nextWidgets;

    const normalizedById = new Map<string, DashboardWidget>();
    for (const widget of nextWidgets) {
      normalizedById.set(widget.id, {
        ...widget,
        layout: clampLayout(widget.layout, gridColumns)
      });
    }

    const isContainerKind = (kind: WidgetKind): boolean =>
      kind === 'layout_list' || kind === 'layout_grid' || kind === 'layout_accordion';

    // Drop orphaned/invalid parent references before normalization.
    for (const [id, widget] of normalizedById) {
      const parentId = widget.parentLayoutId;
      if (!parentId) continue;
      const parent = normalizedById.get(parentId);
      if (!parent || !isContainerKind(parent.kind) || parentId === id) {
        normalizedById.set(id, {
          ...widget,
          parentLayoutId: undefined
        });
      }
    }

    // Break cycles to avoid unstable ordering/positioning loops.
    for (const [id, widget] of normalizedById) {
      if (!widget.parentLayoutId) continue;
      const seen = new Set<string>([id]);
      let cursor = widget.parentLayoutId;
      let cyclic = false;
      while (cursor) {
        if (seen.has(cursor)) {
          cyclic = true;
          break;
        }
        seen.add(cursor);
        const parent = normalizedById.get(cursor);
        cursor = parent?.parentLayoutId;
      }
      if (cyclic) {
        normalizedById.set(id, {
          ...widget,
          parentLayoutId: undefined
        });
      }
    }

    const depthCache = new Map<string, number>();
    const depthVisiting = new Set<string>();
    const hierarchyDepth = (widgetId: string): number => {
      const cached = depthCache.get(widgetId);
      if (cached !== undefined) return cached;
      if (depthVisiting.has(widgetId)) return 0;
      depthVisiting.add(widgetId);
      const widget = normalizedById.get(widgetId);
      const parentId = widget?.parentLayoutId;
      let depth = 0;
      if (parentId && normalizedById.has(parentId)) {
        depth = hierarchyDepth(parentId) + 1;
      }
      depthVisiting.delete(widgetId);
      depthCache.set(widgetId, depth);
      return depth;
    };

    const containerIds = nextWidgets
      .filter(
        (widget) =>
          widget.kind === 'layout_list' || widget.kind === 'layout_grid' || widget.kind === 'layout_accordion'
      )
      .map((widget) => widget.id)
      .sort((a, b) => {
        const depthDiff = hierarchyDepth(a) - hierarchyDepth(b);
        if (depthDiff !== 0) return depthDiff;
        const layoutA = normalizedById.get(a)?.layout ?? { x: 1, y: 1, w: 1, h: 1 };
        const layoutB = normalizedById.get(b)?.layout ?? { x: 1, y: 1, w: 1, h: 1 };
        return layoutA.y - layoutB.y || layoutA.x - layoutB.x || a.localeCompare(b);
      });

    for (const containerId of containerIds) {
      const parent = normalizedById.get(containerId);
      if (!parent || !isContainerKind(parent.kind)) continue;

      const children = Array.from(normalizedById.values())
        .filter((widget) => widget.parentLayoutId === parent.id)
        .sort(
          (a, b) =>
            a.layout.y - b.layout.y ||
            a.layout.x - b.layout.x ||
            a.id.localeCompare(b.id)
        );

      if (parent.kind === 'layout_accordion') {
        const [slotChild, ...overflow] = children;
        if (slotChild) {
          normalizedById.set(slotChild.id, {
            ...slotChild,
            parentLayoutId: parent.id
          });
        }
        for (const extra of overflow) {
          normalizedById.set(extra.id, {
            ...extra,
            parentLayoutId: undefined
          });
        }
        continue;
      }

      for (const child of children) {
        normalizedById.set(child.id, {
          ...child,
          parentLayoutId: parent.id
        });
      }
    }

    return nextWidgets.map((widget) => normalizedById.get(widget.id) ?? widget);
  }

  function updateHistory(signals: SignalRow[], trackedSignalIds: Set<number>) {
    if (trackedSignalIds.size === 0) {
      if (historyBySignal.size > 0) {
        historyBySignal = new Map();
      }
      return;
    }

    const byId = new Map<number, SignalRow>();
    for (const signal of signals) {
      byId.set(signal.signal_id, signal);
    }

    const next = new Map(historyBySignal);

    for (const signalId of [...next.keys()]) {
      if (!trackedSignalIds.has(signalId)) {
        next.delete(signalId);
      }
    }

    for (const signalId of trackedSignalIds) {
      const signal = byId.get(signalId);
      if (!signal) {
        next.delete(signalId);
        continue;
      }
      const numeric = parseNumericSignal(signal);
      if (numeric === null) continue;

      const values = [...(next.get(signalId) ?? [])];
      values.push(numeric);
      if (values.length > 90) {
        values.splice(0, values.length - 90);
      }
      next.set(signalId, values);
    }

    historyBySignal = next;
  }

  const TOPIC_HARDWARE_PRIORITY: WidgetKind[] = [
    'state_machine',
    'dio',
    'motor',
    'encoder',
    'imu'
  ];

  function normalizeTopicPath(path: string): string {
    const trimmed = path.trim();
    if (!trimmed) return '';
    const withoutLeadingSlash = trimmed.replace(/^\/+/, '');
    const withoutAthenaRoot = withoutLeadingSlash.replace(/^athena\/?/i, '');
    const withoutTrailingSlash = withoutAthenaRoot.replace(/\/+$/, '');
    return withoutTrailingSlash ? `/${withoutTrailingSlash}` : '';
  }

  function isSignalInTopic(signalPath: string, topicPath: string): boolean {
    if (!topicPath) return false;
    const normalizedSignalPath = normalizeTopicPath(signalPath);
    if (!normalizedSignalPath) return false;
    return (
      normalizedSignalPath === topicPath || normalizedSignalPath.startsWith(`${topicPath}/`)
    );
  }

  function resolveTopicDrop(
    topicPathRaw: string
  ): { signal: SignalRow; kind: WidgetKind; title: string } | null {
    const topicPath = normalizeTopicPath(topicPathRaw);
    if (!topicPath) return null;

    const scopedSignals = signalRows.filter((signal) => isSignalInTopic(signal.path, topicPath));
    if (scopedSignals.length === 0) return null;

    const topicLabel = leafPath(topicPath);
    const hardwareKindFromTopic = hardwareWidgetKindForPath(topicPath);
    if (hardwareKindFromTopic) {
      const anchorSignal =
        scopedSignals.find(
          (signal) => hardwareWidgetKindForPath(signal.path) === hardwareKindFromTopic
        ) ?? scopedSignals[0];
      return {
        signal: anchorSignal,
        kind: hardwareKindFromTopic,
        title: topicLabel
      };
    }

    for (const preferredKind of TOPIC_HARDWARE_PRIORITY) {
      const anchorSignal = scopedSignals.find(
        (signal) => hardwareWidgetKindForPath(signal.path) === preferredKind
      );
      if (!anchorSignal) continue;
      return {
        signal: anchorSignal,
        kind: preferredKind,
        title: topicLabel
      };
    }

    const fallbackSignal = scopedSignals.find((signal) => !isWritableSignal(signal)) ?? scopedSignals[0];
    return {
      signal: fallbackSignal,
      kind: defaultLeafWidgetKind(fallbackSignal),
      title: topicLabel
    };
  }

  function createWidget(
    signal: SignalRow,
    kind: WidgetKind,
    titleOverride = '',
    preferredPosition?: Pick<WidgetLayout, 'x' | 'y'>,
    parentLayoutId?: string | null
  ) {
    const config = buildDefaultWidgetConfig(kind, signal, signalRows);
    let nextWidget = makeWidget(
      signal,
      kind,
      widgets,
      titleOverride,
      gridColumns,
      preferredPosition,
      parentLayoutId ?? undefined,
      config
    );

    const parentLayout =
      parentLayoutId === null || parentLayoutId === undefined
        ? null
        : widgets.find((entry) => entry.id === parentLayoutId) ?? null;
    const normalizedPreferredPosition = preferredPosition
      ? parentLayout?.kind === 'layout_grid'
        ? preferredPosition
        : {
            x: Math.max(1, Math.round(preferredPosition.x)),
            y: Math.max(1, Math.round(preferredPosition.y))
          }
      : undefined;

    // For direct drag-drop placement, honor the resolved drop position instead of global auto-placement.
    if (normalizedPreferredPosition) {
      nextWidget = {
        ...nextWidget,
        layout: clampLayout(
          {
            ...nextWidget.layout,
            x: normalizedPreferredPosition.x,
            y: normalizedPreferredPosition.y
          },
          gridColumns
        )
      };
    }

    setActiveWidgets([nextWidget, ...widgets]);
    widgetInputs = {
      ...widgetInputs,
      [nextWidget.id]: signal.value === '-' ? '' : signal.value
    };
    selectedWidgetId = nextWidget.id;
    selectedId = signal.signal_id;
  }

  function showAsWidgetKind(kind: WidgetKind) {
    if (!selectedSignal) return;

    if (!selectedWidget) {
      createWidget(selectedSignal, kind);
      return;
    }

    const anchorSignal = signalById.get(selectedWidget.signalId) ?? selectedSignal;
    const nextConfig = buildDefaultWidgetConfig(kind, anchorSignal, signalRows);

    setActiveWidgets(
      widgets.map((widget) =>
        widget.id === selectedWidget.id
          ? {
              ...widget,
              kind,
              config: cloneWidgetConfig(nextConfig)
            }
          : widget
      )
    );
    selectedId = anchorSignal.signal_id;
  }

  function createLayoutWidget(
    kind: LayoutToolKind,
    preferredPosition?: Pick<WidgetLayout, 'x' | 'y'>,
    parentLayoutId?: string | null
  ) {
    let nextWidget = makeLayoutWidget(
      kind,
      widgets,
      '',
      gridColumns,
      preferredPosition,
      undefined,
      parentLayoutId ?? undefined
    );

    const parentLayout =
      parentLayoutId === null || parentLayoutId === undefined
        ? null
        : widgets.find((entry) => entry.id === parentLayoutId) ?? null;
    const normalizedPreferredPosition = preferredPosition
      ? parentLayout?.kind === 'layout_grid'
        ? preferredPosition
        : {
            x: Math.max(1, Math.round(preferredPosition.x)),
            y: Math.max(1, Math.round(preferredPosition.y))
          }
      : undefined;

    if (normalizedPreferredPosition) {
      nextWidget = {
        ...nextWidget,
        layout: clampLayout(
          {
            ...nextWidget.layout,
            x: normalizedPreferredPosition.x,
            y: normalizedPreferredPosition.y
          },
          gridColumns
        )
      };
    }

    setActiveWidgets([nextWidget, ...widgets]);
    selectedWidgetId = nextWidget.id;
    selectedId = null;
  }

  function copySelectedWidget(): boolean {
    if (!selectedWidgetId) return false;

    const widget = widgets.find((entry) => entry.id === selectedWidgetId);
    if (!widget) return false;

    copiedWidget = {
      kind: widget.kind,
      signalId: widget.signalId,
      title: widget.title,
      layout: { ...widget.layout },
      parentLayoutId: widget.parentLayoutId,
      config: cloneWidgetConfig(widget.config),
      inputValue: widgetInputs[widget.id] ?? null
    };

    status = `copied ${widget.title}`;
    return true;
  }

  function closeDashboardMenu() {
    dashboardMenu = null;
  }

  function dashboardMenuStyle(menu: DashboardContextMenuState): string {
    const viewportWidth = typeof window === 'undefined' ? 1920 : window.innerWidth;
    const viewportHeight = typeof window === 'undefined' ? 1080 : window.innerHeight;
    const x = Math.min(menu.x, viewportWidth - 236);
    const y = Math.min(menu.y, viewportHeight - 276);
    return `left:${Math.max(8, x)}px;top:${Math.max(8, y)}px;`;
  }

  function openDashboardMenu(event: MouseEvent) {
    if (railSection !== 'dashboards') return;

    const target = event.target as HTMLElement | null;
    if (!target) return;
    if (target.closest('.widget-card')) return;
    if (target.closest('.widget-context-menu')) return;
    if (target.closest('.floating-dock')) return;
    if (target.closest('.explorer-fab')) return;

    event.preventDefault();
    dashboardMenu = {
      x: event.clientX,
      y: event.clientY
    };
  }

  function openDataExplorerFromMenu() {
    pendingSignalMapRequest = null;
    showExplorer = true;
    query = '';
    closeDashboardMenu();
  }

  function closeExplorerDock() {
    showExplorer = false;
    pendingSignalMapRequest = null;
  }

  function requestSignalMapFromExplorer(request: SignalMapRequest) {
    pendingSignalMapRequest = request;
    showExplorer = true;
    query = '';
    if (selectedWidgetId !== null && selectedId === null) {
      const widget = widgets.find((entry) => entry.id === selectedWidgetId);
      if (widget && widget.signalId > 0) {
        selectedId = widget.signalId;
      }
    }
  }

  function openLayoutToolsFromMenu() {
    showLayoutTools = true;
    layoutToolQuery = '';
    closeDashboardMenu();
  }

  function addPageFromMenu() {
    addTab();
    closeDashboardMenu();
  }

  function renamePageFromMenu() {
    if (!activeTab) return;
    const nextName = window.prompt('Rename current page', activeTab.name);
    if (nextName === null) return;
    const trimmed = nextName.trim();
    if (!trimmed) return;
    renameTab(activeTab.id, trimmed);
    closeDashboardMenu();
  }

  function clearLayoutFromMenu() {
    clearWidgets();
    closeDashboardMenu();
  }

  function suggestedLayoutFilename(): string {
    const now = new Date();
    const stamp = now
      .toISOString()
      .replace(/[:]/g, '-')
      .replace(/\..+$/, '');
    return `arcp-ui-${stamp}.json`;
  }

  function downloadJsonFile(filename: string, contents: string) {
    const blob = new Blob([contents], { type: 'application/json;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement('a');
    anchor.href = url;
    anchor.download = filename;
    anchor.style.display = 'none';
    document.body.appendChild(anchor);
    anchor.click();
    document.body.removeChild(anchor);
    setTimeout(() => URL.revokeObjectURL(url), 0);
  }

  function downloadBinaryFile(filename: string, bytes: Uint8Array) {
    const blob = new Blob([bytes], { type: 'application/octet-stream' });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement('a');
    anchor.href = url;
    anchor.download = filename;
    anchor.style.display = 'none';
    document.body.appendChild(anchor);
    anchor.click();
    document.body.removeChild(anchor);
    setTimeout(() => URL.revokeObjectURL(url), 0);
  }

  async function pickJsonFileText(): Promise<string | null> {
    return new Promise((resolve, reject) => {
      const input = document.createElement('input');
      input.type = 'file';
      input.accept = '.json,application/json';
      input.style.display = 'none';
      document.body.appendChild(input);

      const cleanup = () => {
        input.removeEventListener('change', onChange);
        document.body.removeChild(input);
      };

      const onChange = async () => {
        try {
          const file = input.files?.[0];
          if (!file) {
            cleanup();
            resolve(null);
            return;
          }
          const text = await file.text();
          cleanup();
          resolve(text);
        } catch (err) {
          cleanup();
          reject(err);
        }
      };

      input.addEventListener('change', onChange, { once: true });
      input.click();
    });
  }

  async function exportLayoutToFile() {
    try {
      const serialized = serializeLayoutExport(layoutState);
      downloadJsonFile(suggestedLayoutFilename(), serialized);
      status = `ui saved (${layoutState.tabs.length} tabs)`;
      lastError = '';
    } catch (err) {
      lastError = `save failed: ${String(err)}`;
    }
  }

  async function importLayoutFromFile() {
    try {
      const raw = await pickJsonFileText();
      if (!raw) return;

      const hasExistingWidgets = layoutState.tabs.some((tab) => tab.widgets.length > 0);
      if (hasExistingWidgets) {
        const confirmed = window.confirm(
          'Load UI JSON and replace the current layout? This action can be undone with Ctrl+Z.'
        );
        if (!confirmed) return;
      }

      const imported = parseLayoutImport(raw);
      commitLayout(imported, { recordHistory: true, clearRedo: true });
      selectedWidgetId = null;
      selectedId = null;
      widgetInputs = {};
      status = `ui loaded (${imported.tabs.length} tabs)`;
      lastError = '';
    } catch (err) {
      lastError = `load failed: ${String(err)}`;
    }
  }

  function importLayoutFromMenu() {
    closeDashboardMenu();
    void importLayoutFromFile();
  }

  function exportLayoutFromMenu() {
    closeDashboardMenu();
    void exportLayoutToFile();
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
    selectedRemoteLogPath = '';
    selectedRemoteLogPreview = '';
    selectedRemoteLogPreviewTruncated = false;
    remoteLogDownloadInFlightPath = null;
    remoteLogPreviewInFlight = false;
  }

  function clearRemoteLogs() {
    remoteLogs = [];
    remoteLogsResolvedHost = '';
    remoteLogError = '';
    robotLinkProbe = null;
    clearRemoteLogSelection();
  }

  function setRemoteLogLiveFollow(enabled: boolean) {
    remoteLogLiveFollow = enabled;
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
        selectedRemoteLogPreview = '';
        selectedRemoteLogPreviewTruncated = false;
      }
      if (selectedRemoteLogPath) {
        await loadRemoteLogPreview(selectedRemoteLogPath, { silent: true });
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
      selectedRemoteLogPreview = preview.content;
      selectedRemoteLogPreviewTruncated = preview.truncated;
      if (!options?.silent) {
        lastError = '';
      }
    } catch (err) {
      selectedRemoteLogPreview = '';
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

      commitLayout(imported, { recordHistory: true, clearRedo: true });
      selectedWidgetId = null;
      selectedId = null;
      widgetInputs = {};
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
    handleGridColumnsChange(columns);
  }

  function updateDefaultGridDensity(value: string) {
    const density = parseGridDensity(value);
    defaultGridDensity = density;
    saveGridPreferences(defaultGridColumns, defaultGridDensity);
  }

  function pasteCopiedWidget(): boolean {
    if (!copiedWidget) return false;

    const preferred = {
      x: copiedWidget.layout.x + 1,
      y: copiedWidget.layout.y + 1
    };

    if (isLayoutWidgetKind(copiedWidget.kind)) {
      const nextWidget = makeLayoutWidget(
        copiedWidget.kind,
        widgets,
        copiedWidget.title,
        gridColumns,
        preferred,
        cloneWidgetConfig(copiedWidget.config)
      );
      setActiveWidgets([nextWidget, ...widgets]);
      selectedWidgetId = nextWidget.id;
      selectedId = null;
      status = `pasted ${nextWidget.title}`;
      return true;
    }

    const signal = signalById.get(copiedWidget.signalId);
    if (!signal) {
      lastError = 'cannot paste: source signal is unavailable';
      return false;
    }

    const nextWidget = makeWidget(
      signal,
      copiedWidget.kind,
      widgets,
      copiedWidget.title,
      gridColumns,
      preferred,
      copiedWidget.parentLayoutId,
      cloneWidgetConfig(copiedWidget.config)
    );

    setActiveWidgets([nextWidget, ...widgets]);
    selectedWidgetId = nextWidget.id;
    selectedId = signal.signal_id;
    widgetInputs = {
      ...widgetInputs,
      [nextWidget.id]: copiedWidget.inputValue ?? signal.value
    };
    status = `pasted ${nextWidget.title}`;
    return true;
  }

  function removeWidget(widgetId: string) {
    const removingLayout = widgets.find((widget) => widget.id === widgetId);
    const nextInputs = { ...widgetInputs };
    delete nextInputs[widgetId];
    widgetInputs = nextInputs;

    const filtered = widgets
      .filter((widget) => widget.id !== widgetId)
      .map((widget) => {
        if (!removingLayout || !isLayoutWidgetKind(removingLayout.kind)) {
          return widget;
        }
        if (widget.parentLayoutId !== widgetId) return widget;
        return {
          ...widget,
          parentLayoutId: undefined
        };
      });

    setActiveWidgets(filtered);
    if (selectedWidgetId === widgetId) {
      selectedWidgetId = null;
    }
  }

  function patchWidgetLayout(
    widgetId: string,
    layout: WidgetLayout,
    parentLayoutId?: string | null
  ) {
    const moving = widgets.find((widget) => widget.id === widgetId);
    if (!moving) return;

    const clamped = clampLayout(layout, gridColumns);
    const dx = clamped.x - moving.layout.x;
    const dy = clamped.y - moving.layout.y;
    const hasDelta = dx !== 0 || dy !== 0;
    const widgetsById = new Map(widgets.map((widget) => [widget.id, widget]));

    const isDescendantOfMoving = (candidate: DashboardWidget): boolean => {
      let cursor = candidate.parentLayoutId;
      while (cursor) {
        if (cursor === widgetId) return true;
        const parent = widgetsById.get(cursor);
        cursor = parent?.parentLayoutId;
      }
      return false;
    };

    setActiveWidgets(
      widgets.map((widget) => {
        if (widget.id === widgetId) {
          return {
            ...widget,
            layout: clamped,
            parentLayoutId: parentLayoutId === undefined ? widget.parentLayoutId : parentLayoutId ?? undefined
          };
        }

        if (isLayoutWidgetKind(moving.kind) && hasDelta && isDescendantOfMoving(widget)) {
          return {
            ...widget,
            layout: clampLayout(
              {
                ...widget.layout,
                x: widget.layout.x + dx,
                y: widget.layout.y + dy
              },
              gridColumns
            )
          };
        }

        return widget;
      })
    );
  }

  function updateWidgetConfig(widgetId: string, config: WidgetConfigRecord | undefined) {
    const target = widgets.find((widget) => widget.id === widgetId);
    if (!target) return;

    const nextConfig = cloneWidgetConfig(config);
    if (target.kind !== 'layout_accordion') {
      setActiveWidgets(
        widgets.map((widget) =>
          widget.id === widgetId
            ? {
                ...widget,
                config: nextConfig
              }
            : widget
        )
      );
      return;
    }

    const nextConfigRecord =
      nextConfig && typeof nextConfig === 'object' && !Array.isArray(nextConfig)
        ? nextConfig
        : {};
    const previousAccordion = readLayoutAccordionConfig(target.config);
    const nextAccordion = readLayoutAccordionConfig(nextConfigRecord);
    const previousCollapsed = previousAccordion.collapsed;
    const nextCollapsed = nextAccordion.collapsed;
    const currentRows = Math.max(1, Math.round(target.layout.h));

    let expandedRows = nextAccordion.expandedRows ?? previousAccordion.expandedRows;
    if (expandedRows === null) {
      expandedRows = previousCollapsed
        ? Math.max(DEFAULT_ACCORDION_EXPANDED_ROWS, currentRows)
        : currentRows;
    }
    expandedRows = Math.max(1, Math.min(96, Math.round(expandedRows)));

    let nextLayout = clampLayout(target.layout, gridColumns);
    if (!previousCollapsed && nextCollapsed) {
      expandedRows = Math.max(1, currentRows);
      nextLayout = clampLayout(
        {
          ...nextLayout,
          h: ACCORDION_COLLAPSED_ROWS
        },
        gridColumns
      );
    } else if (previousCollapsed && !nextCollapsed) {
      expandedRows = Math.max(
        ACCORDION_COLLAPSED_ROWS,
        Math.min(96, Math.round(nextAccordion.expandedRows ?? previousAccordion.expandedRows ?? DEFAULT_ACCORDION_EXPANDED_ROWS))
      );
      nextLayout = clampLayout(
        {
          ...nextLayout,
          h: expandedRows
        },
        gridColumns
      );
    } else if (!nextCollapsed) {
      expandedRows = Math.max(ACCORDION_COLLAPSED_ROWS, currentRows);
    }

    const patchedConfig: WidgetConfigRecord = {
      ...nextConfigRecord,
      collapsed: nextCollapsed,
      expandedRows
    };

    setActiveWidgets(
      widgets.map((widget) =>
        widget.id === widgetId
          ? {
              ...widget,
              layout: nextLayout,
              config: patchedConfig
            }
          : widget
      )
    );
  }

  function updateWidgetTitle(widgetId: string, title: string) {
    const nextTitle = title.trim();
    if (!nextTitle) return;
    setActiveWidgets(
      widgets.map((widget) =>
        widget.id === widgetId
          ? {
              ...widget,
              title: nextTitle
            }
          : widget
      )
    );
  }

  function clearWidgets() {
    if (widgets.length === 0) return;
    const confirmed = window.confirm(
      `Clear all widgets from this dashboard tab? (${widgets.length} item${widgets.length === 1 ? '' : 's'})`
    );
    if (!confirmed) return;

    selectedWidgetId = null;
    setActiveWidgets([]);
  }

  function setWidgetInput(widgetId: string, value: string) {
    widgetInputs = { ...widgetInputs, [widgetId]: value };
  }

  function handleGridColumnsChange(columns: number) {
    if (!Number.isFinite(columns) || columns < 1 || columns === gridColumns) {
      return;
    }
    gridColumns = columns;

    if (!activeTab || activeTab.widgets.length === 0) {
      return;
    }

    const nextWidgets = activeTab.widgets.map((widget) => ({
      ...widget,
      layout: clampLayout(widget.layout, columns)
    }));

    const changed = nextWidgets.some((widget, index) => {
      const prev = activeTab.widgets[index];
      return (
        widget.layout.x !== prev.layout.x ||
        widget.layout.y !== prev.layout.y ||
        widget.layout.w !== prev.layout.w ||
        widget.layout.h !== prev.layout.h
      );
    });

    if (changed) {
      setActiveWidgets(nextWidgets);
    }
  }

  function addTab() {
    const tab = createEmptyTab(`Dashboard ${layoutState.tabs.length + 1}`);
    commitLayout({
      activeTabId: tab.id,
      tabs: [...layoutState.tabs, tab]
    });
    selectedWidgetId = null;
    selectedId = null;
  }

  function removeTab(tabId: string) {
    if (layoutState.tabs.length <= 1) return;

    const nextTabs = layoutState.tabs.filter((tab) => tab.id !== tabId);
    const nextActive =
      layoutState.activeTabId === tabId ? nextTabs[0].id : layoutState.activeTabId;

    commitLayout({
      activeTabId: nextActive,
      tabs: nextTabs
    });

    selectedWidgetId = null;
  }

  function renameTab(tabId: string, name: string) {
    const nextTabs = layoutState.tabs.map((tab) => (tab.id === tabId ? { ...tab, name } : tab));
    commitLayout({ ...layoutState, tabs: nextTabs });
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

      if (!next.connected && next.signals.length === 0 && previous && !next.full_snapshot) {
        snapshot = {
          ...previous,
          connected: false,
          status: next.status,
          signal_count: next.signal_count,
          update_count: next.update_count,
          uptime_ms: next.uptime_ms,
          server_cpu_percent: next.server_cpu_percent,
          server_rss_bytes: next.server_rss_bytes,
          host_cpu_percent: next.host_cpu_percent,
          host_rss_bytes: next.host_rss_bytes
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

        snapshot = {
          connected: next.connected,
          status: next.status,
          signal_count: next.signal_count,
          update_count: next.update_count,
          uptime_ms: next.uptime_ms,
          server_cpu_percent: next.server_cpu_percent,
          server_rss_bytes: next.server_rss_bytes,
          host_cpu_percent: next.host_cpu_percent,
          host_rss_bytes: next.host_rss_bytes,
          signals: nextSignals
        };
      }

      connected = next.connected;
      status = next.status;
      needsSessionReconnect = false;

      const filteredSignals = (snapshot?.signals ?? []).filter((signal) => !isIgnoredPublishSignal(signal));
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
            commitLayout({ ...layoutState, tabs: nextTabs }, { recordHistory: false, clearRedo: false });
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

  function openInspectorForSelection() {
    if (selectedWidgetId !== null && selectedId === null) {
      const widget = widgets.find((entry) => entry.id === selectedWidgetId);
      if (widget && widget.signalId > 0) {
        selectedId = widget.signalId;
      }
    }
    showInspector = true;
  }

  function shouldIgnoreGlobalHotkeys(target: EventTarget | null): boolean {
    const el = target as HTMLElement | null;
    if (!el) return false;
    const tag = el.tagName.toLowerCase();
    if (tag === 'input' || tag === 'textarea' || tag === 'select' || tag === 'button') {
      return true;
    }
    return el.isContentEditable;
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

    if (key === 'escape' && dashboardMenu) {
      event.preventDefault();
      closeDashboardMenu();
      return;
    }

    if (event.defaultPrevented || shouldIgnoreGlobalHotkeys(event.target)) {
      return;
    }

    if (hasAccel && key === 's' && !event.shiftKey && !event.altKey) {
      event.preventDefault();
      void exportLayoutToFile();
      return;
    }

    if (hasAccel && key === 'o' && !event.shiftKey && !event.altKey) {
      event.preventDefault();
      void importLayoutFromFile();
      return;
    }

    if (hasAccel && key === 'c') {
      event.preventDefault();
      copySelectedWidget();
      return;
    }

    if (hasAccel && key === 'v') {
      event.preventDefault();
      pasteCopiedWidget();
      return;
    }

    if (key === 'z' && !event.altKey && hasAccel) {
      event.preventDefault();
      if (event.shiftKey) {
        redoLayout();
      } else {
        undoLayout();
      }
      return;
    }

    if ((event.key === 'Delete' || event.key === 'Backspace') && selectedWidgetId) {
      event.preventDefault();
      removeWidget(selectedWidgetId);
      return;
    }

    if (key === 'i' && (selectedWidgetId !== null || selectedId !== null)) {
      event.preventDefault();
      openInspectorForSelection();
    }
  }

  onMount(() => {
    const gridPreferences = loadGridPreferences();
    defaultGridColumns = gridPreferences.columns;
    defaultGridDensity = gridPreferences.density;
    gridColumns = gridPreferences.columns;
    layoutState = loadLayout();
    undoStack = [];
    redoStack = [];
    void refreshWindowModes();
    if (Number(controlPort) > 0) {
      void connect();
    }
    const onFullscreenChange = () => {
      void refreshWindowModes();
    };
    const onPointerDown = (event: PointerEvent) => {
      const target = event.target as HTMLElement | null;
      if (target?.closest('.dashboard-context-menu')) return;
      closeDashboardMenu();
    };
    let refreshCancelled = false;
    let refreshTimer: ReturnType<typeof setTimeout> | null = null;
    const refreshDelayMs = () => {
      if (!connected) return REFRESH_MS_DISCONNECTED;
      return railSection === 'dashboards' ? REFRESH_MS_DASHBOARDS : REFRESH_MS_OTHER;
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
    const remoteLogLiveTimer = setInterval(() => {
      if (railSection !== 'logs') return;
      if (!remoteLogLiveFollow) return;
      if (!selectedRemoteLogPath || remoteLogPreviewInFlight) return;
      void loadRemoteLogPreview(selectedRemoteLogPath, { silent: true });
    }, REMOTE_LOG_LIVE_POLL_MS);
    window.addEventListener('keydown', onGlobalKeyDown);
    window.addEventListener('pointerdown', onPointerDown);
    document.addEventListener('fullscreenchange', onFullscreenChange);
    return () => {
      refreshCancelled = true;
      if (refreshTimer !== null) {
        clearTimeout(refreshTimer);
      }
      clearInterval(serverLayoutTimer);
      clearInterval(robotLinkProbeTimer);
      clearInterval(remoteLogLiveTimer);
      if (replayState?.timerId !== null) {
        window.clearInterval(replayState.timerId);
      }
      window.removeEventListener('keydown', onGlobalKeyDown);
      window.removeEventListener('pointerdown', onPointerDown);
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
      onLoadLayoutFromHost={() => void importLayoutFromFile()}
      onLoadLayoutFromServer={() => void loadLayoutFromServerProfile()}
      onSaveLayoutToHost={() => void exportLayoutToFile()}
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
      {#if railSection === 'dashboards'}
        <WorkspaceTabs
          tabs={tabSummaries}
          activeTabId={layoutState.activeTabId}
          onSelectTab={(tabId) => {
            layoutState = { ...layoutState, activeTabId: tabId };
            selectedWidgetId = null;
            selectedId = null;
            noteLayoutMutation();
            saveLayout(layoutState);
          }}
          onAddTab={addTab}
          onRemoveTab={removeTab}
          onRenameTab={renameTab}
        />
      {/if}

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
              onAddSignal={(signal) => createWidget(signal, defaultWidgetKind(signal))}
              {leafPath}
            />
            {#if showInspector}
              <InspectorPanel
                signals={signalRows}
                {signalById}
                {selectedSignal}
                {selectedWidget}
                widgetKinds={widgetKindsFor(selectedSignal)}
                {widgetKindLabel}
                onShowAsKind={showAsWidgetKind}
                onTriggerAction={sendAction}
                onSelectTunableWidget={() => {
                  if (selectedSignal) createWidget(selectedSignal, 'tunable');
                }}
                onRemoveWidget={removeWidget}
                onUpdateWidgetTitle={updateWidgetTitle}
                onUpdateWidgetConfig={updateWidgetConfig}
                onRequestSignalMap={requestSignalMapFromExplorer}
              />
            {/if}
          </div>
        </section>
      {:else if railSection === 'actions'}
        <section class={`actions-stage ${showInspector ? '' : 'no-inspector'}`}>
          <div class="actions-layout">
            <ActionsScreen
              actions={actionSignals}
              onTriggerAction={sendAction}
              onSelectSignal={(signalId) => {
                selectedId = signalId;
                showInspector = true;
              }}
            />
            {#if showInspector}
              <InspectorPanel
                signals={signalRows}
                {signalById}
                {selectedSignal}
                {selectedWidget}
                widgetKinds={widgetKindsFor(selectedSignal)}
                {widgetKindLabel}
                onShowAsKind={showAsWidgetKind}
                onTriggerAction={sendAction}
                onSelectTunableWidget={() => {
                  if (selectedSignal) createWidget(selectedSignal, 'tunable');
                }}
                onRemoveWidget={removeWidget}
                onUpdateWidgetTitle={updateWidgetTitle}
                onUpdateWidgetConfig={updateWidgetConfig}
                onRequestSignalMap={requestSignalMapFromExplorer}
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
          preview={selectedRemoteLogPreview}
          previewTruncated={selectedRemoteLogPreviewTruncated}
          liveFollow={remoteLogLiveFollow}
          onRefresh={() => {
            void refreshRobotLinkProbe();
            void refreshRemoteLogs(true);
          }}
          onSelectLog={(path) => {
            setRemoteLogLiveFollow(true);
            void loadRemoteLogPreview(path);
          }}
          onDownloadLog={(path) => void downloadSelectedRemoteLog(path)}
          onToggleLiveFollow={(enabled) => setRemoteLogLiveFollow(enabled)}
        />
      {:else if railSection === 'camera_tuning'}
        <CameraTuningScreen
          signals={arcpSignalRows}
          onSelectSignal={(signalId) => {
            selectedId = signalId;
            selectedWidgetId = null;
            railSection = 'signals';
            showInspector = true;
          }}
          onSendSet={sendSet}
        />
      {:else}
        <section
          class="workspace-stage"
          aria-label="Dashboard workspace"
          oncontextmenu={openDashboardMenu}
        >
          <section class="center-column">
            <WidgetCanvas
              {widgets}
              signals={signalRows}
              {signalById}
              baseGridColumns={defaultGridColumns}
              gridDensity={defaultGridDensity}
              {selectedWidgetId}
              {widgetInputs}
              {historyBySignal}
              {sparklinePath}
              {editMode}
              onSelectWidget={(widgetId, signalId) => {
                selectedWidgetId = widgetId;
                selectedId = signalId && signalId > 0 ? signalId : null;
              }}
              onToggleEditMode={() => (editMode = !editMode)}
              onClearWidgets={clearWidgets}
              onPatchWidgetLayout={patchWidgetLayout}
              onUpdateWidgetConfig={updateWidgetConfig}
              onRemoveWidget={removeWidget}
              onWidgetInput={setWidgetInput}
              onSendAction={sendAction}
              onSendSet={sendSet}
              onRequestInspector={openInspectorForSelection}
              onDropSignal={(signalId, position, parentLayoutId) => {
                const signal = signalById.get(signalId);
                if (!signal) return;
                selectedId = signalId;
                createWidget(signal, defaultLeafWidgetKind(signal), '', position, parentLayoutId ?? null);
              }}
              onDropTopic={(topicPath, position, parentLayoutId) => {
                const resolved = resolveTopicDrop(topicPath);
                if (!resolved) return;
                selectedId = resolved.signal.signal_id;
                createWidget(
                  resolved.signal,
                  resolved.kind,
                  resolved.title,
                  position,
                  parentLayoutId ?? null
                );
              }}
              onDropLayoutTool={(kind, position, parentLayoutId) => {
                createLayoutWidget(kind, position, parentLayoutId ?? null);
              }}
              onGridColumnsChange={handleGridColumnsChange}
            />
          </section>

          {#if dashboardMenu}
            <DashboardContextMenu
              style={dashboardMenuStyle(dashboardMenu)}
              canClearLayout={widgets.length > 0}
              canRenamePage={activeTab !== null}
              onImportLayout={importLayoutFromMenu}
              onExportLayout={exportLayoutFromMenu}
              onOpenDataExplorer={openDataExplorerFromMenu}
              onOpenLayoutTools={openLayoutToolsFromMenu}
              onAddPage={addPageFromMenu}
              onRenamePage={renamePageFromMenu}
              onClearLayout={clearLayoutFromMenu}
            />
          {/if}

          {#if showExplorer}
            <FloatingDock
              title={pendingSignalMapRequest ? `Data Explorer - ${pendingSignalMapRequest.title}` : 'Data Explorer'}
              side="left"
              width={370}
              height={560}
              onClose={closeExplorerDock}
            >
              {#if pendingSignalMapRequest}
                <div class="explorer-map-hint panel">
                  <strong>{pendingSignalMapRequest.title}</strong>
                  <div class="explorer-map-actions">
                    {#if pendingSignalMapRequest.allowNone}
                      <button
                        class="btn"
                        onclick={() => {
                          pendingSignalMapRequest?.onPick(null);
                          pendingSignalMapRequest = null;
                          showExplorer = false;
                          query = '';
                        }}
                      >
                        {pendingSignalMapRequest.noneLabel}
                      </button>
                    {/if}
                    <button
                      class="btn"
                      onclick={() => {
                        pendingSignalMapRequest = null;
                        query = '';
                      }}
                    >
                      Cancel mapping
                    </button>
                  </div>
                </div>
              {/if}
              <SignalBrowser
                signals={explorerSignals}
                ntSignals={pendingSignalMapRequest ? [] : filteredNtSignals}
                {selectedId}
                {query}
                onQueryInput={(value) => (query = value)}
                onSelectSignal={(signalId) => {
                  if (pendingSignalMapRequest) {
                    pendingSignalMapRequest.onPick(signalId);
                    selectedId = signalId;
                    pendingSignalMapRequest = null;
                    showExplorer = false;
                    query = '';
                    return;
                  }
                  selectedId = signalId;
                  selectedWidgetId = null;
                }}
                onAddSignal={(signal) => {
                  if (pendingSignalMapRequest) {
                    pendingSignalMapRequest.onPick(signal.signal_id);
                    selectedId = signal.signal_id;
                    pendingSignalMapRequest = null;
                    showExplorer = false;
                    query = '';
                    return;
                  }
                  createWidget(signal, defaultWidgetKind(signal));
                }}
                {leafPath}
                showHeader={false}
              />
            </FloatingDock>
          {/if}

          {#if showLayoutTools}
            <FloatingDock title="Layout Tools" side="left" width={320} height={500} onClose={() => (showLayoutTools = false)}>
              <LayoutToolPalette
                query={layoutToolQuery}
                onQueryInput={(value) => (layoutToolQuery = value)}
                onAddLayoutTool={createLayoutWidget}
                showHeader={false}
              />
            </FloatingDock>
          {/if}

          {#if showInspector}
            <FloatingDock title="Inspector" side="right" width={350} height={560} onClose={() => (showInspector = false)}>
              <InspectorPanel
                signals={signalRows}
                {signalById}
                {selectedSignal}
                {selectedWidget}
                widgetKinds={widgetKindsFor(selectedSignal)}
                {widgetKindLabel}
                showHeader={false}
                onShowAsKind={showAsWidgetKind}
                onTriggerAction={sendAction}
                onSelectTunableWidget={() => {
                  if (selectedSignal) createWidget(selectedSignal, 'tunable');
                }}
                onRemoveWidget={removeWidget}
                onUpdateWidgetTitle={updateWidgetTitle}
                onUpdateWidgetConfig={updateWidgetConfig}
                onRequestSignalMap={requestSignalMapFromExplorer}
              />
            </FloatingDock>
          {/if}

        </section>
        <button
          class="explorer-fab"
          title={showExplorer ? 'Close Explorer' : 'Open Explorer'}
          aria-label={showExplorer ? 'Close Explorer' : 'Open Explorer'}
          onclick={() => {
            if (showExplorer) {
              closeExplorerDock();
            } else {
              showExplorer = true;
            }
          }}
        >
          <FontAwesomeIcon icon={showExplorer ? faXmark : faMagnifyingGlass} class="explorer-fab-icon" />
        </button>
      {/if}
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

  .center-column {
    min-width: 0;
    height: 100%;
    display: grid;
    grid-template-rows: 1fr;
    gap: 0.45rem;
    min-height: 0;
  }

  .workspace-stage {
    position: relative;
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    flex: 1 1 auto;
    min-width: 0;
    min-height: 0;
    overflow: hidden;
  }

  .explorer-map-hint {
    padding: 0.44rem;
    border-radius: 8px;
    border: 1px solid rgba(180, 35, 45, 0.38);
    background: rgba(127, 29, 29, 0.16);
    display: grid;
    gap: 0.34rem;
  }

  .explorer-map-hint strong {
    font-size: 0.74rem;
    color: #ffe4e8;
    font-family: var(--font-display);
  }

  .explorer-map-actions {
    display: inline-flex;
    flex-wrap: wrap;
    gap: 0.3rem;
  }

  .explorer-fab {
    position: fixed;
    right: 1rem;
    bottom: 1rem;
    width: 2.9rem;
    height: 2.9rem;
    border-radius: 999px;
    z-index: 80;
    border: 1px solid rgba(180, 35, 45, 0.62);
    background: rgba(180, 35, 45, 0.9);
    color: #fff5f7;
    display: grid;
    place-items: center;
    padding: 0;
    box-shadow: 0 10px 22px rgba(0, 0, 0, 0.34);
  }

  .explorer-fab-icon {
    width: 1.4rem;
    height: 1.4rem;
    fill: currentColor;
    display: block;
  }

  .signals-stage,
  .actions-stage {
    flex: 1 1 auto;
    min-height: 0;
    display: grid;
    grid-template-rows: 1fr;
    gap: 0;
  }

  .signals-layout,
  .actions-layout {
    min-height: 0;
    display: grid;
    grid-template-columns: minmax(280px, 420px) minmax(320px, 1fr);
    gap: 0.58rem;
  }

  .actions-layout {
    grid-template-columns: minmax(320px, 1fr) minmax(300px, 420px);
  }

  .signals-stage.no-inspector .signals-layout,
  .actions-stage.no-inspector .actions-layout {
    grid-template-columns: 1fr;
  }

  @media (max-width: 1200px) {
    .signals-layout,
    .actions-layout {
      grid-template-columns: 1fr;
    }
  }

  @media (max-width: 900px) {
    .signals-stage,
    .actions-stage {
      grid-template-rows: auto 1fr;
    }
  }
</style>
