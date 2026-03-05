<script lang="ts">
  type GridDensity = 'compact' | 'balanced' | 'comfortable';
  type RecordingSummary = {
    id: string;
    name: string;
    source: string;
    startedAtEpochMs: number;
    durationMs: number;
    signalCount: number;
    frameCount: number;
  };

  type Props = {
    host: string;
    controlPort: string;
    defaultGridColumns: number;
    defaultGridDensity: GridDensity;
    acceptRobotRecordingRequests: boolean;
    recordingStatus: string;
    recordings: RecordingSummary[];
    selectedRecordingId: string;
    connected: boolean;
    status: string;
    serverLayoutName: string;
    availableServerLayouts: string[];
    onHostInput: (value: string) => void;
    onPortInput: (value: string) => void;
    onDefaultGridColumnsInput: (value: string) => void;
    onDefaultGridDensityInput: (value: string) => void;
    onServerLayoutNameInput: (value: string) => void;
    onAcceptRobotRecordingRequestsInput: (enabled: boolean) => void;
    onSelectedRecordingIdInput: (recordingId: string) => void;
    onConnect: () => void;
    onDisconnect: () => void;
    onReplayRecording: () => void;
    onExportRecording: () => void;
    onImportRecording: () => void;
    onDeleteRecording: () => void;
    onDeleteServerLayout: (layoutName: string) => void;
  };

  let {
    host,
    controlPort,
    defaultGridColumns,
    defaultGridDensity,
    acceptRobotRecordingRequests,
    recordingStatus,
    recordings,
    selectedRecordingId,
    connected,
    status,
    serverLayoutName,
    availableServerLayouts,
    onHostInput,
    onPortInput,
    onDefaultGridColumnsInput,
    onDefaultGridDensityInput,
    onServerLayoutNameInput,
    onAcceptRobotRecordingRequestsInput,
    onSelectedRecordingIdInput,
    onConnect,
    onDisconnect,
    onReplayRecording,
    onExportRecording,
    onImportRecording,
    onDeleteRecording,
    onDeleteServerLayout
  }: Props = $props();

  function formatDuration(durationMs: number): string {
    const totalSeconds = Math.max(0, Math.round(durationMs / 1000));
    const minutes = Math.floor(totalSeconds / 60);
    const seconds = totalSeconds % 60;
    return `${minutes}m ${String(seconds).padStart(2, '0')}s`;
  }

  function formatStartedAt(epochMs: number): string {
    if (!Number.isFinite(epochMs) || epochMs <= 0) return 'unknown';
    return new Date(epochMs).toLocaleString();
  }

  const selectedRecordingSummary = $derived(
    recordings.find((entry) => entry.id === selectedRecordingId) ?? null
  );
</script>

<section class="settings-screen panel">
  <header>
    <h2>Connection Settings</h2>
    <p>Configure ARCP endpoint. Use <code>ds</code> (or leave Host blank) for Driver Station auto-targeting.</p>
  </header>

  <div class="settings-grid">
    <label>
      Host
      <input
        value={host}
        placeholder="ds"
        oninput={(e) => onHostInput((e.currentTarget as HTMLInputElement).value)}
      />
    </label>

    <label>
      Control port
      <input
        value={controlPort}
        inputmode="numeric"
        placeholder="5805"
        oninput={(e) => onPortInput((e.currentTarget as HTMLInputElement).value)}
      />
    </label>

    <div class="actions">
      <button class="btn btn-primary" onclick={onConnect}>Connect</button>
      <button class="btn" onclick={onDisconnect}>Disconnect</button>
    </div>
  </div>

  <div class="status-row">
    <span class="status-label">Link status</span>
    <span class="badge" data-online={connected}>{connected ? 'ONLINE' : 'OFFLINE'}</span>
    <code>{status}</code>
  </div>

  <div class="canvas-defaults panel">
    <header>
      <h3>Canvas Defaults</h3>
      <p>Set baseline grid settings for dashboard canvases.</p>
    </header>
    <div class="defaults-grid">
      <label>
        Default columns
        <input
          type="number"
          min="8"
          max="96"
          value={String(defaultGridColumns)}
          oninput={(e) => onDefaultGridColumnsInput((e.currentTarget as HTMLInputElement).value)}
        />
      </label>
      <label>
        Density
        <select
          value={defaultGridDensity}
          onchange={(e) => onDefaultGridDensityInput((e.currentTarget as HTMLSelectElement).value)}
        >
          <option value="compact">Compact</option>
          <option value="balanced">Balanced</option>
          <option value="comfortable">Comfortable</option>
        </select>
      </label>
    </div>
  </div>

  <div class="server-layout panel">
    <header>
      <h3>Server Layout Profiles</h3>
      <p>Select a server profile, then delete it if needed.</p>
    </header>
    <div class="layout-controls">
      <label>
        Server profile
        <select
          value={serverLayoutName}
          onchange={(e) => onServerLayoutNameInput((e.currentTarget as HTMLSelectElement).value)}
          disabled={!connected || availableServerLayouts.length === 0}
        >
          {#if availableServerLayouts.length === 0}
            <option value="">No server layouts</option>
          {:else}
            {#each availableServerLayouts as name}
              <option value={name}>{name}</option>
            {/each}
          {/if}
        </select>
      </label>
      <div class="actions">
        <button
          class="btn btn-danger"
          onclick={() => onDeleteServerLayout(serverLayoutName)}
          disabled={!connected || !serverLayoutName || availableServerLayouts.length === 0}
        >
          Delete
        </button>
      </div>
    </div>
    {#if availableServerLayouts.length === 0}
      <p class="layout-empty">No server layouts discovered yet.</p>
    {/if}
  </div>

  <div class="recording panel">
    <header>
      <h3>Recording & Replay</h3>
      <p>Record ARCP telemetry locally, replay sessions, and honor robot recording requests.</p>
    </header>

    <label class="toggle-row">
      <input
        type="checkbox"
        checked={acceptRobotRecordingRequests}
        onchange={(e) =>
          onAcceptRobotRecordingRequestsInput((e.currentTarget as HTMLInputElement).checked)}
      />
      Accept robot recording requests
    </label>

    <p class="recording-controls-note">
      Start/stop recording and quick-load replay controls are available in the top bar.
    </p>

    <label>
      Saved recordings
      <select
        value={selectedRecordingId}
        onchange={(e) => onSelectedRecordingIdInput((e.currentTarget as HTMLSelectElement).value)}
        disabled={recordings.length === 0}
      >
        {#if recordings.length === 0}
          <option value="">No recordings</option>
        {:else}
          {#each recordings as recording}
            <option value={recording.id}>{recording.name}</option>
          {/each}
        {/if}
      </select>
    </label>

    <div class="recording-actions">
      <button class="btn" onclick={onReplayRecording} disabled={recordings.length === 0 || !selectedRecordingId}>
        Replay
      </button>
      <button class="btn" onclick={onExportRecording} disabled={recordings.length === 0 || !selectedRecordingId}>
        Save to File
      </button>
      <button class="btn" onclick={onImportRecording}>Load File</button>
      <button class="btn btn-danger" onclick={onDeleteRecording} disabled={recordings.length === 0 || !selectedRecordingId}>
        Delete
      </button>
    </div>

    {#if selectedRecordingSummary}
      <p class="recording-meta">
        <strong>{selectedRecordingSummary.source}</strong>
        {` | ${formatDuration(selectedRecordingSummary.durationMs)} | ${selectedRecordingSummary.signalCount} signals | ${selectedRecordingSummary.frameCount} frames | ${formatStartedAt(selectedRecordingSummary.startedAtEpochMs)}`}
      </p>
    {/if}

    <p class="recording-status">{recordingStatus}</p>
  </div>

</section>

<style>
  .settings-screen {
    padding: 0.9rem;
    display: grid;
    grid-template-rows: auto auto auto auto auto;
    gap: 0.86rem;
    min-height: 0;
  }

  header h2 {
    margin: 0;
    font-size: 1rem;
    color: var(--text-strong);
  }

  header p {
    margin: 0.2rem 0 0;
    font-size: 0.78rem;
    color: var(--text-soft);
  }

  .settings-grid {
    display: grid;
    grid-template-columns: minmax(180px, 280px) minmax(120px, 180px) auto;
    gap: 0.6rem;
    align-items: end;
  }

  label {
    display: grid;
    gap: 0.2rem;
    font-size: 0.75rem;
    color: var(--text-soft);
  }

  .actions {
    display: flex;
    gap: 0.45rem;
  }

  .status-row {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    padding: 0.58rem 0.66rem;
    background: var(--surface-2);
  }

  .status-label {
    color: var(--text-soft);
    font-size: 0.74rem;
  }

  code {
    font-family: var(--font-mono);
    font-size: 0.72rem;
    color: var(--text);
    background: var(--surface-3);
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    padding: 0.2rem 0.35rem;
    overflow: auto;
    white-space: nowrap;
  }

  @media (max-width: 960px) {
    .settings-grid {
      grid-template-columns: 1fr;
      align-items: stretch;
    }

    .actions {
      justify-content: flex-start;
    }

    .status-row {
      flex-wrap: wrap;
    }
  }

  .server-layout {
    padding: 0.72rem;
    display: grid;
    gap: 0.6rem;
    border-radius: 10px;
  }

  .server-layout header h3 {
    margin: 0;
    font-size: 0.9rem;
    color: var(--text-strong);
  }

  .server-layout header p {
    margin: 0.15rem 0 0;
    font-size: 0.74rem;
    color: var(--text-soft);
  }

  .canvas-defaults {
    padding: 0.72rem;
    display: grid;
    gap: 0.6rem;
    border-radius: 10px;
  }

  .canvas-defaults header h3 {
    margin: 0;
    font-size: 0.9rem;
    color: var(--text-strong);
  }

  .canvas-defaults header p {
    margin: 0.15rem 0 0;
    font-size: 0.74rem;
    color: var(--text-soft);
  }

  .defaults-grid {
    display: grid;
    grid-template-columns: minmax(140px, 220px) minmax(140px, 220px);
    gap: 0.5rem;
    align-items: end;
  }

  .layout-controls {
    display: grid;
    grid-template-columns: minmax(160px, 260px) auto;
    gap: 0.5rem;
    align-items: end;
  }

  .layout-empty {
    margin: 0;
    font-size: 0.73rem;
    color: var(--text-soft);
  }

  .recording {
    padding: 0.72rem;
    display: grid;
    gap: 0.58rem;
    border-radius: 10px;
  }

  .recording header h3 {
    margin: 0;
    font-size: 0.9rem;
    color: var(--text-strong);
  }

  .recording header p {
    margin: 0.15rem 0 0;
    font-size: 0.74rem;
    color: var(--text-soft);
  }

  .toggle-row {
    display: inline-flex;
    align-items: center;
    gap: 0.45rem;
    font-size: 0.74rem;
    color: var(--text);
  }

  .recording-actions {
    display: flex;
    flex-wrap: wrap;
    gap: 0.45rem;
  }

  .recording-controls-note {
    margin: 0;
    font-size: 0.72rem;
    color: var(--text-soft);
  }

  .recording-meta {
    margin: 0;
    font-size: 0.73rem;
    color: var(--text-soft);
  }

  .recording-status {
    margin: 0;
    font-size: 0.74rem;
    color: var(--text);
  }

  @media (max-width: 960px) {
    .layout-controls {
      grid-template-columns: 1fr;
    }

    .defaults-grid {
      grid-template-columns: 1fr;
    }
  }
</style>
