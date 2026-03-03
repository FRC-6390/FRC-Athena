<script lang="ts">
  import type { RemoteLogEntry } from '../lib/arcp';

  type SourceFilter = 'all' | 'rio' | 'athena';

  type Props = {
    connected: boolean;
    status: string;
    requestedHost: string;
    resolvedHost: string;
    probeHost: string;
    controlPort: number;
    robotReachable: boolean;
    arcpReachable: boolean;
    probeLoading: boolean;
    refreshing: boolean;
    previewLoading: boolean;
    downloadingPath: string | null;
    error: string;
    entries: RemoteLogEntry[];
    selectedPath: string;
    preview: string;
    previewTruncated: boolean;
    liveFollow: boolean;
    onRefresh: () => void;
    onSelectLog: (path: string) => void;
    onDownloadLog: (path: string) => void;
    onToggleLiveFollow: (enabled: boolean) => void;
  };

  let {
    connected,
    status,
    requestedHost,
    resolvedHost,
    probeHost,
    controlPort,
    robotReachable,
    arcpReachable,
    probeLoading,
    refreshing,
    previewLoading,
    downloadingPath,
    error,
    entries,
    selectedPath,
    preview,
    previewTruncated,
    liveFollow,
    onRefresh,
    onSelectLog,
    onDownloadLog,
    onToggleLiveFollow
  }: Props = $props();

  let query = $state('');
  let sourceFilter = $state<SourceFilter>('all');
  let lineViewport = $state<HTMLDivElement | null>(null);

  function formatBytes(raw: number): string {
    if (!Number.isFinite(raw) || raw < 0) return 'n/a';
    if (raw < 1024) return `${Math.round(raw)} B`;
    if (raw < 1024 * 1024) return `${(raw / 1024).toFixed(1)} KiB`;
    if (raw < 1024 * 1024 * 1024) return `${(raw / (1024 * 1024)).toFixed(1)} MiB`;
    return `${(raw / (1024 * 1024 * 1024)).toFixed(1)} GiB`;
  }

  function formatTime(epochSec: number): string {
    if (!Number.isFinite(epochSec) || epochSec <= 0) return 'unknown';
    return new Date(epochSec * 1000).toLocaleString();
  }

  const selectedEntry = $derived(entries.find((entry) => entry.path === selectedPath) ?? null);
  const previewLines = $derived.by(() => {
    const normalized = preview.replace(/\r\n/g, '\n').replace(/\r/g, '\n');
    const lines = normalized.split('\n');
    if (lines.length > 0 && lines[lines.length - 1] === '') {
      lines.pop();
    }
    return lines.map((text, index) => ({
      lineNumber: index + 1,
      text: text.length > 0 ? text : ' '
    }));
  });
  const filteredEntries = $derived.by(() => {
    const token = query.trim().toLowerCase();
    return entries.filter((entry) => {
      if (sourceFilter !== 'all' && entry.source !== sourceFilter) return false;
      if (!token) return true;
      return (
        entry.path.toLowerCase().includes(token) ||
        entry.name.toLowerCase().includes(token) ||
        entry.source.toLowerCase().includes(token)
      );
    });
  });

  $effect(() => {
    const currentPath = selectedPath;
    const loading = previewLoading;
    const lineCount = previewLines.length;
    if (!liveFollow || loading || !currentPath || lineCount === 0) return;
    requestAnimationFrame(() => {
      if (!lineViewport) return;
      lineViewport.scrollTop = lineViewport.scrollHeight;
    });
  });
</script>

<section class="logs-screen panel">
  <header>
    <h2>Robot Logs</h2>
    <p>Browse and download roboRIO and Athena logs through Driver Station network targeting.</p>
  </header>

  <div class="status-row">
    <span class="status-label">Link</span>
    <span class="badge" data-online={connected}>{connected ? 'ONLINE' : 'OFFLINE'}</span>
    <code>{status}</code>
  </div>

  <div class="meta-row panel">
    <span><strong>Requested host:</strong> {requestedHost.trim() || '(blank -> DS auto)'}</span>
    <span><strong>Resolved host:</strong> {resolvedHost || 'n/a'}</span>
    <button class="btn btn-primary" onclick={onRefresh} disabled={refreshing}>
      {refreshing ? 'Refreshing...' : 'Refresh Logs'}
    </button>
  </div>

  <div class="probe-row panel">
    <span>
      <strong>Robot Link:</strong>
      {#if probeLoading}
        probing...
      {:else if robotReachable}
        reachable
      {:else}
        not reachable
      {/if}
    </span>
    <span>
      <strong>ARCP {controlPort > 0 ? `:${controlPort}` : ''}:</strong>
      {#if probeLoading}
        probing...
      {:else if arcpReachable}
        reachable
      {:else}
        not found
      {/if}
    </span>
    <span><strong>Probe host:</strong> {probeHost || 'n/a'}</span>
  </div>

  {#if !connected && !probeLoading && robotReachable && !arcpReachable}
    <section class="link-warning panel">
      Robot is reachable, but no ARCP server was found on port {controlPort > 0 ? controlPort : 'configured'}.
    </section>
  {/if}

  {#if error}
    <section class="error-banner panel">{error}</section>
  {/if}

  <div class="body-grid">
    <section class="log-list panel">
      <div class="list-tools">
        <input
          value={query}
          placeholder="Filter logs by name or path"
          oninput={(event) => (query = (event.currentTarget as HTMLInputElement).value)}
        />
        <select value={sourceFilter} onchange={(event) => (sourceFilter = (event.currentTarget as HTMLSelectElement).value as SourceFilter)}>
          <option value="all">All Sources</option>
          <option value="rio">roboRIO</option>
          <option value="athena">Athena</option>
        </select>
      </div>

      <div class="list-count">{filteredEntries.length} logs</div>

      <div class="entries">
        {#if filteredEntries.length === 0}
          <p class="empty">No logs found for this filter. Refresh to rescan the robot.</p>
        {:else}
          {#each filteredEntries as entry (entry.path)}
            <button
              class={`entry ${selectedPath === entry.path ? 'active' : ''}`}
              onclick={() => onSelectLog(entry.path)}
            >
              <div class="entry-head">
                <strong>{entry.name}</strong>
                <span class={`source ${entry.source}`}>{entry.source}</span>
              </div>
              <div class="entry-meta">
                <span>{formatBytes(entry.sizeBytes)}</span>
                <span>{formatTime(entry.modifiedEpochSec)}</span>
              </div>
              <code>{entry.path}</code>
            </button>
          {/each}
        {/if}
      </div>
    </section>

    <section class="preview panel">
      <div class="preview-head">
        <h3>Log Preview</h3>
        <div class="preview-actions">
          <button class={`btn ${liveFollow ? 'btn-primary' : ''}`} onclick={() => onToggleLiveFollow(!liveFollow)}>
            {liveFollow ? 'Live ON' : 'Live OFF'}
          </button>
          <button
            class="btn"
            onclick={() => selectedPath && onDownloadLog(selectedPath)}
            disabled={!selectedPath || !!downloadingPath}
          >
            {#if downloadingPath === selectedPath}
              Downloading...
            {:else}
              Download File
            {/if}
          </button>
        </div>
      </div>

      {#if !selectedEntry}
        <p class="empty">Select a log to view and download it.</p>
      {:else}
        <p class="preview-meta">
          <strong>{selectedEntry.name}</strong>
          {` | ${formatBytes(selectedEntry.sizeBytes)} | ${formatTime(selectedEntry.modifiedEpochSec)}`}
        </p>
        {#if previewTruncated}
          <p class="preview-note">Showing tail of file (truncated).</p>
        {/if}
        {#if liveFollow}
          <p class="preview-note">Live follow enabled (1s polling).</p>
        {/if}
        {#if previewLoading}
          <p class="empty">Loading preview...</p>
        {:else if previewLines.length === 0}
          <p class="empty">No preview data available (file may be binary or empty).</p>
        {:else}
          <div class="line-viewport" bind:this={lineViewport}>
            {#each previewLines as line (`line-${line.lineNumber}`)}
              <div class="log-line">
                <span class="line-no">{line.lineNumber}</span>
                <span class="line-text">{line.text}</span>
              </div>
            {/each}
          </div>
        {/if}
      {/if}
    </section>
  </div>
</section>

<style>
  .logs-screen {
    padding: 0.78rem;
    display: grid;
    grid-template-rows: auto auto auto auto 1fr;
    gap: 0.58rem;
    min-height: 0;
  }

  header h2 {
    margin: 0;
    font-size: 0.98rem;
    color: var(--text-strong);
  }

  header p {
    margin: 0.2rem 0 0;
    color: var(--text-soft);
    font-size: 0.76rem;
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

  .meta-row {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 0.7rem;
    padding: 0.58rem 0.66rem;
    font-size: 0.74rem;
    color: var(--text-soft);
  }

  .meta-row strong {
    color: var(--text);
  }

  .probe-row {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 0.7rem;
    padding: 0.58rem 0.66rem;
    font-size: 0.74rem;
    color: var(--text-soft);
  }

  .probe-row strong {
    color: var(--text);
  }

  .link-warning {
    padding: 0.54rem 0.66rem;
    border: 1px solid rgba(248, 113, 113, 0.5);
    background: rgba(127, 29, 29, 0.2);
    color: #fecaca;
    font-size: 0.75rem;
    border-radius: 8px;
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

  .body-grid {
    min-height: 0;
    display: grid;
    grid-template-columns: minmax(320px, 420px) 1fr;
    gap: 0.48rem;
  }

  .log-list,
  .preview {
    min-height: 0;
    display: grid;
    gap: 0.38rem;
    padding: 0.52rem;
    border-radius: 10px;
  }

  .log-list {
    grid-template-rows: auto auto 1fr;
  }

  .preview {
    grid-template-rows: auto auto auto 1fr;
  }

  .list-tools {
    display: grid;
    grid-template-columns: 1fr auto;
    gap: 0.42rem;
  }

  .list-count {
    font-size: 0.7rem;
    color: var(--text-soft);
  }

  .entries {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.26rem;
    align-content: start;
  }

  .entry {
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    background: var(--surface-3);
    color: inherit;
    text-align: left;
    padding: 0.35rem 0.4rem;
    display: grid;
    gap: 0.15rem;
  }

  .entry.active {
    border-color: rgba(180, 35, 45, 0.52);
    background: rgba(180, 35, 45, 0.14);
  }

  .entry-head {
    display: flex;
    align-items: center;
    gap: 0.35rem;
  }

  .entry-head strong {
    font-size: 0.72rem;
    color: var(--text-strong);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .source {
    font-size: 0.62rem;
    border-radius: 999px;
    padding: 0.06rem 0.34rem;
    border: 1px solid var(--border-subtle);
    text-transform: uppercase;
    letter-spacing: 0.04em;
    color: var(--text-soft);
  }

  .source.rio {
    border-color: rgba(97, 175, 239, 0.45);
    color: #bfdbfe;
  }

  .source.athena {
    border-color: rgba(248, 113, 113, 0.45);
    color: #fecaca;
  }

  .entry-meta {
    display: flex;
    gap: 0.6rem;
    font-size: 0.64rem;
    color: var(--text-soft);
  }

  .entry code {
    font-size: 0.62rem;
  }

  .preview-head {
    display: flex;
    justify-content: space-between;
    gap: 0.45rem;
    align-items: center;
  }

  .preview-actions {
    display: flex;
    gap: 0.4rem;
    align-items: center;
  }

  .preview-head h3 {
    margin: 0;
    font-size: 0.8rem;
    color: var(--text-strong);
  }

  .preview-meta,
  .preview-note {
    margin: 0;
    font-size: 0.7rem;
    color: var(--text-soft);
  }

  .empty {
    margin: 0;
    font-size: 0.72rem;
    color: var(--text-soft);
  }

  .line-viewport {
    margin: 0;
    min-height: 0;
    overflow: auto;
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: #0f141f;
    color: #dce7ff;
    font-family: var(--font-mono);
    font-size: 0.68rem;
    line-height: 1.34;
    padding: 0.3rem 0;
  }

  .log-line {
    display: grid;
    grid-template-columns: auto 1fr;
    gap: 0.45rem;
    align-items: start;
    padding: 0 0.5rem;
  }

  .line-no {
    color: #7f8ea6;
    text-align: right;
    min-width: 3.2ch;
    user-select: none;
  }

  .line-text {
    white-space: pre-wrap;
    overflow-wrap: anywhere;
    color: #dce7ff;
  }

  @media (max-width: 1220px) {
    .body-grid {
      grid-template-columns: 1fr;
    }
  }
</style>
