<script lang="ts">
  import type { RemoteLogEntry } from '../lib/arcp';

  type SourceFilter = 'all' | 'rio' | 'athena';

  type Props = {
    connected: boolean;
    status: string;
    requestedHost: string;
    resolvedHost: string;
    refreshing: boolean;
    previewLoading: boolean;
    downloadingPath: string | null;
    error: string;
    entries: RemoteLogEntry[];
    selectedPath: string;
    preview: string;
    previewTruncated: boolean;
    onRefresh: () => void;
    onSelectLog: (path: string) => void;
    onDownloadLog: (path: string) => void;
  };

  let {
    connected,
    status,
    requestedHost,
    resolvedHost,
    refreshing,
    previewLoading,
    downloadingPath,
    error,
    entries,
    selectedPath,
    preview,
    previewTruncated,
    onRefresh,
    onSelectLog,
    onDownloadLog
  }: Props = $props();

  let query = $state('');
  let sourceFilter = $state<SourceFilter>('all');

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
        {#if previewLoading}
          <p class="empty">Loading preview...</p>
        {:else if !preview.trim()}
          <p class="empty">No preview data available (file may be binary or empty).</p>
        {:else}
          <pre>{preview}</pre>
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

  pre {
    margin: 0;
    min-height: 0;
    overflow: auto;
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: #0f141f;
    color: #dce7ff;
    font-family: var(--font-mono);
    font-size: 0.68rem;
    line-height: 1.4;
    padding: 0.55rem;
    white-space: pre-wrap;
    overflow-wrap: anywhere;
  }

  @media (max-width: 1220px) {
    .body-grid {
      grid-template-columns: 1fr;
    }
  }
</style>
