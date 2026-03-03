<script lang="ts">
  import type { RemoteLogEntry } from '../lib/arcp';

  type SourceFilter = 'all' | 'rio' | 'athena';
  type LineSeverity = 'error' | 'warn' | 'info' | 'debug' | 'trace' | 'other';
  type PreviewSeverityFilter = 'all' | LineSeverity;
  type LineSearchMatch = {
    leading: string;
    match: string;
    trailing: string;
    hasMatch: boolean;
  };

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
    previewLines: string[];
    previewTruncated: boolean;
    liveFollow: boolean;
    streamConnected: boolean;
    streamState: string;
    streamMessage: string;
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
    previewLines,
    previewTruncated,
    liveFollow,
    streamConnected,
    streamState,
    streamMessage,
    onRefresh,
    onSelectLog,
    onDownloadLog,
    onToggleLiveFollow
  }: Props = $props();

  let query = $state('');
  let sourceFilter = $state<SourceFilter>('all');
  let previewQuery = $state('');
  let previewSeverity = $state<PreviewSeverityFilter>('all');
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

  function parseLineSeverity(text: string): LineSeverity {
    const normalized = text.toLowerCase();
    if (normalized.trim().length === 0) return 'other';
    if (/\b(fatal|panic|critical|segfault|uncaught exception|assert(?:ion)? failed)\b/.test(normalized)) {
      return 'error';
    }
    if (/\b(error|exception|fail(?:ed|ure)?|fault)\b/.test(normalized)) {
      return 'error';
    }
    if (/\b(warn(?:ing)?)\b/.test(normalized)) {
      return 'warn';
    }
    if (/\b(info|notice)\b/.test(normalized)) {
      return 'info';
    }
    if (/\b(debug|dbg)\b/.test(normalized)) {
      return 'debug';
    }
    if (/\b(trace|verbose|vtrace)\b/.test(normalized)) {
      return 'trace';
    }
    return 'other';
  }

  function buildLineSearchMatch(text: string, token: string): LineSearchMatch {
    if (!token) {
      return {
        leading: text,
        match: '',
        trailing: '',
        hasMatch: false
      };
    }
    const index = text.toLowerCase().indexOf(token);
    if (index < 0) {
      return {
        leading: text,
        match: '',
        trailing: '',
        hasMatch: false
      };
    }
    return {
      leading: text.slice(0, index),
      match: text.slice(index, index + token.length),
      trailing: text.slice(index + token.length),
      hasMatch: true
    };
  }

  const selectedEntry = $derived(entries.find((entry) => entry.path === selectedPath) ?? null);
  const showNoArcpWarning = $derived(!connected && !probeLoading && robotReachable && !arcpReachable);
  const renderedPreviewLines = $derived.by(() =>
    previewLines.map((text, index) => {
      const normalized = text.length > 0 ? text : ' ';
      return {
        lineNumber: index + 1,
        text: normalized,
        normalized: normalized.toLowerCase(),
        severity: parseLineSeverity(normalized)
      };
    })
  );
  const normalizedPreviewQuery = $derived(previewQuery.trim().toLowerCase());
  const previewSeverityCounts = $derived.by(() => {
    const counts: Record<LineSeverity, number> = {
      error: 0,
      warn: 0,
      info: 0,
      debug: 0,
      trace: 0,
      other: 0
    };
    for (const line of renderedPreviewLines) {
      counts[line.severity] += 1;
    }
    return counts;
  });
  const visiblePreviewLines = $derived.by(() => {
    const token = normalizedPreviewQuery;
    return renderedPreviewLines
      .filter((line) => {
        if (previewSeverity !== 'all' && line.severity !== previewSeverity) return false;
        if (!token) return true;
        return line.normalized.includes(token);
      })
      .map((line) => ({
        lineNumber: line.lineNumber,
        text: line.text,
        severity: line.severity,
        highlight: buildLineSearchMatch(line.text, token)
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
    const lineCount = visiblePreviewLines.length;
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

  <section class="link-warning panel" data-visible={showNoArcpWarning}>
    Robot is reachable, but no ARCP server was found on port {controlPort > 0 ? controlPort : 'configured'}.
  </section>

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
        {#if liveFollow}
          <p class="preview-note">
            Stream: {streamConnected ? 'connected' : streamState}
            {streamMessage ? ` (${streamMessage})` : ''}
          </p>
        {/if}
        {#if previewTruncated}
          <p class="preview-note">Showing tail of file (truncated).</p>
        {/if}
        {#if liveFollow}
          <p class="preview-note">Live follow enabled (streaming).</p>
        {/if}
        <div class="preview-tools">
          <input
            value={previewQuery}
            placeholder="Search log lines"
            oninput={(event) => (previewQuery = (event.currentTarget as HTMLInputElement).value)}
          />
          <select
            value={previewSeverity}
            onchange={(event) =>
              (previewSeverity = (event.currentTarget as HTMLSelectElement).value as PreviewSeverityFilter)}
          >
            <option value="all">All Levels ({renderedPreviewLines.length})</option>
            <option value="error">Errors ({previewSeverityCounts.error})</option>
            <option value="warn">Warnings ({previewSeverityCounts.warn})</option>
            <option value="info">Info ({previewSeverityCounts.info})</option>
            <option value="debug">Debug ({previewSeverityCounts.debug})</option>
            <option value="trace">Trace ({previewSeverityCounts.trace})</option>
            <option value="other">Other ({previewSeverityCounts.other})</option>
          </select>
          <button
            class="btn btn-mini"
            onclick={() => {
              previewQuery = '';
              previewSeverity = 'all';
            }}
            disabled={previewQuery.length === 0 && previewSeverity === 'all'}
          >
            Clear
          </button>
        </div>
        <p class="preview-note preview-filter-count">
          Showing {visiblePreviewLines.length} of {renderedPreviewLines.length} lines
        </p>
        {#if previewLoading}
          <p class="empty">Loading preview...</p>
        {:else if renderedPreviewLines.length === 0}
          <p class="empty">No preview data available (file may be binary or empty).</p>
        {:else if visiblePreviewLines.length === 0}
          <p class="empty">No log lines matched the current search/filter.</p>
        {:else}
          <div class="line-viewport" bind:this={lineViewport}>
            {#each visiblePreviewLines as line (`line-${line.lineNumber}`)}
              <div class={`log-line ${line.severity}`}>
                <span class="line-no">{line.lineNumber}</span>
                <span class="line-text">
                  {#if line.highlight.hasMatch}
                    <span>{line.highlight.leading}</span><mark class="line-match">{line.highlight.match}</mark><span>{line.highlight.trailing}</span>
                  {:else}
                    {line.text}
                  {/if}
                </span>
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
    min-height: 2.05rem;
    display: flex;
    align-items: center;
    transition:
      opacity 0.14s ease,
      border-color 0.14s ease,
      background 0.14s ease;
  }

  .link-warning[data-visible='false'] {
    opacity: 0;
    visibility: hidden;
    border-color: transparent;
    background: transparent;
    pointer-events: none;
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

  .preview-tools {
    display: grid;
    grid-template-columns: minmax(190px, 1fr) auto auto;
    gap: 0.38rem;
    align-items: center;
  }

  .preview-tools select {
    min-width: 10.5rem;
  }

  .preview-filter-count {
    font-size: 0.66rem;
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
    padding: 0.06rem 0.5rem;
    border-left: 2px solid transparent;
    background: transparent;
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

  .line-match {
    background: rgba(250, 204, 21, 0.4);
    color: #fffbe6;
    border-radius: 2px;
    padding: 0 0.08rem;
  }

  .log-line.error {
    border-left-color: rgba(248, 113, 113, 0.9);
    background: rgba(127, 29, 29, 0.24);
  }

  .log-line.error .line-text {
    color: #fecaca;
  }

  .log-line.warn {
    border-left-color: rgba(250, 204, 21, 0.9);
    background: rgba(120, 88, 18, 0.24);
  }

  .log-line.warn .line-text {
    color: #fde68a;
  }

  .log-line.info {
    border-left-color: rgba(96, 165, 250, 0.9);
    background: rgba(30, 64, 175, 0.2);
  }

  .log-line.info .line-text {
    color: #bfdbfe;
  }

  .log-line.debug {
    border-left-color: rgba(110, 231, 183, 0.9);
    background: rgba(6, 78, 59, 0.2);
  }

  .log-line.debug .line-text {
    color: #bbf7d0;
  }

  .log-line.trace {
    border-left-color: rgba(148, 163, 184, 0.85);
    background: rgba(51, 65, 85, 0.22);
  }

  .log-line.trace .line-text {
    color: #cbd5e1;
  }

  .log-line.other {
    border-left-color: rgba(71, 85, 105, 0.65);
  }

  @media (max-width: 1220px) {
    .body-grid {
      grid-template-columns: 1fr;
    }
  }
</style>
