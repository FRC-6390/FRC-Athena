<script lang="ts">
  import type { SignalRow } from '../lib/arcp';

  type Props = {
    signals: SignalRow[];
    onSelectSignal: (signalId: number) => void;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  type CameraGroup = {
    key: string;
    label: string;
    signals: SignalRow[];
  };

  let { signals, onSelectSignal, onSendSet }: Props = $props();

  let selectedGroupKey = $state('');
  let query = $state('');
  let draftBySignal = $state<Record<number, string>>({});

  function splitPath(path: string): string[] {
    return path
      .split('/')
      .map((part) => part.trim())
      .filter((part) => part.length > 0);
  }

  function normalizeToken(value: string): string {
    return value.toLowerCase().replace(/[^a-z0-9]+/g, '');
  }

  function leafPath(path: string): string {
    const parts = splitPath(path);
    return parts[parts.length - 1] ?? path;
  }

  function toTitle(value: string): string {
    if (!value) return 'Camera';
    return value
      .replace(/[_-]+/g, ' ')
      .split(' ')
      .filter(Boolean)
      .map((part) => part[0].toUpperCase() + part.slice(1))
      .join(' ');
  }

  function parentPath(path: string): string {
    const parts = splitPath(path);
    if (parts.length <= 1) return '';
    return `/${parts.slice(0, -1).join('/')}`;
  }

  function isCameraSignal(signal: SignalRow): boolean {
    const token = normalizeToken(signal.path);
    return (
      token.includes('camera') ||
      token.includes('stream') ||
      token.includes('vision') ||
      token.includes('exposure') ||
      token.includes('whitebalance') ||
      token.includes('gain') ||
      token.includes('brightness') ||
      token.includes('contrast') ||
      token.includes('saturation') ||
      token.includes('pipeline')
    );
  }

  function cameraGroupKey(signal: SignalRow): string {
    const parts = splitPath(signal.path);
    const lower = parts.map((part) => normalizeToken(part));

    for (let idx = 0; idx < lower.length; idx++) {
      const token = lower[idx];
      if (token === 'cameras' || token === 'camera') {
        const end = Math.min(parts.length, idx + 2);
        return `/${parts.slice(0, end).join('/')}`;
      }
      if (token === 'vision' && parts[idx + 1]) {
        const end = Math.min(parts.length, idx + 2);
        return `/${parts.slice(0, end).join('/')}`;
      }
    }

    const parent = parentPath(signal.path);
    return parent || '/camera';
  }

  const cameraGroups = $derived.by(() => {
    const map = new Map<string, SignalRow[]>();

    for (const signal of signals) {
      if (!isCameraSignal(signal)) continue;
      const key = cameraGroupKey(signal);
      if (!map.has(key)) map.set(key, []);
      map.get(key)!.push(signal);
    }

    return [...map.entries()]
      .map(([key, groupSignals]): CameraGroup => {
        const label = toTitle(leafPath(key) || key);
        return {
          key,
          label,
          signals: [...groupSignals].sort((a, b) => a.path.localeCompare(b.path))
        };
      })
      .sort((a, b) => a.label.localeCompare(b.label));
  });

  const selectedGroup = $derived(
    cameraGroups.find((group) => group.key === selectedGroupKey) ?? null
  );

  const visibleGroups = $derived.by(() => {
    const q = query.trim().toLowerCase();
    if (!q) return cameraGroups;
    return cameraGroups.filter(
      (group) =>
        group.label.toLowerCase().includes(q) ||
        group.key.toLowerCase().includes(q) ||
        group.signals.some((signal) => signal.path.toLowerCase().includes(q))
    );
  });

  const streamSignal = $derived.by(() => {
    if (!selectedGroup) return null;
    return (
      selectedGroup.signals.find((signal) => {
        if (signal.signal_type !== 'string' && signal.signal_type !== 'string[]') return false;
        const token = normalizeToken(leafPath(signal.path));
        return token.includes('stream') || token.includes('url') || token.includes('video') || token.includes('mjpeg');
      }) ?? null
    );
  });

  const streamUrl = $derived.by(() => {
    if (!streamSignal) return '';
    const raw = streamSignal.value.trim();
    if (!raw || raw === '-') return '';
    return raw;
  });

  const telemetrySignals = $derived.by(() => {
    if (!selectedGroup) return [];
    return selectedGroup.signals.filter((signal) => signal.access !== 'write').slice(0, 16);
  });

  const tuningSignals = $derived.by(() => {
    if (!selectedGroup) return [];
    return selectedGroup.signals.filter((signal) => {
      if (signal.access !== 'write') return false;
      const token = normalizeToken(leafPath(signal.path));
      return (
        token.includes('exposure') ||
        token.includes('gain') ||
        token.includes('brightness') ||
        token.includes('contrast') ||
        token.includes('saturation') ||
        token.includes('gamma') ||
        token.includes('whitebalance') ||
        token.includes('temperature') ||
        token.includes('fps') ||
        token.includes('framerate') ||
        token.includes('pipeline') ||
        token.includes('led') ||
        token.includes('mode') ||
        token.includes('resolution') ||
        token.includes('stream')
      );
    });
  });

  $effect(() => {
    if (cameraGroups.length === 0) {
      selectedGroupKey = '';
      return;
    }
    if (cameraGroups.some((group) => group.key === selectedGroupKey)) return;
    selectedGroupKey = cameraGroups[0].key;
  });

  $effect(() => {
    const next = { ...draftBySignal };
    let changed = false;
    const writableIds = new Set<number>();

    for (const signal of tuningSignals) {
      writableIds.add(signal.signal_id);
      if (next[signal.signal_id] === undefined) {
        next[signal.signal_id] = signal.value === '-' ? '' : signal.value;
        changed = true;
      }
    }

    for (const key of Object.keys(next)) {
      const signalId = Number(key);
      if (!writableIds.has(signalId)) {
        delete next[signalId];
        changed = true;
      }
    }

    if (changed) {
      draftBySignal = next;
    }
  });

  function formatValue(raw: string): string {
    if (raw === '-') return '--';
    return raw;
  }

  function inputKind(signal: SignalRow): 'bool' | 'number' | 'text' {
    if (signal.signal_type === 'bool') return 'bool';
    if (signal.signal_type === 'f64' || signal.signal_type === 'i64') return 'number';
    return 'text';
  }

  function setDraft(signalId: number, value: string) {
    draftBySignal = {
      ...draftBySignal,
      [signalId]: value
    };
  }

  function applySignal(signal: SignalRow) {
    const raw = draftBySignal[signal.signal_id] ?? '';
    const kind = inputKind(signal);

    if (kind === 'bool') {
      const normalized = raw.trim().toLowerCase();
      if (normalized !== 'true' && normalized !== 'false') return;
      onSendSet(signal.signal_id, normalized);
      return;
    }

    if (kind === 'number') {
      const numeric = Number(raw);
      if (!Number.isFinite(numeric)) return;
      onSendSet(
        signal.signal_id,
        signal.signal_type === 'i64' ? String(Math.round(numeric)) : String(numeric)
      );
      return;
    }

    if (!raw.trim()) return;
    onSendSet(signal.signal_id, raw);
  }
</script>

<section class="camera-screen panel">
  <header>
    <h2>Camera Tuning</h2>
    <p>Tune camera controls and monitor stream telemetry by camera group.</p>
  </header>

  <div class="toolbar">
    <input
      value={query}
      placeholder="Filter camera groups"
      oninput={(event) => (query = (event.currentTarget as HTMLInputElement).value)}
    />
    <span class="summary-chip">{cameraGroups.length} groups</span>
  </div>

  <div class="layout">
    <aside class="group-list panel-card">
      <h3>Groups</h3>
      <div class="scroll-list">
        {#if visibleGroups.length === 0}
          <p class="empty">No camera groups found.</p>
        {:else}
          {#each visibleGroups as group (group.key)}
            <button
              class={`group-item ${selectedGroupKey === group.key ? 'active' : ''}`}
              onclick={() => (selectedGroupKey = group.key)}
            >
              <strong>{group.label}</strong>
              <span>{group.signals.length} signals</span>
            </button>
          {/each}
        {/if}
      </div>
    </aside>

    <section class="camera-view panel-card">
      {#if selectedGroup}
        <div class="view-head">
          <h3>{selectedGroup.label}</h3>
          <code>{selectedGroup.key}</code>
        </div>

        <div class="preview">
          {#if streamUrl}
            <img src={streamUrl} alt={`${selectedGroup.label} stream`} loading="lazy" />
            <small title={streamUrl}>{streamSignal?.path}</small>
          {:else}
            <p class="empty">No stream URL signal mapped for this group.</p>
          {/if}
        </div>

        <div class="sections">
          <section class="sub-card">
            <h4>Tuning Controls</h4>
            {#if tuningSignals.length === 0}
              <p class="empty">No writable camera tuning signals found.</p>
            {:else}
              <div class="control-list">
                {#each tuningSignals as signal (signal.signal_id)}
                  {@const kind = inputKind(signal)}
                  <article class="control-row">
                    <button class="signal-label" onclick={() => onSelectSignal(signal.signal_id)}>
                      <strong>{leafPath(signal.path)}</strong>
                      <span>#{signal.signal_id} · {signal.signal_type}</span>
                    </button>

                    <div class="inline-controls">
                      {#if kind === 'bool'}
                        <select
                          value={draftBySignal[signal.signal_id] ?? signal.value}
                          onchange={(event) => setDraft(signal.signal_id, (event.currentTarget as HTMLSelectElement).value)}
                        >
                          <option value="true">true</option>
                          <option value="false">false</option>
                        </select>
                      {:else}
                        <input
                          value={draftBySignal[signal.signal_id] ?? signal.value}
                          oninput={(event) => setDraft(signal.signal_id, (event.currentTarget as HTMLInputElement).value)}
                        />
                      {/if}
                      <button class="btn btn-primary" onclick={() => applySignal(signal)}>Set</button>
                    </div>
                  </article>
                {/each}
              </div>
            {/if}
          </section>

          <section class="sub-card">
            <h4>Telemetry</h4>
            {#if telemetrySignals.length === 0}
              <p class="empty">No telemetry signals in this group.</p>
            {:else}
              <div class="telemetry-list">
                {#each telemetrySignals as signal (signal.signal_id)}
                  <button class="telemetry-row" onclick={() => onSelectSignal(signal.signal_id)}>
                    <strong>{leafPath(signal.path)}</strong>
                    <span>{formatValue(signal.value)}</span>
                  </button>
                {/each}
              </div>
            {/if}
          </section>
        </div>
      {:else}
        <p class="empty">Select a camera group to view controls.</p>
      {/if}
    </section>
  </div>
</section>

<style>
  .camera-screen {
    padding: 0.76rem;
    display: grid;
    grid-template-rows: auto auto 1fr;
    gap: 0.56rem;
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

  .toolbar {
    display: flex;
    gap: 0.45rem;
    align-items: center;
  }

  .toolbar input {
    flex: 1 1 auto;
    min-width: 0;
  }

  .summary-chip {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.22rem 0.48rem;
    font-size: 0.7rem;
    color: var(--text-soft);
    background: var(--surface-3);
    white-space: nowrap;
  }

  .layout {
    min-height: 0;
    display: grid;
    grid-template-columns: minmax(220px, 290px) minmax(0, 1fr);
    gap: 0.54rem;
  }

  .panel-card {
    border: 1px solid var(--border-subtle);
    border-radius: 10px;
    background: var(--surface-2);
    padding: 0.48rem;
    min-height: 0;
    display: grid;
    gap: 0.36rem;
  }

  .panel-card h3 {
    margin: 0;
    font-size: 0.79rem;
    color: var(--text-strong);
  }

  .scroll-list {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.28rem;
    align-content: start;
  }

  .group-item {
    width: 100%;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-3);
    color: var(--text);
    padding: 0.34rem 0.42rem;
    text-align: left;
    display: grid;
    gap: 0.16rem;
  }

  .group-item strong {
    font-size: 0.74rem;
    color: var(--text-strong);
    line-height: 1.1;
  }

  .group-item span {
    font-size: 0.66rem;
    color: var(--text-soft);
  }

  .group-item.active {
    border-color: rgba(180, 35, 45, 0.64);
    background: rgba(180, 35, 45, 0.2);
  }

  .camera-view {
    grid-template-rows: auto auto 1fr;
  }

  .view-head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.4rem;
  }

  .view-head code {
    margin: 0;
    font-family: var(--font-mono);
    font-size: 0.62rem;
    color: var(--text-soft);
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .preview {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: #0f1522;
    min-height: 180px;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    overflow: hidden;
  }

  .preview img {
    width: 100%;
    height: 100%;
    object-fit: contain;
    background: #0b1220;
  }

  .preview small {
    display: block;
    padding: 0.28rem 0.36rem;
    font-size: 0.62rem;
    color: var(--text-soft);
    font-family: var(--font-mono);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .sections {
    min-height: 0;
    display: grid;
    grid-template-columns: minmax(0, 1.2fr) minmax(0, 0.8fr);
    gap: 0.42rem;
  }

  .sub-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: rgba(27, 33, 44, 0.68);
    padding: 0.38rem;
    min-height: 0;
    display: grid;
    grid-template-rows: auto 1fr;
    gap: 0.3rem;
  }

  .sub-card h4 {
    margin: 0;
    font-size: 0.73rem;
    color: var(--text-strong);
  }

  .control-list,
  .telemetry-list {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.24rem;
    align-content: start;
  }

  .control-row,
  .telemetry-row {
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-3);
    padding: 0.3rem 0.34rem;
    display: grid;
    gap: 0.2rem;
  }

  .signal-label,
  .telemetry-row {
    border: 0;
    background: transparent;
    color: inherit;
    text-align: left;
    padding: 0;
  }

  .signal-label strong,
  .telemetry-row strong {
    font-size: 0.68rem;
    color: var(--text-strong);
  }

  .signal-label span {
    font-size: 0.62rem;
    color: var(--text-soft);
  }

  .inline-controls {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 0.22rem;
    align-items: center;
  }

  .inline-controls input,
  .inline-controls select {
    width: 100%;
    min-width: 0;
    padding: 0.24rem 0.32rem;
    font-size: 0.66rem;
  }

  .telemetry-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 0.32rem;
    width: 100%;
  }

  .telemetry-row span {
    font-family: var(--font-mono);
    font-size: 0.66rem;
    color: var(--text-soft);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    max-width: 48%;
  }

  .empty {
    margin: 0;
    color: var(--text-soft);
    font-size: 0.74rem;
  }

  @media (max-width: 1240px) {
    .layout {
      grid-template-columns: 1fr;
      grid-template-rows: minmax(180px, 30vh) minmax(0, 1fr);
    }

    .sections {
      grid-template-columns: 1fr;
    }
  }
</style>
