<script lang="ts">
  import { decodeReplay } from '../../lib/data/decoder';
  import { buildReplayModel, type ReplayModel } from '../../lib/analysis/replay';
  import { buildCsv } from '../../lib/analysis/csv';
  import ReplayScrubber from './ReplayScrubber.svelte';
  import SignalPicker from './SignalPicker.svelte';
  import PlotPanel from './PlotPanel.svelte';
  import FieldView from './FieldView.svelte';

  /** Plots shown at once. Each plot is one WebGL canvas, so the cap keeps the frame budget bounded. */
  const MAX_PLOTS = 8;
  const ZOOM_OPTIONS = [
    { label: 'Full', micros: 0 },
    { label: '60 s', micros: 60_000_000 },
    { label: '10 s', micros: 10_000_000 },
    { label: '2 s', micros: 2_000_000 }
  ];

  let model = $state<ReplayModel | null>(null);
  let fileName = $state('');
  let loading = $state(false);
  let loadError = $state<string | null>(null);
  let cursor = $state(0);
  let zoomMicros = $state(0);
  let selected = $state<number[]>([]);
  let exportNote = $state<string | null>(null);

  const signals = $derived(model ? model.signals() : []);
  const poseIds = $derived(signals.filter((s) => s.type === 'struct:Pose2d').map((s) => s.signalId));
  const fieldSignal = $derived(
    selected.find((id) => poseIds.includes(id)) ?? poseIds[0] ?? null
  );

  /** Visible window: the full replay, or a zoom window that follows the cursor. */
  const view = $derived.by(() => {
    if (!model) return { from: 0, to: 0 };
    if (zoomMicros === 0) return { from: model.startMicros, to: model.endMicros };
    const half = zoomMicros / 2;
    let from = cursor - half;
    let to = cursor + half;
    if (from < model.startMicros) {
      to += model.startMicros - from;
      from = model.startMicros;
    }
    if (to > model.endMicros) {
      from = Math.max(model.startMicros, from - (to - model.endMicros));
      to = model.endMicros;
    }
    return { from, to };
  });

  async function load(file: File) {
    loading = true;
    loadError = null;
    exportNote = null;
    try {
      const records = await decodeReplay(new Uint8Array(await file.arrayBuffer()));
      const next = buildReplayModel(records);
      model = next;
      fileName = file.name;
      cursor = next.startMicros;
      selected = [];
      zoomMicros = 0;
    } catch (error) {
      model = null;
      loadError = `Could not read ${file.name}: ${error instanceof Error ? error.message : String(error)}`;
    } finally {
      loading = false;
    }
  }

  function onFile(event: Event) {
    const file = (event.currentTarget as HTMLInputElement).files?.[0];
    if (file) void load(file);
  }

  function toggle(signalId: number) {
    selected = selected.includes(signalId)
      ? selected.filter((id) => id !== signalId)
      : selected.length < MAX_PLOTS
        ? [...selected, signalId]
        : selected;
  }

  function exportCsv() {
    if (!model || selected.length === 0) return;
    const text = buildCsv(model, selected, view.from, view.to);
    const blob = new Blob([text], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    const base = fileName.replace(/\.athrp$/i, '') || 'replay';
    link.href = url;
    link.download = `${base}-${Math.round((view.from - model.startMicros) / 1000)}ms.csv`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
    exportNote = `Exported ${selected.length} signal(s) over the visible range.`;
  }

  function jumpTo(micros: number) {
    cursor = micros;
  }
</script>

<section class="analysis">
  <div class="toolbar">
    <label class="file">
      <span>Open replay</span>
      <input type="file" accept=".athrp" onchange={onFile} />
    </label>
    {#if model}
      <span class="meta">{fileName} | {model.frameCount} frames | {model.markers.length} markers
        {#if model.sequenceGaps > 0}| <span class="warn">{model.sequenceGaps} gaps</span>{/if}
      </span>
      <div class="zoom" role="group" aria-label="visible window">
        {#each ZOOM_OPTIONS as option (option.label)}
          <button type="button" class:active={zoomMicros === option.micros} onclick={() => (zoomMicros = option.micros)}>
            {option.label}
          </button>
        {/each}
      </div>
      <button type="button" class="export" onclick={exportCsv} disabled={selected.length === 0}>Export CSV</button>
    {/if}
  </div>

  {#if loading}
    <p class="note">Reading replay...</p>
  {:else if loadError}
    <p class="error">{loadError}</p>
  {/if}

  {#if exportNote}<p class="note">{exportNote}</p>{/if}

  {#if !model}
    <div class="empty">
      <p>Open a <code>.athrp</code> replay to scrub the timeline, plot signals in sync, and export CSV.</p>
      <p class="muted">Everything is read in this window; no data leaves the browser.</p>
    </div>
  {:else}
    <div class="scrub">
      <ReplayScrubber {model} {cursor} onscrub={jumpTo} />
      {#if model.markers.length > 0}
        <ol class="markers" aria-label="markers">
          {#each model.markers as marker (marker.timestampMicros + marker.kind + marker.message)}
            <li>
              <button type="button" onclick={() => jumpTo(marker.timestampMicros)}>
                <span class="when">{((marker.timestampMicros - model.startMicros) / 1_000_000).toFixed(2)} s</span>
                <span>{marker.label}</span>
                <span class="muted">{marker.message}</span>
              </button>
            </li>
          {/each}
        </ol>
      {/if}
    </div>

    <div class="body">
      <aside>
        <SignalPicker {signals} {selected} max={MAX_PLOTS} ontoggle={toggle} />
      </aside>
      <div class="plots">
        {#if fieldSignal !== null}
          <FieldView {model} signalId={fieldSignal} {cursor} />
        {/if}
        {#each selected.filter((id) => !poseIds.includes(id)) as id (id)}
          <PlotPanel
            {model}
            signalId={id}
            from={view.from}
            to={view.to}
            {cursor}
            onremove={() => toggle(id)}
          />
        {:else}
          {#if fieldSignal === null}
            <p class="muted">Pick signals on the left to plot them against each other.</p>
          {/if}
        {/each}
      </div>
    </div>
  {/if}
</section>

<style>
  .analysis {
    display: grid;
    gap: 12px;
  }
  .toolbar {
    display: flex;
    flex-wrap: wrap;
    gap: 10px;
    align-items: center;
  }
  .file {
    display: inline-flex;
    gap: 8px;
    align-items: center;
    font-size: 13px;
  }
  .file span,
  .export,
  .zoom button {
    background: var(--surface-3);
    color: var(--text-strong);
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    padding: 6px 10px;
    font: inherit;
    cursor: pointer;
  }
  .zoom {
    display: inline-flex;
    gap: 4px;
  }
  .zoom button.active {
    border-color: #38bdf8;
  }
  .export:disabled {
    opacity: 0.5;
    cursor: default;
  }
  .meta {
    font-size: 12px;
    color: var(--text-soft);
  }
  .warn {
    color: #fbbf24;
  }
  .note {
    margin: 0;
    font-size: 13px;
    color: var(--text-soft);
  }
  .error {
    margin: 0;
    color: var(--danger);
  }
  .empty {
    padding: 32px;
    border: 1px dashed var(--border);
    border-radius: 12px;
    color: var(--text);
  }
  .muted {
    color: var(--text-soft);
  }
  .scrub {
    display: grid;
    gap: 8px;
    padding: 10px 12px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
  }
  .markers {
    list-style: none;
    margin: 0;
    padding: 0;
    display: flex;
    flex-wrap: wrap;
    gap: 6px;
  }
  .markers button {
    display: inline-flex;
    gap: 6px;
    align-items: baseline;
    background: transparent;
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 3px 8px;
    color: var(--text);
    font: inherit;
    font-size: 12px;
    cursor: pointer;
  }
  .when {
    font-family: var(--font-mono);
    color: #fbbf24;
  }
  .body {
    display: grid;
    grid-template-columns: minmax(220px, 300px) minmax(0, 1fr);
    gap: 12px;
    align-items: start;
  }
  .plots {
    display: grid;
    gap: 10px;
    min-width: 0;
  }
  @media (max-width: 860px) {
    .body {
      grid-template-columns: 1fr;
    }
  }
</style>
