<script lang="ts">
  import { onMount } from 'svelte';
  import { decodeReplay } from '../../lib/data/decoder';
  import { pendingReplay } from '../../lib/replay-handoff';
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

  /** A replay downloaded from the robot in the Pit workspace opens here. */
  onMount(() =>
    pendingReplay.subscribe((pending) => {
      if (!pending) return;
      pendingReplay.set(null);
      void load(new File([pending.bytes as BlobPart], pending.name));
    })
  );

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
  <div class="ui-panel toolbar">
    <label class="ui-btn" data-variant={model ? undefined : 'primary'}>
      Open replay
      <input type="file" accept=".athrp" onchange={onFile} hidden />
    </label>
    {#if model}
      <span class="meta">
        <b>{fileName}</b>
        <span class="ui-mono">{model.frameCount} frames · {model.markers.length} markers</span>
        {#if model.sequenceGaps > 0}<span class="ui-badge" data-tone="warn">{model.sequenceGaps} gaps</span>{/if}
      </span>
      <span class="ui-spacer"></span>
      <div class="segmented" role="group" aria-label="visible window">
        {#each ZOOM_OPTIONS as option (option.label)}
          <button type="button" class:active={zoomMicros === option.micros} onclick={() => (zoomMicros = option.micros)}>
            {option.label}
          </button>
        {/each}
      </div>
      <button type="button" class="ui-btn" onclick={exportCsv} disabled={selected.length === 0}>Export CSV</button>
    {/if}
  </div>
  {#if loading}
    <p class="note">Reading replay...</p>
  {:else if loadError}
    <p class="error">{loadError}</p>
  {/if}

  {#if exportNote}<p class="note">{exportNote}</p>{/if}

  {#if !model}
    <div class="ui-panel empty">
      <h3>No replay open</h3>
      <p>Open a <code>.athrp</code> file to scrub the timeline, plot signals in sync and export CSV.</p>
      <p class="muted">Replays recorded on the robot can be downloaded from Health → Robot files.</p>
    </div>
  {:else}
    <div class="ui-panel scrub">
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
      <aside class="ui-panel picker">
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
    gap: var(--sp-3);
  }
  .toolbar {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: var(--sp-3);
    padding: var(--sp-2) var(--sp-3);
  }
  .meta {
    display: inline-flex;
    align-items: center;
    gap: var(--sp-2);
    font-size: var(--fs-sm);
    color: var(--text-soft);
    min-width: 0;
  }
  .meta b {
    color: var(--text-strong);
    font-weight: 600;
  }
  .meta .ui-mono {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .segmented {
    display: inline-flex;
    padding: 2px;
    border: 1px solid var(--border-subtle);
    border-radius: var(--radius);
    background: var(--bg-1);
  }
  .segmented button {
    height: 24px;
    padding: 0 10px;
    border: none;
    border-radius: var(--radius-sm);
    background: transparent;
    color: var(--text-soft);
    font: inherit;
    font-size: var(--fs-xs);
    font-weight: 500;
    cursor: pointer;
  }
  .segmented button.active {
    background: var(--surface-3);
    color: var(--text-strong);
  }
  .note {
    margin: 0;
    font-size: var(--fs-sm);
    color: var(--text-soft);
  }
  .error {
    margin: 0;
    font-size: var(--fs-sm);
    color: var(--fault);
  }
  .empty {
    display: grid;
    justify-items: center;
    gap: var(--sp-2);
    padding: 64px var(--sp-4);
    text-align: center;
  }
  .empty h3 {
    margin: 0;
    font-size: var(--fs-lg);
    color: var(--text-strong);
  }
  .empty p {
    margin: 0;
    font-size: var(--fs-sm);
    color: var(--text);
  }
  .empty code {
    font-family: var(--font-mono);
  }
  .muted {
    color: var(--text-faint);
  }
  .scrub {
    display: grid;
    gap: var(--sp-2);
    padding: var(--sp-3);
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
    height: 24px;
    padding: 0 8px;
    border: 1px solid var(--border);
    border-radius: var(--radius-sm);
    background: var(--surface-2);
    color: var(--text);
    font: inherit;
    font-size: var(--fs-xs);
    cursor: pointer;
  }
  .markers button:hover {
    border-color: var(--border-emphasis);
  }
  .when {
    font-family: var(--font-mono);
    color: var(--warn);
  }
  .body {
    display: grid;
    grid-template-columns: minmax(220px, 300px) minmax(0, 1fr);
    gap: var(--sp-3);
    align-items: start;
  }
  .picker {
    padding: var(--sp-2);
  }
  .plots {
    display: grid;
    gap: var(--sp-3);
    min-width: 0;
  }
  @media (max-width: 860px) {
    .body {
      grid-template-columns: 1fr;
    }
  }
</style>
