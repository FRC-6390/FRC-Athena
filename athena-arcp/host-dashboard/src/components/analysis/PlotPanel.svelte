<script lang="ts">
  import { onDestroy } from 'svelte';
  import type { ReplayModel } from '../../lib/analysis/replay';
  import { decimate, toClipSpace, windowPoints, yRangeOf, type YRange } from '../../lib/analysis/plot-data';
  import { WebGLLinePlot } from '../../lib/analysis/webgl-lines';

  type Props = {
    model: ReplayModel;
    signalId: number;
    /** Visible window in robot micros. */
    from: number;
    to: number;
    cursor: number;
    onremove: () => void;
  };

  let { model, signalId, from, to, cursor, onremove }: Props = $props();

  let canvas = $state<HTMLCanvasElement | null>(null);
  let wrap = $state<HTMLDivElement | null>(null);
  let pixelWidth = $state(640);
  let plot: WebGLLinePlot | null = null;
  let error = $state<string | null>(null);

  const descriptor = $derived(model.descriptor(signalId));
  const numeric = $derived(model.numericTrack(signalId));
  const xMin = $derived((from - model.startMicros) / 1_000_000);
  const xMax = $derived((to - model.startMicros) / 1_000_000);

  // Derived values only: the effect below just draws them, so it never writes state it reads.
  const points = $derived.by(() => {
    const track = numeric;
    if (!track) return [];
    return decimate(windowPoints(track.t, track.v, from, to, model.startMicros), pixelWidth);
  });
  const range = $derived<YRange>(yRangeOf(points));
  const latest = $derived.by(() => {
    void cursor;
    const at = model.valueAt(signalId, cursor);
    return at === undefined ? '--' : typeof at === 'boolean' ? (at ? 'true' : 'false') : String(at);
  });

  $effect(() => {
    const target = canvas;
    if (!target) return;
    if (!plot) {
      try {
        plot = new WebGLLinePlot(target);
      } catch (e) {
        error = e instanceof Error ? e.message : String(e);
        return;
      }
    }
    plot.resize();
    plot.draw([{ vertices: toClipSpace(points, xMin, xMax, range), color: [0.35, 0.78, 1, 1] }]);
  });

  // Decimation follows the drawn width, so each plot does work proportional to its pixels.
  $effect(() => {
    const element = wrap;
    if (!element) return;
    const observer = new ResizeObserver(() => {
      pixelWidth = Math.max(16, Math.round(element.clientWidth * Math.min(2, window.devicePixelRatio || 1)));
    });
    observer.observe(element);
    return () => observer.disconnect();
  });

  const cursorPercent = $derived(
    to > from ? Math.min(100, Math.max(0, ((cursor - from) / (to - from)) * 100)) : 0
  );

  onDestroy(() => plot?.dispose());
</script>

<article class="plot" data-path={descriptor?.path}>
  <header>
    <span class="name" title={descriptor?.path}>{descriptor?.path ?? `#${signalId}`}</span>
    <span class="latest">{latest}{descriptor?.unit ? ` ${descriptor.unit}` : ''}</span>
    <button type="button" onclick={onremove} aria-label="remove plot">x</button>
  </header>
  <div class="canvas-wrap" bind:this={wrap}>
    {#if error}
      <p class="error">{error}</p>
    {:else}
      <canvas bind:this={canvas} aria-label="plot of {descriptor?.path ?? signalId}"></canvas>
      <div class="cursor" style="left: {cursorPercent}%"></div>
      <span class="range top">{range.max.toFixed(2)}</span>
      <span class="range bottom">{range.min.toFixed(2)}</span>
    {/if}
  </div>
</article>

<style>
  .plot {
    display: grid;
    gap: 4px;
    padding: 8px 10px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    min-width: 0;
  }
  header {
    display: flex;
    gap: 8px;
    align-items: center;
    min-width: 0;
  }
  .name {
    flex: 1;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-family: var(--font-mono);
    font-size: 12px;
    color: var(--text-soft);
  }
  .latest {
    font-family: var(--font-mono);
    font-size: 13px;
    color: var(--text-strong);
  }
  header button {
    background: transparent;
    border: 1px solid var(--border);
    color: var(--text-soft);
    border-radius: 5px;
    padding: 0 6px;
    cursor: pointer;
  }
  .canvas-wrap {
    position: relative;
    height: 120px;
  }
  canvas {
    width: 100%;
    height: 100%;
    display: block;
  }
  .cursor {
    position: absolute;
    top: 0;
    bottom: 0;
    width: 1px;
    background: rgba(251, 191, 36, 0.8);
    pointer-events: none;
  }
  .range {
    position: absolute;
    left: 4px;
    font-family: var(--font-mono);
    font-size: 10px;
    color: var(--text-soft);
    pointer-events: none;
  }
  .range.top {
    top: 0;
  }
  .range.bottom {
    bottom: 0;
  }
  .error {
    color: var(--danger);
    font-size: 12px;
  }
</style>
