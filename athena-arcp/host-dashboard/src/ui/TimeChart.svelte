<script lang="ts">
  import uPlot from 'uplot';
  import 'uplot/dist/uPlot.min.css';
  import { untrack } from 'svelte';
  import { formatNumber } from '../lib/format';
  import { themeColor } from '../lib/board/widgets/robot-canvas';
  import {
    buildChartData,
    relativeLabel,
    valueHeldAt,
    visibleX,
    yDomainFor,
    type ChartData,
    type ChartSeries,
    type Range
  } from '../lib/chart/time-chart';

  /**
   * Time-series chart over the store's samples. The x axis is the trailing `windowSeconds` (ending at the
   * newest sample); the y axis is automatic unless `yMin` / `yMax` fix a side. Hover shows every visible
   * series at the cursor time; drag across the plot to zoom in time, double-click to reset; pause freezes the
   * picture while data keeps arriving; the legend shows values (cursor or latest) and hides/shows series.
   */
  type Props = {
    series: ChartSeries[];
    windowSeconds?: number;
    yMin?: number | null;
    yMax?: number | null;
    /** Show the legend row under the plot. */
    legend?: boolean;
    /** Fixed height in CSS pixels; when omitted the chart fills its parent. */
    height?: number;
  };

  let { series, windowSeconds = 10, yMin = null, yMax = null, legend = true, height }: Props = $props();

  type Hover = { x: number; values: (number | null)[]; left: number; top: number };

  let paused = $state(false);
  let zoom = $state<Range | null>(null);
  let hidden = $state<Record<string, boolean>>({});
  let latest = $state<(number | null)[]>([]);
  let hover = $state<Hover | null>(null);
  let plotEl = $state<HTMLDivElement | undefined>(undefined);
  let themeVersion = $state(0);

  /** Changes when the set, order, id or colour of the lines changes: the uPlot instance is rebuilt then. */
  const structure = $derived(series.map((s) => `${s.id}|${s.color}|${s.dashed ? 1 : 0}`).join(';'));

  let plot: uPlot | null = null;
  let frozen: ChartData | null = null;
  let newest = 0;
  let plotWidth = 0;

  const FONT = '11px ui-monospace, SFMono-Regular, Menlo, monospace';

  /** uPlot draws to canvas, which cannot read `var(--token)`: resolve theme tokens to colours at build time. */
  function resolveColor(color: string): string {
    const m = /^var\((--[\w-]+)\)$/.exec(color.trim());
    return m ? themeColor(m[1], '#888') : color;
  }

  function onCursor(u: uPlot) {
    const idx = u.cursor.idx;
    if (idx === null || idx === undefined || idx < 0 || u.cursor.left === undefined || u.cursor.left < 0) {
      hover = null;
      return;
    }
    const xs = u.data[0];
    const cursorLeft = u.cursor.left ?? 0;
    const cursorTop = u.cursor.top ?? 0;
    hover = {
      x: xs[idx] - newest,
      values: u.data.slice(1).map((col) => valueHeldAt(col as (number | null)[], idx)),
      left: u.over.offsetLeft + cursorLeft,
      top: u.over.offsetTop + cursorTop
    };
  }

  function onSelect(u: uPlot) {
    if (u.select.width < 4) return;
    const a = u.posToVal(u.select.left, 'x');
    const b = u.posToVal(u.select.left + u.select.width, 'x');
    u.setSelect({ left: 0, top: 0, width: 0, height: 0 }, false);
    zoom = { min: Math.min(a, b), max: Math.max(a, b) };
  }

  function resetZoom() {
    zoom = null;
  }

  // Build (or rebuild) the uPlot instance when the element, theme or line structure changes. Data is pushed
  // by the effect below, so live updates never recreate the chart.
  $effect(() => {
    void themeVersion;
    void structure;
    const el = plotEl;
    if (!el) return;
    const specs = untrack(() => series);
    const faint = resolveColor('var(--text-faint)');
    const border = resolveColor('var(--border)');
    const grid = { show: true, stroke: border, width: 1 };
    const ticks = { show: true, stroke: border, width: 1, size: 4 };

    const opts: uPlot.Options = {
      width: Math.max(50, el.clientWidth),
      height: Math.max(50, el.clientHeight),
      legend: { show: false },
      cursor: {
        x: true,
        y: false,
        drag: { x: true, y: false, setScale: false, dist: 4 }
      },
      scales: { x: { time: false }, y: {} },
      axes: [
        {
          stroke: faint,
          font: FONT,
          grid,
          ticks,
          gap: 4,
          size: 22,
          values: (_u, vals) => vals.map((v) => relativeLabel(v - newest))
        },
        {
          stroke: faint,
          font: FONT,
          grid,
          ticks,
          gap: 4,
          size: 58,
          values: (_u, vals) => vals.map((v) => formatNumber(v))
        }
      ],
      series: [
        {},
        ...specs.map((s) => ({
          label: s.label,
          stroke: resolveColor(s.color),
          width: 1.6,
          dash: s.dashed ? [4, 3] : undefined,
          spanGaps: true,
          points: { show: false }
        }))
      ],
      hooks: {
        setCursor: [(u) => onCursor(u)],
        setSelect: [(u) => onSelect(u)]
      }
    };

    const u = new uPlot(opts, [[0, 1], ...specs.map(() => [null, null])] as unknown as uPlot.AlignedData, el);
    plot = u;
    frozen = null;
    u.over.addEventListener('dblclick', resetZoom);
    const ro = new ResizeObserver(() => {
      const w = el.clientWidth;
      const h = el.clientHeight;
      if (w > 0 && h > 0) {
        plotWidth = w;
        u.setSize({ width: w, height: h });
      }
    });
    ro.observe(el);
    untrack(() => apply());

    return () => {
      ro.disconnect();
      u.over.removeEventListener('dblclick', resetZoom);
      u.destroy();
      if (plot === u) plot = null;
    };
  });

  // Dark/light follows the system setting; tokens are re-read when it changes.
  $effect(() => {
    const mq = window.matchMedia('(prefers-color-scheme: dark)');
    const bump = () => (themeVersion += 1);
    mq.addEventListener('change', bump);
    return () => mq.removeEventListener('change', bump);
  });

  /** Pushes the current samples (or the frozen table while paused) and the axis ranges into the plot. */
  function apply() {
    const u = plot;
    if (!u) return;
    const list = series;
    const W = Math.max(1, windowSeconds);
    const zoomNow = zoom;
    const isHidden = list.map((s) => Boolean(hidden[s.id]));

    let data: ChartData;
    if (paused && frozen && frozen.length === list.length + 1) {
      data = frozen;
    } else {
      const built = buildChartData(list, W, zoomNow ? zoomNow.min : null);
      data = built.data;
      newest = built.newestSeconds;
      frozen = data;
    }

    const x = visibleX(newest, W, zoomNow);
    const y = yDomainFor(data, x, isHidden, { min: yMin ?? null, max: yMax ?? null });
    latest = data.slice(1).map((col) => valueHeldAt(col as (number | null)[], col.length - 1));

    u.batch(() => {
      u.setData(data as unknown as uPlot.AlignedData, false);
      list.forEach((_, i) => u.setSeries(i + 1, { show: !isHidden[i] }, false));
      u.setScale('x', x);
      u.setScale('y', y);
    });
  }

  // Re-run whenever the samples, window, bounds, zoom or visibility change. Reading them here is what tracks them.
  $effect(() => {
    series;
    windowSeconds;
    yMin;
    yMax;
    zoom;
    paused;
    hidden;
    apply();
  });

  function toggleSeries(id: string) {
    hidden = { ...hidden, [id]: !hidden[id] };
  }

  function formatCell(v: number | null | undefined): string {
    return v === null || v === undefined ? '--' : formatNumber(v);
  }

  const tipFlipped = $derived(hover !== null && hover.left > plotWidth / 2);
</script>

<div class="chart" style={height !== undefined ? `height:${height}px` : undefined}>
  <div class="bar">
    {#if paused}<span class="ui-badge" data-tone="warn">paused</span>{/if}
    <span class="hint">{zoom ? 'zoomed in time' : 'drag to zoom'} · double-click to reset</span>
    <span class="ui-spacer"></span>
    {#if zoom}
      <button type="button" class="ui-btn" data-variant="ghost" onclick={resetZoom}>Reset zoom</button>
    {/if}
    <button type="button" class="ui-btn" data-variant="ghost" aria-pressed={paused} onclick={() => (paused = !paused)}>
      {paused ? 'Resume' : 'Pause'}
    </button>
  </div>

  <div class="plot" bind:this={plotEl}>
    {#if hover}
      <div class="tip ui-mono" class:flipped={tipFlipped} style="left:{hover.left}px;top:{hover.top}px">
        <div class="tip-time">{relativeLabel(hover.x)}</div>
        {#each series as s, i (s.id)}
          {#if !hidden[s.id]}
            <div class="tip-row">
              <i style="background:{resolveColor(s.color)}"></i>
              <span>{s.label}</span>
              <b>{formatCell(hover.values[i])}</b>
            </div>
          {/if}
        {/each}
      </div>
    {/if}
  </div>

  {#if legend}
    <div class="legend">
      {#each series as s, i (s.id)}
        <button
          type="button"
          class="item"
          class:off={hidden[s.id]}
          aria-pressed={!hidden[s.id]}
          title={hidden[s.id] ? 'Show this series' : 'Hide this series'}
          onclick={() => toggleSeries(s.id)}
        >
          <i style="background:{resolveColor(s.color)}"></i>
          <span class="name">{s.label}</span>
          <b class="ui-mono">{formatCell(hover ? hover.values[i] : latest[i])}</b>
        </button>
      {/each}
      <span class="ui-spacer"></span>
      <span class="window ui-mono">{windowSeconds}s</span>
    </div>
  {/if}
</div>

<style>
  .chart {
    display: flex;
    flex-direction: column;
    gap: 4px;
    min-height: 0;
    height: 100%;
    min-width: 0;
  }
  .bar {
    display: flex;
    align-items: center;
    gap: var(--sp-2);
    min-height: 22px;
    font-size: var(--fs-xs);
  }
  .hint {
    color: var(--text-faint);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .plot {
    position: relative;
    flex: 1;
    min-height: 80px;
    min-width: 0;
    border-radius: var(--radius);
    background: var(--surface-2);
    border: 1px solid var(--border);
    overflow: hidden;
  }
  .plot :global(.uplot) {
    font-family: var(--font-mono);
  }
  .plot :global(.u-over) {
    cursor: crosshair;
  }
  .plot :global(.u-select) {
    background: var(--brand-soft);
    border-left: 1px solid var(--border-emphasis);
    border-right: 1px solid var(--border-emphasis);
  }
  .tip {
    position: absolute;
    z-index: 2;
    margin: 10px 0 0 10px;
    padding: 6px 8px;
    min-width: 120px;
    border-radius: var(--radius-sm);
    border: 1px solid var(--border-emphasis);
    background: var(--surface-3);
    color: var(--text);
    font-size: var(--fs-xs);
    pointer-events: none;
    white-space: nowrap;
  }
  .tip.flipped {
    transform: translateX(calc(-100% - 20px));
  }
  .tip-time {
    color: var(--text-soft);
    margin-bottom: 2px;
  }
  .tip-row {
    display: grid;
    grid-template-columns: 8px 1fr auto;
    align-items: center;
    gap: 6px;
  }
  .tip-row i,
  .item i {
    width: 10px;
    height: 2px;
    border-radius: 1px;
    display: inline-block;
  }
  .tip-row b {
    color: var(--text-strong);
    font-weight: 500;
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 4px var(--sp-4);
    font-size: var(--fs-xs);
  }
  .item {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 0;
    border: none;
    background: transparent;
    color: var(--text-soft);
    font: inherit;
    font-size: inherit;
    cursor: pointer;
  }
  .item.off {
    opacity: 0.4;
  }
  .item b {
    color: var(--text-strong);
    font-weight: 500;
  }
  .window {
    color: var(--text-faint);
  }
</style>
