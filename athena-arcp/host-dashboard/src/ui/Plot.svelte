<script lang="ts">
  import type { ValueStore } from '../lib/data/store';
  import type { SignalDescriptor } from '../lib/data/types';
  import { formatValue } from '../lib/format';
  import { windowSeries, yDomain, toSvgPoints, type PlotPoint } from '../lib/mechanisms/plot';

  type Props = {
    signals: SignalDescriptor[];
    store: ValueStore;
    live: { readonly current: number };
    windowSeconds?: number;
    height?: number;
  };

  let { signals, store, live, windowSeconds = 10, height = 150 }: Props = $props();

  const SERIES: Record<string, string> = {
    target: 'var(--series-target)',
    reference: 'var(--series-reference)',
    measured: 'var(--series-measured)'
  };
  const FALLBACK = ['var(--series-4)', 'var(--series-5)', 'var(--text)'];
  const WIDTH = 600;

  const plotted = $derived.by(() => {
    void live.current;
    const newest = Math.max(0, ...signals.map((s) => store.latestSample(s.signalId)?.timestampMicros ?? 0));
    const series = signals.map((s) => ({
      signal: s,
      points: windowSeries(store.samples(s.signalId), newest, windowSeconds) as PlotPoint[]
    }));
    const raw = yDomain(series.map((s) => s.points), { min: null, max: null });
    // Keep a minimum visible span so sensor jitter around a steady value stays flat instead of filling the plot.
    const mid = (raw.max + raw.min) / 2;
    const minSpan = Math.max(Math.abs(mid) * 0.2, 1);
    const domain = raw.max - raw.min >= minSpan ? raw : { min: mid - minSpan / 2, max: mid + minSpan / 2 };
    const ticks = [domain.max, (domain.max + domain.min) / 2, domain.min];
    return {
      ticks,
      lines: series.map((s, i) => ({
        signal: s.signal,
        color: SERIES[s.signal.name] ?? FALLBACK[i % FALLBACK.length],
        points: toSvgPoints(s.points, { width: WIDTH, height, padding: 8 }, windowSeconds, domain),
        latest: store.latestSample(s.signal.signalId)?.value
      }))
    };
  });
</script>

<figure class="plot">
  <div class="canvas" style="height:{height}px">
    <svg viewBox="0 0 {WIDTH} {height}" preserveAspectRatio="none" role="img" aria-label="signal trace">
      {#each [0.25, 0.5, 0.75] as f (f)}
        <line x1="0" x2={WIDTH} y1={height * f} y2={height * f} class="grid" />
      {/each}
      {#each plotted.lines as line (line.signal.signalId)}
        <polyline points={line.points} fill="none" stroke={line.color} stroke-width="1.5" vector-effect="non-scaling-stroke" stroke-linejoin="round" />
      {/each}
    </svg>
    <div class="axis ui-mono">
      {#each plotted.ticks as tick, i (i)}<span>{formatValue(tick)}</span>{/each}
    </div>
  </div>
  <figcaption class="legend">
    {#each plotted.lines as line (line.signal.signalId)}
      <span class="item">
        <i style="background:{line.color}"></i>
        <span class="name">{line.signal.name}</span>
        <span class="val ui-mono">{formatValue(line.latest)}</span>
      </span>
    {/each}
    <span class="ui-spacer"></span>
    <span class="window ui-mono">{windowSeconds}s</span>
  </figcaption>
</figure>

<style>
  .plot {
    margin: 0;
    display: grid;
    gap: var(--sp-2);
  }
  .canvas {
    position: relative;
    border-radius: var(--radius);
    background: var(--bg-1);
    border: 1px solid var(--border);
    overflow: hidden;
  }
  svg {
    display: block;
    width: 100%;
    height: 100%;
  }
  .grid {
    stroke: var(--border);
    stroke-width: 1;
    vector-effect: non-scaling-stroke;
  }
  .axis {
    position: absolute;
    inset: 4px auto 4px 6px;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    font-size: 10px;
    color: var(--text-faint);
    pointer-events: none;
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: var(--sp-4);
    font-size: var(--fs-xs);
  }
  .item {
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }
  .item i {
    width: 10px;
    height: 2px;
    border-radius: 1px;
  }
  .name {
    color: var(--text-soft);
  }
  .val {
    color: var(--text-strong);
  }
  .window {
    color: var(--text-faint);
  }
</style>
