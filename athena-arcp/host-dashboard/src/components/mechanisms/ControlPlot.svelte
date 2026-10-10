<script lang="ts">
  import type { ValueStore } from '../../lib/data/store';
  import type { SignalDescriptor } from '../../lib/data/types';
  import { formatValue } from '../../lib/format';
  import { windowSeries, yDomain, toSvgPoints, type PlotPoint } from '../../lib/mechanisms/plot';

  type Props = {
    signals: SignalDescriptor[];
    store: ValueStore;
    live: { readonly current: number };
    windowSeconds?: number;
    yMin?: number | null;
    yMax?: number | null;
  };

  let { signals, store, live, windowSeconds = 20, yMin = null, yMax = null }: Props = $props();

  const COLORS: Record<string, string> = {
    target: '#f59e0b',
    reference: '#38bdf8',
    measured: '#34d399'
  };
  const WIDTH = 480;
  const HEIGHT = 140;

  const plotted = $derived.by(() => {
    void live.current;
    const newest = Math.max(
      0,
      ...signals.map((s) => store.latestSample(s.signalId)?.timestampMicros ?? 0)
    );
    const series = signals.map((s) => ({
      signal: s,
      points: windowSeries(store.samples(s.signalId), newest, windowSeconds) as PlotPoint[]
    }));
    const domain = yDomain(
      series.map((s) => s.points),
      { min: yMin, max: yMax }
    );
    return {
      domain,
      lines: series.map((s) => ({
        signal: s.signal,
        color: COLORS[s.signal.name] ?? '#e2e8f0',
        points: toSvgPoints(s.points, { width: WIDTH, height: HEIGHT, padding: 6 }, windowSeconds, domain),
        latest: store.latestSample(s.signal.signalId)?.value
      }))
    };
  });
</script>

<figure class="plot">
  <svg viewBox="0 0 {WIDTH} {HEIGHT}" preserveAspectRatio="none" role="img" aria-label="control trace">
    <line x1="0" x2={WIDTH} y1={HEIGHT / 2} y2={HEIGHT / 2} class="grid" />
    {#each plotted.lines as line (line.signal.signalId)}
      <polyline points={line.points} fill="none" stroke={line.color} stroke-width="1.6" vector-effect="non-scaling-stroke" />
    {/each}
  </svg>
  <div class="scale">
    <span>{formatValue(plotted.domain.max)}</span>
    <span>{windowSeconds} s</span>
    <span>{formatValue(plotted.domain.min)}</span>
  </div>
  <ul class="legend">
    {#each plotted.lines as line (line.signal.signalId)}
      <li>
        <i style="background:{line.color}"></i>
        <span>{line.signal.name}</span>
        <b>{formatValue(line.latest)}</b>
      </li>
    {/each}
  </ul>
</figure>

<style>
  .plot {
    margin: 0;
    display: grid;
    gap: 6px;
  }
  svg {
    width: 100%;
    height: 140px;
    background: var(--bg-2);
    border: 1px solid var(--border);
    border-radius: 8px;
  }
  .grid {
    stroke: var(--border);
    stroke-dasharray: 4 4;
  }
  .scale {
    display: flex;
    justify-content: space-between;
    font-size: 10px;
    color: var(--text-soft);
    font-family: var(--font-mono);
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 12px;
    list-style: none;
    margin: 0;
    padding: 0;
    font-size: 12px;
  }
  .legend li {
    display: flex;
    gap: 6px;
    align-items: center;
  }
  .legend i {
    width: 10px;
    height: 10px;
    border-radius: 2px;
    display: inline-block;
  }
  .legend b {
    font-family: var(--font-mono);
    color: var(--text-strong);
    font-weight: 500;
  }
</style>
