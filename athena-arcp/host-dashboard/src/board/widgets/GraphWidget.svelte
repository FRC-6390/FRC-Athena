<script lang="ts">
  import type { WidgetContext } from '../../lib/board/types';
  import { formatValue } from '../../lib/format';
  import { windowSeries, yDomain, toSvgPoints, type PlotPoint } from '../../lib/mechanisms/plot';

  let { ctx }: { ctx: WidgetContext } = $props();

  const PALETTE = ['var(--series-measured)', 'var(--series-reference)', 'var(--series-target)', 'var(--series-4)', 'var(--series-5)', 'var(--text)'];
  const NAMED: Record<string, string> = { target: 'var(--series-target)', reference: 'var(--series-reference)', measured: 'var(--series-measured)' };

  const series = $derived(ctx.bindings.series ?? []);
  const windowSeconds = $derived(ctx.setting<number>('window'));
  const finite = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? v : null);
  const fixedMin = $derived(finite(ctx.setting<number>('min')));
  const fixedMax = $derived(finite(ctx.setting<number>('max')));
  const legend = $derived(ctx.setting<boolean>('legend'));
  const plotWidth = $derived(Math.max(50, ctx.size.width));
  const plotHeight = $derived(Math.max(30, ctx.size.height - (legend ? 22 : 0)));

  const plotted = $derived.by(() => {
    void ctx.live.current;
    const newest = Math.max(0, ...series.map((s) => ctx.session.store.latestSample(s.signalId)?.timestampMicros ?? 0));
    const points = series.map((s) => windowSeries(ctx.samples(s), newest, windowSeconds) as PlotPoint[]);
    const raw = yDomain(points, { min: fixedMin, max: fixedMax });
    const mid = (raw.max + raw.min) / 2;
    const minSpan = Math.max(Math.abs(mid) * 0.1, 0.5);
    const domain =
      fixedMin !== null && fixedMax !== null
        ? raw
        : raw.max - raw.min >= minSpan
          ? raw
          : { min: mid - minSpan / 2, max: mid + minSpan / 2 };
    return {
      domain,
      lines: series.map((s, i) => ({
        signal: s,
        color: NAMED[s.name] ?? PALETTE[i % PALETTE.length],
        points: toSvgPoints(points[i], { width: plotWidth, height: plotHeight, padding: 6 }, windowSeconds, domain),
        latest: ctx.latest(s)
      }))
    };
  });
</script>

<div class="graph">
  <svg width={plotWidth} height={plotHeight} viewBox="0 0 {plotWidth} {plotHeight}" role="img" aria-label="graph">
    {#each [0.25, 0.5, 0.75] as f (f)}
      <line x1="0" x2={plotWidth} y1={plotHeight * f} y2={plotHeight * f} class="grid" />
    {/each}
    {#each plotted.lines as line (line.signal.signalId)}
      <polyline points={line.points} fill="none" stroke={line.color} stroke-width="1.6" stroke-linejoin="round" />
    {/each}
  </svg>
  <span class="axis top ui-mono">{formatValue(plotted.domain.max)}</span>
  <span class="axis bottom ui-mono" style="bottom:{legend ? 26 : 4}px">{formatValue(plotted.domain.min)}</span>
  {#if legend}
    <div class="legend">
      {#each plotted.lines as line (line.signal.signalId)}
        <span class="item"><i style="background:{line.color}"></i>{line.signal.name}<b class="ui-mono">{formatValue(line.latest)}</b></span>
      {/each}
    </div>
  {/if}
</div>

<style>
  .graph {
    position: relative;
    height: 100%;
    overflow: hidden;
  }
  svg {
    display: block;
  }
  .grid {
    stroke: var(--border);
    stroke-width: 1;
  }
  .axis {
    position: absolute;
    left: 6px;
    font-size: 10px;
    color: var(--text-faint);
    pointer-events: none;
  }
  .axis.top {
    top: 4px;
  }
  .legend {
    display: flex;
    gap: 12px;
    height: 22px;
    align-items: center;
    padding: 0 8px;
    overflow: hidden;
    white-space: nowrap;
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .item {
    display: inline-flex;
    align-items: center;
    gap: 5px;
  }
  .item i {
    width: 10px;
    height: 2px;
  }
  .item b {
    color: var(--text-strong);
    font-weight: 500;
  }
</style>
