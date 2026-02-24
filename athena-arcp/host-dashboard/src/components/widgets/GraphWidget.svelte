<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readGraphConfig, type GraphSeriesStyle } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signalById: Map<number, SignalRow>;
    historyBySignal: Map<number, number[]>;
    configRaw?: WidgetConfigRecord;
  };

  type SeriesRender = {
    signalId: number;
    label: string;
    color: string;
    role: string;
    style: GraphSeriesStyle;
    values: number[];
    latest: number | null;
  };

  let { signal, signalById, historyBySignal, configRaw }: Props = $props();

  function numericFromSignal(value: string): number | null {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  function yFor(value: number, min: number, max: number): number {
    const range = Math.max(1e-9, max - min);
    const ratio = (value - min) / range;
    return 34 - ratio * 30;
  }

  function xFor(index: number, total: number): number {
    if (total <= 1) return 50;
    return (index / (total - 1)) * 100;
  }

  function linePath(values: number[], maxSamples: number, min: number, max: number): string {
    if (values.length === 0) return 'M 0 18 L 100 18';
    const offset = Math.max(0, maxSamples - values.length);
    return values
      .map((value, index) => {
        const x = xFor(offset + index, maxSamples);
        const y = yFor(value, min, max);
        return `${index === 0 ? 'M' : 'L'} ${x.toFixed(2)} ${y.toFixed(2)}`;
      })
      .join(' ');
  }

  function stepPath(values: number[], maxSamples: number, min: number, max: number): string {
    if (values.length === 0) return 'M 0 18 L 100 18';

    const offset = Math.max(0, maxSamples - values.length);
    const points = values.map((value, index) => {
      return {
        x: xFor(offset + index, maxSamples),
        y: yFor(value, min, max)
      };
    });

    let path = `M ${points[0].x.toFixed(2)} ${points[0].y.toFixed(2)}`;
    for (let i = 1; i < points.length; i++) {
      path += ` L ${points[i].x.toFixed(2)} ${points[i - 1].y.toFixed(2)}`;
      path += ` L ${points[i].x.toFixed(2)} ${points[i].y.toFixed(2)}`;
    }
    return path;
  }

  const computed = $derived.by(() => {
    const config = readGraphConfig(configRaw, signal);

    const series = config.series
      .map((entry): SeriesRender | null => {
        const boundSignal = signalById.get(entry.signalId);
        const samples = [...(historyBySignal.get(entry.signalId) ?? [])];
        if (samples.length === 0 && boundSignal) {
          const current = numericFromSignal(boundSignal.value);
          if (current !== null) {
            samples.push(current);
          }
        }

        const latest = samples.length > 0 ? samples[samples.length - 1] ?? null : null;

        return {
          signalId: entry.signalId,
          label: entry.label,
          color: entry.color,
          role: entry.role,
          style: entry.style,
          values: samples,
          latest
        };
      })
      .filter((entry): entry is SeriesRender => entry !== null);

    let min = Number.POSITIVE_INFINITY;
    let max = Number.NEGATIVE_INFINITY;
    let maxSamples = 0;

    for (const entry of series) {
      if (entry.values.length > maxSamples) {
        maxSamples = entry.values.length;
      }

      for (const value of entry.values) {
        if (value < min) min = value;
        if (value > max) max = value;
      }
    }

    if (config.yMin !== null) min = config.yMin;
    if (config.yMax !== null) max = config.yMax;

    if (!Number.isFinite(min) || !Number.isFinite(max)) {
      min = 0;
      max = 1;
    }

    if (max <= min) {
      max = min + 1;
    }

    return {
      config,
      series,
      min,
      max,
      maxSamples: Math.max(2, maxSamples)
    };
  });
</script>

{#if computed.series.length === 0}
  <div class="graph-empty">No numeric series configured.</div>
{:else}
  <div class="graph-root">
    <svg viewBox="0 0 100 36" preserveAspectRatio="none" aria-label="graph widget">
      <line class="guide" x1="0" y1="4" x2="100" y2="4" />
      <line class="guide" x1="0" y1="18" x2="100" y2="18" />
      <line class="guide" x1="0" y1="32" x2="100" y2="32" />

      {#each computed.series as series (series.signalId)}
        <path
          d={
            series.style === 'step'
              ? stepPath(series.values, computed.maxSamples, computed.min, computed.max)
              : linePath(series.values, computed.maxSamples, computed.min, computed.max)
          }
          stroke={series.color}
          class={`series ${series.style}`}
        />

        {#if series.style === 'dot'}
          {#each series.values.slice(-18) as value, index (`${series.signalId}-${index}`)}
            {@const offset = Math.max(0, computed.maxSamples - series.values.length)}
            <circle
              cx={xFor(offset + series.values.length - Math.min(series.values.length, 18) + index, computed.maxSamples)}
              cy={yFor(value, computed.min, computed.max)}
              r="0.85"
              fill={series.color}
            />
          {/each}
        {/if}
      {/each}
    </svg>

    <div class="range-row">
      <span>{computed.min.toFixed(2)}</span>
      <span>{computed.max.toFixed(2)}</span>
    </div>

    {#if computed.config.showLegend}
      <div class="legend">
        {#each computed.series as series (series.signalId)}
          <span class="legend-item">
            <i style={`background:${series.color};`}></i>
            <strong>{series.label}</strong>
            <em>{series.role}</em>
            <b>{series.latest === null ? 'n/a' : series.latest.toFixed(3)}</b>
          </span>
        {/each}
      </div>
    {/if}
  </div>
{/if}

<style>
  .graph-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto auto;
    gap: 0.24rem;
  }

  .graph-empty {
    color: var(--text-soft);
    font-size: 0.72rem;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 2.4rem;
    border-radius: 6px;
    background: #1b2534;
    border: 1px solid rgba(99, 115, 140, 0.48);
  }

  .guide {
    stroke: rgba(153, 164, 180, 0.16);
    stroke-width: 0.45;
  }

  .series {
    fill: none;
    stroke-width: 1.2;
    vector-effect: non-scaling-stroke;
  }

  .series.dot {
    stroke-opacity: 0.45;
    stroke-dasharray: 1 1.5;
  }

  .range-row {
    display: flex;
    justify-content: space-between;
    color: var(--text-soft);
    font-family: var(--font-mono);
    font-size: 0.62rem;
    line-height: 1;
  }

  .legend {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
    gap: 0.2rem 0.34rem;
  }

  .legend-item {
    display: inline-grid;
    grid-template-columns: auto minmax(0, 1fr) auto;
    align-items: center;
    gap: 0.2rem 0.34rem;
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    background: var(--surface-3);
    padding: 0.15rem 0.22rem;
    font-size: 0.62rem;
  }

  .legend-item i {
    width: 0.48rem;
    height: 0.48rem;
    border-radius: 999px;
    display: block;
  }

  .legend-item strong {
    min-width: 0;
    color: var(--text);
    font-weight: 600;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .legend-item em {
    color: var(--text-soft);
    font-style: normal;
    text-transform: uppercase;
    letter-spacing: 0.02em;
  }

  .legend-item b {
    grid-column: 2 / 4;
    justify-self: end;
    color: var(--text-strong);
    font-family: var(--font-mono);
    font-weight: 600;
  }
</style>
