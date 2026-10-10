<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { batteryReading, sparklinePoints } from '../../../lib/board/widgets/match-battery';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.volts?.[0]);
  const warnBelow = $derived(ctx.setting<number>('warnBelow'));
  const faultBelow = $derived(ctx.setting<number>('faultBelow'));
  const windowSeconds = $derived(ctx.setting<number>('window'));

  const reading = $derived.by(() => {
    void ctx.live.current;
    if (!signal) return null;
    return batteryReading(ctx.samples(signal), { warnBelow, faultBelow, windowSeconds });
  });
  const chart = $derived(reading ? sparklinePoints(reading.points, 300, 60, 3) : '');
</script>

<div class="battery" data-tone={reading?.tone ?? 'none'}>
  <div class="number-row">
    <span class="volts ui-mono">{reading?.volts !== null && reading?.volts !== undefined ? reading.volts.toFixed(2) : '--'}</span>
    <span class="unit">V</span>
  </div>
  <div class="sag">
    <span class="ui-mono">min {reading?.min !== null && reading?.min !== undefined ? reading.min.toFixed(2) : '--'} V</span>
    <span class="ui-mono">{windowSeconds} s</span>
  </div>
  <svg class="chart" viewBox="0 0 300 60" preserveAspectRatio="none" aria-hidden="true">
    <line x1="0" x2="300" y1="30" y2="30" class="grid" />
    {#if chart}
      <polyline points={chart} class="line" vector-effect="non-scaling-stroke" />
    {/if}
  </svg>
</div>

<style>
  .battery {
    width: 100%;
    height: 100%;
    display: grid;
    grid-template-rows: 1fr auto minmax(24px, 30%);
    gap: 4px;
    padding: 8px 12px;
    box-sizing: border-box;
    container-type: size;
    color: var(--text-strong);
  }
  .number-row {
    display: flex;
    align-items: baseline;
    justify-content: center;
    gap: 0.15em;
    min-height: 0;
  }
  .volts {
    /* The number row gets about half the card; the sag line and chart take the rest. */
    font-size: min(40cqh, 28cqw);
    font-weight: 700;
    line-height: 1;
    letter-spacing: -0.02em;
  }
  .unit {
    font-size: min(16cqh, 10cqw);
    font-weight: 600;
    color: var(--text-soft);
  }
  .battery[data-tone='ok'] .volts {
    color: var(--ok);
  }
  .battery[data-tone='warn'] .volts {
    color: var(--warn);
  }
  .battery[data-tone='fault'] .volts {
    color: var(--fault);
  }
  .sag {
    display: flex;
    justify-content: space-between;
    gap: 6px;
    font-size: var(--fs-sm);
    color: var(--text-soft);
    white-space: nowrap;
    min-width: 0;
  }
  .chart {
    width: 100%;
    height: 100%;
    min-height: 24px;
  }
  .grid {
    stroke: var(--border-subtle);
    stroke-width: 1;
    stroke-dasharray: 4 4;
    vector-effect: non-scaling-stroke;
  }
  .line {
    fill: none;
    stroke: var(--ok);
    stroke-width: 2;
    stroke-linejoin: round;
  }
  .battery[data-tone='warn'] .line {
    stroke: var(--warn);
  }
  .battery[data-tone='fault'] .line {
    stroke: var(--fault);
  }
</style>
