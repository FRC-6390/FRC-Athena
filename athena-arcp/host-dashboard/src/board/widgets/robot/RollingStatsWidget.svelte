<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { formatNumber } from '../../../lib/format';
  import { rollingStats } from '../../../lib/board/widgets/robot-series';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const windowSeconds = $derived(Math.max(1, ctx.setting<number>('window')));
  const bins = $derived(Math.min(32, Math.max(2, Math.round(ctx.setting<number>('bins')))));

  const stats = $derived.by(() => {
    void ctx.live.current;
    if (!signal) return null;
    const samples = ctx.samples(signal);
    const now = ctx.session.store.latestSample(signal.signalId)?.timestampMicros ?? 0;
    return rollingStats(samples, now, windowSeconds, bins);
  });

  const peak = $derived(stats ? Math.max(1, ...stats.histogram) : 1);
  const unit = $derived(signal?.unit ?? '');
  const tiles = $derived.by(() => {
    const s = stats;
    const f = (v: number | undefined) => (s && v !== undefined && Number.isFinite(v) ? formatNumber(v) : '--');
    return [
      { label: 'min', value: f(s?.min) },
      { label: 'max', value: f(s?.max) },
      { label: 'mean', value: f(s?.mean) },
      { label: 'std', value: f(s?.std) },
      { label: 'rate', value: s ? (s.rateHz >= 10 ? s.rateHz.toFixed(0) : s.rateHz.toFixed(1)) : '--', unit: 'Hz' }
    ];
  });
  const compact = $derived(ctx.size.height < 130);
</script>

<div class="stats" class:compact>
  <div class="tiles">
    {#each tiles as tile (tile.label)}
      <div class="tile">
        <span class="label">{tile.label}</span>
        <span class="value ui-mono">{tile.value}{#if tile.label !== 'rate' && unit && tile.value !== '--'}<small>{unit}</small>{:else if tile.unit}<small>{tile.unit}</small>{/if}</span>
      </div>
    {/each}
  </div>

  <div class="hist" aria-label="histogram">
    {#if stats && stats.histogram.length > 0}
      <svg viewBox="0 0 {bins * 10} 40" preserveAspectRatio="none" role="img">
        {#each stats.histogram as count, i (i)}
          {@const h = (count / peak) * 36}
          <rect x={i * 10 + 1} y={40 - h} width="8" height={h} rx="1" />
        {/each}
      </svg>
      <div class="axis ui-mono"><span>{formatNumber(stats.min)}</span><span>{stats.count} samples · {windowSeconds} s</span><span>{formatNumber(stats.max)}</span></div>
    {:else}
      <div class="empty">{signal ? 'no numeric samples in window' : 'bind a number'}</div>
    {/if}
  </div>
</div>

<style>
  .stats {
    display: flex;
    flex-direction: column;
    gap: 8px;
    height: 100%;
    padding: 8px 10px;
    box-sizing: border-box;
    min-height: 0;
  }
  .tiles {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(64px, 1fr));
    gap: 6px 10px;
  }
  .tile {
    display: flex;
    flex-direction: column;
    min-width: 0;
  }
  .label {
    font-size: var(--fs-xs);
    letter-spacing: 0.05em;
    text-transform: uppercase;
    color: var(--text-faint);
  }
  .value {
    font-size: var(--fs-lg);
    font-weight: 600;
    color: var(--text-strong);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .compact .value {
    font-size: var(--fs-md);
  }
  .value small {
    margin-left: 3px;
    font-size: var(--fs-xs);
    font-weight: 500;
    color: var(--text-faint);
  }
  .hist {
    display: flex;
    flex-direction: column;
    flex: 1;
    min-height: 40px;
  }
  svg {
    width: 100%;
    flex: 1;
    min-height: 24px;
    overflow: visible;
  }
  rect {
    fill: var(--series-reference);
    opacity: 0.85;
  }
  .axis {
    display: flex;
    justify-content: space-between;
    font-size: 10px;
    color: var(--text-faint);
  }
  .empty {
    display: grid;
    place-items: center;
    flex: 1;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
