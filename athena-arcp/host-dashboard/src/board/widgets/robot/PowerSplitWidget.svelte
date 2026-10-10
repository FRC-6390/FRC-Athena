<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { numericOf } from '../../../lib/board/widgets/robot-series';
  import { powerBreakdown } from '../../../lib/board/widgets/robot-power';
  import { formatNumber } from '../../../lib/format';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signals = $derived(ctx.bindings.currents ?? []);
  const showChannels = $derived(ctx.setting<boolean>('channels'));

  /** Channel label: the last two path segments, e.g. `turret/current`. */
  const labelOf = (path: string) => path.split('/').slice(-2).join('/');

  const breakdown = $derived.by(() => {
    void ctx.live.current;
    return powerBreakdown(
      signals.map((s) => ({ label: labelOf(s.path), amps: numericOf(ctx.latest(s)) ?? Number.NaN }))
    );
  });
  const hasData = $derived(signals.some((s) => numericOf(ctx.latest(s)) !== null));
  const PALETTE = ['var(--series-measured)', 'var(--series-reference)', 'var(--series-target)', 'var(--series-4)', 'var(--series-5)', 'var(--text-soft)'];
  const color = (i: number) => PALETTE[i % PALETTE.length];
</script>

<div class="power">
  <div class="head">
    <span class="total ui-mono">{hasData ? formatNumber(Math.round(breakdown.total * 10) / 10) : '--'}<small>A total</small></span>
    <span class="count ui-mono">{signals.length} channel{signals.length === 1 ? '' : 's'}</span>
  </div>

  <div class="stack" role="img" aria-label="current split">
    {#if hasData && breakdown.magnitude > 0}
      {#each breakdown.shares as share, i (share.label + i)}
        {#if share.fraction > 0}
          <i style="width:{share.fraction * 100}%; background:{color(i)}" title="{share.label} {formatNumber(share.amps)} A"></i>
        {/if}
      {/each}
    {:else}
      <i class="idle"></i>
    {/if}
  </div>

  {#if showChannels}
    <ul>
      {#each breakdown.shares as share, i (share.label + i)}
        <li>
          <span class="swatch" style="background:{color(i)}"></span>
          <span class="label" title={signals[i]?.path}>{share.label}</span>
          <span class="amps ui-mono">{Number.isFinite(share.amps) && hasData ? formatNumber(Math.round(share.amps * 10) / 10) : '--'}<small>A</small></span>
          <span class="pct ui-mono">{hasData ? `${Math.round(share.fraction * 100)}%` : '--'}</span>
        </li>
      {/each}
    </ul>
  {/if}
</div>

<style>
  .power {
    display: flex;
    flex-direction: column;
    gap: 10px;
    height: 100%;
    padding: 10px 12px;
    box-sizing: border-box;
    min-height: 0;
    overflow: hidden;
  }
  .head {
    display: flex;
    align-items: baseline;
    justify-content: space-between;
    gap: 8px;
  }
  .total {
    font-size: var(--fs-xl);
    font-weight: 600;
    color: var(--text-strong);
  }
  .total small {
    margin-left: 4px;
    font-size: var(--fs-xs);
    font-weight: 500;
    color: var(--text-faint);
  }
  .count {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .stack {
    display: flex;
    height: 14px;
    border-radius: 4px;
    background: var(--surface-3);
    overflow: hidden;
  }
  .stack i {
    display: block;
    height: 100%;
    border-right: 1px solid var(--surface);
  }
  .stack i:last-child {
    border-right: none;
  }
  .stack i.idle {
    width: 100%;
    background: var(--surface-3);
  }
  ul {
    list-style: none;
    margin: 0;
    padding: 0;
    overflow: auto;
    min-height: 0;
  }
  li {
    display: grid;
    grid-template-columns: 10px minmax(0, 1fr) auto 40px;
    align-items: center;
    gap: 8px;
    min-height: 24px;
    border-top: 1px solid var(--border);
    font-size: var(--fs-sm);
  }
  li:first-child {
    border-top: none;
  }
  .swatch {
    width: 10px;
    height: 10px;
    border-radius: 2px;
  }
  .label {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    color: var(--text-soft);
  }
  .amps {
    color: var(--text-strong);
    font-weight: 500;
    white-space: nowrap;
  }
  .amps small {
    margin-left: 2px;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .pct {
    text-align: right;
    color: var(--text-faint);
    font-size: var(--fs-xs);
  }
</style>
