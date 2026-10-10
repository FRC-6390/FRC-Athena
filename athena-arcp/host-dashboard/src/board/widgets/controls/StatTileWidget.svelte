<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { formatNumber } from '../../../lib/format';
  import { finiteSetting } from '../../../lib/board/widgets/controls-numeric';
  import { humanLabel, sparkPath, windowDelta } from '../../../lib/board/widgets/controls-values';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const windowSeconds = $derived(Math.max(1, finiteSetting(ctx.setting<number>('window')) ?? 30));
  const decimals = $derived(finiteSetting(ctx.setting<number>('decimals')) ?? -1);
  const showDelta = $derived(ctx.setting<boolean>('showDelta') !== false);
  const unit = $derived(ctx.setting<string>('unit') || signal?.unit || '');

  const value = $derived.by(() => {
    void ctx.live.current;
    const v = ctx.latest(signal);
    return typeof v === 'number' ? v : typeof v === 'bigint' ? Number(v) : null;
  });
  // The window ends at the newest sample so a stalled link freezes the trace instead of drifting.
  const newest = $derived.by(() => {
    void ctx.live.current;
    return signal ? (ctx.session.store.latestSample(signal.signalId)?.timestampMicros ?? 0) : 0;
  });
  const history = $derived.by(() => {
    void ctx.live.current;
    return signal ? ctx.samples(signal) : [];
  });
  const spark = $derived(sparkPath(history, newest, windowSeconds, 100, 24));
  const delta = $derived(windowDelta(history, newest, windowSeconds));
  const text = $derived(value === null ? '--' : decimals >= 0 ? value.toFixed(decimals) : formatNumber(value));
  const deltaText = $derived.by(() => {
    if (delta === null) return '--';
    const d = decimals >= 0 ? delta.toFixed(decimals) : formatNumber(delta);
    return `${delta > 0 ? '+' : ''}${d}`;
  });
  const fontSize = $derived(Math.max(18, Math.min(ctx.size.height * 0.42, ctx.size.width / Math.max(3, text.length * 0.62))));
</script>

<div class="tile">
  <span class="label">{signal ? humanLabel(signal.name) : 'Stat'}</span>
  <span class="number ui-mono" style="font-size:{fontSize}px">{text}{#if unit}<small>{unit}</small>{/if}</span>
  <div class="foot">
    {#if showDelta}
      <span class="delta ui-mono" title="Change over the last {windowSeconds} s">{deltaText} <span class="win">/ {windowSeconds} s</span></span>
    {/if}
  </div>
  {#if spark}
    <svg class="spark" viewBox="0 0 100 24" preserveAspectRatio="none" aria-hidden="true">
      <path d={spark} vector-effect="non-scaling-stroke" fill="none" />
    </svg>
  {/if}
</div>

<style>
  .tile {
    position: relative;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr) auto;
    height: 100%;
    padding: 8px 10px;
    min-width: 0;
    overflow: hidden;
  }
  .label {
    font-size: var(--fs-xs);
    font-weight: 600;
    letter-spacing: 0.04em;
    text-transform: uppercase;
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .number {
    align-self: center;
    font-weight: 600;
    line-height: 1.05;
    color: var(--text-strong);
    white-space: nowrap;
    overflow: hidden;
  }
  .number small {
    margin-left: 4px;
    font-size: 0.45em;
    font-weight: 500;
    color: var(--text-faint);
  }
  .foot {
    position: relative;
    z-index: 1;
    min-height: 16px;
    font-size: var(--fs-xs);
    color: var(--text);
  }
  .win {
    color: var(--text-faint);
  }
  .spark {
    position: absolute;
    left: 0;
    right: 0;
    bottom: 0;
    width: 100%;
    height: 40%;
    opacity: 0.9;
    pointer-events: none;
  }
  .spark path {
    stroke: var(--info);
    stroke-width: 1.5;
    stroke-linejoin: round;
  }
</style>
