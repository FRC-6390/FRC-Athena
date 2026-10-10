<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { formatNumber } from '../../../lib/format';
  import { finiteSetting } from '../../../lib/board/widgets/controls-numeric';
  import { alertTone, fraction } from '../../../lib/board/widgets/controls-values';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const targetSignal = $derived(ctx.bindings.target?.[0]);
  const num = (v: unknown) => (typeof v === 'number' ? v : typeof v === 'bigint' ? Number(v) : null);
  const value = $derived.by(() => {
    void ctx.live.current;
    return num(ctx.latest(signal));
  });
  const target = $derived.by(() => {
    void ctx.live.current;
    return num(ctx.latest(targetSignal));
  });
  const min = $derived(finiteSetting(ctx.setting<number>('min')) ?? 0);
  const max = $derived(finiteSetting(ctx.setting<number>('max')) ?? 100);
  const vertical = $derived(ctx.setting<string>('orientation') === 'vertical');
  const alertWhen = $derived(ctx.setting<string>('alertWhen') === 'low' ? 'low' : 'high');
  const warn = $derived(ctx.setting<number>('warn'));
  const fault = $derived(ctx.setting<number>('fault'));
  const tone = $derived(alertTone(value, { warn, fault, alertWhen }));
  const fill = $derived(fraction(value, min, max));
  const marker = $derived(target === null ? null : fraction(target, min, max));
  const unit = $derived(signal?.unit ?? '');
</script>

<div class="bar" class:vertical data-tone={tone ?? 'ok'}>
  <div class="head">
    <span class="label">{signal?.name ?? 'Bar'}</span>
    <span class="value ui-mono">{value === null ? '--' : formatNumber(value)}{#if unit}<small>{unit}</small>{/if}</span>
  </div>
  <div class="track-wrap">
    <div class="track">
      <div class="fill" style={vertical ? `height:${fill * 100}%` : `width:${fill * 100}%`}></div>
      {#if marker !== null}
        <div
          class="marker"
          title={`Target ${formatNumber(target ?? 0)}`}
          style={vertical ? `bottom:${marker * 100}%` : `left:${marker * 100}%`}
        ></div>
      {/if}
    </div>
    <div class="scale ui-mono">
      <span>{formatNumber(min)}</span>
      {#if marker !== null && target !== null}<span class="target">target {formatNumber(target)}</span>{/if}
      <span>{formatNumber(max)}</span>
    </div>
  </div>
</div>

<style>
  .bar {
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    gap: 6px;
    height: 100%;
    padding: 8px 10px;
    min-width: 0;
  }
  .head {
    display: flex;
    align-items: baseline;
    justify-content: space-between;
    gap: 8px;
    min-width: 0;
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
  .value {
    font-size: var(--fs-lg);
    font-weight: 600;
    color: var(--text-strong);
    white-space: nowrap;
  }
  .bar[data-tone='warn'] .value {
    color: var(--warn);
  }
  .bar[data-tone='fault'] .value {
    color: var(--fault);
  }
  .value small {
    margin-left: 3px;
    font-size: 0.6em;
    font-weight: 500;
    color: var(--text-faint);
  }
  .track-wrap {
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 4px;
    min-height: 0;
  }
  .track {
    position: relative;
    min-height: 12px;
    border-radius: var(--radius-sm);
    background: var(--surface-3);
    border: 1px solid var(--border);
    overflow: visible;
  }
  .fill {
    position: absolute;
    left: 0;
    top: 0;
    bottom: 0;
    width: 0;
    border-radius: var(--radius-sm);
    background: var(--info);
    transition: width 150ms;
  }
  .bar[data-tone='ok'] .fill {
    background: var(--ok);
  }
  .bar[data-tone='warn'] .fill {
    background: var(--warn);
  }
  .bar[data-tone='fault'] .fill {
    background: var(--fault);
  }
  .marker {
    position: absolute;
    background: var(--series-target);
    border-radius: 1px;
    pointer-events: none;
  }
  .bar:not(.vertical) .track {
    height: 100%;
    min-height: 16px;
  }
  .bar:not(.vertical) .marker {
    top: -4px;
    bottom: -4px;
    width: 2px;
    margin-left: -1px;
  }
  .scale {
    display: flex;
    justify-content: space-between;
    gap: 6px;
    font-size: 10px;
    color: var(--text-faint);
    min-width: 0;
  }
  .target {
    color: var(--series-target);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  /* Vertical: the track fills the column, the scale sits beside it. */
  .bar.vertical {
    grid-template-rows: auto minmax(0, 1fr);
  }
  .bar.vertical .track-wrap {
    grid-template-rows: minmax(0, 1fr);
    grid-template-columns: minmax(0, 1fr) auto;
  }
  .bar.vertical .track {
    width: 22px;
    justify-self: center;
    height: 100%;
    min-height: 0;
  }
  .bar.vertical .fill {
    top: auto;
    width: 100%;
    height: 0;
  }
  .bar.vertical .marker {
    left: -4px;
    right: -4px;
    height: 2px;
    margin-bottom: -1px;
  }
  .bar.vertical .scale {
    flex-direction: column-reverse;
    justify-content: space-between;
    align-items: flex-start;
  }
</style>
