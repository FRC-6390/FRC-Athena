<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { clamp, mapRange } from '../../../lib/board/widgets/robot-geometry';
  import { numericOf } from '../../../lib/board/widgets/robot-series';
  import { formatNumber } from '../../../lib/format';

  let { ctx }: { ctx: WidgetContext } = $props();

  const heightSignal = $derived(ctx.bindings.height?.[0]);
  const targetSignal = $derived(ctx.bindings.target?.[0]);
  const minH = $derived(Number(ctx.setting<number>('min')));
  const maxH = $derived(Number(ctx.setting<number>('max')));
  const unit = $derived(ctx.setting<string>('unit'));

  const height = $derived.by(() => {
    void ctx.live.current;
    return numericOf(ctx.latest(heightSignal));
  });
  const target = $derived.by(() => {
    void ctx.live.current;
    return numericOf(ctx.latest(targetSignal));
  });
  const outOfRange = $derived(height !== null && (height < minH || height > maxH));

  const W = $derived(ctx.size.width);
  const H = $derived(ctx.size.height);
  const shaftW = $derived(Math.max(16, Math.min(90, W * 0.3)));
  const shaftX = $derived(W * 0.5 - shaftW / 2);
  const top = 14;
  const bottom = $derived(H - 14);
  const travel = $derived(Math.max(1e-9, bottom - top));
  // Height to pixel y. Values outside the configured range are drawn at the limit they passed.
  const yOf = (value: number) => bottom - mapRange(clamp(value, minH, maxH), minH, maxH, 0, travel);
  const carriageH = $derived(Math.max(8, travel * 0.08));
  const carriageY = $derived(height === null ? bottom : yOf(height) - carriageH / 2);
  const targetY = $derived(target === null ? null : yOf(target));
  const ticks = $derived(Array.from({ length: 5 }, (_, i) => minH + ((maxH - minH) * i) / 4));
  const fontPx = $derived(Math.max(9, Math.min(11, shaftW * 0.5)));
  const railX = $derived(shaftX + shaftW * 0.15);
  const showLabels = $derived(W > 120);
</script>

<div class="elevator" data-tone={outOfRange ? 'fault' : null}>
  <svg width={W} height={H} viewBox="0 0 {W} {H}" role="img" aria-label="elevator">
    <rect x={shaftX} y={top} width={shaftW} height={travel} rx="3" class="shaft" />
    <rect x={railX} y={top} width={shaftW * 0.7} height={travel} class="rail" />
    {#each ticks as t, i (i)}
      {@const y = yOf(t)}
      <line x1={shaftX - 6} x2={shaftX} y1={y} y2={y} class="tick" />
      {#if showLabels}
        <text x={shaftX - 9} y={y} class="scale" font-size={fontPx} text-anchor="end" dominant-baseline="central">{formatNumber(t)}</text>
      {/if}
    {/each}
    {#if targetY !== null}
      <line x1={shaftX} x2={shaftX + shaftW} y1={targetY} y2={targetY} class="target" />
      <polygon points="{shaftX + shaftW + 2},{targetY} {shaftX + shaftW + 9},{targetY - 5} {shaftX + shaftW + 9},{targetY + 5}" class="target-marker" />
    {/if}
    <rect x={shaftX - 4} y={carriageY} width={shaftW + 8} height={carriageH} rx="3" class="carriage" data-tone={outOfRange ? 'fault' : 'info'} />
  </svg>
  <div class="readout">
    <span class="value ui-mono">{height === null ? '--' : formatNumber(height)}<small>{unit}</small></span>
    {#if target !== null}<span class="sub ui-mono">target {formatNumber(target)}{unit}</span>{/if}
  </div>
</div>

<style>
  .elevator {
    position: relative;
    width: 100%;
    height: 100%;
    overflow: hidden;
  }
  svg {
    display: block;
  }
  .shaft {
    fill: var(--surface-2);
    stroke: var(--border-emphasis);
    stroke-width: 1;
  }
  .rail {
    fill: var(--surface-3);
  }
  .tick {
    stroke: var(--text-faint);
  }
  .scale {
    fill: var(--text-faint);
    font-family: var(--font-mono);
  }
  .target {
    stroke: var(--series-target);
    stroke-width: 1.5;
    stroke-dasharray: 4 3;
  }
  .target-marker {
    fill: var(--series-target);
  }
  .carriage {
    fill: var(--info);
  }
  .carriage[data-tone='fault'] {
    fill: var(--fault);
  }
  .readout {
    position: absolute;
    top: 4px;
    right: 6px;
    display: flex;
    flex-direction: column;
    align-items: flex-end;
    pointer-events: none;
  }
  .value {
    font-size: var(--fs-lg);
    font-weight: 600;
    color: var(--text-strong);
  }
  .elevator[data-tone='fault'] .value {
    color: var(--fault);
  }
  .value small {
    margin-left: 3px;
    font-size: var(--fs-xs);
    font-weight: 500;
    color: var(--text-faint);
  }
  .sub {
    font-size: var(--fs-xs);
    color: var(--series-target);
  }
</style>
