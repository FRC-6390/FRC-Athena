<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import {
    compassArcPath,
    compassPoint,
    normalizeDegrees,
    shortestDelta,
    toDegrees,
    type AngleUnit
  } from '../../../lib/board/widgets/robot-geometry';
  import { numericOf } from '../../../lib/board/widgets/robot-series';
  import { formatNumber } from '../../../lib/format';

  let { ctx }: { ctx: WidgetContext } = $props();

  const angleSignal = $derived(ctx.bindings.angle?.[0]);
  const targetSignal = $derived(ctx.bindings.target?.[0]);
  const unit = $derived(ctx.setting<string>('unit') as AngleUnit);
  const minAngle = $derived(Number(ctx.setting<number>('minAngle')));
  const maxAngle = $derived(Number(ctx.setting<number>('maxAngle')));

  /** Angles in degrees, 0 at the top and clockwise positive (the same frame as the compass). */
  const angle = $derived.by(() => {
    void ctx.live.current;
    const v = numericOf(ctx.latest(angleSignal));
    return v === null ? null : toDegrees(v, unit);
  });
  const target = $derived.by(() => {
    void ctx.live.current;
    const v = numericOf(ctx.latest(targetSignal));
    return v === null ? null : toDegrees(v, unit);
  });

  // Range: the travel from min to max clockwise. A min above max wraps through 0 degrees.
  const travel = $derived(normalizeDegrees(maxAngle - minAngle));
  const inRange = $derived(angle === null || !Number.isFinite(minAngle) || !Number.isFinite(maxAngle) || travel === 0
    ? true
    : normalizeDegrees(angle - minAngle) <= travel + 1e-9);
  const offset = $derived(angle === null || target === null ? null : shortestDelta(angle, target));

  const size = $derived(Math.max(60, Math.min(ctx.size.width, ctx.size.height)));
  const c = $derived(size / 2);
  const R = $derived(Math.max(22, c - 12));
  const band = $derived(Math.max(5, R * 0.1));
  const fontPx = $derived(Math.max(9, Math.min(12, R * 0.16)));
  const needle = $derived(angle === null ? null : compassPoint(c, c, R - band - 4, normalizeDegrees(angle)));
  const needleBase = $derived(angle === null ? null : compassPoint(c, c, R * 0.12, normalizeDegrees(angle)));
  const targetPoint = $derived(target === null ? null : compassPoint(c, c, R - band / 2, normalizeDegrees(target)));
  const allowedPath = $derived(compassArcPath(c, c, R - band / 2, minAngle, minAngle + travel));
  const forbiddenPath = $derived(compassArcPath(c, c, R - band / 2, minAngle + travel, minAngle + 360));
  const minLabel = $derived(compassPoint(c, c, R - band - fontPx, minAngle));
  const maxLabel = $derived(compassPoint(c, c, R - band - fontPx, maxAngle));
  const tone = $derived(angle === null ? null : inRange ? 'ok' : 'fault');
</script>

<div class="dial" data-tone={tone}>
  <svg width={size} height={size} viewBox="0 0 {size} {size}" role="img" aria-label="angle dial">
    <circle cx={c} cy={c} r={R} class="face" />
    <circle cx={c} cy={c} r={R - band} class="inner" />
    <path d={forbiddenPath} class="forbidden" stroke-width={band} />
    <path d={allowedPath} class="allowed" stroke-width={band} />
    {#each [0, 90, 180, 270] as deg (deg)}
      {@const p1 = compassPoint(c, c, R - band, deg)}
      {@const p2 = compassPoint(c, c, R - band - 6, deg)}
      <line x1={p1.x} y1={p1.y} x2={p2.x} y2={p2.y} class="tick" />
    {/each}
    {#if minAngle !== maxAngle}
      <text x={minLabel.x} y={minLabel.y} class="limit" font-size={fontPx} text-anchor="middle" dominant-baseline="central">{formatNumber(minAngle)}°</text>
      <text x={maxLabel.x} y={maxLabel.y} class="limit" font-size={fontPx} text-anchor="middle" dominant-baseline="central">{formatNumber(maxAngle)}°</text>
    {/if}
    {#if targetPoint}
      <circle cx={targetPoint.x} cy={targetPoint.y} r={Math.max(3, band * 0.6)} class="target" />
    {/if}
    {#if needle && needleBase}
      <line x1={needleBase.x} y1={needleBase.y} x2={needle.x} y2={needle.y} class="needle" />
    {/if}
    <circle cx={c} cy={c} r={Math.max(3, R * 0.05)} class="hub" />
  </svg>
  <div class="readout">
    <span class="value ui-mono">{angle === null ? '--' : formatNumber(angle)}<small>°</small></span>
    {#if target !== null}
      <span class="sub ui-mono">→ {formatNumber(target)}°{offset !== null ? ` (${offset >= 0 ? '+' : ''}${formatNumber(offset)})` : ''}</span>
    {/if}
  </div>
</div>

<style>
  .dial {
    position: relative;
    width: 100%;
    height: 100%;
    display: grid;
    place-items: center;
    overflow: hidden;
  }
  svg {
    display: block;
  }
  .face {
    fill: var(--surface-2);
    stroke: var(--border-emphasis);
    stroke-width: 1;
  }
  .inner {
    fill: none;
    stroke: var(--border);
    stroke-width: 1;
  }
  .allowed {
    fill: none;
    stroke: var(--ok);
    opacity: 0.55;
  }
  .forbidden {
    fill: none;
    stroke: var(--fault);
    opacity: 0.45;
  }
  .tick {
    stroke: var(--text-faint);
    stroke-width: 1.5;
  }
  .limit {
    fill: var(--text-faint);
    font-family: var(--font-mono);
  }
  .target {
    fill: var(--series-target);
    stroke: var(--surface-2);
    stroke-width: 1.5;
  }
  .needle {
    stroke: var(--info);
    stroke-width: 3;
    stroke-linecap: round;
  }
  .hub {
    fill: var(--info);
  }
  .readout {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    display: flex;
    flex-direction: column;
    align-items: center;
    pointer-events: none;
  }
  .value {
    font-size: var(--fs-xl);
    font-weight: 600;
    color: var(--text-strong);
    text-shadow: 0 0 6px var(--surface-2);
  }
  .dial[data-tone='fault'] .value {
    color: var(--fault);
  }
  .value small {
    margin-left: 2px;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
  .sub {
    font-size: var(--fs-xs);
    color: var(--text-soft);
    text-shadow: 0 0 6px var(--surface-2);
  }
</style>
