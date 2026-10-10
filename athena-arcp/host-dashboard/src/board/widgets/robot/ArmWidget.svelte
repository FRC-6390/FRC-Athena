<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { fitScale, normalizeDegrees, toDegrees, twoLinkJoints, type AngleUnit } from '../../../lib/board/widgets/robot-geometry';
  import { numericOf } from '../../../lib/board/widgets/robot-series';
  import { formatNumber } from '../../../lib/format';

  let { ctx }: { ctx: WidgetContext } = $props();

  const shoulderSignal = $derived(ctx.bindings.shoulder?.[0]);
  const elbowSignal = $derived(ctx.bindings.elbow?.[0]);
  const unit = $derived(ctx.setting<string>('unit') as AngleUnit);
  const upper = $derived(Math.max(0.01, Number(ctx.setting<number>('upper')) || 0.5));
  const lower = $derived(Math.max(0.01, Number(ctx.setting<number>('lower')) || 0.5));
  const elbowRelative = $derived(ctx.setting<boolean>('elbowRelative'));

  const shoulder = $derived.by(() => {
    void ctx.live.current;
    const v = numericOf(ctx.latest(shoulderSignal));
    return v === null ? null : toDegrees(v, unit);
  });
  const elbow = $derived.by(() => {
    void ctx.live.current;
    if (!elbowSignal) return null;
    const v = numericOf(ctx.latest(elbowSignal));
    return v === null ? null : toDegrees(v, unit);
  });

  const width = $derived(ctx.size.width);
  const height = $derived(ctx.size.height);
  const reach = $derived(upper + lower);
  const scale = $derived(fitScale(reach, width, height, 22));
  const origin = $derived({ x: width / 2, y: height / 2 + reach * scale * 0.25 });
  // Keep the pivot inside the box even when the arm is tall.
  const pivot = $derived({
    x: origin.x,
    y: Math.min(height - 12, Math.max(12, origin.y))
  });

  /** Joint positions in pixels. Without an elbow binding only the upper link is drawn. */
  const joints = $derived.by(() => {
    if (shoulder === null) return null;
    const relative = elbow === null ? 0 : elbowRelative ? elbow : elbow - shoulder;
    return twoLinkJoints(pivot, shoulder, relative, upper, elbow === null ? 0 : lower, scale);
  });
  const reachRadius = $derived(reach * scale);
  const fontPx = $derived(Math.max(9, Math.min(12, Math.min(width, height) * 0.06)));
  const shoulderLabel = $derived(shoulder === null ? '--' : `${formatNumber(normalizeDegrees(shoulder))}°`);
  const elbowLabel = $derived(elbow === null ? null : `${formatNumber(elbow)}°`);
</script>

<div class="arm">
  <svg width={width} height={height} viewBox="0 0 {width} {height}" role="img" aria-label="arm">
    <circle cx={pivot.x} cy={pivot.y} r={reachRadius} class="reach" />
    <line x1={0} x2={width} y1={pivot.y + 0.5} y2={pivot.y + 0.5} class="ground" />
    {#if joints}
      <line x1={pivot.x} y1={pivot.y} x2={joints.elbow.x} y2={joints.elbow.y} class="upper" />
      {#if elbow !== null}
        <line x1={joints.elbow.x} y1={joints.elbow.y} x2={joints.tip.x} y2={joints.tip.y} class="lower" />
      {/if}
      <circle cx={pivot.x} cy={pivot.y} r="5" class="base" />
      {#if elbow !== null}<circle cx={joints.elbow.x} cy={joints.elbow.y} r="4" class="joint" />{/if}
      <circle cx={joints.tip.x} cy={joints.tip.y} r="4" class="tip" />
      <text x={pivot.x + 12} y={pivot.y - 10} font-size={fontPx} class="label">{shoulderLabel}</text>
      {#if elbowLabel}
        <text x={joints.elbow.x + 8} y={joints.elbow.y - 8} font-size={fontPx} class="label">{elbowLabel}</text>
      {/if}
    {:else}
      <text x={width / 2} y={height / 2} font-size={fontPx + 2} text-anchor="middle" class="empty">bind a shoulder angle</text>
    {/if}
  </svg>
</div>

<style>
  .arm {
    width: 100%;
    height: 100%;
    overflow: hidden;
  }
  svg {
    display: block;
  }
  .reach {
    fill: none;
    stroke: var(--border);
    stroke-dasharray: 3 4;
  }
  .ground {
    stroke: var(--border-emphasis);
    stroke-width: 1;
  }
  .upper,
  .lower {
    stroke-width: 6;
    stroke-linecap: round;
  }
  .upper {
    stroke: var(--series-measured);
  }
  .lower {
    stroke: var(--series-reference);
  }
  .base {
    fill: var(--surface-3);
    stroke: var(--text-soft);
    stroke-width: 1.5;
  }
  .joint {
    fill: var(--surface-3);
    stroke: var(--text-soft);
    stroke-width: 1.5;
  }
  .tip {
    fill: var(--info);
  }
  .label {
    fill: var(--text-soft);
    font-family: var(--font-mono);
  }
  .empty {
    fill: var(--text-faint);
  }
</style>
