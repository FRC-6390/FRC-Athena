<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { compassPoint, normalizeDegrees, shortestDelta, toDegrees, type AngleUnit } from '../../../lib/board/widgets/robot-geometry';
  import { numericOf } from '../../../lib/board/widgets/robot-series';
  import { formatNumber } from '../../../lib/format';

  let { ctx }: { ctx: WidgetContext } = $props();

  const headingSignal = $derived(ctx.bindings.heading?.[0]);
  const targetSignal = $derived(ctx.bindings.target?.[0]);
  const unit = $derived(ctx.setting<string>('unit') as AngleUnit);
  const offset = $derived(Number(ctx.setting<number>('offset')) || 0);

  /** Heading and target in degrees, compass convention (0 = north, clockwise positive), or null. */
  const heading = $derived.by(() => {
    void ctx.live.current;
    const v = numericOf(ctx.latest(headingSignal));
    return v === null ? null : normalizeDegrees(toDegrees(v, unit) + offset);
  });
  const target = $derived.by(() => {
    void ctx.live.current;
    const v = numericOf(ctx.latest(targetSignal));
    return v === null ? null : normalizeDegrees(toDegrees(v, unit) + offset);
  });
  const error = $derived(heading !== null && target !== null ? shortestDelta(heading, target) : null);

  const size = $derived(Math.max(40, Math.min(ctx.size.width, ctx.size.height)));
  const c = $derived(size / 2);
  const R = $derived(Math.max(14, c - 12));
  const fontPx = $derived(Math.max(9, Math.min(13, R * 0.18)));
  const ticks = Array.from({ length: 24 }, (_, i) => i * 15);
  const cardinals = [
    { label: 'N', deg: 0 },
    { label: 'E', deg: 90 },
    { label: 'S', deg: 180 },
    { label: 'W', deg: 270 }
  ];
  const ringPath = $derived(`M ${c} ${c - R} A ${R} ${R} 0 1 1 ${c} ${c + R} A ${R} ${R} 0 1 1 ${c} ${c - R}`);
  const targetPoint = $derived(target === null ? null : compassPoint(c, c, R - 8, target));
</script>

<div class="compass">
  <svg width={size} height={size} viewBox="0 0 {size} {size}" role="img" aria-label="compass heading">
    <!-- The dial turns so that north sits at the negative of the heading; the pointer at the top reads the heading. -->
    <g transform="rotate({heading === null ? 0 : -heading} {c} {c})">
      <path d={ringPath} class="ring" />
      {#each ticks as deg (deg)}
        {@const major = deg % 45 === 0}
        {@const p1 = compassPoint(c, c, R, deg)}
        {@const p2 = compassPoint(c, c, R - (major ? 9 : 5), deg)}
        <line x1={p1.x} y1={p1.y} x2={p2.x} y2={p2.y} class:major />
      {/each}
      {#each cardinals as card (card.label)}
        {@const p = compassPoint(c, c, R - 9 - fontPx * 0.6, card.deg)}
        <text
          x={p.x}
          y={p.y}
          class:north={card.label === 'N'}
          font-size={fontPx}
          text-anchor="middle"
          dominant-baseline="central"
          transform="rotate({heading === null ? 0 : heading} {p.x} {p.y})">{card.label}</text
        >
      {/each}
      {#if targetPoint}
        <circle cx={targetPoint.x} cy={targetPoint.y} r="4" class="target" />
      {/if}
    </g>
    <!-- Fixed pointer: the robot's heading. -->
    <polygon points="{c},{c - R + 2} {c - 5},{c - R + 14} {c + 5},{c - R + 14}" class="pointer" />
    <circle cx={c} cy={c} r={R * 0.42} class="hub" />
  </svg>
  <div class="readout">
    <span class="value ui-mono">{heading === null ? '--' : formatNumber(heading)}<small>°</small></span>
    {#if target !== null}
      <span class="sub ui-mono">target {formatNumber(target)}° {error !== null ? `· Δ ${error >= 0 ? '+' : ''}${formatNumber(error)}°` : ''}</span>
    {/if}
  </div>
</div>

<style>
  .compass {
    position: relative;
    display: grid;
    place-items: center;
    width: 100%;
    height: 100%;
    overflow: hidden;
  }
  svg {
    display: block;
  }
  .ring {
    fill: var(--surface-2);
    stroke: var(--border-emphasis);
    stroke-width: 1.5;
  }
  line {
    stroke: var(--text-faint);
    stroke-width: 1;
  }
  line.major {
    stroke: var(--text-soft);
  }
  text {
    fill: var(--text-soft);
    font-family: var(--font-body);
    font-weight: 600;
  }
  text.north {
    fill: var(--fault);
  }
  .target {
    fill: var(--series-target);
    stroke: var(--surface-2);
    stroke-width: 1.5;
  }
  .pointer {
    fill: var(--info);
  }
  .hub {
    fill: var(--surface-3);
    stroke: var(--border);
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
  .value small {
    margin-left: 2px;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
  .sub {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
</style>
