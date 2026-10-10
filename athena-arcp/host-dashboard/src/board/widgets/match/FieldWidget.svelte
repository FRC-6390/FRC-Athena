<script lang="ts">
  import fieldUrl from '../../../assets/fields/FieldImage2026.svg?url';
  import type { WidgetContext } from '../../../lib/board/types';
  import { parseAlliance, type Alliance } from '../../../lib/board/widgets/match-alliance';
  import {
    FIELD_CENTRE,
    FIELD_VIEW,
    ROBOT_SIZE,
    decodePose2d,
    formatPose,
    headingDegrees,
    isRotated,
    screenPoint,
    trailPoses,
    trailPointsAttr,
    type Perspective
  } from '../../../lib/board/widgets/match-field';

  let { ctx }: { ctx: WidgetContext } = $props();

  const poseSignals = $derived(ctx.bindings.pose ?? []);
  const allianceSignal = $derived(ctx.bindings.alliance?.[0]);
  const trailSeconds = $derived(ctx.setting<number>('trail'));
  const perspective = $derived(ctx.setting<Perspective>('perspective'));
  const trueMeans = $derived(ctx.setting<Alliance>('trueIs'));
  const showImage = $derived(ctx.setting<boolean>('showImage'));

  const alliance = $derived.by(() => {
    void ctx.live.current;
    return allianceSignal ? parseAlliance(ctx.latest(allianceSignal), trueMeans) : null;
  });
  const rotated = $derived(isRotated(perspective, alliance));

  /** Robot first (slot order), then extra poses such as targets or vision estimates. */
  const scene = $derived.by(() => {
    void ctx.live.current;
    return poseSignals.map((signal, i) => {
      const samples = ctx.samples(signal);
      const latest = decodePose2d(ctx.latest(signal));
      return {
        index: i,
        signalId: signal.signalId,
        latest,
        trail: i === 0 && trailSeconds > 0 ? trailPoses(samples, trailSeconds) : [],
        name: signal.name
      };
    });
  });
  const robot = $derived(scene[0]);
  const extras = $derived(scene.slice(1).filter((s) => s.latest !== null));
  const hasPose = $derived(robot?.latest !== null && robot?.latest !== undefined);

  const bumper = $derived<'red' | 'blue' | 'unknown'>(alliance ?? 'unknown');
  const cx = FIELD_CENTRE.x;
  const cy = FIELD_CENTRE.y;
  const EXTRA_CLASSES = ['target', 'vision', 'other'];
</script>

<div class="field" class:empty={!hasPose}>
  <svg viewBox="{FIELD_VIEW.x} {FIELD_VIEW.y} {FIELD_VIEW.width} {FIELD_VIEW.height}" role="img" aria-label="Field view">
    <g transform={rotated ? `rotate(180 ${cx} ${cy})` : undefined}>
      {#if showImage}
        <image href={fieldUrl} x={FIELD_VIEW.x} y={FIELD_VIEW.y} width={FIELD_VIEW.width} height={FIELD_VIEW.height} preserveAspectRatio="none" />
      {:else}
        <rect x={FIELD_VIEW.x} y={FIELD_VIEW.y} width={FIELD_VIEW.width} height={FIELD_VIEW.height} class="plain" />
      {/if}

      {#if robot && robot.trail.length > 1}
        <polyline points={trailPointsAttr(robot.trail)} class="trail" />
      {/if}

      {#each extras as extra, i (extra.signalId)}
        {@const s = screenPoint(extra.latest!)}
        <g transform="translate({s.x} {s.y}) rotate({headingDegrees(extra.latest!.theta)})" class="extra {EXTRA_CLASSES[i % EXTRA_CLASSES.length]}">
          <circle r="0.22" />
          <line x1="0" y1="0" x2="0.4" y2="0" />
        </g>
      {/each}

      {#if robot?.latest}
        {@const s = screenPoint(robot.latest)}
        <g transform="translate({s.x} {s.y}) rotate({headingDegrees(robot.latest.theta)})">
          <rect x={-ROBOT_SIZE / 2} y={-ROBOT_SIZE / 2} width={ROBOT_SIZE} height={ROBOT_SIZE} rx="0.08" class="bumper {bumper}" />
          <rect x={-ROBOT_SIZE / 2 + 0.12} y={-ROBOT_SIZE / 2 + 0.12} width={ROBOT_SIZE - 0.24} height={ROBOT_SIZE - 0.24} rx="0.04" class="body" />
          <path d="M {ROBOT_SIZE / 2 - 0.05} 0 L {ROBOT_SIZE / 2 - 0.3} -0.22 L {ROBOT_SIZE / 2 - 0.3} 0.22 Z" class="heading" />
        </g>
      {/if}
    </g>
  </svg>

  {#if !hasPose}
    <div class="notice">No pose yet</div>
  {/if}

  <div class="readout ui-mono">
    {#if robot?.name}<span class="name">{robot.name}</span>{/if}
    <span>{formatPose(robot?.latest ?? null)}</span>
    {#if alliance}<span class="side" data-side={alliance}>{alliance === 'red' ? 'Red' : 'Blue'} side up</span>{/if}
  </div>
</div>

<style>
  .field {
    position: relative;
    width: 100%;
    height: 100%;
    display: grid;
    place-items: center;
    background: var(--bg-1);
    container-type: size;
  }
  svg {
    width: 100%;
    height: 100%;
    display: block;
  }
  .plain {
    fill: var(--surface-2);
  }
  .trail {
    fill: none;
    stroke: var(--ok);
    stroke-width: 0.06;
    stroke-linejoin: round;
    stroke-linecap: round;
    opacity: 0.85;
  }
  .bumper {
    stroke-width: 0.1;
  }
  .bumper.red {
    fill: var(--brand);
    stroke: var(--brand);
  }
  .bumper.blue {
    fill: var(--info);
    stroke: var(--info);
  }
  .bumper.unknown {
    fill: var(--text-faint);
    stroke: var(--text-faint);
  }
  .body {
    fill: var(--surface-3);
  }
  .heading {
    fill: var(--text-strong);
  }
  .extra circle {
    fill: none;
    stroke-width: 0.06;
  }
  .extra line {
    stroke-width: 0.06;
  }
  .extra.target {
    color: var(--series-target);
  }
  .extra.vision {
    color: var(--series-5);
  }
  .extra.other {
    color: var(--series-4);
  }
  .extra circle,
  .extra line {
    stroke: currentColor;
  }
  .notice {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    font-size: min(9cqh, 9cqw);
    color: var(--text-soft);
    pointer-events: none;
  }
  .readout {
    position: absolute;
    left: 8px;
    bottom: 6px;
    display: flex;
    flex-wrap: wrap;
    gap: 4px 10px;
    align-items: baseline;
    max-width: calc(100% - 16px);
    font-size: var(--fs-xs);
    color: var(--text-strong);
    text-shadow: 0 0 4px var(--bg-0);
    pointer-events: none;
  }
  .name {
    color: var(--text-soft);
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }
  .side {
    color: var(--text-soft);
  }
  .side[data-side='red'] {
    color: var(--brand);
  }
  .side[data-side='blue'] {
    color: var(--info);
  }
</style>
