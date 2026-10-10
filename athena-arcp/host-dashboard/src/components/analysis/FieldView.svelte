<script lang="ts">
  import fieldUrl from '../../assets/fields/FieldImage2026.svg?url';
  import type { ReplayModel } from '../../lib/analysis/replay';
  import { lastIndexAtOrBefore } from '../../lib/analysis/replay';

  type Props = {
    model: ReplayModel;
    signalId: number;
    cursor: number;
    /** Trail length in samples before the cursor. */
    trail?: number;
  };

  let { model, signalId, cursor, trail = 200 }: Props = $props();

  // Field image extent in metres (FieldImage2026.svg viewBox). Field y is flipped in the image.
  const VIEW = { x: -0.5, y: -0.5, width: 17.541, height: 9.0692 };
  const FIELD_HEIGHT = 8.0692;
  const ROBOT = 0.8;

  const pose = $derived.by(() => {
    const track = model.poseTrack(signalId);
    if (!track || track.pose.length === 0) return null;
    const i = lastIndexAtOrBefore(track.t, cursor);
    if (i < 0) return null;
    const from = Math.max(0, i - trail);
    const path = track.pose.slice(from, i + 1).map((p) => `${p.x},${FIELD_HEIGHT - p.y}`).join(' ');
    const current = track.pose[i];
    return { current, path, count: i - from + 1 };
  });

  const deg = (rad: number) => (-rad * 180) / Math.PI;
</script>

<figure class="field">
  <svg viewBox="{VIEW.x} {VIEW.y} {VIEW.width} {VIEW.height}" role="img" aria-label="robot pose on the field">
    <image href={fieldUrl} x={VIEW.x} y={VIEW.y} width={VIEW.width} height={VIEW.height} preserveAspectRatio="none" />
    {#if pose}
      {#if pose.count > 1}
        <polyline points={pose.path} class="trail" />
      {/if}
      <g transform="translate({pose.current.x} {FIELD_HEIGHT - pose.current.y}) rotate({deg(pose.current.theta)})">
        <rect x={-ROBOT / 2} y={-ROBOT / 2} width={ROBOT} height={ROBOT} class="robot" />
        <line x1="0" y1="0" x2={ROBOT / 2} y2="0" class="heading" />
      </g>
    {/if}
  </svg>
  {#if pose}
    <figcaption>
      x {pose.current.x.toFixed(2)} m, y {pose.current.y.toFixed(2)} m, heading {((pose.current.theta * 180) / Math.PI).toFixed(0)} deg
    </figcaption>
  {:else}
    <figcaption>No pose sample at this time.</figcaption>
  {/if}
</figure>

<style>
  .field {
    margin: 0;
    display: grid;
    gap: 6px;
  }
  svg {
    width: 100%;
    height: auto;
    border-radius: var(--radius-lg);
    border: 1px solid var(--border);
    background: #0b0d12;
  }
  .trail {
    fill: none;
    stroke: rgba(134, 239, 172, 0.8);
    stroke-width: 0.04;
  }
  .robot {
    fill: rgba(56, 189, 248, 0.35);
    stroke: var(--info);
    stroke-width: 0.05;
  }
  .heading {
    stroke: #f8fafc;
    stroke-width: 0.06;
  }
  figcaption {
    font-size: 12px;
    color: var(--text-soft);
    font-family: var(--font-mono);
  }
</style>
