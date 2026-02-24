<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';

  type Props = {
    signal: SignalRow;
  };

  let { signal }: Props = $props();

  const heading = $derived.by(() => {
    const parsed = Number(signal.value);
    if (!Number.isFinite(parsed)) return 0;
    return ((parsed % 360) + 360) % 360;
  });

  const majorTicks = $derived.by(() => {
    const ticks: Array<{ angle: number; inner: number; outer: number }> = [];
    for (let angle = 0; angle < 360; angle += 30) {
      ticks.push({ angle, inner: 12, outer: 7.2 });
    }
    return ticks;
  });

  const minorTicks = $derived.by(() => {
    const ticks: Array<{ angle: number; inner: number; outer: number }> = [];
    for (let angle = 0; angle < 360; angle += 10) {
      if (angle % 30 === 0) continue;
      ticks.push({ angle, inner: 10.7, outer: 8.1 });
    }
    return ticks;
  });
</script>

<div class="compass-root">
  <svg viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet" aria-label="360 compass">
    <circle class="ring" cx="50" cy="50" r="42" />

    {#each minorTicks as tick, idx (`mi-${idx}`)}
      <line
        class="tick-minor"
        x1="50"
        y1={tick.inner}
        x2="50"
        y2={tick.outer}
        transform={`rotate(${tick.angle} 50 50)`}
      />
    {/each}

    {#each majorTicks as tick, idx (`ma-${idx}`)}
      <line
        class="tick-major"
        x1="50"
        y1={tick.inner}
        x2="50"
        y2={tick.outer}
        transform={`rotate(${tick.angle} 50 50)`}
      />
    {/each}

    <text class="cardinal n" x="50" y="12.5">N</text>
    <text class="cardinal e" x="87.5" y="53.5">E</text>
    <text class="cardinal s" x="50" y="92.5">S</text>
    <text class="cardinal w" x="12.5" y="53.5">W</text>

    <g transform={`rotate(${heading.toFixed(2)} 50 50)`}>
      <line class="needle" x1="50" y1="50" x2="50" y2="15.5" />
      <polygon class="needle-head" points="50,8.2 46.9,16.6 53.1,16.6" />
      <line class="tail" x1="50" y1="50" x2="50" y2="69.5" />
    </g>

    <circle class="hub" cx="50" cy="50" r="2.2" />
  </svg>

  <div class="meta">
    <strong>{heading.toFixed(1)} deg</strong>
  </div>
</div>

<style>
  .compass-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 0.18rem;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 2.4rem;
    border-radius: 7px;
    border: 1px solid var(--border-subtle);
    background: #101827;
  }

  .ring {
    fill: rgba(15, 23, 42, 0.68);
    stroke: rgba(148, 163, 184, 0.52);
    stroke-width: 1.4;
  }

  .tick-minor {
    stroke: rgba(148, 163, 184, 0.28);
    stroke-width: 0.8;
    stroke-linecap: round;
  }

  .tick-major {
    stroke: rgba(203, 213, 225, 0.64);
    stroke-width: 1.25;
    stroke-linecap: round;
  }

  .cardinal {
    fill: rgba(226, 232, 240, 0.92);
    font-family: var(--font-display);
    font-size: 6.8px;
    font-weight: 700;
    text-anchor: middle;
    dominant-baseline: middle;
  }

  .cardinal.n {
    fill: rgba(248, 113, 113, 0.96);
  }

  .needle {
    stroke: rgba(248, 113, 113, 0.96);
    stroke-width: 1.8;
    stroke-linecap: round;
  }

  .needle-head {
    fill: rgba(248, 113, 113, 0.96);
  }

  .tail {
    stroke: rgba(148, 163, 184, 0.85);
    stroke-width: 1.2;
    stroke-linecap: round;
  }

  .hub {
    fill: #e2e8f0;
  }

  .meta {
    display: flex;
    justify-content: center;
    align-items: baseline;
  }

  .meta strong {
    color: var(--text-strong);
    font-size: 0.78rem;
    font-family: var(--font-mono);
    letter-spacing: 0.01em;
  }
</style>
