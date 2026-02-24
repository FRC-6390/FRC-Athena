<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseMech2dScene, readMech2dConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
  };

  let { signal, configRaw }: Props = $props();

  const config = $derived(readMech2dConfig(configRaw));
  const scene = $derived(parseMech2dScene(signal.value));

  function toSvgY(y: number): number {
    return scene.viewportHeight - y;
  }

  const gridLines = $derived.by(() => {
    if (!config.showGrid) return [];
    const cols = 8;
    const rows = 8;
    const lines: Array<{ x1: number; y1: number; x2: number; y2: number }> = [];
    for (let col = 1; col < cols; col++) {
      const x = (col / cols) * scene.viewportWidth;
      lines.push({ x1: x, y1: 0, x2: x, y2: scene.viewportHeight });
    }
    for (let row = 1; row < rows; row++) {
      const y = (row / rows) * scene.viewportHeight;
      lines.push({ x1: 0, y1: y, x2: scene.viewportWidth, y2: y });
    }
    return lines;
  });
</script>

<div class="mech-root">
  <svg
    viewBox={`0 0 ${scene.viewportWidth} ${scene.viewportHeight}`}
    preserveAspectRatio="none"
    aria-label="mechanism2d viewer"
  >
    <rect class="mech-bg" x="0" y="0" width={scene.viewportWidth} height={scene.viewportHeight} />

    {#if config.showGrid}
      {#each gridLines as line, idx (`g-${idx}`)}
        <line
          class="grid-line"
          x1={line.x1}
          y1={toSvgY(line.y1)}
          x2={line.x2}
          y2={toSvgY(line.y2)}
        />
      {/each}
    {/if}

    {#each scene.segments as segment, idx (`s-${idx}`)}
      <line
        x1={segment.x1}
        y1={toSvgY(segment.y1)}
        x2={segment.x2}
        y2={toSvgY(segment.y2)}
        stroke={segment.color}
        stroke-width={segment.width || config.lineWidth}
        stroke-linecap="round"
      />
    {/each}

    {#each scene.joints as joint, idx (`j-${idx}`)}
      <circle
        cx={joint.x}
        cy={toSvgY(joint.y)}
        r={joint.r}
        fill={joint.color}
        stroke="rgba(15,23,42,0.7)"
        stroke-width={joint.r * 0.35}
      />
    {/each}
  </svg>

  {#if scene.segments.length === 0 && scene.joints.length === 0}
    <p class="hint">No mechanism segments published yet.</p>
  {/if}
</div>

<style>
  .mech-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 0.18rem;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 4.2rem;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: #0b1020;
  }

  .mech-bg {
    fill: #0f172a;
  }

  .grid-line {
    stroke: rgba(148, 163, 184, 0.18);
    stroke-width: 0.01;
  }

  .hint {
    margin: 0;
    color: var(--text-soft);
    font-size: 0.62rem;
  }
</style>
