<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readDialConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
  };

  let { signal, configRaw }: Props = $props();

  const config = $derived(readDialConfig(configRaw));

  const value = $derived.by(() => {
    const parsed = Number(signal.value);
    return Number.isFinite(parsed) ? parsed : config.min;
  });

  const normalized = $derived.by(() => {
    const range = Math.max(1e-9, config.max - config.min);
    return Math.max(0, Math.min(1, (value - config.min) / range));
  });

  function polar(cx: number, cy: number, radius: number, angleDeg: number) {
    const radians = ((angleDeg - 90) * Math.PI) / 180;
    return {
      x: cx + radius * Math.cos(radians),
      y: cy + radius * Math.sin(radians)
    };
  }

  function arcPath(startAngle: number, endAngle: number): string {
    const start = polar(50, 50, 34, endAngle);
    const end = polar(50, 50, 34, startAngle);
    const largeArcFlag = endAngle - startAngle <= 180 ? '0' : '1';
    return `M ${start.x.toFixed(2)} ${start.y.toFixed(2)} A 34 34 0 ${largeArcFlag} 0 ${end.x.toFixed(2)} ${end.y.toFixed(2)}`;
  }

  const sweepAngle = $derived(-135 + normalized * 270);
</script>

<div class="dial-root">
  <svg viewBox="0 0 100 74" preserveAspectRatio="xMidYMid meet" aria-label="dial widget">
    <path class="track" d={arcPath(-135, 135)} />
    <path class="value" d={arcPath(-135, sweepAngle)} />
    <line
      class="needle"
      x1="50"
      y1="50"
      x2={polar(50, 50, 28, sweepAngle).x.toFixed(2)}
      y2={polar(50, 50, 28, sweepAngle).y.toFixed(2)}
    />
    <circle cx="50" cy="50" r="1.8" class="hub" />
  </svg>

  <div class="dial-meta">
    <strong>{value.toFixed(2)}{config.unit ? ` ${config.unit}` : ''}</strong>
    <span>{config.min} - {config.max}</span>
  </div>
</div>

<style>
  .dial-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 0.18rem;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 2.2rem;
  }

  .track {
    fill: none;
    stroke: rgba(148, 163, 184, 0.28);
    stroke-width: 6;
    stroke-linecap: round;
  }

  .value {
    fill: none;
    stroke: rgba(239, 68, 68, 0.92);
    stroke-width: 6;
    stroke-linecap: round;
    filter: drop-shadow(0 0 2px rgba(239, 68, 68, 0.32));
  }

  .needle {
    stroke: #e2e8f0;
    stroke-width: 1.2;
    stroke-linecap: round;
  }

  .hub {
    fill: #e2e8f0;
  }

  .dial-meta {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    gap: 0.3rem;
  }

  .dial-meta strong {
    color: var(--text-strong);
    font-size: 0.78rem;
    font-family: var(--font-display);
  }

  .dial-meta span {
    color: var(--text-soft);
    font-size: 0.62rem;
    font-family: var(--font-mono);
  }
</style>
