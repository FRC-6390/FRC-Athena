<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseNumericArray, readSwerveModuleConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
  };

  let { signal, signals, signalById, configRaw }: Props = $props();

  const config = $derived(readSwerveModuleConfig(configRaw, signal, signals));
  const packed = $derived(parseNumericArray(signal.value));

  function numericSignalValue(signalId: number | null): number | null {
    if (signalId === null) return null;
    const row = signalById.get(signalId);
    if (!row) return null;
    const parsed = Number(row.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  function isAnglePath(path: string): boolean {
    const token = path.toLowerCase();
    return (
      token.includes('angle') || token.includes('heading') || token.includes('yaw') || token.includes('theta')
    );
  }

  function isSpeedPath(path: string): boolean {
    const token = path.toLowerCase();
    return token.includes('speed') || token.includes('velocity') || token.includes('vel');
  }

  const angleDeg = $derived.by(() => {
    const explicit = numericSignalValue(config.angleSignalId);
    if (explicit !== null) return explicit;
    if ((signal.signal_type === 'f64' || signal.signal_type === 'i64') && isAnglePath(signal.path)) {
      const parsed = Number(signal.value);
      return Number.isFinite(parsed) ? parsed : 0;
    }
    return packed[0] ?? 0;
  });

  const speedMps = $derived.by(() => {
    const explicit = numericSignalValue(config.speedSignalId);
    if (explicit !== null) return explicit;
    if ((signal.signal_type === 'f64' || signal.signal_type === 'i64') && isSpeedPath(signal.path)) {
      const parsed = Number(signal.value);
      return Number.isFinite(parsed) ? parsed : 0;
    }
    return packed[1] ?? 0;
  });

  const speedRatio = $derived.by(() => {
    const max = Math.max(0.1, config.maxSpeed);
    return Math.max(-1, Math.min(1, speedMps / max));
  });

  const arrowLength = $derived(0.13 + Math.abs(speedRatio) * 0.2);
  const speedClass = $derived.by(() => {
    if (Math.abs(speedRatio) < 0.05) return 'idle';
    return speedRatio >= 0 ? 'forward' : 'reverse';
  });
</script>

<div class="module-root">
  <svg viewBox="-1 -1 2 2" preserveAspectRatio="xMidYMid meet" aria-label="swerve module">
    <circle class="module-ring" cx="0" cy="0" r="0.68" />
    <line class="axis" x1="-0.68" y1="0" x2="0.68" y2="0" />
    <line class="axis" x1="0" y1="-0.68" x2="0" y2="0.68" />
    <g transform={`rotate(${angleDeg.toFixed(2)})`}>
      <line
        class={`module-arrow ${speedClass}`}
        x1="0"
        y1="0"
        x2="0"
        y2={(-arrowLength).toFixed(4)}
      />
      <line
        class={`module-arrow-head ${speedClass}`}
        x1="0"
        y1={(-arrowLength).toFixed(4)}
        x2="-0.075"
        y2={(-arrowLength + 0.09).toFixed(4)}
      />
      <line
        class={`module-arrow-head ${speedClass}`}
        x1="0"
        y1={(-arrowLength).toFixed(4)}
        x2="0.075"
        y2={(-arrowLength + 0.09).toFixed(4)}
      />
    </g>
  </svg>

  <div class="meta">
    <span class="label">{config.label}</span>
    <span>{angleDeg.toFixed(1)} deg</span>
    <span>{speedMps.toFixed(2)} m/s</span>
  </div>
</div>

<style>
  .module-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 0.2rem;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 3.8rem;
    border-radius: 7px;
    border: 1px solid var(--border-subtle);
    background: #131a2a;
  }

  .module-ring {
    fill: rgba(15, 23, 42, 0.65);
    stroke: rgba(148, 163, 184, 0.5);
    stroke-width: 0.055;
  }

  .axis {
    stroke: rgba(148, 163, 184, 0.22);
    stroke-width: 0.035;
  }

  .module-arrow {
    stroke-width: 0.078;
    stroke-linecap: round;
  }

  .module-arrow-head {
    stroke-width: 0.068;
    stroke-linecap: round;
  }

  .module-arrow.idle,
  .module-arrow-head.idle {
    stroke: #94a3b8;
    fill: #94a3b8;
  }

  .module-arrow.forward,
  .module-arrow-head.forward {
    stroke: #38bdf8;
    fill: #38bdf8;
  }

  .module-arrow.reverse,
  .module-arrow-head.reverse {
    stroke: #f97316;
    fill: #f97316;
  }

  .meta {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 0.2rem;
    color: var(--text-soft);
    font-size: 0.62rem;
    font-family: var(--font-mono);
  }

  .label {
    color: var(--text-strong);
    font-family: var(--font-display);
    font-size: 0.64rem;
  }
</style>
