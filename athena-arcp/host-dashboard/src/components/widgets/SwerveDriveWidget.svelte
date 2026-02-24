<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseNumericArray, readSwerveDriveConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
  };

  type ModuleSample = {
    key: string;
    label: string;
    angleDeg: number;
    speedMps: number;
    ratio: number;
  };

  let { signal, signals, signalById, configRaw }: Props = $props();

  const config = $derived(readSwerveDriveConfig(configRaw, signal, signals));
  const packed = $derived(parseNumericArray(signal.value));

  function numericSignalValue(signalId: number | null): number | null {
    if (signalId === null) return null;
    const row = signalById.get(signalId);
    if (!row) return null;
    const parsed = Number(row.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  const modules = $derived.by(() => {
    const maxSpeed = Math.max(0.1, config.maxSpeed);
    const out: ModuleSample[] = [];
    for (let idx = 0; idx < config.modules.length; idx++) {
      const binding = config.modules[idx];
      const packedAngle = packed[idx * 2] ?? 0;
      const packedSpeed = packed[idx * 2 + 1] ?? 0;
      const angleDeg = numericSignalValue(binding.angleSignalId) ?? packedAngle;
      const speedMps = numericSignalValue(binding.speedSignalId) ?? packedSpeed;
      out.push({
        key: binding.key,
        label: binding.label || binding.key.toUpperCase(),
        angleDeg,
        speedMps,
        ratio: Math.max(-1, Math.min(1, speedMps / maxSpeed))
      });
    }
    return out;
  });

  const headingDeg = $derived.by(() => {
    const explicit = numericSignalValue(config.headingSignalId);
    if (explicit !== null) return explicit;
    const headingFromPacked = packed.length >= 9 ? packed[8] : null;
    return headingFromPacked ?? 0;
  });

  function arrowLength(ratio: number): number {
    return 0.12 + Math.abs(ratio) * 0.2;
  }

  function speedClass(ratio: number): string {
    if (Math.abs(ratio) < 0.05) return 'idle';
    return ratio >= 0 ? 'forward' : 'reverse';
  }
</script>

<div class="swerve-root">
  <div class="heading">
    <span>Heading</span>
    <strong>{headingDeg.toFixed(1)} deg</strong>
  </div>

  <div class="module-grid">
    {#each modules as module (module.key)}
      {@const len = arrowLength(module.ratio)}
      {@const klass = speedClass(module.ratio)}
      <div class="module-card">
        <div class="module-title">{module.label}</div>
        <svg viewBox="-1 -1 2 2" preserveAspectRatio="xMidYMid meet" aria-label={`${module.label} module`}>
          <circle class="ring" cx="0" cy="0" r="0.66" />
          <g transform={`rotate(${module.angleDeg.toFixed(2)})`}>
            <line class={`arrow ${klass}`} x1="0" y1="0" x2="0" y2={(-len).toFixed(4)} />
            <line
              class={`arrow-head ${klass}`}
              x1="0"
              y1={(-len).toFixed(4)}
              x2="-0.07"
              y2={(-len + 0.085).toFixed(4)}
            />
            <line
              class={`arrow-head ${klass}`}
              x1="0"
              y1={(-len).toFixed(4)}
              x2="0.07"
              y2={(-len + 0.085).toFixed(4)}
            />
          </g>
        </svg>
        <div class="module-meta">
          <span>{module.angleDeg.toFixed(0)} deg</span>
          <span>{module.speedMps.toFixed(2)} m/s</span>
        </div>
      </div>
    {/each}
  </div>
</div>

<style>
  .swerve-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    gap: 0.24rem;
  }

  .heading {
    display: flex;
    justify-content: space-between;
    align-items: center;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    padding: 0.24rem 0.36rem;
    background: rgba(20, 29, 44, 0.92);
    color: var(--text-soft);
    font-size: 0.64rem;
  }

  .heading strong {
    color: var(--text-strong);
    font-family: var(--font-mono);
    font-size: 0.72rem;
  }

  .module-grid {
    min-height: 0;
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.24rem;
  }

  .module-card {
    min-height: 0;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: #141d2e;
    padding: 0.2rem;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr) auto;
    gap: 0.16rem;
  }

  .module-title {
    color: var(--text-strong);
    font-size: 0.62rem;
    font-family: var(--font-display);
    text-transform: uppercase;
    letter-spacing: 0.03em;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 2.6rem;
    border-radius: 6px;
    border: 1px solid rgba(148, 163, 184, 0.2);
    background: #0f172a;
  }

  .ring {
    fill: rgba(15, 23, 42, 0.7);
    stroke: rgba(148, 163, 184, 0.45);
    stroke-width: 0.058;
  }

  .arrow {
    stroke-width: 0.08;
    stroke-linecap: round;
  }

  .arrow-head {
    stroke-width: 0.068;
    stroke-linecap: round;
  }

  .arrow.idle,
  .arrow-head.idle {
    stroke: #94a3b8;
    fill: #94a3b8;
  }

  .arrow.forward,
  .arrow-head.forward {
    stroke: #38bdf8;
    fill: #38bdf8;
  }

  .arrow.reverse,
  .arrow-head.reverse {
    stroke: #fb923c;
    fill: #fb923c;
  }

  .module-meta {
    display: flex;
    justify-content: space-between;
    gap: 0.2rem;
    color: var(--text-soft);
    font-size: 0.58rem;
    font-family: var(--font-mono);
  }
</style>
