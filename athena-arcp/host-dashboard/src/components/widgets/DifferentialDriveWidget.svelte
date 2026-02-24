<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseNumericArray, readDifferentialDriveConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
  };

  let { signal, signals, signalById, configRaw }: Props = $props();

  const config = $derived(readDifferentialDriveConfig(configRaw, signal, signals));
  const packed = $derived(parseNumericArray(signal.value));

  function numericSignalValue(signalId: number | null): number | null {
    if (signalId === null) return null;
    const row = signalById.get(signalId);
    if (!row) return null;
    const parsed = Number(row.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  const leftSpeed = $derived.by(() => numericSignalValue(config.leftSpeedSignalId) ?? packed[0] ?? 0);
  const rightSpeed = $derived.by(() => numericSignalValue(config.rightSpeedSignalId) ?? packed[1] ?? 0);
  const headingDeg = $derived.by(() => numericSignalValue(config.headingSignalId) ?? packed[2] ?? 0);

  const leftRatio = $derived.by(() => Math.max(-1, Math.min(1, leftSpeed / Math.max(0.1, config.maxSpeed))));
  const rightRatio = $derived.by(() => Math.max(-1, Math.min(1, rightSpeed / Math.max(0.1, config.maxSpeed))));
</script>

<div class="diff-root">
  <div class="heading-row">
    <span>Heading</span>
    <strong>{headingDeg.toFixed(1)} deg</strong>
  </div>

  <div class="body">
    <div class="bar-col">
      <span class="label">Left</span>
      <div class="track">
        <div class="fill {leftRatio >= 0 ? 'forward' : 'reverse'}" style={`height:${Math.abs(leftRatio) * 100}%`}></div>
      </div>
      <span class="value">{leftSpeed.toFixed(2)} m/s</span>
    </div>

    <div class="chassis">
      <svg viewBox="-1 -1 2 2" preserveAspectRatio="xMidYMid meet" aria-label="differential heading">
        <rect class="robot-body" x="-0.46" y="-0.7" width="0.92" height="1.4" rx="0.12" />
        <g transform={`rotate(${headingDeg.toFixed(2)})`}>
          <line class="heading-arrow" x1="0" y1="0.12" x2="0" y2="-0.62" />
          <polygon class="heading-arrow-head" points="0,-0.77 -0.09,-0.57 0.09,-0.57" />
        </g>
      </svg>
    </div>

    <div class="bar-col">
      <span class="label">Right</span>
      <div class="track">
        <div class="fill {rightRatio >= 0 ? 'forward' : 'reverse'}" style={`height:${Math.abs(rightRatio) * 100}%`}></div>
      </div>
      <span class="value">{rightSpeed.toFixed(2)} m/s</span>
    </div>
  </div>
</div>

<style>
  .diff-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    gap: 0.2rem;
  }

  .heading-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: rgba(20, 29, 44, 0.9);
    padding: 0.2rem 0.34rem;
    color: var(--text-soft);
    font-size: 0.64rem;
  }

  .heading-row strong {
    color: var(--text-strong);
    font-family: var(--font-mono);
    font-size: 0.7rem;
  }

  .body {
    min-height: 0;
    display: grid;
    grid-template-columns: minmax(2.4rem, 3rem) minmax(0, 1fr) minmax(2.4rem, 3rem);
    gap: 0.24rem;
  }

  .bar-col {
    min-height: 0;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr) auto;
    gap: 0.14rem;
    align-items: center;
  }

  .label {
    color: var(--text-soft);
    font-size: 0.58rem;
    text-transform: uppercase;
    letter-spacing: 0.03em;
  }

  .track {
    width: 100%;
    height: 100%;
    min-height: 2.9rem;
    border-radius: 999px;
    border: 1px solid rgba(148, 163, 184, 0.35);
    background: rgba(15, 23, 42, 0.6);
    display: flex;
    align-items: flex-end;
    overflow: hidden;
  }

  .fill {
    width: 100%;
    min-height: 2px;
    transition: height 70ms linear;
  }

  .fill.forward {
    background: linear-gradient(180deg, #38bdf8, #0ea5e9);
  }

  .fill.reverse {
    background: linear-gradient(180deg, #fb923c, #f97316);
  }

  .value {
    color: var(--text-soft);
    font-size: 0.56rem;
    font-family: var(--font-mono);
  }

  .chassis {
    min-height: 0;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: #101827;
    padding: 0.16rem;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 3.5rem;
  }

  .robot-body {
    fill: rgba(148, 163, 184, 0.22);
    stroke: rgba(148, 163, 184, 0.5);
    stroke-width: 0.08;
  }

  .heading-arrow {
    stroke: #f87171;
    stroke-width: 0.12;
    stroke-linecap: round;
  }

  .heading-arrow-head {
    fill: #f87171;
  }
</style>
