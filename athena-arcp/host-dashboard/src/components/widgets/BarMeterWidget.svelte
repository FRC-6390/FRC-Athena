<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readBarConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
  };

  let { signal, configRaw }: Props = $props();

  const config = $derived(readBarConfig(configRaw));

  const value = $derived.by(() => {
    const parsed = Number(signal.value);
    return Number.isFinite(parsed) ? parsed : 0;
  });

  const normalized = $derived.by(() => {
    const range = Math.max(1e-9, config.max - config.min);
    return Math.max(0, Math.min(1, (value - config.min) / range));
  });

  const tone = $derived.by(() => {
    if (config.mode !== 'usage') return 'normal';
    if (normalized >= config.crit) return 'critical';
    if (normalized >= config.warn) return 'warn';
    return 'normal';
  });
</script>

<div class="bar-root" data-tone={tone}>
  <div class="value-row">
    <strong>{value.toFixed(2)}{config.unit ? ` ${config.unit}` : ''}</strong>
    <span>{Math.round(normalized * 100)}%</span>
  </div>

  <div class="bar-track" aria-label="value meter">
    <span class="bar-fill" style={`width:${(normalized * 100).toFixed(1)}%;`}></span>
  </div>

  <div class="range-row">
    <span>{config.min}</span>
    <span>{config.max}</span>
  </div>
</div>

<style>
  .bar-root {
    display: grid;
    gap: 0.22rem;
  }

  .value-row {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    gap: 0.3rem;
  }

  .value-row strong {
    font-size: 0.84rem;
    color: var(--text-strong);
    font-family: var(--font-display);
  }

  .value-row span {
    color: var(--text-soft);
    font-size: 0.68rem;
    font-family: var(--font-mono);
  }

  .bar-track {
    width: 100%;
    height: 0.62rem;
    border-radius: 999px;
    overflow: hidden;
    border: 1px solid var(--border-subtle);
    background: rgba(148, 163, 184, 0.16);
  }

  .bar-fill {
    display: block;
    height: 100%;
    border-radius: inherit;
    background: linear-gradient(90deg, rgba(239, 68, 68, 0.9), rgba(251, 113, 133, 0.95));
  }

  .bar-root[data-tone='warn'] .bar-fill {
    background: linear-gradient(90deg, rgba(245, 158, 11, 0.9), rgba(251, 191, 36, 0.95));
  }

  .bar-root[data-tone='critical'] .bar-fill {
    background: linear-gradient(90deg, rgba(185, 28, 28, 0.95), rgba(239, 68, 68, 0.95));
  }

  .range-row {
    display: flex;
    justify-content: space-between;
    color: var(--text-soft);
    font-size: 0.62rem;
    font-family: var(--font-mono);
  }
</style>
