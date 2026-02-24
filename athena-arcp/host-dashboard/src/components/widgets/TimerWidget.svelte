<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readTimerConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
  };

  let { signal, configRaw }: Props = $props();

  const config = $derived(readTimerConfig(configRaw));

  const sourceSeconds = $derived.by(() => {
    const parsed = Number(signal.value);
    return Number.isFinite(parsed) ? parsed : 0;
  });

  const displaySeconds = $derived.by(() => {
    if (config.mode === 'countdown') {
      return Math.max(0, config.durationSec - sourceSeconds);
    }
    return Math.max(0, sourceSeconds);
  });

  const progress = $derived.by(() => {
    const duration = Math.max(1, config.durationSec);
    if (config.mode === 'countdown') {
      return Math.max(0, Math.min(1, displaySeconds / duration));
    }
    return Math.max(0, Math.min(1, sourceSeconds / duration));
  });

  function formatClock(totalSeconds: number): string {
    const seconds = Math.max(0, Math.floor(totalSeconds));
    const minutes = Math.floor(seconds / 60);
    const rem = seconds % 60;
    return `${minutes}:${String(rem).padStart(2, '0')}`;
  }
</script>

<div class="timer-root" data-mode={config.mode}>
  <div class="timer-clock">{formatClock(displaySeconds)}</div>
  <div class="timer-sub">{config.mode === 'countdown' ? 'remaining' : 'elapsed'}</div>
  <div class="timer-track" aria-label="timer progress">
    <span class="timer-fill" style={`width:${(progress * 100).toFixed(1)}%;`}></span>
  </div>
</div>

<style>
  .timer-root {
    display: grid;
    gap: 0.18rem;
    align-content: center;
  }

  .timer-clock {
    color: var(--text-strong);
    font-size: clamp(1.25rem, 3.1vh, 2.35rem);
    line-height: 1;
    font-family: var(--font-display);
    font-variant-numeric: tabular-nums;
    letter-spacing: 0.03em;
  }

  .timer-sub {
    color: var(--text-soft);
    font-size: 0.7rem;
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }

  .timer-track {
    height: 0.46rem;
    border-radius: 999px;
    border: 1px solid var(--border-subtle);
    overflow: hidden;
    background: rgba(148, 163, 184, 0.16);
  }

  .timer-fill {
    display: block;
    height: 100%;
    border-radius: inherit;
    background: linear-gradient(90deg, rgba(239, 68, 68, 0.92), rgba(248, 113, 113, 0.95));
  }

  .timer-root[data-mode='countdown'] .timer-fill {
    background: linear-gradient(90deg, rgba(245, 158, 11, 0.92), rgba(251, 191, 36, 0.95));
  }
</style>
