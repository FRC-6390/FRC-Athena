<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import { sparklinePath } from '../../lib/dashboard';

  let { signal, historyBySignal }: { signal: SignalRow; historyBySignal: Map<number, number[]> } = $props();
</script>

<svg viewBox="0 0 100 34" preserveAspectRatio="none" aria-label="trend chart">
  <path d={sparklinePath(historyBySignal.get(signal.signal_id) ?? [])} />
</svg>
<div class="meta-line">Current: {signal.value}</div>

<style>
  .meta-line {
    color: var(--text-soft);
    font-size: 0.68rem;
    line-height: 1.2;
    white-space: normal;
    overflow-wrap: anywhere;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 0;
    border-radius: 6px;
    background: #20293a;
    border: 1px solid var(--border-subtle);
  }

  path {
    fill: none;
    stroke: var(--brand);
    stroke-width: 1.6;
  }
</style>
