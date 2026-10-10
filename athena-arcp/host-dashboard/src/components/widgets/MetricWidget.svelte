<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';

  let { signal }: { signal: SignalRow } = $props();
</script>

<div class="metric-value">{signal.value}</div>

<style>
  .metric-value {
    font-size: 1.08rem;
    font-family: var(--font-display);
    line-height: 1.08;
    white-space: normal;
    overflow-wrap: anywhere;
    color: var(--text-strong);
  }
</style>
