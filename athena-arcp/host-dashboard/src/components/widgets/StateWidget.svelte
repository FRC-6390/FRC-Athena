<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';

  let { signal }: { signal: SignalRow } = $props();
</script>

<span class={`state-pill ${signal.value === 'true' ? 'on' : ''}`}>{signal.value}</span>

<style>
  .state-pill {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    border-radius: 999px;
    border: 1px solid var(--border-subtle);
    min-width: 74px;
    padding: 0.18rem 0.45rem;
    font-family: var(--font-display);
    background: var(--surface-3);
    color: var(--text);
    font-size: 0.72rem;
  }

  .state-pill.on {
    background: rgba(180, 35, 45, 0.24);
    color: #ffd7dd;
    border-color: rgba(180, 35, 45, 0.48);
  }
</style>
