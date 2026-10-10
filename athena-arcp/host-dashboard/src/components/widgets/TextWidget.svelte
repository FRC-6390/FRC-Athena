<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';

  let { signal }: { signal: SignalRow } = $props();
</script>

<pre>{signal.value}</pre>

<style>
  pre {
    margin: 0;
    white-space: pre-wrap;
    word-break: break-word;
    font-family: var(--font-mono);
    font-size: 0.7rem;
  }
</style>
