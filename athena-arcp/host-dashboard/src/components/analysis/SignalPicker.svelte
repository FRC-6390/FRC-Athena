<script lang="ts">
  import type { SignalDescriptor } from '../../lib/data/types';

  type Props = {
    signals: SignalDescriptor[];
    selected: number[];
    max: number;
    ontoggle: (signalId: number) => void;
  };

  let { signals, selected, max, ontoggle }: Props = $props();
  let query = $state('');

  const shown = $derived.by(() => {
    const q = query.trim().toLowerCase();
    return signals
      .filter((s) => s.type === 'bool' || s.type === 'i64' || s.type === 'f64' || s.type === 'struct:Pose2d')
      .filter((s) => q === '' || s.path.toLowerCase().includes(q))
      .slice(0, 300);
  });
</script>

<div class="picker">
  <input type="search" placeholder="filter signals" bind:value={query} aria-label="filter signals" />
  <p class="count">{selected.length} of {max} plots</p>
  <ul>
    {#each shown as signal (signal.signalId)}
      {@const checked = selected.includes(signal.signalId)}
      <li>
        <label class:checked>
          <input
            type="checkbox"
            {checked}
            disabled={!checked && selected.length >= max}
            onchange={() => ontoggle(signal.signalId)}
          />
          <span class="path" title={signal.path}>{signal.path}</span>
          <span class="type">{signal.type}</span>
        </label>
      </li>
    {:else}
      <li class="empty">No plottable signals match.</li>
    {/each}
  </ul>
</div>

<style>
  .picker {
    display: grid;
    gap: 6px;
    min-height: 0;
  }
  input[type='search'] {
    background: var(--bg-2);
    color: var(--text);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 6px 8px;
    font: inherit;
  }
  .count {
    margin: 0;
    font-size: 12px;
    color: var(--text-soft);
  }
  ul {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    gap: 2px;
    max-height: 420px;
    overflow: auto;
  }
  label {
    display: grid;
    grid-template-columns: auto 1fr auto;
    gap: 6px;
    align-items: center;
    padding: 4px 6px;
    border-radius: 6px;
    font-size: 12px;
    cursor: pointer;
  }
  label:hover {
    background: var(--surface-2, rgba(255, 255, 255, 0.04));
  }
  label.checked .path {
    color: var(--text-strong);
  }
  .path {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-family: var(--font-mono);
  }
  .type {
    color: var(--text-soft);
    font-size: 11px;
  }
  .empty {
    color: var(--text-soft);
    font-size: 12px;
    padding: 6px;
  }
</style>
