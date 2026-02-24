<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readDropdownConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, configRaw, onSendSet }: Props = $props();

  let selected = $state('');
  let selectedSignalId = $state<number | null>(null);

  const config = $derived(readDropdownConfig(configRaw, signal));

  $effect(() => {
    if (selectedSignalId !== signal.signal_id) {
      selectedSignalId = signal.signal_id;
      selected = signal.value;
      return;
    }

    if (config.commit === 'auto') {
      selected = signal.value;
      return;
    }

    if (!config.options.some((option) => option.value === selected)) {
      selected = signal.value;
    }
  });

  function applySelection() {
    onSendSet(signal.signal_id, selected);
  }
</script>

<div class="dropdown-root">
  <select
    value={selected}
    onchange={(event) => {
      selected = (event.currentTarget as HTMLSelectElement).value;
      if (config.commit === 'auto') {
        applySelection();
      }
    }}
  >
    {#if config.options.length === 0}
      <option value={signal.value}>{signal.value || '(empty)'}</option>
    {:else}
      {#each config.options as option (`${option.label}-${option.value}`)}
        <option value={option.value}>{option.label}</option>
      {/each}
    {/if}
  </select>

  {#if config.commit === 'button'}
    <button class="btn btn-primary set-btn" onclick={applySelection}>Set</button>
  {/if}
</div>

<style>
  .dropdown-root {
    display: flex;
    align-items: center;
    gap: 0.3rem;
    width: 100%;
    min-width: 0;
  }

  select {
    flex: 1 1 auto;
    width: auto;
    min-width: 0;
    font-size: 0.7rem;
    padding: 0.26rem 0.4rem;
    border: 1px solid var(--border-subtle);
    background: #2b3445;
    color: var(--text-strong);
  }

  select option {
    background: #1f2735;
    color: var(--text-strong);
  }

  .set-btn {
    flex: 0 0 auto;
    padding: 0.22rem 0.44rem;
    font-size: 0.66rem;
    white-space: nowrap;
  }
</style>
