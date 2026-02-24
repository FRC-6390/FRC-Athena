<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readChoiceConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, configRaw, onSendSet }: Props = $props();

  const config = $derived(readChoiceConfig(configRaw, signal));
  const canWrite = $derived(signal.access === 'write');

  let selected = $state('');
  let selectedSignalId = $state<number | null>(null);

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

  function choose(valueRaw: string) {
    selected = valueRaw;
    if (config.commit === 'auto' && canWrite) {
      onSendSet(signal.signal_id, valueRaw);
    }
  }

  function commit() {
    if (!canWrite) return;
    onSendSet(signal.signal_id, selected);
  }
</script>

<div class="choice-root" data-direction={config.direction}>
  <div class="button-list">
    {#each config.options as option (`${option.value}-${option.label}`)}
      <button
        class="choice-btn"
        class:active={selected === option.value}
        disabled={!canWrite}
        onclick={() => choose(option.value)}
      >
        {option.label}
      </button>
    {/each}
  </div>

  {#if config.commit === 'button'}
    <button class="btn btn-primary" disabled={!canWrite} onclick={commit}>{config.buttonLabel}</button>
  {/if}
</div>

<style>
  .choice-root {
    display: grid;
    gap: 0.28rem;
  }

  .button-list {
    display: grid;
    gap: 0.24rem;
    grid-template-columns: repeat(auto-fit, minmax(84px, 1fr));
  }

  .choice-root[data-direction='vertical'] .button-list {
    grid-template-columns: 1fr;
  }

  .choice-btn {
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-3);
    color: var(--text);
    min-height: 1.64rem;
    padding: 0.2rem 0.42rem;
    font-size: 0.7rem;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .choice-btn.active {
    border-color: rgba(180, 35, 45, 0.62);
    background: rgba(180, 35, 45, 0.22);
    color: #ffe4e6;
  }

  .choice-btn:disabled {
    opacity: 0.55;
  }

  .btn {
    justify-self: end;
    padding: 0.22rem 0.44rem;
    font-size: 0.66rem;
  }
</style>
