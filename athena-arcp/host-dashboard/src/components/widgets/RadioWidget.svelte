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

  function pick(valueRaw: string) {
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

<div class="radio-root" data-direction={config.direction}>
  <fieldset class="radio-list">
    {#each config.options as option (`${option.value}-${option.label}`)}
      <label>
        <input
          type="radio"
          name={`radio-${signal.signal_id}`}
          checked={selected === option.value}
          disabled={!canWrite}
          onchange={() => pick(option.value)}
        />
        <span>{option.label}</span>
      </label>
    {/each}
  </fieldset>

  {#if config.commit === 'button'}
    <button class="btn btn-primary" disabled={!canWrite} onclick={commit}>{config.buttonLabel}</button>
  {/if}
</div>

<style>
  .radio-root {
    display: grid;
    gap: 0.24rem;
  }

  .radio-list {
    border: 0;
    padding: 0;
    margin: 0;
    min-width: 0;
    display: grid;
    gap: 0.22rem;
  }

  .radio-root[data-direction='horizontal'] .radio-list {
    grid-template-columns: repeat(auto-fit, minmax(96px, 1fr));
  }

  label {
    display: inline-flex;
    align-items: center;
    gap: 0.34rem;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-3);
    min-height: 1.56rem;
    padding: 0.12rem 0.34rem;
    color: var(--text);
    font-size: 0.7rem;
  }

  input[type='radio'] {
    width: 0.82rem;
    height: 0.82rem;
    margin: 0;
    accent-color: #dc2626;
  }

  .btn {
    justify-self: end;
    padding: 0.22rem 0.44rem;
    font-size: 0.66rem;
  }
</style>
