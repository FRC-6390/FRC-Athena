<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readInputConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, configRaw, onSendSet }: Props = $props();

  const config = $derived(readInputConfig(configRaw, signal));
  const canWrite = $derived(signal.access === 'write');

  let draft = $state('');
  let selectedSignalId = $state<number | null>(null);

  $effect(() => {
    if (selectedSignalId !== signal.signal_id) {
      selectedSignalId = signal.signal_id;
      draft = signal.value;
      return;
    }

    if (config.commit === 'auto') {
      draft = signal.value;
    }
  });

  function commit() {
    if (!canWrite) return;
    onSendSet(signal.signal_id, draft);
  }

  function onInput(valueRaw: string) {
    draft = valueRaw;
    if (config.commit === 'auto') {
      commit();
    }
  }

  function onKeydown(event: KeyboardEvent) {
    if (config.commit !== 'enter') return;
    if (event.key !== 'Enter') return;
    event.preventDefault();
    commit();
  }

  const showButton = $derived(config.commit === 'button');
</script>

<div class="input-root" data-type={config.inputType} data-has-button={showButton}>
  <input
    type={config.inputType}
    value={draft}
    placeholder={config.placeholder}
    disabled={!canWrite}
    min={config.min || undefined}
    max={config.max || undefined}
    step={config.step || undefined}
    pattern={config.pattern || undefined}
    autocomplete={config.autocomplete || undefined}
    spellcheck={config.spellcheck}
    oninput={(event) => onInput((event.currentTarget as HTMLInputElement).value)}
    onkeydown={onKeydown}
    onblur={() => {
      if (config.commit === 'blur') commit();
    }}
  />

  {#if showButton}
    <button class="btn btn-primary set-btn" disabled={!canWrite} onclick={commit}>
      {config.buttonLabel}
    </button>
  {/if}
</div>

<style>
  .input-root {
    display: flex;
    align-items: center;
    gap: 0.24rem;
    width: 100%;
    min-width: 0;
  }

  input {
    flex: 1 1 auto;
    width: auto;
    min-width: 0;
    font-size: 0.7rem;
    padding: 0.26rem 0.36rem;
  }

  .input-root[data-type='range'] input {
    padding: 0;
  }

  .set-btn {
    flex: 0 0 auto;
    padding: 0.22rem 0.44rem;
    font-size: 0.66rem;
    white-space: nowrap;
  }
</style>
