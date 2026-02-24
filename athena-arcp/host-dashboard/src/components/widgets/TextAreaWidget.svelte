<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readTextAreaConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, configRaw, onSendSet }: Props = $props();

  const config = $derived(readTextAreaConfig(configRaw));
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

  function onKeydown(event: KeyboardEvent) {
    if (config.commit !== 'ctrl-enter') return;
    if (event.key !== 'Enter' || !(event.ctrlKey || event.metaKey)) return;
    event.preventDefault();
    commit();
  }
  const showButton = $derived(config.commit === 'button' || config.commit === 'ctrl-enter');
</script>

<div class="textarea-root" data-has-button={showButton}>
  <textarea
    value={draft}
    rows={config.rows}
    placeholder={config.placeholder}
    disabled={!canWrite}
    maxlength={config.maxLength ?? undefined}
    spellcheck={config.spellcheck}
    oninput={(event) => {
      draft = (event.currentTarget as HTMLTextAreaElement).value;
      if (config.commit === 'auto') {
        commit();
      }
    }}
    onkeydown={onKeydown}
    onblur={() => {
      if (config.commit === 'blur') commit();
    }}
  ></textarea>

  {#if showButton}
    <button class="btn btn-primary set-btn" disabled={!canWrite} onclick={commit}>
      {config.buttonLabel}
    </button>
  {/if}
</div>

<style>
  .textarea-root {
    display: grid;
    gap: 0.24rem;
    height: 100%;
    min-width: 0;
  }

  .textarea-root[data-has-button='true'] {
    grid-template-columns: minmax(0, 1fr) auto;
    grid-template-rows: minmax(0, 1fr);
    align-items: stretch;
  }

  textarea {
    width: 100%;
    min-width: 0;
    min-height: 0;
    resize: vertical;
    font-size: 0.7rem;
    padding: 0.3rem 0.36rem;
    font-family: var(--font-mono);
  }

  .set-btn {
    align-self: stretch;
    flex: 0 0 auto;
    padding: 0.22rem 0.44rem;
    font-size: 0.66rem;
    white-space: nowrap;
  }

  .textarea-root[data-has-button='false'] .set-btn {
    align-self: auto;
  }
</style>
