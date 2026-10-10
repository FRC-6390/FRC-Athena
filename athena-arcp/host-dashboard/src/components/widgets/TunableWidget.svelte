<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';

  let {
    signal,
    inputValue,
    onInput,
    onSendSet
  }: {
    signal: SignalRow;
    inputValue: string | undefined;
    onInput: (value: string) => void;
    onSendSet: (signalId: number, valueRaw: string) => void;
  } = $props();
</script>

<div class="inline-set-row">
  <input value={inputValue ?? signal.value} oninput={(e) => onInput((e.currentTarget as HTMLInputElement).value)} />
  <button class="btn btn-primary" onclick={() => onSendSet(signal.signal_id, inputValue ?? '')}>Set value</button>
</div>

<style>
  .inline-set-row {
    display: flex;
    align-items: center;
    gap: 0.3rem;
    width: 100%;
    min-width: 0;
  }

  .inline-set-row input {
    flex: 1 1 auto;
    min-width: 0;
  }

  .inline-set-row .btn {
    flex: 0 0 auto;
    white-space: nowrap;
  }
</style>
