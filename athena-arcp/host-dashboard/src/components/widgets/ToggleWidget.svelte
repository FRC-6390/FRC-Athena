<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readToggleConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, configRaw, onSendSet }: Props = $props();

  const config = $derived(readToggleConfig(configRaw));
  const canWrite = $derived(signal.access === 'write');

  const value = $derived.by(() => {
    const normalized = signal.value.trim().toLowerCase();
    return normalized === 'true' || normalized === '1' || normalized === 'on' || normalized === 'yes';
  });

  function toggleValue() {
    if (!canWrite) return;
    onSendSet(signal.signal_id, value ? 'false' : 'true');
  }
</script>

<div class="toggle-root" data-style={config.style} data-on={value}>
  <button class="toggle-btn" disabled={!canWrite} onclick={toggleValue}>
    {#if config.style === 'switch'}
      <span class="track"><span class="thumb"></span></span>
      <span class="label">{value ? config.trueLabel : config.falseLabel}</span>
    {:else}
      <span class="label">{value ? config.trueLabel : config.falseLabel}</span>
    {/if}
  </button>
</div>

<style>
  .toggle-root {
    display: grid;
    align-content: center;
    height: 100%;
  }

  .toggle-btn {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    background: var(--surface-3);
    color: var(--text);
    height: 1.72rem;
    padding: 0.14rem 0.48rem;
    font-size: 0.72rem;
    font-weight: 600;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 0.42rem;
    min-width: 0;
  }

  .toggle-root[data-on='true'] .toggle-btn {
    border-color: rgba(239, 68, 68, 0.58);
    background: rgba(180, 35, 45, 0.26);
    color: #ffe4e6;
  }

  .toggle-btn:disabled {
    opacity: 0.55;
  }

  .track {
    width: 1.9rem;
    height: 1rem;
    border-radius: 999px;
    background: rgba(148, 163, 184, 0.42);
    border: 1px solid var(--border-subtle);
    position: relative;
    flex: 0 0 auto;
  }

  .thumb {
    position: absolute;
    top: 0.08rem;
    left: 0.08rem;
    width: 0.72rem;
    height: 0.72rem;
    border-radius: 999px;
    background: #cbd5e1;
    transition: transform 0.14s ease;
  }

  .toggle-root[data-on='true'] .thumb {
    transform: translateX(0.86rem);
    background: #fee2e2;
  }

  .toggle-root[data-style='button'] .toggle-btn {
    border-radius: 8px;
    width: 100%;
  }

  .label {
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
</style>
