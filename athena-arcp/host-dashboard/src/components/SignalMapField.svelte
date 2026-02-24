<script lang="ts">
  import { getContext } from 'svelte';
  import type { SignalRow } from '../lib/arcp';
  import SignalMappingPicker from './SignalMappingPicker.svelte';

  type SignalMapRequest = {
    title: string;
    candidates: SignalRow[];
    selectedSignalId: number | null;
    allowNone: boolean;
    noneLabel: string;
    onPick: (signalId: number | null) => void;
  };

  type Props = {
    label: string;
    selectedSignalId: number | null;
    candidates: SignalRow[];
    signalById: Map<number, SignalRow>;
    optional?: boolean;
    enabled?: boolean;
    onToggleEnabled?: (enabled: boolean) => void;
    allowNone?: boolean;
    noneLabel?: string;
    onChange: (signalId: number | null) => void;
  };

  let {
    label,
    selectedSignalId,
    candidates,
    signalById,
    optional = false,
    enabled = selectedSignalId !== null,
    onToggleEnabled,
    allowNone = true,
    noneLabel = 'None',
    onChange
  }: Props = $props();

  let pickerOpen = $state(false);

  const candidateIds = $derived.by(() => new Set(candidates.map((signal) => signal.signal_id)));
  const selectedSignal = $derived(
    selectedSignalId === null ? null : signalById.get(selectedSignalId) ?? null
  );
  const unsupportedSelection = $derived(
    selectedSignalId !== null && !candidateIds.has(selectedSignalId)
  );

  const displayPath = $derived.by(() => {
    if (selectedSignal) return selectedSignal.path;
    if (unsupportedSelection) return `Signal #${selectedSignalId} (unsupported)`;
    return noneLabel;
  });

  const displayType = $derived.by(() => {
    if (selectedSignal) return selectedSignal.signal_type;
    return unsupportedSelection ? 'unsupported' : 'none';
  });

  const requestSignalMap =
    getContext<((request: SignalMapRequest) => void) | undefined>('arcp.signal-map-request');
</script>

<div class="map-field">
  <div class="map-head">
    <span class="map-label">{label}</span>
    {#if optional}
      <label class="map-enabled">
        <input
          type="checkbox"
          checked={enabled}
          onchange={(event) => {
            onToggleEnabled?.((event.currentTarget as HTMLInputElement).checked);
          }}
        />
        include
      </label>
    {/if}
  </div>
  <button
    type="button"
    class="map-trigger"
    data-unsupported={unsupportedSelection}
    onclick={() => {
      if (requestSignalMap) {
        requestSignalMap({
          title: `Map ${label}`,
          candidates,
          selectedSignalId: unsupportedSelection ? null : selectedSignalId,
          allowNone,
          noneLabel,
          onPick: (signalId) => onChange(signalId)
        });
        return;
      }
      pickerOpen = true;
    }}
  >
    <span class="path">{displayPath}</span>
    <span class="chip">{displayType}</span>
  </button>
</div>

<SignalMappingPicker
  open={pickerOpen}
  title={`Map ${label}`}
  signals={candidates}
  selectedSignalId={unsupportedSelection ? null : selectedSignalId}
  {allowNone}
  {noneLabel}
  onPick={(signalId) => onChange(signalId)}
  onClose={() => {
    pickerOpen = false;
  }}
/>

<style>
  .map-field {
    display: grid;
    gap: 0.18rem;
    min-width: 0;
  }

  .map-label {
    color: var(--text-soft);
    font-size: 0.68rem;
  }

  .map-head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.32rem;
    min-width: 0;
  }

  .map-enabled {
    display: inline-flex;
    align-items: center;
    gap: 0.24rem;
    font-size: 0.62rem;
    color: var(--text-soft);
    white-space: nowrap;
  }

  .map-enabled input {
    margin: 0;
  }

  .map-trigger {
    width: 100%;
    min-width: 0;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-3);
    color: var(--text);
    padding: 0.22rem 0.3rem;
    display: flex;
    align-items: center;
    gap: 0.28rem;
    text-align: left;
  }

  .map-trigger:hover {
    border-color: var(--border-emphasis);
  }

  .map-trigger[data-unsupported='true'] {
    border-color: rgba(248, 113, 113, 0.5);
    background: rgba(127, 29, 29, 0.22);
  }

  .path {
    flex: 1 1 auto;
    min-width: 0;
    font-size: 0.68rem;
    line-height: 1.2;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    color: var(--text-strong);
  }

  .chip {
    flex: 0 0 auto;
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    background: var(--surface-2);
    color: var(--text-soft);
    font-size: 0.6rem;
    line-height: 1;
    padding: 0.1rem 0.28rem;
    text-transform: uppercase;
    letter-spacing: 0.02em;
  }
</style>
