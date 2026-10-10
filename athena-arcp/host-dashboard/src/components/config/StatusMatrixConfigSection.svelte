<script lang="ts">
  import { getContext } from 'svelte';
  import { addStatusMatrixSignal, readStatusMatrixConfig } from '../../lib/widget-config';
  import type { SignalSectionProps, SignalMapRequest } from './types';

  let {
    selectedWidget,
    primarySignal,
    selectedSignal,
    signals,
    signalById,
    pools,
    setConfig
  }: SignalSectionProps = $props();

  const numericSignals = $derived(pools.numericSignals);
  const boolSignals = $derived(pools.boolSignals);
  const requestSignalMap =
    getContext<((request: SignalMapRequest) => void) | undefined>('arcp.signal-map-request');

  function addSelectedBoolToMatrix() {
    if (!primarySignal || !selectedSignal) return;
    if (selectedSignal.signal_type !== 'bool') return;
    setConfig(addStatusMatrixSignal(selectedWidget.config, primarySignal, signals, selectedSignal));
  }
  function requestStatusMatrixSignalFromExplorer() {
    if (!primarySignal) return;
    if (!requestSignalMap) {
      addSelectedBoolToMatrix();
      return;
    }
    requestSignalMap({
      title: 'Add status board signal',
      candidates: boolSignals,
      selectedSignalId: selectedSignal?.signal_type === 'bool' ? selectedSignal.signal_id : null,
      allowNone: false,
      noneLabel: 'Required',
      onPick: (signalId) => {
        if (signalId === null) return;
        const signal = signalById.get(signalId);
        if (!signal || signal.signal_type !== 'bool') return;
        setConfig(addStatusMatrixSignal(selectedWidget.config, primarySignal, signals, signal));
      }
    });
  }
</script>

{#if selectedWidget.kind === 'status_matrix'}
  {@const matrix = readStatusMatrixConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Status Board</h4>
    <div class="row-grid">
      <label>
        Columns
        <input
          type="number"
          min="1"
          max="8"
          value={matrix.columns}
          oninput={(event) =>
            setConfig({
              ...matrix,
              columns: Math.max(1, Math.min(8, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || 4)))
            })}
        />
      </label>

      <label>
        Mode
        <select
          value={matrix.mode}
          onchange={(event) =>
            setConfig({
              ...matrix,
              mode: (event.currentTarget as HTMLSelectElement).value as 'status' | 'toggle' | 'toggle_value'
            })}
        >
          <option value="status">Status only</option>
          <option value="toggle">Toggle</option>
          <option value="toggle_value">Toggle + value</option>
        </select>
      </label>
    </div>

    <label class="row-inline">
      <input
        type="checkbox"
        checked={matrix.showSummary}
        onchange={(event) => setConfig({ ...matrix, showSummary: (event.currentTarget as HTMLInputElement).checked })}
      />
      Show summary chips
    </label>

    <button class="btn" disabled={boolSignals.length === 0} onclick={requestStatusMatrixSignalFromExplorer}>
      Add bool signal from explorer
    </button>

    <div class="series-list">
      {#each matrix.items as item, index (`${item.signalId}-${index}`)}
        {@const bound = signalById.get(item.signalId)}
        <div class="series-row">
          <strong>{bound?.path ?? `Signal ${item.signalId}`}</strong>
          <select
            value={item.healthyWhen}
            onchange={(event) =>
              setConfig({
                ...matrix,
                items: matrix.items.map((entry, itemIndex) =>
                  itemIndex === index
                    ? { ...entry, healthyWhen: (event.currentTarget as HTMLSelectElement).value as 'true' | 'false' }
                    : entry
                )
              })}
          >
            <option value="true">Healthy when true</option>
            <option value="false">Healthy when false</option>
          </select>
          <input
            value={item.label}
            oninput={(event) =>
              setConfig({
                ...matrix,
                items: matrix.items.map((entry, itemIndex) =>
                  itemIndex === index
                    ? { ...entry, label: (event.currentTarget as HTMLInputElement).value }
                    : entry
                )
              })}
          />
          {#if matrix.mode === 'toggle' || matrix.mode === 'toggle_value'}
            <select
              value={item.toggleSignalId ?? ''}
              onchange={(event) => {
                const raw = (event.currentTarget as HTMLSelectElement).value;
                setConfig({
                  ...matrix,
                  items: matrix.items.map((entry, itemIndex) =>
                    itemIndex === index
                      ? { ...entry, toggleSignalId: raw ? Number(raw) : null }
                      : entry
                  )
                });
              }}
            >
              <option value="">No toggle signal</option>
              {#each boolSignals as option (option.signal_id)}
                <option value={option.signal_id}>{option.path}</option>
              {/each}
            </select>
          {/if}
          {#if matrix.mode === 'toggle_value'}
            <select
              value={item.valueSignalId ?? ''}
              onchange={(event) => {
                const raw = (event.currentTarget as HTMLSelectElement).value;
                setConfig({
                  ...matrix,
                  items: matrix.items.map((entry, itemIndex) =>
                    itemIndex === index
                      ? { ...entry, valueSignalId: raw ? Number(raw) : null }
                      : entry
                  )
                });
              }}
            >
              <option value="">No value signal</option>
              {#each numericSignals as option (option.signal_id)}
                <option value={option.signal_id}>{option.path}</option>
              {/each}
            </select>
          {/if}
          <button
            class="btn btn-danger"
            onclick={() =>
              setConfig({
                ...matrix,
                items: matrix.items.filter((_, itemIndex) => itemIndex !== index)
              })}
          >
            Remove
          </button>
        </div>
      {/each}
    </div>
  </section>
{/if}
