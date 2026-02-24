<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readControllerConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, signals, signalById, configRaw, onSendSet }: Props = $props();

  let draftValues = $state<Record<number, string>>({});

  const rows = $derived.by(() => {
    const config = readControllerConfig(configRaw, signal, signals);
    return config.params
      .map((param) => {
        const bound = signalById.get(param.signalId);
        if (!bound) return null;
        return {
          ...param,
          signal: bound
        };
      })
      .filter((entry): entry is { key: string; label: string; signalId: number; signal: SignalRow } => entry !== null);
  });

  $effect(() => {
    const next = { ...draftValues };
    let changed = false;

    const ids = new Set<number>();
    for (const row of rows) {
      ids.add(row.signalId);
      if (next[row.signalId] === undefined) {
        next[row.signalId] = row.signal.value;
        changed = true;
      }
    }

    for (const signalId of Object.keys(next).map(Number)) {
      if (!ids.has(signalId)) {
        delete next[signalId];
        changed = true;
      }
    }

    if (changed) {
      draftValues = next;
    }
  });

  function setOne(signalId: number) {
    const value = draftValues[signalId] ?? '';
    onSendSet(signalId, value);
  }

  function setAll() {
    for (const row of rows) {
      setOne(row.signalId);
    }
  }
</script>

{#if rows.length === 0}
  <div class="empty">No writable PID/FF parameters found near this signal path.</div>
{:else}
  <div class="controller-root">
    <div class="controller-head">
      <span>Parameters</span>
      <button class="btn" onclick={setAll}>Set all</button>
    </div>

    <div class="controller-rows">
      {#each rows as row (row.signalId)}
        <div class="controller-row">
          <span class="row-label">{row.label}</span>
          <input
            value={draftValues[row.signalId] ?? row.signal.value}
            oninput={(event) => {
              draftValues = {
                ...draftValues,
                [row.signalId]: (event.currentTarget as HTMLInputElement).value
              };
            }}
          />
          <button class="btn btn-primary" onclick={() => setOne(row.signalId)}>Set</button>
        </div>
      {/each}
    </div>
  </div>
{/if}

<style>
  .controller-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: auto 1fr;
    gap: 0.24rem;
  }

  .controller-head {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 0.4rem;
    color: var(--text-soft);
    font-size: 0.68rem;
  }

  .controller-head .btn {
    padding: 0.22rem 0.4rem;
    font-size: 0.64rem;
  }

  .controller-rows {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.22rem;
    align-content: start;
  }

  .controller-row {
    display: grid;
    grid-template-columns: 60px minmax(0, 1fr) auto;
    gap: 0.24rem;
    align-items: center;
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    background: var(--surface-3);
    padding: 0.2rem;
  }

  .controller-row .row-label {
    color: var(--text-soft);
    font-size: 0.66rem;
    font-family: var(--font-display);
  }

  .controller-row input {
    min-width: 0;
    font-family: var(--font-mono);
    font-size: 0.68rem;
    padding: 0.22rem 0.36rem;
  }

  .controller-row .btn {
    padding: 0.22rem 0.4rem;
    font-size: 0.64rem;
  }

  .empty {
    color: var(--text-soft);
    font-size: 0.72rem;
  }
</style>
