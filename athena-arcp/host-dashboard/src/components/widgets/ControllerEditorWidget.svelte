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
    onSendAction: (signalId: number) => void;
  };

  let { signal, signals, signalById, configRaw, onSendSet, onSendAction }: Props = $props();

  let draftValues = $state<Record<number, string>>({});
  let modeDraft = $state('');
  let modeSeedSignalId = $state<number | null>(null);

  const config = $derived(readControllerConfig(configRaw, signal, signals));

  const rows = $derived.by(() =>
    config.params
      .map((param) => {
        const bound = signalById.get(param.signalId);
        if (!bound) return null;
        return {
          ...param,
          signal: bound
        };
      })
      .filter(
        (entry): entry is { key: string; label: string; signalId: number; signal: SignalRow } => entry !== null
      )
  );

  const autotuneModeRow = $derived.by(() => {
    const id = config.autotune.modeSignalId;
    return id ? signalById.get(id) ?? null : null;
  });

  const autotuneStatusRow = $derived.by(() => {
    const id = config.autotune.statusSignalId;
    return id ? signalById.get(id) ?? null : null;
  });

  const autotuneResultRow = $derived.by(() => {
    const id = config.autotune.resultSignalId;
    return id ? signalById.get(id) ?? null : null;
  });

  const autotuneStartActionRow = $derived.by(() => {
    const id = config.autotune.startActionSignalId;
    return id ? signalById.get(id) ?? null : null;
  });

  const autotuneStopActionRow = $derived.by(() => {
    const id = config.autotune.stopActionSignalId;
    return id ? signalById.get(id) ?? null : null;
  });

  const showAutotune = $derived(
    config.autotune.enabled ||
      autotuneModeRow !== null ||
      autotuneStatusRow !== null ||
      autotuneResultRow !== null ||
      autotuneStartActionRow !== null ||
      autotuneStopActionRow !== null
  );

  const modeWritable = $derived(autotuneModeRow?.access === 'write');

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

  $effect(() => {
    if (!autotuneModeRow) {
      modeSeedSignalId = null;
      return;
    }
    if (modeSeedSignalId === autotuneModeRow.signal_id) return;
    modeSeedSignalId = autotuneModeRow.signal_id;
    modeDraft = autotuneModeRow.value === '-' ? '' : autotuneModeRow.value;
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

  function triggerAction(row: SignalRow | null) {
    if (!row) return;
    onSendAction(row.signal_id);
  }

  function sendMode() {
    if (!autotuneModeRow || !modeWritable) return;
    const raw = modeDraft.trim();
    if (!raw) return;

    if (autotuneModeRow.signal_type === 'bool') {
      const normalized = raw.toLowerCase();
      if (normalized !== 'true' && normalized !== 'false') return;
      onSendSet(autotuneModeRow.signal_id, normalized);
      return;
    }

    if (autotuneModeRow.signal_type === 'f64' || autotuneModeRow.signal_type === 'i64') {
      const numeric = Number(raw);
      if (!Number.isFinite(numeric)) return;
      onSendSet(
        autotuneModeRow.signal_id,
        autotuneModeRow.signal_type === 'i64' ? String(Math.round(numeric)) : String(numeric)
      );
      return;
    }

    onSendSet(autotuneModeRow.signal_id, raw);
  }

  function compactValue(value: string): string {
    return value === '-' ? '--' : value;
  }
</script>

{#if rows.length === 0 && !showAutotune}
  <div class="empty">No writable PID/FF parameters found near this signal path.</div>
{:else}
  <div class="controller-root">
    {#if rows.length > 0}
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
    {/if}

    {#if showAutotune}
      <section class="autotune-block">
        <header>
          <strong>Autotuner</strong>
          <span>{config.autotune.enabled ? 'enabled' : 'mapped'}</span>
        </header>

        <div class="autotune-grid">
          <label>
            Mode
            <div class="mode-row">
              <input
                value={autotuneModeRow ? modeDraft : ''}
                disabled={!autotuneModeRow || !modeWritable}
                placeholder={autotuneModeRow ? autotuneModeRow.value : 'not mapped'}
                oninput={(event) => {
                  modeDraft = (event.currentTarget as HTMLInputElement).value;
                }}
              />
              <button class="btn btn-primary" disabled={!autotuneModeRow || !modeWritable} onclick={sendMode}>Set</button>
            </div>
          </label>

          <div class="status-row">
            Status
            <code>{compactValue(autotuneStatusRow?.value ?? '-')}</code>
          </div>

          <div class="status-row">
            Result
            <code>{compactValue(autotuneResultRow?.value ?? '-')}</code>
          </div>
        </div>

        <div class="autotune-actions">
          <button
            class="btn btn-primary"
            disabled={autotuneStartActionRow?.access !== 'invoke'}
            onclick={() => triggerAction(autotuneStartActionRow)}
          >
            Start
          </button>
          <button
            class="btn btn-danger"
            disabled={autotuneStopActionRow?.access !== 'invoke'}
            onclick={() => triggerAction(autotuneStopActionRow)}
          >
            Stop
          </button>
        </div>
      </section>
    {/if}
  </div>
{/if}

<style>
  .controller-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr) auto;
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

  .autotune-block {
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: rgba(27, 33, 44, 0.86);
    display: grid;
    gap: 0.3rem;
    padding: 0.32rem;
  }

  .autotune-block header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.32rem;
  }

  .autotune-block header strong {
    color: var(--text-strong);
    font-size: 0.69rem;
  }

  .autotune-block header span {
    color: var(--text-soft);
    font-size: 0.62rem;
    text-transform: uppercase;
    letter-spacing: 0.04em;
  }

  .autotune-grid {
    display: grid;
    gap: 0.26rem;
  }

  .autotune-grid label {
    display: grid;
    gap: 0.16rem;
    font-size: 0.62rem;
    color: var(--text-soft);
  }

  .status-row {
    display: grid;
    gap: 0.16rem;
    font-size: 0.62rem;
    color: var(--text-soft);
  }

  .mode-row {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 0.24rem;
  }

  .mode-row input {
    min-width: 0;
    font-size: 0.67rem;
    padding: 0.22rem 0.32rem;
    font-family: var(--font-mono);
  }

  .autotune-grid code {
    margin: 0;
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    background: rgba(30, 41, 59, 0.78);
    color: var(--text);
    padding: 0.24rem 0.34rem;
    font-family: var(--font-mono);
    font-size: 0.65rem;
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .autotune-actions {
    display: inline-flex;
    gap: 0.24rem;
  }

  .autotune-actions .btn {
    padding: 0.22rem 0.44rem;
    font-size: 0.63rem;
  }

  .empty {
    color: var(--text-soft);
    font-size: 0.72rem;
  }
</style>
