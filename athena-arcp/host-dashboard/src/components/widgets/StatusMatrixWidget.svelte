<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readStatusMatrixConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
  };

  type ItemRender = {
    signalId: number;
    label: string;
    group: string;
    healthyWhen: 'true' | 'false';
    value: string;
    state: 'ok' | 'bad' | 'unknown';
  };

  let { signal, signals, signalById, configRaw }: Props = $props();

  function boolState(raw: string): boolean | null {
    const value = raw.trim().toLowerCase();
    if (value === 'true' || value === '1' || value === 'on' || value === 'yes') return true;
    if (value === 'false' || value === '0' || value === 'off' || value === 'no') return false;
    return null;
  }

  const computed = $derived.by(() => {
    const config = readStatusMatrixConfig(configRaw, signal, signals);
    const rows = config.items.map((item): ItemRender => {
      const bound = signalById.get(item.signalId);
      const value = bound?.value ?? '-';
      const boolValue = boolState(value);

      if (boolValue === null) {
        return {
          ...item,
          value,
          state: 'unknown'
        };
      }

      const expected = item.healthyWhen === 'true';
      return {
        ...item,
        value,
        state: boolValue === expected ? 'ok' : 'bad'
      };
    });

    const ok = rows.filter((row) => row.state === 'ok').length;
    const bad = rows.filter((row) => row.state === 'bad').length;
    const unknown = rows.length - ok - bad;

    return {
      config,
      rows,
      ok,
      bad,
      unknown
    };
  });
</script>

{#if computed.rows.length === 0}
  <div class="empty">No boolean signals configured.</div>
{:else}
  <div class="matrix-root">
    {#if computed.config.showSummary}
      <div class="summary-row">
        <span class="summary-chip ok">OK {computed.ok}</span>
        <span class="summary-chip bad">Bad {computed.bad}</span>
        <span class="summary-chip unknown">Unknown {computed.unknown}</span>
      </div>
    {/if}

    <div class="grid" style={`--cols:${computed.config.columns};`}>
      {#each computed.rows as row (`${row.signalId}-${row.label}`)}
        <article class={`tile ${row.state}`} title={`Signal #${row.signalId}: ${row.value}`}>
          <strong>{row.label}</strong>
          <span class="tile-group">{row.group}</span>
          <span class="tile-value">{row.value}</span>
        </article>
      {/each}
    </div>
  </div>
{/if}

<style>
  .matrix-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    gap: 0.26rem;
  }

  .empty {
    color: var(--text-soft);
    font-size: 0.72rem;
  }

  .summary-row {
    display: inline-flex;
    gap: 0.24rem;
    flex-wrap: wrap;
  }

  .summary-chip {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.12rem 0.34rem;
    font-size: 0.62rem;
    color: var(--text-soft);
    background: var(--surface-3);
  }

  .summary-chip.ok {
    border-color: rgba(16, 185, 129, 0.45);
    color: #86efac;
    background: rgba(6, 78, 59, 0.45);
  }

  .summary-chip.bad {
    border-color: rgba(248, 113, 113, 0.45);
    color: #fecaca;
    background: rgba(127, 29, 29, 0.35);
  }

  .summary-chip.unknown {
    border-color: rgba(148, 163, 184, 0.45);
  }

  .grid {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.24rem;
    grid-template-columns: repeat(var(--cols, 4), minmax(0, 1fr));
    align-content: start;
  }

  .tile {
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-3);
    padding: 0.22rem 0.28rem;
    display: grid;
    gap: 0.12rem;
  }

  .tile strong {
    color: var(--text-strong);
    font-size: 0.68rem;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .tile-group {
    color: var(--text-soft);
    font-size: 0.58rem;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .tile-value {
    color: var(--text);
    font-family: var(--font-mono);
    font-size: 0.64rem;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .tile.ok {
    border-color: rgba(16, 185, 129, 0.52);
    background: rgba(6, 78, 59, 0.35);
  }

  .tile.bad {
    border-color: rgba(248, 113, 113, 0.52);
    background: rgba(127, 29, 29, 0.3);
  }

  .tile.unknown {
    border-color: rgba(148, 163, 184, 0.4);
    background: rgba(51, 65, 85, 0.45);
  }

  @media (max-width: 820px) {
    .grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
  }
</style>
