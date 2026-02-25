<script lang="ts">
  import type { DashboardSnapshot, SignalRow } from '../lib/arcp';

  type Props = {
    connected: boolean;
    status: string;
    lastError: string;
    snapshot: DashboardSnapshot | null;
    signals: SignalRow[];
    onSelectSignal: (signalId: number) => void;
  };

  type Counter = {
    key: string;
    value: number;
  };

  let { connected, status, lastError, snapshot, signals, onSelectSignal }: Props = $props();

  function formatMemory(bytes: number | null): string {
    if (bytes === null) return 'n/a';
    return `${(bytes / (1024 * 1024)).toFixed(1)} MiB`;
  }

  function formatCpu(value: number | null): string {
    if (value === null) return 'n/a';
    return `${value.toFixed(1)}%`;
  }

  function formatUptime(ms: number | null): string {
    if (ms === null) return 'n/a';
    const totalSec = Math.max(0, Math.floor(ms / 1000));
    const hours = Math.floor(totalSec / 3600);
    const minutes = Math.floor((totalSec % 3600) / 60);
    const seconds = totalSec % 60;
    if (hours > 0) return `${hours}h ${minutes}m ${seconds}s`;
    return `${minutes}m ${seconds}s`;
  }

  function normalizeToken(value: string): string {
    return value.toLowerCase().replace(/[^a-z0-9]+/g, '');
  }

  function boolValue(raw: string): boolean | null {
    const token = raw.trim().toLowerCase();
    if (token === 'true' || token === '1' || token === 'yes' || token === 'on') return true;
    if (token === 'false' || token === '0' || token === 'no' || token === 'off') return false;
    return null;
  }

  const byKind = $derived.by(() => {
    const counts = new Map<string, number>();
    for (const signal of signals) {
      counts.set(signal.kind, (counts.get(signal.kind) ?? 0) + 1);
    }
    return [...counts.entries()]
      .map(([key, value]) => ({ key, value }))
      .sort((a, b) => b.value - a.value || a.key.localeCompare(b.key));
  });

  const byAccess = $derived.by(() => {
    const counts = new Map<string, number>();
    for (const signal of signals) {
      counts.set(signal.access, (counts.get(signal.access) ?? 0) + 1);
    }
    return [...counts.entries()]
      .map(([key, value]) => ({ key, value }))
      .sort((a, b) => b.value - a.value || a.key.localeCompare(b.key));
  });

  const byPolicy = $derived.by(() => {
    const counts = new Map<string, number>();
    for (const signal of signals) {
      counts.set(signal.policy, (counts.get(signal.policy) ?? 0) + 1);
    }
    return [...counts.entries()]
      .map(([key, value]) => ({ key, value }))
      .sort((a, b) => b.value - a.value || a.key.localeCompare(b.key));
  });

  const warningSignals = $derived.by(() => {
    const results: SignalRow[] = [];
    for (const signal of signals) {
      if (signal.signal_type !== 'bool') continue;
      const flag = boolValue(signal.value);
      if (flag !== true) continue;
      const token = normalizeToken(signal.path);
      if (
        token.includes('fault') ||
        token.includes('error') ||
        token.includes('stalled') ||
        token.includes('overcurrent') ||
        token.includes('overtemp') ||
        token.includes('disconnected') ||
        token.includes('blocked')
      ) {
        results.push(signal);
      }
    }
    return results.slice(0, 50);
  });

  const warnings = $derived.by(() => {
    const items: string[] = [];
    if (!connected) items.push('Dashboard is not connected to ARCP runtime.');
    if (lastError.trim()) items.push(`Last error: ${lastError}`);
    if ((snapshot?.server_cpu_percent ?? 0) >= 80) items.push('Server CPU is above 80%.');
    if ((snapshot?.host_cpu_percent ?? 0) >= 80) items.push('Host UI CPU is above 80%.');
    if ((snapshot?.server_rss_bytes ?? 0) >= 512 * 1024 * 1024) items.push('Server RSS is above 512 MiB.');
    if (warningSignals.length > 0) items.push(`${warningSignals.length} fault-like boolean signals are asserted.`);
    return items;
  });

  function topN(counters: Counter[], limit = 8): Counter[] {
    return counters.slice(0, limit);
  }
</script>

<section class="diagnostics-screen panel">
  <header>
    <h2>Diagnostics</h2>
    <p>Runtime health, signal distribution, and active warning indicators.</p>
  </header>

  <div class="summary-grid">
    <article class="summary-card">
      <span>Link</span>
      <strong data-online={connected}>{connected ? 'ONLINE' : 'OFFLINE'}</strong>
      <small>{status || 'unknown'}</small>
    </article>
    <article class="summary-card">
      <span>Signals</span>
      <strong>{signals.length}</strong>
      <small>{snapshot?.update_count ?? 0} updates</small>
    </article>
    <article class="summary-card">
      <span>Server</span>
      <strong>{formatCpu(snapshot?.server_cpu_percent ?? null)}</strong>
      <small>{formatMemory(snapshot?.server_rss_bytes ?? null)}</small>
    </article>
    <article class="summary-card">
      <span>UI Host</span>
      <strong>{formatCpu(snapshot?.host_cpu_percent ?? null)}</strong>
      <small>{formatMemory(snapshot?.host_rss_bytes ?? null)}</small>
    </article>
    <article class="summary-card">
      <span>Uptime</span>
      <strong>{formatUptime(snapshot?.uptime_ms ?? null)}</strong>
      <small>runtime</small>
    </article>
  </div>

  <div class="body-grid">
    <section class="panel-card">
      <h3>Warnings</h3>
      {#if warnings.length === 0}
        <p class="empty">No diagnostic warnings detected.</p>
      {:else}
        <ul class="simple-list">
          {#each warnings as warning (`warn-${warning}`)}
            <li>{warning}</li>
          {/each}
        </ul>
      {/if}
    </section>

    <section class="panel-card">
      <h3>Active Fault Signals</h3>
      {#if warningSignals.length === 0}
        <p class="empty">No fault-like bool signals are currently true.</p>
      {:else}
        <div class="signal-table">
          {#each warningSignals as signal (signal.signal_id)}
            <button class="signal-row" onclick={() => onSelectSignal(signal.signal_id)}>
              <strong>{signal.path}</strong>
              <span>#{signal.signal_id} · {signal.kind} · {signal.access}</span>
            </button>
          {/each}
        </div>
      {/if}
    </section>

    <section class="panel-card compact">
      <h3>By Kind</h3>
      <div class="chip-grid">
        {#each topN(byKind) as item (`kind-${item.key}`)}
          <span class="chip">{item.key}: {item.value}</span>
        {/each}
      </div>
    </section>

    <section class="panel-card compact">
      <h3>By Access</h3>
      <div class="chip-grid">
        {#each topN(byAccess) as item (`access-${item.key}`)}
          <span class="chip">{item.key}: {item.value}</span>
        {/each}
      </div>
    </section>

    <section class="panel-card compact">
      <h3>By Policy</h3>
      <div class="chip-grid">
        {#each topN(byPolicy) as item (`policy-${item.key}`)}
          <span class="chip">{item.key}: {item.value}</span>
        {/each}
      </div>
    </section>
  </div>
</section>

<style>
  .diagnostics-screen {
    padding: 0.78rem;
    display: grid;
    grid-template-rows: auto auto 1fr;
    gap: 0.58rem;
    min-height: 0;
  }

  header h2 {
    margin: 0;
    font-size: 0.98rem;
    color: var(--text-strong);
  }

  header p {
    margin: 0.2rem 0 0;
    color: var(--text-soft);
    font-size: 0.76rem;
  }

  .summary-grid {
    display: grid;
    grid-template-columns: repeat(5, minmax(0, 1fr));
    gap: 0.44rem;
  }

  .summary-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.42rem;
    display: grid;
    gap: 0.1rem;
  }

  .summary-card span {
    font-size: 0.64rem;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    color: var(--text-soft);
  }

  .summary-card strong {
    font-size: 0.8rem;
    font-family: var(--font-mono);
    color: var(--text-strong);
  }

  .summary-card strong[data-online='true'] {
    color: #bbf7d0;
  }

  .summary-card strong[data-online='false'] {
    color: #fecaca;
  }

  .summary-card small {
    font-size: 0.64rem;
    color: var(--text-soft);
  }

  .body-grid {
    min-height: 0;
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.48rem;
    align-content: start;
  }

  .panel-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.46rem;
    min-height: 0;
    display: grid;
    grid-template-rows: auto 1fr;
    gap: 0.32rem;
  }

  .panel-card.compact {
    grid-template-rows: auto;
  }

  .panel-card h3 {
    margin: 0;
    font-size: 0.78rem;
    color: var(--text-strong);
  }

  .empty {
    margin: 0;
    font-size: 0.72rem;
    color: var(--text-soft);
  }

  .simple-list {
    margin: 0;
    padding-left: 1rem;
    display: grid;
    gap: 0.2rem;
    color: var(--text);
    font-size: 0.72rem;
  }

  .signal-table {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.24rem;
    align-content: start;
  }

  .signal-row {
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    background: var(--surface-3);
    color: inherit;
    text-align: left;
    padding: 0.34rem 0.4rem;
    display: grid;
    gap: 0.08rem;
  }

  .signal-row strong {
    font-size: 0.68rem;
    color: var(--text-strong);
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .signal-row span {
    font-size: 0.62rem;
    color: var(--text-soft);
  }

  .chip-grid {
    display: flex;
    flex-wrap: wrap;
    gap: 0.24rem;
  }

  .chip {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    background: var(--surface-3);
    color: var(--text-soft);
    font-size: 0.65rem;
    padding: 0.12rem 0.4rem;
  }

  @media (max-width: 1260px) {
    .summary-grid {
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }

    .body-grid {
      grid-template-columns: 1fr;
    }
  }

  @media (max-width: 860px) {
    .summary-grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
  }
</style>
