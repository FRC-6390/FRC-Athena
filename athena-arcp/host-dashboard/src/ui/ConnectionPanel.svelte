<script lang="ts" module>
  /** What the panel needs from a robot data source (LiveSource in the desktop app; the WS bridge in a browser). */
  export type Connector = {
    connect(options: { host: string; replay?: boolean }): Promise<unknown>;
    disconnect(): Promise<void>;
    onStatus(listener: (status: import('../lib/data/live-source').LiveStatus) => void): () => void;
    onEvent?(listener: (event: import('../lib/data/live-source').LiveEvent) => void): () => void;
    timeSync?(samples?: number): Promise<unknown>;
    recordStart?(): Promise<string>;
  };
</script>

<script lang="ts">
  import { onDestroy, onMount } from 'svelte';
  import type { LiveStatus } from '../lib/data/live-source';

  type Props = { connector: Connector | null };
  let { connector }: Props = $props();

  type TargetKey = 'sim' | 'usb' | 'radio' | 'mdns' | 'custom';
  const TARGET_KEY = 'athena.dashboard.target';
  const TEAM_KEY = 'athena.dashboard.team';
  const HOST_KEY = 'athena.dashboard.host';

  function remembered(key: string, fallback: string): string {
    try {
      return localStorage.getItem(key) ?? fallback;
    } catch {
      return fallback;
    }
  }
  function remember(key: string, value: string) {
    try {
      localStorage.setItem(key, value);
    } catch {
      /* storage unavailable: a per-viewer convenience only */
    }
  }

  let target = $state<TargetKey>((remembered(TARGET_KEY, 'sim') as TargetKey) || 'sim');
  let team = $state(remembered(TEAM_KEY, ''));
  let custom = $state(remembered(HOST_KEY, ''));
  let record = $state(false);
  let busy = $state(false);
  let error = $state<string | null>(null);
  let status = $state<LiveStatus | null>(null);
  const offs: Array<() => void> = [];

  const teamNumber = $derived(Number(team));
  const teamValid = $derived(Number.isInteger(teamNumber) && teamNumber >= 1 && teamNumber <= 9999);
  const targets = $derived([
    { key: 'sim' as const, label: 'Simulator', host: '127.0.0.1', needsTeam: false },
    { key: 'usb' as const, label: 'Robot · USB', host: '172.22.11.2', needsTeam: false },
    {
      key: 'radio' as const,
      label: 'Robot · radio',
      host: teamValid ? `10.${Math.floor(teamNumber / 100)}.${teamNumber % 100}.2` : '10.TE.AM.2',
      needsTeam: true
    },
    { key: 'mdns' as const, label: 'Robot · mDNS', host: teamValid ? `roborio-${teamNumber}-frc.local` : 'roborio-TEAM-frc.local', needsTeam: true },
    { key: 'custom' as const, label: 'Custom host', host: custom || 'hostname or IP', needsTeam: false }
  ]);
  const selected = $derived(targets.find((t) => t.key === target) ?? targets[0]);
  const ready = $derived(selected.needsTeam ? teamValid : selected.key === 'custom' ? custom.trim() !== '' : true);
  const connected = $derived(status?.connected === true);

  onMount(() => {
    if (!connector) return;
    offs.push(connector.onStatus((next) => (status = next)));
    if (connector.onEvent) {
      offs.push(
        connector.onEvent((event) => {
          if (event.kind === 'disconnected') error = `Disconnected: ${event.reason}`;
        })
      );
    }
  });
  onDestroy(() => offs.forEach((off) => off()));

  async function connect() {
    if (!connector || !ready) return;
    busy = true;
    error = null;
    remember(TARGET_KEY, target);
    remember(TEAM_KEY, team);
    remember(HOST_KEY, custom);
    const host = selected.key === 'custom' ? custom.trim() : selected.host;
    try {
      await connector.connect({ host, replay: record });
      await connector.timeSync?.(8).catch(() => undefined);
      if (record) await connector.recordStart?.().catch((e) => (error = `Not recording: ${String(e)}`));
    } catch (e) {
      error = `Could not connect to ${host}: ${e instanceof Error ? e.message : String(e)}`;
    } finally {
      busy = false;
    }
  }

  async function disconnect() {
    if (!connector) return;
    busy = true;
    try {
      await connector.disconnect();
    } finally {
      busy = false;
    }
  }

  const ms = (micros: number | null | undefined) => (micros === null || micros === undefined ? '—' : `${(micros / 1000).toFixed(1)} ms`);
</script>

<div class="panel">
  {#if !connector}
    <p class="note">
      This window is showing demo data. Connect to a robot or the simulator from the desktop app, or run
      <code>arcp-ws-bridge</code> and reload.
    </p>
  {:else}
    <div class="targets" role="radiogroup" aria-label="Connection target">
      {#each targets as t (t.key)}
        <button
          type="button"
          role="radio"
          aria-checked={target === t.key}
          class="target"
          class:active={target === t.key}
          disabled={connected}
          onclick={() => (target = t.key)}
        >
          <span class="t-label">{t.label}</span>
          <span class="t-host ui-mono">{t.host}</span>
        </button>
      {/each}
    </div>

    <div class="row">
      {#if selected.needsTeam}
        <label class="field">
          <span class="ui-label">Team</span>
          <input class="ui-input team" inputmode="numeric" bind:value={team} disabled={connected} placeholder="6390" />
        </label>
      {:else if selected.key === 'custom'}
        <label class="field grow">
          <span class="ui-label">Host</span>
          <input class="ui-input" bind:value={custom} disabled={connected} placeholder="10.63.90.2" />
        </label>
      {/if}
      <label class="check">
        <input type="checkbox" bind:checked={record} disabled={connected} />
        Record session (.athrp)
      </label>
      <span class="ui-spacer"></span>
      {#if connected}
        <button type="button" class="ui-btn" onclick={disconnect} disabled={busy}>Disconnect</button>
      {:else}
        <button type="button" class="ui-btn" data-variant="primary" onclick={connect} disabled={busy || !ready}>
          {busy ? 'Connecting…' : 'Connect'}
        </button>
      {/if}
    </div>

    {#if status}
      <div class="stats">
        <div><span class="ui-label">Status</span><span class="v" data-tone={connected ? 'ok' : 'fault'}>{connected ? 'Connected' : 'Disconnected'}</span></div>
        <div><span class="ui-label">Round trip</span><span class="v ui-mono">{ms(status.rttMicros)}</span></div>
        <div><span class="ui-label">Clock offset</span><span class="v ui-mono">{ms(status.offsetMicros)}</span></div>
        <div><span class="ui-label">Lost cycles</span><span class="v ui-mono">{status.lostCycles}</span></div>
        <div><span class="ui-label">Backfills</span><span class="v ui-mono">{status.backfills}</span></div>
        {#if status.server}<div><span class="ui-label">Clients</span><span class="v ui-mono">{status.server.clients}</span></div>{/if}
      </div>
      {#if status.server?.throttled}
        <p class="warn">Bandwidth budget is throttling: {status.server.throttledPatterns.join(', ')}</p>
      {/if}
    {/if}
    {#if error}<p class="error">{error}</p>{/if}
  {/if}
</div>

<style>
  .panel {
    display: grid;
    gap: var(--sp-3);
  }
  .targets {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
    gap: var(--sp-2);
  }
  .target {
    display: grid;
    gap: 2px;
    padding: var(--sp-2) var(--sp-3);
    border: 1px solid var(--border-subtle);
    border-radius: var(--radius);
    background: var(--surface-2);
    color: var(--text);
    font: inherit;
    text-align: left;
    cursor: pointer;
  }
  .target:hover:not(:disabled) {
    border-color: var(--border-emphasis);
  }
  .target.active {
    border-color: var(--brand);
    background: var(--brand-soft);
  }
  .target:disabled {
    cursor: default;
    opacity: 0.6;
  }
  .target.active:disabled {
    opacity: 1;
  }
  .t-label {
    font-size: var(--fs-sm);
    font-weight: 600;
    color: var(--text-strong);
  }
  .t-host {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .row {
    display: flex;
    align-items: flex-end;
    gap: var(--sp-3);
    flex-wrap: wrap;
  }
  .field {
    display: grid;
    gap: 4px;
  }
  .field.grow {
    flex: 1;
  }
  .team {
    width: 90px;
  }
  .check {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    height: 28px;
    font-size: var(--fs-sm);
    color: var(--text-soft);
  }
  .stats {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(100px, 1fr));
    gap: var(--sp-2);
    padding-top: var(--sp-2);
    border-top: 1px solid var(--border);
  }
  .stats div {
    display: grid;
    gap: 2px;
  }
  .v {
    font-size: var(--fs-sm);
    color: var(--text-strong);
  }
  .v[data-tone='ok'] { color: var(--ok); }
  .v[data-tone='fault'] { color: var(--fault); }
  .note,
  .warn,
  .error {
    margin: 0;
    font-size: var(--fs-sm);
  }
  .note {
    color: var(--text-soft);
  }
  .note code {
    font-family: var(--font-mono);
    color: var(--text);
  }
  .warn {
    color: var(--warn);
  }
  .error {
    color: var(--fault);
  }
</style>
