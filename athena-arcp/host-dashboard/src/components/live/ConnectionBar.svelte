<script lang="ts">
  import { onDestroy, onMount } from 'svelte';
  import { teamHost, type LiveSource, type LiveStatus } from '../../lib/data/live-source';

  type Props = { live: LiveSource };
  let { live }: Props = $props();

  const TEAM_KEY = 'athena.dashboard.team';
  const MODE_KEY = 'athena.dashboard.mode';

  /** Per-viewer convenience only: remembered team and mode. Failures are ignored. */
  function remembered(key: string, fallback: string): string {
    try {
      return localStorage.getItem(key) ?? fallback;
    } catch {
      return fallback;
    }
  }
  function remember(key: string, value: string): void {
    try {
      localStorage.setItem(key, value);
    } catch {
      /* private window or blocked storage */
    }
  }

  let mode = $state<'robot' | 'sim'>(remembered(MODE_KEY, 'robot') === 'sim' ? 'sim' : 'robot');
  let team = $state(remembered(TEAM_KEY, ''));
  let replay = $state(false);
  let busy = $state(false);
  let error = $state<string | null>(null);
  let status = $state<LiveStatus | null>(null);
  let offStatus: (() => void) | null = null;
  let offEvent: (() => void) | null = null;

  const host = $derived(mode === 'sim' ? 'localhost' : teamHost(Number(team)));
  const connected = $derived(status?.connected === true);

  onMount(() => {
    offStatus = live.onStatus((next) => (status = next));
    offEvent = live.onEvent((event) => {
      if (event.kind === 'disconnected') error = `Disconnected: ${event.reason}`;
    });
  });
  onDestroy(() => {
    offStatus?.();
    offEvent?.();
  });

  const ms = (micros: number | null | undefined) =>
    micros === null || micros === undefined ? '-' : `${(micros / 1000).toFixed(1)} ms`;

  async function connect() {
    if (!host) {
      error = 'Enter a team number between 1 and 9999, or use the simulator';
      return;
    }
    busy = true;
    error = null;
    remember(TEAM_KEY, team);
    remember(MODE_KEY, mode);
    try {
      await live.connect({ host, replay });
      // Time sync gives the RTT and clock offset shown in the status line.
      await live.timeSync(8).catch(() => undefined);
      // Recording keeps the live stream as a .athrp in the app data folder until disconnect.
      if (replay) await live.recordStart().catch((e) => (error = `Not recording: ${String(e)}`));
    } catch (e) {
      error = `Could not connect to ${host}: ${e instanceof Error ? e.message : String(e)}`;
    } finally {
      busy = false;
    }
  }

  async function disconnect() {
    busy = true;
    try {
      await live.disconnect();
    } finally {
      busy = false;
    }
  }
</script>

<div class="bar">
  <div class="inputs">
    <label>
      <span>Robot</span>
      <select bind:value={mode} disabled={connected}>
        <option value="robot">Team number</option>
        <option value="sim">Simulator (localhost)</option>
      </select>
    </label>
    {#if mode === 'robot'}
      <label>
        <span>Team</span>
        <input type="number" min="1" max="9999" bind:value={team} disabled={connected} placeholder="6390" />
      </label>
    {/if}
    <span class="host" title="Address used for the control connection">{host ?? 'no address'}</span>
    <label class="check"><input type="checkbox" bind:checked={replay} disabled={connected} /> record .athrp</label>
    {#if connected}
      <button type="button" onclick={disconnect} disabled={busy}>Disconnect</button>
    {:else}
      <button type="button" onclick={connect} disabled={busy || !host}>Connect</button>
    {/if}
  </div>

  {#if status}
    <div class="stats" class:down={!connected}>
      <span class="dot" class:up={connected}>{connected ? 'connected' : 'disconnected'}</span>
      <span title="Round trip, from the last time sync">RTT {ms(status.rttMicros)}</span>
      <span title="Robot clock minus dashboard clock">offset {ms(status.offsetMicros)}</span>
      <span title="Frame cycles lost to sequence gaps">drops {status.lostCycles}</span>
      <span title="Gaps seen">gaps {status.gaps}</span>
      <span title="Automatic backfills completed">backfills {status.backfills}</span>
      {#if status.server}
        <span title="Clients connected to the robot">clients {status.server.clients}</span>
        {#if status.server.throttled}
          <span class="warn" title="Bandwidth budget has lowered these subscriptions">
            throttled: {status.server.throttledPatterns.join(', ')}
          </span>
        {/if}
        {#if status.server.droppedFrames > 0}
          <span class="warn">robot dropped {status.server.droppedFrames} datagrams</span>
        {/if}
      {/if}
    </div>
  {/if}

  {#if error}<p class="error">{error}</p>{/if}
</div>

<style>
  .bar {
    display: grid;
    gap: 6px;
    font-size: 12px;
    color: var(--text-soft);
  }
  .inputs,
  .stats {
    display: flex;
    gap: 10px;
    align-items: center;
    flex-wrap: wrap;
  }
  label {
    display: inline-flex;
    gap: 6px;
    align-items: center;
  }
  input[type='number'] {
    width: 84px;
  }
  .host {
    font-family: var(--font-mono, monospace);
  }
  .dot::before {
    content: '';
    display: inline-block;
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #f87171;
    margin-right: 4px;
  }
  .dot.up::before {
    background: #34d399;
  }
  .down {
    opacity: 0.8;
  }
  .warn {
    color: #fbbf24;
  }
  .error {
    margin: 0;
    color: #f87171;
  }
</style>
