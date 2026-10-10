<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { SignalDescriptor } from '../../lib/data/types';
  import { deviceHealth, type DeviceSection } from '../../lib/mechanisms/model';
  import ValueRow from '../../ui/ValueRow.svelte';

  type Props = {
    devices: DeviceSection[];
    session: Session;
    live: { readonly current: number };
  };

  let { devices, session, live }: Props = $props();
  let expanded = $state<Record<string, boolean>>({});

  /** Live readings first; Info/Config/Setup/Health details stay behind a toggle. */
  const isLive = (signal: SignalDescriptor) => /\/State\//.test(signal.localPath) || ['current', 'temperature', 'derate'].includes(signal.name);

  function toneOf(signal: SignalDescriptor): 'warn' | 'fault' | null {
    void live.current;
    const sample = session.store.latestSample(signal.signalId);
    const value = typeof sample?.value === 'number' ? sample.value : undefined;
    const health = deviceHealth(signal, value);
    return health === 'warn' ? 'warn' : health === 'derate' ? 'fault' : null;
  }
</script>

<section class="ui-panel">
  <header class="ui-panel-head">
    <h3 class="ui-panel-title">Devices</h3>
    <span class="ui-spacer"></span>
    <span class="count ui-mono">{devices.length}</span>
  </header>
  {#if devices.length === 0}
    <p class="empty">No devices published.</p>
  {/if}
  {#each devices as device (device.path)}
    <div class="device">
      <div class="device-name">{device.name}</div>
      {#each device.signals.filter((sig) => expanded[device.path] || isLive(sig)) as signal (signal.signalId)}
        <ValueRow {signal} {session} {live} tone={toneOf(signal)} label={signal.localPath.split('/').slice(2).join('/') || signal.name} />
      {/each}
      {#if device.signals.some((sig) => !isLive(sig))}
        <button type="button" class="more" onclick={() => (expanded[device.path] = !expanded[device.path])}>
          {expanded[device.path] ? 'Hide details' : `Show ${device.signals.filter((sig) => !isLive(sig)).length} more`}
        </button>
      {/if}
    </div>
  {/each}
</section>

<style>
  .device + .device {
    border-top: 1px solid var(--border-subtle);
  }
  .device-name {
    padding: var(--sp-2) var(--sp-3) 4px;
    font-size: var(--fs-sm);
    font-weight: 600;
    color: var(--text-strong);
  }
  .more {
    width: 100%;
    padding: 6px var(--sp-3);
    border: none;
    border-top: 1px solid var(--border);
    background: transparent;
    color: var(--text-soft);
    font: inherit;
    font-size: var(--fs-xs);
    text-align: left;
    cursor: pointer;
  }
  .more:hover {
    color: var(--text);
    background: var(--surface-2);
  }
  .count {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .empty {
    margin: 0;
    padding: var(--sp-3);
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
</style>
