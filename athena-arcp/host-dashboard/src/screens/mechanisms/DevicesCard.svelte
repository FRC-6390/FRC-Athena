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
      {#each device.signals as signal (signal.signalId)}
        <ValueRow {signal} {session} {live} tone={toneOf(signal)} />
      {/each}
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
