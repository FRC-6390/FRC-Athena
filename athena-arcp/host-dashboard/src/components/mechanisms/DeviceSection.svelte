<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { SignalDescriptor } from '../../lib/data/types';
  import { deviceHealth, type DeviceSection } from '../../lib/mechanisms/model';
  import SignalField from './SignalField.svelte';

  type Props = {
    devices: DeviceSection[];
    session: Session;
    live: { readonly current: number };
  };

  let { devices, session, live }: Props = $props();

  function healthOf(signal: SignalDescriptor) {
    void live.current;
    const sample = session.store.latestSample(signal.signalId);
    const value = typeof sample?.value === 'number' ? sample.value : undefined;
    return deviceHealth(signal, value);
  }
</script>

<section class="card" aria-labelledby="devices-heading">
  <h3 id="devices-heading">Devices</h3>
  {#if devices.length === 0}
    <p class="empty">No devices published.</p>
  {/if}
  {#each devices as device (device.path)}
    <div class="device">
      <h4>{device.name}</h4>
      <div class="grid">
        {#each device.signals as signal (signal.signalId)}
          <div class="health-{healthOf(signal)}">
            <SignalField {signal} {session} {live} />
          </div>
        {/each}
      </div>
    </div>
  {/each}
</section>

<style>
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 12px;
    display: grid;
    gap: 10px;
  }
  h3 {
    margin: 0;
    font-family: var(--font-display);
    font-size: 13px;
    color: var(--text-strong);
  }
  h4 {
    margin: 0 0 6px;
    font-family: var(--font-mono);
    font-size: 12px;
    color: var(--text-soft);
  }
  .device {
    display: grid;
    gap: 4px;
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(170px, 1fr));
    gap: 8px;
  }
  .health-warn :global(.field) {
    border-color: #b45309;
  }
  .health-derate :global(.field) {
    border-color: #7c3aed;
  }
  .empty {
    margin: 0;
    color: var(--text-soft);
    font-size: 12px;
  }
</style>
