<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { deviceSiblings } from '../../../lib/board/widgets/robot-siblings';
  import { numericOf } from '../../../lib/board/widgets/robot-series';
  import { formatNumber } from '../../../lib/format';
  import type { SignalDescriptor } from '../../../lib/data/types';

  let { ctx }: { ctx: WidgetContext } = $props();

  const anchor = $derived(ctx.bindings.signal?.[0]);
  const warnAt = $derived(Number(ctx.setting<number>('tempWarn')));
  const faultAt = $derived(Number(ctx.setting<number>('tempFault')));

  const device = $derived.by(() => {
    void ctx.live.current;
    if (!anchor) return null;
    return deviceSiblings(ctx.session.store.allSignals(), anchor.path);
  });

  const reading = (s: SignalDescriptor | null | undefined) => {
    void ctx.live.current;
    return s ? numericOf(ctx.latest(s)) : null;
  };
  const connected = $derived.by(() => {
    void ctx.live.current;
    const s = device?.connected;
    if (!s) return null;
    const v = ctx.latest(s);
    return typeof v === 'boolean' ? v : null;
  });
  const current = $derived(reading(device?.current));
  const voltage = $derived(reading(device?.voltage));
  const temperature = $derived(reading(device?.temperature));
  const derate = $derived(reading(device?.derate));
  const derateStatus = $derived(derate === null ? null : derate < 0.999 ? 'derating' : 'full');

  // Temperature bar scale runs from 0 to 25% above the fault line.
  const scaleMax = $derived(Math.max(faultAt, warnAt, 1) * 1.25);
  const tempTone = $derived(
    temperature === null ? null : temperature >= faultAt ? 'fault' : temperature >= warnAt ? 'warn' : 'ok'
  );
  const tempPct = $derived(temperature === null ? 0 : Math.min(100, Math.max(0, (temperature / scaleMax) * 100)));
  const derateValue = $derived(derate === null ? null : Math.round(Math.min(1, Math.max(0, derate)) * 100));
</script>

{#if !anchor}
  <div class="problem">Bind a device signal</div>
{:else if !device}
  <div class="problem">{anchor.path} is not under a Devices/ group</div>
{:else}
  <div class="card">
    <header>
      <span class="name" title={device.prefix}>{device.name}</span>
      {#if connected === null}
        <span class="ui-badge" data-tone="stale">no link</span>
      {:else}
        <span class="ui-badge" data-tone={connected ? 'ok' : 'fault'}>{connected ? 'connected' : 'disconnected'}</span>
      {/if}
    </header>

    <div class="grid">
      <div class="stat">
        <span class="label">Current</span>
        <span class="value ui-mono">{current === null ? '--' : formatNumber(current)}<small>{device.current?.unit ?? 'A'}</small></span>
      </div>
      <div class="stat">
        <span class="label">Voltage</span>
        <span class="value ui-mono">{voltage === null ? '--' : formatNumber(voltage)}<small>{device.voltage?.unit ?? 'V'}</small></span>
      </div>
    </div>

    <div class="bar-row">
      <span class="label">Temp</span>
      <div class="bar" role="meter" aria-valuemin="0" aria-valuemax={scaleMax} aria-valuenow={temperature ?? 0} data-tone={tempTone}>
        <i style="width:{tempPct}%"></i>
        <span class="mark warn" style="left:{(warnAt / scaleMax) * 100}%"></span>
        <span class="mark fault" style="left:{(faultAt / scaleMax) * 100}%"></span>
      </div>
      <span class="value ui-mono">{temperature === null ? '--' : formatNumber(temperature)}<small>°C</small></span>
    </div>

    <div class="bar-row">
      <span class="label">Derate</span>
      <div class="bar" data-tone={derateStatus === 'derating' ? 'warn' : 'ok'}>
        <i style="width:{derateValue ?? 0}%"></i>
      </div>
      <span class="value ui-mono">{derateValue === null ? '--' : `${derateValue}`}<small>%</small></span>
    </div>
  </div>
{/if}

<style>
  .card {
    display: flex;
    flex-direction: column;
    gap: 10px;
    height: 100%;
    padding: 10px 12px;
    box-sizing: border-box;
    overflow: hidden;
  }
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    min-width: 0;
  }
  .name {
    font-size: var(--fs-md);
    font-weight: 600;
    color: var(--text-strong);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }
  .stat,
  .bar-row {
    display: flex;
    flex-direction: column;
    min-width: 0;
  }
  .bar-row {
    flex-direction: row;
    align-items: center;
    gap: 8px;
  }
  .label {
    font-size: var(--fs-xs);
    letter-spacing: 0.05em;
    text-transform: uppercase;
    color: var(--text-faint);
    min-width: 46px;
  }
  .stat .label {
    min-width: 0;
  }
  .value {
    font-size: var(--fs-lg);
    font-weight: 600;
    color: var(--text-strong);
    white-space: nowrap;
  }
  .bar-row .value {
    min-width: 62px;
    text-align: right;
  }
  .value small {
    margin-left: 3px;
    font-size: var(--fs-xs);
    font-weight: 500;
    color: var(--text-faint);
  }
  .bar {
    position: relative;
    flex: 1;
    height: 8px;
    border-radius: 4px;
    background: var(--surface-3);
    overflow: hidden;
  }
  .bar i {
    position: absolute;
    inset: 0 auto 0 0;
    background: var(--ok);
    border-radius: 4px;
  }
  .bar[data-tone='warn'] i {
    background: var(--warn);
  }
  .bar[data-tone='fault'] i {
    background: var(--fault);
  }
  .mark {
    position: absolute;
    top: 0;
    bottom: 0;
    width: 1px;
    background: var(--border-emphasis);
  }
  .problem {
    display: grid;
    place-items: center;
    height: 100%;
    padding: 8px;
    text-align: center;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
