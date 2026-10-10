<script lang="ts">
  import type { Snippet } from 'svelte';
  import { formatClock, type DriverState } from '../lib/driver/driver-state';

  type Props = {
    title: string;
    subtitle?: string;
    /** Connection summary for the target button. */
    connection: { label: string; detail: string; tone: 'ok' | 'warn' | 'fault' | 'info' | 'stale' };
    driver: DriverState;
    onconnection?: () => void;
    actions?: Snippet;
  };

  let { title, subtitle, connection, driver, onconnection, actions }: Props = $props();

  const modeTone = $derived.by(() => {
    const mode = (driver.mode ?? '').toUpperCase();
    if (mode.includes('AUTO')) return 'info';
    if (mode.includes('TELEOP') || mode.includes('ENABLED')) return 'ok';
    if (mode.includes('TEST')) return 'warn';
    return 'stale';
  });
  const batteryTone = $derived(
    driver.battery.volts === null ? 'stale' : driver.battery.volts < 11 ? 'fault' : driver.battery.volts < 12 ? 'warn' : 'ok'
  );
  const clock = $derived(
    driver.match.remainingSeconds !== null
      ? formatClock(driver.match.remainingSeconds)
      : driver.match.phase === 'idle'
        ? null
        : formatClock(driver.match.elapsedSeconds)
  );
  const faults = $derived(driver.alerts.filter((a) => a.level === 'fault').length);
  const warnings = $derived(driver.alerts.length - faults);
</script>

<header class="header">
  <div class="title">
    <h1>{title}</h1>
    {#if subtitle}<span class="sub">{subtitle}</span>{/if}
  </div>

  <div class="ui-spacer"></div>

  <div class="status">
    {#if faults + warnings > 0}
      <span class="ui-badge" data-tone={faults > 0 ? 'fault' : 'warn'} title={driver.alerts.map((a) => a.text).join('\n')}>
        {faults > 0 ? `${faults} fault${faults === 1 ? '' : 's'}` : `${warnings} warning${warnings === 1 ? '' : 's'}`}
      </span>
    {/if}
    <span class="ui-badge" data-tone={modeTone}>{driver.mode ?? 'NO MODE'}</span>
    {#if clock}<span class="clock">{clock}</span>{/if}
    <span class="metric" title="Battery voltage">
      <span class="ui-dot" data-tone={batteryTone}></span>
      <span class="ui-mono">{driver.battery.volts === null ? '--.-' : driver.battery.volts.toFixed(1)}</span>
      <span class="unit">V</span>
    </span>
    {#if actions}{@render actions()}{/if}
    <button type="button" class="connection" onclick={onconnection} title="Robot connection">
      <span class="ui-dot" data-tone={connection.tone}></span>
      <span class="conn-label">{connection.label}</span>
      <span class="conn-detail ui-mono">{connection.detail}</span>
    </button>
  </div>
</header>

<style>
  .header {
    display: flex;
    align-items: center;
    gap: var(--sp-3);
    height: var(--topbar-height);
    padding: 0 var(--sp-4);
    border-bottom: 1px solid var(--border);
    background: var(--bg-1);
    min-width: 0;
  }
  .title {
    display: flex;
    align-items: baseline;
    gap: var(--sp-2);
    min-width: 0;
  }
  h1 {
    margin: 0;
    font-size: var(--fs-lg);
    font-weight: 600;
    color: var(--text-strong);
    white-space: nowrap;
  }
  .sub {
    font-family: var(--font-mono);
    font-size: var(--fs-xs);
    color: var(--text-faint);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .status {
    display: flex;
    align-items: center;
    gap: var(--sp-3);
    flex: none;
  }
  .clock {
    font-variant-numeric: tabular-nums;
    font-size: var(--fs-lg);
    font-weight: 600;
    color: var(--text-strong);
  }
  .metric {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    color: var(--text-strong);
    font-size: var(--fs-md);
  }
  .unit {
    color: var(--text-faint);
    font-size: var(--fs-xs);
  }
  .connection {
    display: inline-flex;
    align-items: center;
    gap: var(--sp-2);
    height: 30px;
    padding: 0 var(--sp-3);
    border: 1px solid var(--border-subtle);
    border-radius: var(--radius);
    background: var(--surface-2);
    color: var(--text);
    font: inherit;
    cursor: pointer;
  }
  .connection:hover {
    border-color: var(--border-emphasis);
    background: var(--surface-3);
  }
  .conn-label {
    font-size: var(--fs-sm);
    font-weight: 600;
    color: var(--text-strong);
  }
  .conn-detail {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  @media (max-width: 900px) {
    .sub,
    .conn-detail {
      display: none;
    }
  }
</style>
