<script lang="ts">
  import type { Session } from '../lib/data/session';
  import { buildMechanismTree, flattenMechanisms } from '../lib/data/tree';
  import { summarize } from '../lib/mechanisms/model';
  import { formatClock, type DriverState } from '../lib/driver/driver-state';
  import { reasonTone } from './mechanisms/tones';

  type Props = {
    session: Session;
    live: { readonly current: number };
    driver: DriverState;
    onmechanism: (path: string) => void;
  };

  let { session, live, driver, onmechanism }: Props = $props();

  const mechanisms = $derived.by(() => {
    void live.current;
    const roots = buildMechanismTree(session.store.allSignals());
    const robot = roots.find((r) => r.name === 'robot');
    return flattenMechanisms(robot ? robot.children : roots).map((node) => {
      const summary = summarize(node);
      const reason = summary.reason ? String(session.store.latestSample(summary.reason.signalId)?.value ?? '') : '';
      return { node, reason };
    });
  });

  const phaseLabel = $derived(
    { idle: 'Pre-match', auto: 'Autonomous', teleop: 'Teleop', ended: 'Match over' }[driver.match.phase]
  );
  const clock = $derived(
    driver.match.remainingSeconds !== null ? driver.match.remainingSeconds : driver.match.elapsedSeconds
  );
  const voltsPct = $derived(
    driver.battery.volts === null ? 0 : Math.max(0, Math.min(1, (driver.battery.volts - 10) / 3)) * 100
  );
  const batteryTone = $derived(
    driver.battery.volts === null ? 'stale' : driver.battery.volts < 11 ? 'fault' : driver.battery.volts < 12 ? 'warn' : 'ok'
  );

  /** The biggest draws (at least 0.5 A), largest first; the rest are counted, not listed. */
  const ranked = $derived(
    driver.power.consumers
      .map((c, index) => ({ ...c, index }))
      .sort((a, b) => Math.abs(b.amps) - Math.abs(a.amps))
  );
  const topConsumers = $derived(ranked.filter((c) => Math.abs(c.amps) >= 0.5).slice(0, 6));
  const idleCount = $derived(ranked.length - topConsumers.length);
</script>

<div class="overview">
  <div class="grid">
    <section class="ui-panel tile match">
      <header class="ui-panel-head"><h3 class="ui-panel-title">Match</h3></header>
      <div class="ui-panel-body match-body">
        <div class="clock">{formatClock(clock)}</div>
        <div class="phase">{phaseLabel}</div>
        <div class="mode ui-badge">{driver.mode ?? 'No mode'}</div>
      </div>
    </section>

    <section class="ui-panel tile">
      <header class="ui-panel-head"><h3 class="ui-panel-title">Battery</h3></header>
      <div class="ui-panel-body battery">
        <div class="big ui-mono" data-tone={batteryTone}>
          {driver.battery.volts === null ? '--.-' : driver.battery.volts.toFixed(2)}<span>V</span>
        </div>
        <div class="meter"><i style="width:{voltsPct}%" data-tone={batteryTone}></i></div>
        <div class="ui-label split-head">Current · {driver.power.totalAmps.toFixed(1)} A</div>
        <div class="split">
          {#each driver.power.consumers as c, i (c.name)}
            <i style="flex:{Math.max(c.share, 0.02)}" class="seg s{i % 5}" title="{c.name} {c.amps.toFixed(1)} A"></i>
          {/each}
        </div>
        <ul class="consumers">
          {#each topConsumers as c (c.name)}
            <li><i class="seg s{c.index % 5}"></i><span>{c.name}</span><b class="ui-mono">{c.amps.toFixed(1)} A</b></li>
          {:else}
            <li class="idle">All motors idle</li>
          {/each}
          {#if idleCount > 0 && topConsumers.length > 0}<li class="idle">{idleCount} more under 0.5 A</li>{/if}
        </ul>
      </div>
    </section>

    <section class="ui-panel tile alerts">
      <header class="ui-panel-head">
        <h3 class="ui-panel-title">Alerts</h3>
        <span class="ui-spacer"></span>
        {#if driver.alerts.length === 0}<span class="ui-badge" data-tone="ok">All clear</span>{/if}
      </header>
      {#if driver.alerts.length === 0}
        <p class="quiet">No faults or active stages.</p>
      {:else}
        <ul class="alert-list">
          {#each driver.alerts as alert (alert.key)}
            <li><span class="ui-dot" data-tone={alert.level}></span>{alert.text}</li>
          {/each}
        </ul>
      {/if}
    </section>

    <section class="ui-panel tile mechs">
      <header class="ui-panel-head"><h3 class="ui-panel-title">Mechanisms</h3></header>
      <div class="mech-grid">
        {#each mechanisms as m (m.node.path)}
          <button type="button" class="mech" onclick={() => onmechanism(m.node.path)}>
            <span class="ui-dot" data-tone={m.reason ? reasonTone(m.reason) : 'stale'}></span>
            <span class="mname">{m.node.name}</span>
            <span class="mreason">{m.reason || '—'}</span>
          </button>
        {/each}
      </div>
    </section>
  </div>
</div>

<style>
  .overview {
    height: 100%;
    min-height: 0;
    overflow: auto;
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(12, minmax(0, 1fr));
    grid-auto-rows: min-content;
    gap: var(--sp-3);
    overflow: auto;
    align-content: start;
  }
  .tile {
    grid-column: span 4;
  }
  .mechs {
    grid-column: span 12;
  }
  .match-body {
    display: grid;
    justify-items: start;
    gap: 6px;
  }
  .clock {
    font-variant-numeric: tabular-nums;
    font-size: 44px;
    font-weight: 600;
    line-height: 1;
    color: var(--text-strong);
    letter-spacing: -0.02em;
  }
  .phase {
    color: var(--text-soft);
    font-size: var(--fs-sm);
  }
  .battery {
    display: grid;
    gap: var(--sp-2);
  }
  .big {
    font-size: 32px;
    font-weight: 600;
    color: var(--text-strong);
    line-height: 1;
  }
  .big span {
    margin-left: 4px;
    font-size: var(--fs-md);
    color: var(--text-faint);
  }
  .big[data-tone='warn'] { color: var(--warn); }
  .big[data-tone='fault'] { color: var(--fault); }
  .meter {
    height: 6px;
    border-radius: 3px;
    background: var(--surface-3);
    overflow: hidden;
  }
  .meter i {
    display: block;
    height: 100%;
    background: var(--ok);
  }
  .meter i[data-tone='warn'] { background: var(--warn); }
  .meter i[data-tone='fault'] { background: var(--fault); }
  .split-head {
    margin-top: var(--sp-2);
  }
  .split {
    display: flex;
    gap: 2px;
    height: 8px;
    border-radius: 3px;
    overflow: hidden;
  }
  .seg {
    display: block;
  }
  .s0 { background: var(--series-reference); }
  .s1 { background: var(--series-measured); }
  .s2 { background: var(--series-target); }
  .s3 { background: var(--series-4); }
  .s4 { background: var(--series-5); }
  .consumers {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    gap: 4px;
    font-size: var(--fs-sm);
  }
  .consumers li {
    display: grid;
    grid-template-columns: 8px minmax(0, 1fr) auto;
    align-items: center;
    gap: var(--sp-2);
    color: var(--text-soft);
  }
  .consumers li i {
    width: 8px;
    height: 8px;
    border-radius: 2px;
  }
  .consumers b {
    color: var(--text-strong);
    font-weight: 500;
  }
  .quiet {
    margin: 0;
    padding: var(--sp-3);
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
  .alert-list {
    list-style: none;
    margin: 0;
    padding: var(--sp-2) var(--sp-3);
    display: grid;
    gap: var(--sp-2);
    font-size: var(--fs-sm);
  }
  .alert-list li {
    display: flex;
    align-items: center;
    gap: var(--sp-2);
  }
  .mech-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(220px, 1fr));
  }
  .mech {
    display: grid;
    grid-template-columns: 8px minmax(0, 1fr);
    grid-template-rows: auto auto;
    column-gap: var(--sp-2);
    align-items: center;
    padding: var(--sp-3);
    border: none;
    border-right: 1px solid var(--border);
    border-bottom: 1px solid var(--border);
    background: transparent;
    color: inherit;
    font: inherit;
    text-align: left;
    cursor: pointer;
  }
  .mech:hover {
    background: var(--surface-2);
  }
  .mname {
    font-weight: 600;
    color: var(--text-strong);
    text-transform: capitalize;
  }
  .mreason {
    grid-column: 2;
    font-size: var(--fs-xs);
    color: var(--text-faint);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  @media (max-width: 1100px) {
    .tile {
      grid-column: span 6;
    }
  }
  @media (max-width: 720px) {
    .tile {
      grid-column: span 12;
    }
  }
  .consumers .idle {
    color: var(--text-faint);
    font-size: var(--fs-xs);
  }
</style>
