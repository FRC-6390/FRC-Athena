<script lang="ts">
  import type { Session } from '../lib/data/session';
  import { MARKER_KIND, type SignalDescriptor } from '../lib/data/types';
  import { buildMechanismTree, flattenMechanisms } from '../lib/data/tree';
  import { deviceSections, summarize, TEMPERATURE_WARN_C } from '../lib/mechanisms/model';
  import { formatSeconds } from '../lib/format';
  import type { DriverState } from '../lib/driver/driver-state';
  import { reasonTone } from './mechanisms/tones';

  type Props = {
    session: Session;
    live: { readonly current: number };
    driver: DriverState;
    onmechanism: (path: string) => void;
    files?: import('svelte').Snippet;
  };

  let { session, live, driver, onmechanism, files }: Props = $props();

  const num = (signal: SignalDescriptor | undefined) => {
    if (!signal) return null;
    const v = session.store.latestSample(signal.signalId)?.value;
    return typeof v === 'number' ? v : typeof v === 'bigint' ? Number(v) : null;
  };

  /** One row per motor-like device: current, temperature and derate are read from its Devices signals. */
  const motors = $derived.by(() => {
    void live.current;
    const nodes = flattenMechanisms(buildMechanismTree(session.store.allSignals()));
    return nodes.flatMap((node) =>
      deviceSections(node)
        .map((device) => {
          const by = (name: string) => device.signals.find((s) => s.name === name);
          return {
            key: device.path,
            mechanism: node.name,
            mechanismPath: node.path,
            device: device.name,
            current: num(by('current')),
            temperature: num(by('temperature')),
            derate: num(by('derate'))
          };
        })
        .filter((m) => m.current !== null || m.temperature !== null)
    );
  });

  const mechanisms = $derived.by(() => {
    void live.current;
    const roots = buildMechanismTree(session.store.allSignals());
    const robot = roots.find((r) => r.name === 'robot');
    return flattenMechanisms(robot ? robot.children : roots).map((node) => {
      const s = summarize(node);
      const text = (sig: SignalDescriptor | null) => (sig ? String(session.store.latestSample(sig.signalId)?.value ?? '') : '');
      return { node, state: text(s.state), reason: text(s.reason), stage: text(s.stage) };
    });
  });

  const events = $derived.by(() => {
    void live.current;
    const kinds = new Set<number>([MARKER_KIND.FAULT, MARKER_KIND.STAGE_CHANGE, MARKER_KIND.CHECK, MARKER_KIND.OVERRIDE]);
    return session.store
      .markers()
      .filter((m) => kinds.has(m.kind))
      .slice(-30)
      .reverse();
  });

  const hottest = $derived(
    motors.reduce<number | null>((max, m) => (m.temperature !== null && (max === null || m.temperature > max) ? m.temperature : max), null)
  );
  const derating = $derived(motors.filter((m) => m.derate !== null && m.derate < 0.999).length);
  const faults = $derived(driver.alerts.filter((a) => a.level === 'fault').length);

  const tempTone = (t: number | null) => (t === null ? 'stale' : t >= TEMPERATURE_WARN_C + 15 ? 'fault' : t >= TEMPERATURE_WARN_C ? 'warn' : 'ok');
  const tempPct = (t: number | null) => (t === null ? 0 : Math.max(0, Math.min(1, (t - 20) / 80)) * 100);
  const markerTone = (kind: number) => (kind === MARKER_KIND.FAULT ? 'fault' : kind === MARKER_KIND.CHECK ? 'info' : 'warn');
  const markerLabel = (kind: number) =>
    kind === MARKER_KIND.FAULT ? 'Fault' : kind === MARKER_KIND.STAGE_CHANGE ? 'Stage' : kind === MARKER_KIND.CHECK ? 'Check' : 'Override';
</script>

<div class="screen">
  <div class="stats">
    <div class="ui-panel stat">
      <span class="ui-label">Battery</span>
      <span class="value ui-mono" data-tone={driver.battery.volts === null ? null : driver.battery.volts < 11 ? 'fault' : driver.battery.volts < 12 ? 'warn' : null}>
        {driver.battery.volts === null ? '--.-' : driver.battery.volts.toFixed(2)}<small>V</small>
      </span>
    </div>
    <div class="ui-panel stat">
      <span class="ui-label">Total current</span>
      <span class="value ui-mono">{driver.power.totalAmps.toFixed(1)}<small>A</small></span>
    </div>
    <div class="ui-panel stat">
      <span class="ui-label">Hottest motor</span>
      <span class="value ui-mono" data-tone={hottest === null ? null : tempTone(hottest) === 'ok' ? null : tempTone(hottest)}>
        {hottest === null ? '--' : hottest.toFixed(0)}<small>°C</small>
      </span>
    </div>
    <div class="ui-panel stat">
      <span class="ui-label">Derating</span>
      <span class="value ui-mono" data-tone={derating > 0 ? 'warn' : null}>{derating}<small>motors</small></span>
    </div>
    <div class="ui-panel stat">
      <span class="ui-label">Faults</span>
      <span class="value ui-mono" data-tone={faults > 0 ? 'fault' : null}>{faults}</span>
    </div>
  </div>

  <div class="columns">
    <section class="ui-panel">
      <header class="ui-panel-head"><h3 class="ui-panel-title">Motors</h3></header>
      <div class="table motors">
        <div class="thead">
          <span>Device</span><span class="num">Current</span><span>Temperature</span><span class="num">Derate</span>
        </div>
        {#each motors as m (m.key)}
          <button type="button" class="tr" onclick={() => onmechanism(m.mechanismPath)}>
            <span class="dev"><b>{m.mechanism}</b> / {m.device}</span>
            <span class="num ui-mono">{m.current === null ? '—' : `${m.current.toFixed(1)} A`}</span>
            <span class="temp">
              <span class="bar"><i style="width:{tempPct(m.temperature)}%" data-tone={tempTone(m.temperature)}></i></span>
              <span class="ui-mono">{m.temperature === null ? '—' : `${m.temperature.toFixed(0)}°`}</span>
            </span>
            <span class="num ui-mono" class:derated={m.derate !== null && m.derate < 0.999}>
              {m.derate === null ? '—' : `${Math.round(m.derate * 100)}%`}
            </span>
          </button>
        {:else}
          <p class="empty">No motor telemetry yet.</p>
        {/each}
      </div>
    </section>

    <section class="ui-panel">
      <header class="ui-panel-head"><h3 class="ui-panel-title">Mechanisms</h3></header>
      <div class="table">
        {#each mechanisms as m (m.node.path)}
          <button type="button" class="tr mech" onclick={() => onmechanism(m.node.path)}>
            <span class="ui-dot" data-tone={m.reason ? reasonTone(m.reason) : 'stale'}></span>
            <span class="mname">{m.node.name}</span>
            <span class="mreason">{m.reason || m.state || '—'}</span>
            {#if m.stage && m.stage !== 'NORMAL'}<span class="ui-badge" data-tone="warn">{m.stage}</span>{/if}
          </button>
        {/each}
      </div>
    </section>

    <section class="ui-panel">
      <header class="ui-panel-head"><h3 class="ui-panel-title">Recent events</h3></header>
      {#if events.length === 0}
        <p class="empty">No faults, stage changes or checks recorded.</p>
      {:else}
        <ul class="events">
          {#each events as e, i (i)}
            <li>
              <span class="ui-badge" data-tone={markerTone(e.kind)}>{markerLabel(e.kind)}</span>
              <span class="msg">{e.message}</span>
              <span class="time ui-mono">{formatSeconds(e.timestampMicros)}</span>
            </li>
          {/each}
        </ul>
      {/if}
    </section>

    {#if files}
      <section class="ui-panel">
        <header class="ui-panel-head"><h3 class="ui-panel-title">Robot files</h3></header>
        <div class="ui-panel-body">{@render files()}</div>
      </section>
    {/if}
  </div>
</div>

<style>
  .screen {
    display: grid;
    gap: var(--sp-3);
  }
  .stats {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
    gap: var(--sp-3);
  }
  .stat {
    display: grid;
    gap: 6px;
    padding: var(--sp-3);
  }
  .value {
    font-size: var(--fs-2xl);
    font-weight: 600;
    color: var(--text-strong);
    line-height: 1;
  }
  .value small {
    margin-left: 4px;
    font-size: var(--fs-sm);
    font-weight: 400;
    color: var(--text-faint);
  }
  .value[data-tone='warn'] { color: var(--warn); }
  .value[data-tone='fault'] { color: var(--fault); }
  .columns {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(420px, 1fr));
    gap: var(--sp-3);
    align-items: start;
  }
  .thead,
  .tr {
    display: grid;
    grid-template-columns: minmax(0, 1.6fr) 80px minmax(0, 1.4fr) 64px;
    align-items: center;
    gap: var(--sp-3);
    padding: 0 var(--sp-3);
  }
  .thead {
    min-height: 30px;
    font-size: var(--fs-xs);
    letter-spacing: 0.04em;
    text-transform: uppercase;
    color: var(--text-faint);
  }
  .tr {
    width: 100%;
    min-height: 36px;
    border: none;
    border-top: 1px solid var(--border);
    background: transparent;
    color: var(--text);
    font: inherit;
    font-size: var(--fs-sm);
    text-align: left;
    cursor: pointer;
  }
  .tr:hover {
    background: var(--surface-2);
  }
  .tr.mech {
    grid-template-columns: 8px 110px minmax(0, 1fr) auto;
  }
  .dev b {
    color: var(--text-strong);
    font-weight: 600;
    text-transform: capitalize;
  }
  .num {
    text-align: right;
  }
  .derated {
    color: var(--warn);
  }
  .temp {
    display: grid;
    grid-template-columns: minmax(0, 1fr) 36px;
    align-items: center;
    gap: var(--sp-2);
    text-align: right;
  }
  .bar {
    height: 6px;
    border-radius: 3px;
    background: var(--surface-3);
    overflow: hidden;
  }
  .bar i {
    display: block;
    height: 100%;
    background: var(--ok);
  }
  .bar i[data-tone='warn'] { background: var(--warn); }
  .bar i[data-tone='fault'] { background: var(--fault); }
  .mname {
    font-weight: 600;
    color: var(--text-strong);
    text-transform: capitalize;
  }
  .mreason {
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .events {
    list-style: none;
    margin: 0;
    padding: 0;
  }
  .events li {
    display: grid;
    grid-template-columns: 72px minmax(0, 1fr) auto;
    align-items: center;
    gap: var(--sp-2);
    min-height: 34px;
    padding: 0 var(--sp-3);
    border-top: 1px solid var(--border);
    font-size: var(--fs-sm);
  }
  .events li:first-child {
    border-top: none;
  }
  .msg {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .time {
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
