<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import { deriveDriverState, formatClock } from '../../lib/driver/driver-state';

  type Props = {
    session: Session;
    live: { readonly current: number };
  };

  let { session, live }: Props = $props();

  /**
   * Driver station view. Derived from the store on each coalesced refresh; it is only mounted while the
   * Driver workspace is active, so hidden workspaces cost nothing.
   */
  const state = $derived.by(() => {
    void live.current;
    return deriveDriverState(session.store);
  });

  const autoSelection = $derived.by(() => {
    void live.current;
    const signal = session.store.allSignals().find((s) => s.type === 'string' && /auto/i.test(s.path) && s.name === 'selected');
    if (!signal) return null;
    return { path: signal.path, value: String(session.store.latestSample(signal.signalId)?.value ?? '') };
  });

  const phaseLabel: Record<string, string> = {
    idle: 'Disabled',
    auto: 'Autonomous',
    teleop: 'Teleop',
    ended: 'Match over'
  };
</script>

<div class="driver">
  <section class="banner" class:clear={state.alerts.length === 0} aria-live="polite">
    {#if state.alerts.length === 0}
      <strong>All clear</strong>
    {:else}
      {#each state.alerts as alert (alert.key)}
        <span class="alert {alert.level}">{alert.text}</span>
      {/each}
    {/if}
  </section>

  <div class="grid">
    <section class="card match">
      <p class="label">Match</p>
      <p class="phase">{phaseLabel[state.match.phase]}</p>
      <p class="clock" data-testid="match-clock">
        {state.match.remainingSeconds !== null ? formatClock(state.match.remainingSeconds) : formatClock(state.match.elapsedSeconds)}
      </p>
      <p class="sub">
        {state.match.remainingSeconds !== null ? 'remaining' : state.match.elapsedSeconds !== null ? 'elapsed' : 'waiting for match start'}
      </p>
    </section>

    <section class="card mode">
      <p class="label">Operator mode</p>
      <p class="big">{state.mode ?? 'Not reported'}</p>
    </section>

    <section class="card power">
      <p class="label">Battery</p>
      <p class="big">{state.battery.volts !== null ? `${state.battery.volts.toFixed(2)} V` : '--'}</p>
      <p class="label">Current split ({state.power.totalAmps.toFixed(1)} A)</p>
      {#if state.power.consumers.length > 0}
        <div class="split" role="img" aria-label="current split by consumer">
          {#each state.power.consumers as consumer (consumer.name)}
            <span class="seg" style="width: {consumer.share * 100}%" title="{consumer.name}: {consumer.amps.toFixed(1)} A"></span>
          {/each}
        </div>
        <ul>
          {#each state.power.consumers as consumer (consumer.name)}
            <li><span>{consumer.name}</span><span>{consumer.amps.toFixed(1)} A</span><span>{Math.round(consumer.share * 100)}%</span></li>
          {/each}
        </ul>
      {:else}
        <p class="sub">No power telemetry published.</p>
      {/if}
    </section>

    <section class="card camera">
      <p class="label">Camera</p>
      {#if state.camera}
        <p class="sub">{state.camera.path}: {state.camera.value}</p>
      {:else}
        <div class="placeholder">Camera feed not connected</div>
      {/if}
    </section>

    {#if autoSelection}
      <section class="card auto">
        <p class="label">Autonomous routine</p>
        <p class="big">{autoSelection.value || 'None selected'}</p>
        <p class="sub">{autoSelection.path}</p>
      </section>
    {/if}
  </div>
</div>

<style>
  .driver {
    display: grid;
    gap: 16px;
    font-size: 18px;
  }
  .banner {
    display: flex;
    flex-wrap: wrap;
    gap: 10px;
    align-items: center;
    min-height: 64px;
    padding: 12px 18px;
    border-radius: 14px;
    background: #7f1d1d;
    color: #fff;
  }
  .banner.clear {
    background: #14532d;
  }
  .banner strong {
    font-size: 26px;
  }
  .alert {
    font-size: 22px;
    font-weight: 700;
    padding: 4px 12px;
    border-radius: 8px;
    background: rgba(0, 0, 0, 0.25);
  }
  .alert.warn {
    background: #92400e;
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
    gap: 14px;
  }
  .card {
    display: grid;
    gap: 6px;
    align-content: start;
    padding: 18px;
    border-radius: 16px;
    background: var(--surface);
    border: 1px solid var(--border);
    min-width: 0;
  }
  .match {
    grid-row: span 2;
  }
  .label {
    margin: 0;
    font-size: 14px;
    text-transform: uppercase;
    letter-spacing: 0.06em;
    color: var(--text-soft);
  }
  .phase {
    margin: 0;
    font-size: 30px;
    font-weight: 700;
    color: var(--text-strong);
  }
  .clock {
    margin: 0;
    font-family: var(--font-mono);
    font-size: clamp(64px, 12vw, 120px);
    line-height: 1;
    color: var(--text-strong);
  }
  .big {
    margin: 0;
    font-family: var(--font-mono);
    font-size: 40px;
    color: var(--text-strong);
    overflow-wrap: anywhere;
  }
  .sub {
    margin: 0;
    color: var(--text-soft);
    font-size: 15px;
  }
  .split {
    display: flex;
    height: 22px;
    border-radius: 6px;
    overflow: hidden;
    background: var(--bg-2);
  }
  .seg {
    display: block;
    height: 100%;
    background: linear-gradient(90deg, #38bdf8, #818cf8);
  }
  .seg + .seg {
    border-left: 2px solid var(--bg);
  }
  ul {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    gap: 4px;
  }
  li {
    display: grid;
    grid-template-columns: 1fr auto auto;
    gap: 10px;
    font-family: var(--font-mono);
    font-size: 16px;
  }
  .placeholder {
    display: grid;
    place-items: center;
    min-height: 120px;
    border: 2px dashed var(--border);
    border-radius: 12px;
    color: var(--text-soft);
  }
</style>
