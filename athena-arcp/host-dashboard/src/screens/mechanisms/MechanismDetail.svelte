<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { MechanismNode } from '../../lib/data/tree';
  import { groupSignals } from '../../lib/data/tree';
  import { actionSignals, controlSections, deviceSections, summarize, valueSignals } from '../../lib/mechanisms/model';
  import { formatValue } from '../../lib/format';
  import { reasonTone, stateTone } from './tones';
  import ControlCard from './ControlCard.svelte';
  import ActionsCard from './ActionsCard.svelte';
  import DevicesCard from './DevicesCard.svelte';
  import ValueRow from '../../ui/ValueRow.svelte';

  type Props = {
    node: MechanismNode;
    session: Session;
    live: { readonly current: number };
  };

  let { node, session, live }: Props = $props();

  const summary = $derived(summarize(node));
  const controls = $derived(controlSections(node));
  const devices = $derived(deviceSections(node));
  const actions = $derived(actionSignals(node));
  const values = $derived(valueSignals(node));
  const extraState = $derived(
    groupSignals(node).State.filter((s) => s !== summary.state && s !== summary.reason && s !== summary.stage)
  );

  const latest = (id: number | undefined) => {
    void live.current;
    return id === undefined ? undefined : session.store.latestSample(id)?.value;
  };
  const state = $derived(formatValue(latest(summary.state?.signalId)));
  const reason = $derived(summary.reason ? String(latest(summary.reason.signalId) ?? '') : '');
  const stage = $derived(summary.stage ? String(latest(summary.stage.signalId) ?? '') : '');
</script>

<div class="detail">
  <header class="head">
    <div class="names">
      <h2>{node.name}</h2>
      <span class="path ui-mono">{node.path}</span>
    </div>
    <div class="badges">
      {#if summary.state}<span class="ui-badge" data-tone={stateTone(state)}>{state}</span>{/if}
      {#if summary.stage && stage}<span class="ui-badge" data-tone={stage === 'NORMAL' ? 'stale' : 'warn'}>stage {stage}</span>{/if}
    </div>
  </header>

  {#if reason}
    <div class="reason" data-tone={reasonTone(reason)} data-testid="reason">
      <span class="ui-label">Reason</span>
      <span class="text">{reason}</span>
    </div>
  {/if}

  <div class="columns">
    <div class="main">
      {#each controls as section (section.path)}
        <ControlCard {section} {session} {live} />
      {/each}
      {#if values.length > 0 || extraState.length > 0}
        <section class="ui-panel">
          <header class="ui-panel-head"><h3 class="ui-panel-title">Values</h3></header>
          {#each [...extraState, ...values] as signal (signal.signalId)}
            <ValueRow {signal} {session} {live} label={signal.localPath.replace(/^(Values|State)\//, '')} />
          {/each}
        </section>
      {/if}
    </div>
    <div class="side">
      <ActionsCard {actions} {session} {live} />
      <DevicesCard {devices} {session} {live} />
    </div>
  </div>
</div>

<style>
  .detail {
    display: grid;
    gap: var(--sp-3);
    min-width: 0;
  }
  .head {
    display: flex;
    align-items: center;
    gap: var(--sp-3);
    flex-wrap: wrap;
  }
  .names {
    display: flex;
    align-items: baseline;
    gap: var(--sp-2);
    min-width: 0;
  }
  h2 {
    margin: 0;
    font-size: var(--fs-xl);
    font-weight: 600;
    color: var(--text-strong);
    text-transform: capitalize;
  }
  .path {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .badges {
    display: flex;
    gap: 6px;
  }
  .reason {
    display: flex;
    align-items: center;
    gap: var(--sp-3);
    min-height: 34px;
    padding: 0 var(--sp-3);
    border-radius: var(--radius);
    border: 1px solid var(--border);
    background: var(--surface);
  }
  .reason .text {
    font-size: var(--fs-sm);
    color: var(--text);
  }
  .reason[data-tone='fault'] { border-color: color-mix(in srgb, var(--fault) 45%, transparent); background: var(--fault-soft); }
  .reason[data-tone='warn'] { border-color: color-mix(in srgb, var(--warn) 45%, transparent); background: var(--warn-soft); }
  .reason[data-tone='ok'] .text { color: var(--text-soft); }
  .columns {
    display: grid;
    grid-template-columns: minmax(0, 1fr) minmax(280px, 360px);
    gap: var(--sp-3);
    align-items: start;
  }
  .main,
  .side {
    display: grid;
    gap: var(--sp-3);
    min-width: 0;
  }
  @media (max-width: 1100px) {
    .columns {
      grid-template-columns: minmax(0, 1fr);
    }
  }
</style>
