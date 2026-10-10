<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { MechanismNode } from '../../lib/data/tree';
  import { groupSignals } from '../../lib/data/tree';
  import {
    actionSignals,
    controlSections,
    deviceSections,
    summarize,
    valueSignals
  } from '../../lib/mechanisms/model';
  import ActionsSection from './ActionsSection.svelte';
  import ControlSection from './ControlSection.svelte';
  import DeviceSection from './DeviceSection.svelte';
  import SignalField from './SignalField.svelte';
  import { formatValue } from '../../lib/format';

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
  const stateValue = $derived.by(() => {
    void live.current;
    return summary.state ? session.store.latestSample(summary.state.signalId)?.value : undefined;
  });
  const reasonValue = $derived.by(() => {
    void live.current;
    return summary.reason ? session.store.latestSample(summary.reason.signalId)?.value : undefined;
  });
  const stageValue = $derived.by(() => {
    void live.current;
    return summary.stage ? session.store.latestSample(summary.stage.signalId)?.value : undefined;
  });
  const otherState = $derived(
    groupSignals(node).State.filter((s) => s !== summary.state && s !== summary.reason && s !== summary.stage)
  );
</script>

<article class="page">
  <header class="head">
    <div>
      <p class="crumbs">{node.path}</p>
      <h2>{node.name}</h2>
    </div>
    <div class="status">
      <span class="pill state">{formatValue(stateValue)}</span>
      {#if summary.stage}
        <span class="pill">stage {formatValue(stageValue)}</span>
      {/if}
    </div>
  </header>

  {#if summary.reason}
    <p class="reason" data-testid="reason">{formatValue(reasonValue)}</p>
  {/if}

  {#if otherState.length > 0}
    <div class="grid">
      {#each otherState as signal (signal.signalId)}
        <SignalField {signal} {session} {live} />
      {/each}
    </div>
  {/if}

  <div class="layout">
    <ActionsSection {actions} {session} {live} />

    {#each controls as section (section.path)}
      <ControlSection {section} {session} {live} />
    {/each}

    <DeviceSection {devices} {session} {live} />

    {#if values.length > 0}
      <section class="card" aria-labelledby="values-heading">
        <h3 id="values-heading">Values</h3>
        <div class="grid">
          {#each values as signal (signal.signalId)}
            <SignalField {signal} {session} {live} />
          {/each}
        </div>
      </section>
    {/if}
  </div>
</article>

<style>
  .page {
    display: grid;
    gap: 14px;
    min-width: 0;
  }
  .head {
    display: flex;
    justify-content: space-between;
    gap: 12px;
    align-items: flex-start;
    flex-wrap: wrap;
  }
  .crumbs {
    margin: 0;
    font-family: var(--font-mono);
    font-size: 11px;
    color: var(--text-soft);
  }
  h2 {
    margin: 2px 0 0;
    font-family: var(--font-display);
    font-size: 22px;
    color: var(--text-strong);
  }
  .status {
    display: flex;
    gap: 6px;
    flex-wrap: wrap;
  }
  .pill {
    font-size: 12px;
    padding: 3px 9px;
    border-radius: 999px;
    border: 1px solid var(--border-subtle);
    color: var(--text);
    font-family: var(--font-mono);
  }
  .pill.state {
    border-color: var(--brand);
    color: var(--text-strong);
  }
  .reason {
    margin: 0;
    font-size: 13px;
    color: var(--text);
    background: var(--surface-2);
    border-left: 3px solid var(--brand);
    padding: 6px 10px;
    border-radius: 6px;
  }
  .layout {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(340px, 1fr));
    gap: 12px;
    align-items: start;
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(190px, 1fr));
    gap: 8px;
  }
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 12px;
    display: grid;
    gap: 8px;
  }
  h3 {
    margin: 0;
    font-family: var(--font-display);
    font-size: 13px;
    color: var(--text-strong);
  }
</style>
