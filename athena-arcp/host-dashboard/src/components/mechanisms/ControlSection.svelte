<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { ControlSection } from '../../lib/mechanisms/model';
  import ControlPlot from './ControlPlot.svelte';
  import SignalField from './SignalField.svelte';

  type Props = {
    section: ControlSection;
    session: Session;
    live: { readonly current: number };
  };

  let { section, session, live }: Props = $props();
</script>

<section class="card" aria-label="control {section.name}">
  <header>
    <h3>{section.name}</h3>
    <span class="path">{section.path}</span>
  </header>

  {#if section.plot.length > 0}
    <ControlPlot signals={section.plot} store={session.store} {live} />
  {/if}

  {#if section.gains.length > 0}
    <h4>Gains</h4>
    <div class="grid">
      {#each section.gains as signal (signal.signalId)}
        <SignalField {signal} {session} {live} label={signal.name} />
      {/each}
    </div>
  {/if}

  {#if section.constraints.length > 0}
    <h4>Constraints</h4>
    <div class="grid">
      {#each section.constraints as signal (signal.signalId)}
        <SignalField {signal} {session} {live} label={signal.name} />
      {/each}
    </div>
  {/if}

  {#if section.other.length > 0}
    <h4>Other</h4>
    <div class="grid">
      {#each section.other as signal (signal.signalId)}
        <SignalField {signal} {session} {live} label={signal.name} />
      {/each}
    </div>
  {/if}
</section>

<style>
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 12px;
    display: grid;
    gap: 10px;
    min-width: 0;
  }
  header {
    display: flex;
    gap: 10px;
    align-items: baseline;
    flex-wrap: wrap;
  }
  h3 {
    margin: 0;
    font-family: var(--font-display);
    font-size: 14px;
    color: var(--text-strong);
  }
  h4 {
    margin: 4px 0 0;
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.06em;
    color: var(--text-soft);
  }
  .path {
    font-family: var(--font-mono);
    font-size: 11px;
    color: var(--text-soft);
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(190px, 1fr));
    gap: 8px;
  }
</style>
