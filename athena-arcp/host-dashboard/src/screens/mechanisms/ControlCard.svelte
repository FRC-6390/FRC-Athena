<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { ControlSection } from '../../lib/mechanisms/model';
  import Plot from '../../ui/Plot.svelte';
  import ValueRow from '../../ui/ValueRow.svelte';

  type Props = {
    section: ControlSection;
    session: Session;
    live: { readonly current: number };
  };

  let { section, session, live }: Props = $props();

  const constraintLabel = (name: string) =>
    name.replace(/^max/, 'max ').replace(/^min/, 'min ').replace(/([a-z])([A-Z])/g, '$1 $2').toLowerCase();
</script>

<section class="ui-panel card">
  <header class="ui-panel-head">
    <h3 class="ui-panel-title">Control · {section.name}</h3>
    <span class="ui-panel-sub">{section.path}</span>
  </header>
  {#if section.plot.length > 0}
    <div class="ui-panel-body">
      <Plot signals={section.plot} store={session.store} {live} />
    </div>
  {/if}
  <div class="tables">
    {#if section.gains.length > 0}
      <div class="table">
        <div class="ui-label head">Gains</div>
        {#each section.gains as signal (signal.signalId)}
          <ValueRow {signal} {session} {live} />
        {/each}
      </div>
    {/if}
    {#if section.constraints.length > 0}
      <div class="table">
        <div class="ui-label head">Constraints</div>
        {#each section.constraints as signal (signal.signalId)}
          <ValueRow {signal} {session} {live} label={constraintLabel(signal.name)} />
        {/each}
      </div>
    {/if}
    {#if section.other.length > 0}
      <div class="table">
        <div class="ui-label head">Other</div>
        {#each section.other as signal (signal.signalId)}
          <ValueRow {signal} {session} {live} />
        {/each}
      </div>
    {/if}
  </div>
</section>

<style>
  .card {
    display: flex;
    flex-direction: column;
  }
  .tables {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    border-top: 1px solid var(--border);
  }
  .table + .table {
    border-left: 1px solid var(--border);
  }
  .head {
    padding: var(--sp-2) var(--sp-3) 6px;
  }
</style>
